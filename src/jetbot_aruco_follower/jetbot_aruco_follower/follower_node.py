"""Convert selected ArUco targets into exclusive /cmd_vel_joy commands."""

import json
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

from .control import FollowConfig, FollowController, Target


ENABLE_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class ArucoFollower(Node):
    """Follow detector observations only while explicitly enabled."""

    def __init__(self, parameter_overrides=None):
        super().__init__(
            'aruco_follower', parameter_overrides=parameter_overrides)
        defaults = {
            'target_topic': '/aruco/target',
            'enabled_topic': '/aruco/enabled',
            'velocity_topic': '/cmd_vel_joy',
            'status_topic': '/aruco/follower_status',
            'stop_distance_m': 0.5,
            'max_linear_mps': 0.08,
            'max_angular_radps': 0.6,
            'marker_to_front_offset_m': 0.0,
            'linear_kp': 0.45,
            'angular_kp': 1.8,
            'stop_tolerance_m': 0.02,
            'align_before_forward_rad': 0.35,
            'loss_grace_s': 0.8,
            'search_pause_s': 0.5,
            'search_min_angle_rad': 0.5,
            'search_max_angle_rad': 1.5,
            'camera_timeout_s': 1.0,
            'target_timeout_s': 0.4,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        def param(name):
            return self.get_parameter(name).value

        self.controller = FollowController(FollowConfig(
            stop_distance_m=float(param('stop_distance_m')),
            max_linear_mps=float(param('max_linear_mps')),
            max_angular_radps=float(param('max_angular_radps')),
            marker_to_front_offset_m=float(
                param('marker_to_front_offset_m')),
            linear_kp=float(param('linear_kp')),
            angular_kp=float(param('angular_kp')),
            stop_tolerance_m=float(param('stop_tolerance_m')),
            align_before_forward_rad=float(
                param('align_before_forward_rad')),
            loss_grace_s=float(param('loss_grace_s')),
            search_pause_s=float(param('search_pause_s')),
            search_min_angle_rad=float(param('search_min_angle_rad')),
            search_max_angle_rad=float(param('search_max_angle_rad')),
        ))
        self.camera_timeout_s = float(param('camera_timeout_s'))
        self.target_timeout_s = float(param('target_timeout_s'))
        if self.camera_timeout_s <= 0 or self.target_timeout_s <= 0:
            raise ValueError(
                'camera and target timeouts must be positive')

        self.enabled = False
        self.last_frame_at = None
        self.last_detection_at = None
        self.last_target = None
        self.last_marker_id = None
        self.velocity_pub = self.create_publisher(
            Twist, str(param('velocity_topic')), 10)
        self.status_pub = self.create_publisher(
            String, str(param('status_topic')), 10)
        self.create_subscription(
            String,
            str(param('target_topic')),
            self._on_target,
            10,
        )
        self.create_subscription(
            Bool,
            str(param('enabled_topic')),
            self._on_enabled,
            ENABLE_QOS,
        )
        self.create_timer(0.1, self._tick)
        self.get_logger().info(
            'Follower ready and disabled; commands use /cmd_vel_joy')

    def _on_enabled(self, msg):
        enabled = bool(msg.data)
        if enabled == self.enabled:
            return
        self.enabled = enabled
        self.controller.reset()
        self.last_detection_at = None
        self.last_target = None
        self.last_marker_id = None
        if not enabled and self.context.ok():
            self.velocity_pub.publish(Twist())
        self.get_logger().info(
            'ArUco follower enabled' if enabled
            else 'ArUco follower disabled')

    def _on_target(self, msg):
        try:
            payload = json.loads(msg.data)
            if not isinstance(payload, dict):
                return
        except (json.JSONDecodeError, TypeError):
            return
        now = time.monotonic()
        if not bool(payload.get('camera_ok')):
            self.last_frame_at = None
            self.last_target = None
            self.last_marker_id = None
            return
        self.last_frame_at = now
        if not bool(payload.get('detected')):
            self.last_target = None
            self.last_marker_id = None
            return
        try:
            distance = float(payload['distance_m'])
            bearing = float(payload['bearing_rad'])
            marker_id = int(payload['marker_id'])
        except (KeyError, TypeError, ValueError):
            self.last_target = None
            self.last_marker_id = None
            return
        if (not math.isfinite(distance)
                or not math.isfinite(bearing)
                or distance <= 0):
            self.last_target = None
            self.last_marker_id = None
            return
        self.last_target = Target(distance, bearing)
        self.last_marker_id = marker_id
        self.last_detection_at = now

    def _tick(self):
        now = time.monotonic()
        if not self.enabled:
            self._publish_status('DISABLED', False, None, 0.0, 0.0)
            return
        camera_ok = (
            self.last_frame_at is not None
            and now - self.last_frame_at <= self.camera_timeout_s
        )
        target = self.last_target
        if (target is not None
                and self.last_detection_at is not None
                and now - self.last_detection_at
                > self.target_timeout_s):
            target = None
        command = self.controller.command(
            target,
            now,
            self.last_detection_at,
            camera_ok=camera_ok,
        )
        twist = Twist()
        twist.linear.x = command.linear_mps
        twist.angular.z = command.angular_radps
        self.velocity_pub.publish(twist)
        self._publish_status(
            command.state,
            camera_ok,
            target,
            command.linear_mps,
            command.angular_radps,
        )

    def _publish_status(
            self, state, camera_ok, target, linear_mps, angular_radps):
        payload = {
            'enabled': self.enabled,
            'state': state,
            'camera_ok': camera_ok,
            'marker_id': self.last_marker_id if target else None,
            'distance_m': (
                round(target.distance_m, 3) if target else None),
            'bearing_rad': (
                round(target.bearing_rad, 3) if target else None),
            'linear_mps': round(linear_mps, 3),
            'angular_radps': round(angular_radps, 3),
        }
        message = String()
        message.data = json.dumps(payload, separators=(',', ':'))
        self.status_pub.publish(message)

    def stop(self):
        """Send a final stop if shutdown occurs in autonomous mode."""
        if self.enabled and self.context.ok():
            self.velocity_pub.publish(Twist())


def main(args=None):
    """Run the exclusive ArUco follower node."""
    rclpy.init(args=args)
    node = None
    try:
        node = ArucoFollower()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.stop()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
