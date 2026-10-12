"""Republish fresh JPEGs from the host camera server to ROS 2."""

import json
import math
import time
import urllib.error
import urllib.request

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage


class HttpCameraBridge(Node):
    """Bridge the single-owner CSI HTTP stream without reopening the camera."""

    def __init__(self, parameter_overrides=None):
        super().__init__(
            'aruco_http_camera_bridge',
            parameter_overrides=parameter_overrides,
        )
        self.declare_parameter('base_url', 'http://127.0.0.1:8080')
        self.declare_parameter(
            'camera_topic', '/camera/image/compressed')
        self.declare_parameter('frame_id', 'camera_link')
        self.declare_parameter('publish_hz', 10.0)
        self.declare_parameter('request_timeout_s', 0.4)
        self.base_url = str(
            self.get_parameter('base_url').value).rstrip('/')
        self.frame_id = str(self.get_parameter('frame_id').value)
        publish_hz = float(self.get_parameter('publish_hz').value)
        self.request_timeout_s = float(
            self.get_parameter('request_timeout_s').value)
        if not self.base_url.startswith('http://'):
            raise ValueError('base_url must be an HTTP URL')
        if not math.isfinite(publish_hz) or publish_hz <= 0:
            raise ValueError('publish_hz must be positive and finite')
        if (not math.isfinite(self.request_timeout_s)
                or self.request_timeout_s <= 0):
            raise ValueError(
                'request_timeout_s must be positive and finite')
        topic = str(self.get_parameter('camera_topic').value)
        self.publisher = self.create_publisher(
            CompressedImage, topic, qos_profile_sensor_data)
        self.last_seq = -1
        self.last_warning_at = 0.0
        self.create_timer(1.0 / publish_hz, self._tick)
        self.get_logger().info(
            f'Bridging fresh JPEGs from {self.base_url} to {topic}')

    def _fetch(self, path, max_bytes):
        request = urllib.request.Request(
            self.base_url + path,
            headers={'Cache-Control': 'no-cache'},
        )
        with urllib.request.urlopen(
                request, timeout=self.request_timeout_s) as response:
            data = response.read(max_bytes + 1)
            if len(data) > max_bytes:
                raise ValueError(f'{path} response too large')
            return data

    def _tick(self):
        try:
            stats = json.loads(
                self._fetch('/stats', 4096).decode('utf-8'))
            seq = int(stats['seq'])
            if seq <= 0 or seq == self.last_seq:
                return
            jpeg = self._fetch('/snapshot.jpg', 2_000_000)
            if (len(jpeg) < 4
                    or not jpeg.startswith(b'\xff\xd8')
                    or not jpeg.endswith(b'\xff\xd9')):
                raise ValueError(
                    'camera server did not return a complete JPEG')
        except (
                urllib.error.URLError, TimeoutError, OSError, ValueError,
                TypeError, KeyError, UnicodeDecodeError,
                json.JSONDecodeError) as error:
            now = time.monotonic()
            if now - self.last_warning_at >= 5.0:
                self.get_logger().warning(
                    f'Camera bridge waiting for fresh frames: {error}')
                self.last_warning_at = now
            return
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.format = 'jpeg'
        msg.data = jpeg
        self.publisher.publish(msg)
        self.last_seq = seq


def main(args=None):
    """Run the HTTP camera bridge node."""
    rclpy.init(args=args)
    node = None
    try:
        node = HttpCameraBridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
