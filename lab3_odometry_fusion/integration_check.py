"""Actual adapters + Humble EKF integration on a private localhost domain ONLY."""
import json
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time
import urllib.error
import urllib.request


def main():
    domain = os.environ.get('ROS_DOMAIN_ID', '')
    if not domain.isdigit() or int(domain) in (0, 2) or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise SystemExit('Refusing test: private numeric ROS_DOMAIN_ID (not 0/2), ROS_LOCALHOST_ONLY=1 required')
    with socket.socket() as probe:
        if probe.connect_ex(('127.0.0.1', 8766)) == 0:
            raise SystemExit('Refusing test: HTTP port 8766 already occupied')
    import rclpy
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan
    from tf2_msgs.msg import TFMessage
    from rclpy.qos import qos_profile_sensor_data

    root = Path(__file__).resolve().parent
    rclpy.init(args=[])
    node = rclpy.create_node('lab3_isolated_measurement_fixture')
    wheel = node.create_publisher(Odometry, '/diffbot_base_controller/odom', 10)
    laser = node.create_publisher(Odometry, '/odom_rf2o', 10)
    scan = node.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
    received = []
    sent_stamps = set()
    tf_received = []
    node.create_subscription(Odometry, '/lab3/ekf/lidar', lambda m: received.append(m), 10)
    node.create_subscription(TFMessage, '/tf', lambda m: tf_received.append(m), 10)
    child = subprocess.Popen([sys.executable, '-B', str(root/'run_stack.py')])

    def status():
        try:
            with urllib.request.urlopen('http://127.0.0.1:8766/healthz', timeout=1) as response:
                return json.load(response)
        except (urllib.error.URLError, TimeoutError):
            return None

    def send(duration, displacement=0.0, velocity=0.0):
        deadline = time.monotonic()+duration
        while time.monotonic() < deadline:
            stamp = node.get_clock().now().to_msg()
            sent_stamps.add((stamp.sec, stamp.nanosec))
            for publisher, x, y in ((wheel, 1.+displacement, 2.), (laser, 10.+displacement, -3.)):
                msg = Odometry()
                msg.header.stamp = stamp
                msg.header.frame_id, msg.child_frame_id = 'odom', 'base_footprint'
                msg.pose.pose.position.x, msg.pose.pose.position.y = x, y
                msg.pose.pose.orientation.w = 1.
                msg.twist.twist.linear.x = velocity
                publisher.publish(msg)
            msg = LaserScan()
            msg.header.stamp, msg.header.frame_id = stamp, 'lidar_link'
            msg.range_min, msg.range_max = .15, 12.
            msg.ranges = [2.]*360
            scan.publish(msg)
            rclpy.spin_once(node, timeout_sec=.02)
            time.sleep(.03)

    try:
        deadline = time.monotonic()+30
        initial = None
        while time.monotonic() < deadline:
            assert child.poll() is None, 'Fusion supervisor exited'
            send(.5)
            initial = status()
            if initial and initial['input_health'] and initial['sources']['fused']['status']=='live':
                break
        assert initial and initial['input_health'], 'No fresh aligned sensor pair'
        assert initial['sources']['fused']['status']=='live', 'No actual EKF output'
        assert abs(initial['alignment']['x']+9)<1e-6
        assert abs(initial['alignment']['y']-5)<1e-6
        assert received and received[-1].pose.covariance[0]>.001, 'Missing positive covariance adapter'
        assert abs(received[-1].pose.pose.position.x-1)<1e-6
        assert (received[-1].header.stamp.sec, received[-1].header.stamp.nanosec) in sent_stamps
        assert .0 <= node.get_clock().now().nanoseconds*1e-9-(received[-1].header.stamp.sec+received[-1].header.stamp.nanosec*1e-9)<2
        for step in range(1, 11):
            send(.2, .01*step, .05)
        send(3., .1)
        moved = status()
        pose = moved['sources']['fused']['latest']
        assert pose and math.isfinite(pose['x']) and abs(pose['x']-1.1)<.07, pose
        # No new measurements: EKF may predict, but input health must become false.
        time.sleep(2.)
        stale = status()
        assert stale and not stale['input_health'], 'Input loss not detected'
        assert stale['sources']['wheel']['status']=='stale'
        assert stale['sources']['lidar']['status']=='stale'
        assert not tf_received, 'Unexpected TF broadcast in isolated fusion test'
        print('PASS actual Humble EKF, fixed alignment, positive covariance, preserved stamps, finite moving estimate, input-loss health, zero TF broadcasts; private domain only')
    finally:
        child.send_signal(signal.SIGTERM)
        try:
            child.wait(timeout=10)
        except subprocess.TimeoutExpired:
            child.kill()
            child.wait()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
