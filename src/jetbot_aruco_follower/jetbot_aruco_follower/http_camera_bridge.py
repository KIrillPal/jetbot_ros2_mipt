"""Republish fresh JPEGs from the host camera server to ROS 2."""

import json
import math
from pathlib import Path
import time
import urllib.error
import urllib.request

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, CompressedImage
import yaml


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
        self.declare_parameter('camera_info_file', '')
        self.declare_parameter(
            'camera_info_topic', '/camera/camera_info')
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
        info_file = str(self.get_parameter('camera_info_file').value)
        self.camera_info = (
            load_camera_info(info_file, self.frame_id) if info_file else None)
        info_topic = str(self.get_parameter('camera_info_topic').value)
        self.info_publisher = (
            self.create_publisher(
                CameraInfo, info_topic, qos_profile_sensor_data)
            if self.camera_info is not None else None)
        self.last_seq = -1
        self.last_warning_at = 0.0
        self.create_timer(1.0 / publish_hz, self._tick)
        if self.camera_info is None:
            self.get_logger().info(
                f'Bridging fresh JPEGs from {self.base_url} to {topic}')
        else:
            self.get_logger().info(
                f'Bridging fresh JPEGs from {self.base_url} to {topic}; '
                f'camera info {self.camera_info.width}x'
                f'{self.camera_info.height} from {info_file}')

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
        if self.camera_info is not None:
            self.camera_info.header.stamp = msg.header.stamp
            self.info_publisher.publish(self.camera_info)
        self.last_seq = seq


def load_camera_info(path, frame_id):
    """Load a ROS calibration YAML file into a CameraInfo message."""
    try:
        data = yaml.safe_load(Path(path).read_text(encoding='utf-8'))
    except (OSError, UnicodeError, yaml.YAMLError) as error:
        raise ValueError(f'cannot read camera info {path}: {error}') from error
    if not isinstance(data, dict):
        raise ValueError(f'camera info {path} must be a YAML mapping')
    try:
        width = int(data['image_width'])
        height = int(data['image_height'])
        matrix = [float(value) for value in data['camera_matrix']['data']]
        distortion = [
            float(value) for value in data['distortion_coefficients']['data']]
        rectification = [
            float(value) for value in data['rectification_matrix']['data']]
        projection = [
            float(value) for value in data['projection_matrix']['data']]
        model = str(data['distortion_model'])
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError(
            f'camera info {path} is incomplete: {error}') from error
    if (width <= 0 or height <= 0 or len(matrix) != 9
            or len(distortion) not in (4, 5, 8, 12, 14)
            or len(rectification) != 9 or len(projection) != 12
            or matrix[0] <= 10 or matrix[4] <= 10):
        raise ValueError(f'camera info {path} has invalid calibration values')
    message = CameraInfo()
    message.header.frame_id = frame_id
    message.width = width
    message.height = height
    message.distortion_model = model
    message.d = distortion
    message.k = matrix
    message.r = rectification
    message.p = projection
    return message


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
