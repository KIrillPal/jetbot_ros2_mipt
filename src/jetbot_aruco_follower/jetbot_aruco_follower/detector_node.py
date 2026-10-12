"""Detect every ArUco marker and publish the one nearest image center."""

import json
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, CompressedImage
from std_msgs.msg import String

from .vision import CameraCalibration, MarkerDetector, estimated_calibration


class ArucoDetector(Node):
    """Decode camera JPEGs and publish one centrally selected target."""

    def __init__(self, parameter_overrides=None):
        super().__init__(
            'aruco_detector', parameter_overrides=parameter_overrides)
        cv2.setNumThreads(1)
        defaults = {
            'camera_topic': '/camera/image/compressed',
            'camera_info_topic': '/camera/camera_info',
            'target_topic': '/aruco/target',
            'marker_size_m': 0.10,
            'aruco_dictionary': 'DICT_4X4_50',
            'min_side_px': 14.0,
            'adaptive_thresh_constant': 7.0,
            'min_otsu_std_dev': 5.0,
            'max_erroneous_bits_in_border_rate': 0.35,
            'error_correction_rate': 0.6,
            'camera_focal_length_px': 0.0,
            'camera_horizontal_fov_deg': 62.0,
            'max_detection_hz': 10.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        def param(name):
            return self.get_parameter(name).value

        self.detector = MarkerDetector(
            float(param('marker_size_m')),
            target_marker_id=-1,
            dictionary_name=str(param('aruco_dictionary')),
            min_side_px=float(param('min_side_px')),
            adaptive_thresh_constant=float(
                param('adaptive_thresh_constant')),
            min_otsu_std_dev=float(param('min_otsu_std_dev')),
            max_erroneous_bits_in_border_rate=float(
                param('max_erroneous_bits_in_border_rate')),
            error_correction_rate=float(param('error_correction_rate')),
        )
        self.focal_length_px = float(
            param('camera_focal_length_px'))
        self.horizontal_fov_deg = float(
            param('camera_horizontal_fov_deg'))
        max_detection_hz = float(param('max_detection_hz'))
        if max_detection_hz <= 0:
            raise ValueError('max_detection_hz must be positive')
        self.min_detection_period = 1.0 / max_detection_hz
        self.last_processed_at = None
        self.calibration = None
        self.calibration_size = None
        self.publisher = self.create_publisher(
            String, str(param('target_topic')), 10)
        self.create_subscription(
            CompressedImage,
            str(param('camera_topic')),
            self._on_image,
            qos_profile_sensor_data,
        )
        info_topic = str(param('camera_info_topic'))
        if info_topic:
            self.create_subscription(
                CameraInfo,
                info_topic,
                self._on_camera_info,
                qos_profile_sensor_data,
            )
        self.get_logger().info(
            'Selecting the ArUco marker nearest the image center')

    def _on_camera_info(self, msg):
        if msg.k[0] <= 10 or msg.k[4] <= 10:
            return
        distortion = (
            tuple(float(value) for value in msg.d)
            if len(msg.d) in (4, 5, 8, 12, 14)
            else (0.0, 0.0, 0.0, 0.0, 0.0)
        )
        self.calibration = CameraCalibration(
            float(msg.k[0]),
            float(msg.k[4]),
            float(msg.k[2]),
            float(msg.k[5]),
            distortion,
        )
        self.calibration_size = (msg.width, msg.height)

    def _on_image(self, msg):
        now = time.monotonic()
        if (self.last_processed_at is not None
                and now - self.last_processed_at
                < self.min_detection_period):
            return
        self.last_processed_at = now
        try:
            image = cv2.imdecode(
                np.frombuffer(msg.data, dtype=np.uint8),
                cv2.IMREAD_COLOR,
            )
        except cv2.error:
            image = None
        if image is None:
            self._publish({'camera_ok': False, 'detected': False})
            return

        height, width = image.shape[:2]
        calibration = self._calibration_for(width, height)
        detection = self.detector.detect(image, calibration)
        payload = {
            'camera_ok': True,
            'detected': detection is not None,
            'image_width': width,
            'image_height': height,
        }
        if detection is not None:
            payload.update({
                'marker_id': detection.marker_id,
                'distance_m': round(detection.distance_m, 6),
                'bearing_rad': round(detection.bearing_rad, 6),
                'side_px': round(detection.side_px, 3),
                'center_x_px': round(detection.center_x_px, 3),
                'center_y_px': round(detection.center_y_px, 3),
            })
        self._publish(payload)

    def _calibration_for(self, width, height):
        calibration = (
            None if self.focal_length_px > 0 else self.calibration)
        if (calibration is not None
                and self.calibration_size != (width, height)):
            source_width, source_height = self.calibration_size
            if source_width > 0 and source_height > 0:
                scale_x = width / source_width
                scale_y = height / source_height
                calibration = CameraCalibration(
                    calibration.focal_x_px * scale_x,
                    calibration.focal_y_px * scale_y,
                    calibration.center_x_px * scale_x,
                    calibration.center_y_px * scale_y,
                    calibration.distortion,
                )
            else:
                calibration = None
        if calibration is None:
            calibration = estimated_calibration(
                width,
                height,
                self.horizontal_fov_deg,
                self.focal_length_px,
            )
        return calibration

    def _publish(self, payload):
        message = String()
        message.data = json.dumps(payload, separators=(',', ':'))
        self.publisher.publish(message)


def main(args=None):
    """Run the central ArUco detector node."""
    rclpy.init(args=args)
    node = None
    try:
        node = ArucoDetector()
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
