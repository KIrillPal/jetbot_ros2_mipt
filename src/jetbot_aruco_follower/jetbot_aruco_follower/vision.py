"""ArUco detection, central-target selection, and metric pose estimation."""

from dataclasses import dataclass
import math

import cv2
import numpy as np


@dataclass(frozen=True)
class CameraCalibration:
    """Pinhole camera intrinsics used by pose estimation."""

    focal_x_px: float
    focal_y_px: float
    center_x_px: float
    center_y_px: float
    distortion: tuple = (0.0, 0.0, 0.0, 0.0, 0.0)


@dataclass(frozen=True)
class Detection:
    """One detected marker and its location in image/camera coordinates."""

    marker_id: int
    distance_m: float
    bearing_rad: float
    side_px: float
    center_x_px: float
    center_y_px: float


def estimated_calibration(
        width, height, horizontal_fov_deg, focal_length_px=0.0):
    """Build approximate intrinsics from image size and horizontal FOV."""
    if width <= 0 or height <= 0:
        raise ValueError('image dimensions must be positive')
    if not math.isfinite(focal_length_px):
        raise ValueError('focal_length_px must be finite')
    if focal_length_px > 0:
        focal = float(focal_length_px)
    else:
        if not 1.0 < horizontal_fov_deg < 179.0:
            raise ValueError(
                'horizontal_fov_deg must be between 1 and 179')
        focal = width / (
            2.0 * math.tan(math.radians(horizontal_fov_deg) / 2.0))
    return CameraCalibration(
        focal,
        focal,
        (width - 1) / 2.0,
        (height - 1) / 2.0,
    )


def select_central(detections, calibration):
    """Choose the marker whose center is nearest the principal point."""
    if not detections:
        return None
    return min(
        detections,
        key=lambda item: (
            (item.center_x_px - calibration.center_x_px) ** 2
            + (item.center_y_px - calibration.center_y_px) ** 2
        ),
    )


class MarkerDetector:
    """Detect configured ArUco markers and estimate planar range/bearing."""

    def __init__(
            self, marker_size_m, target_marker_id=-1,
            dictionary_name='DICT_4X4_50', min_side_px=14.0,
            adaptive_thresh_constant=7.0, min_otsu_std_dev=5.0,
            max_erroneous_bits_in_border_rate=0.35,
            error_correction_rate=0.6):
        if not math.isfinite(marker_size_m) or marker_size_m <= 0:
            raise ValueError(
                'marker_size_m must be positive and finite')
        if target_marker_id < -1:
            raise ValueError(
                'target_marker_id must be -1 or a nonnegative ID')
        if not math.isfinite(min_side_px) or min_side_px <= 0:
            raise ValueError('min_side_px must be positive and finite')
        self._validate_sensitivity(
            adaptive_thresh_constant,
            min_otsu_std_dev,
            max_erroneous_bits_in_border_rate,
            error_correction_rate,
        )
        if not hasattr(cv2, 'aruco'):
            raise RuntimeError(
                'OpenCV ArUco support is missing; install python3-opencv')
        if not dictionary_name.startswith('DICT_'):
            raise ValueError('invalid ArUco dictionary')
        dictionary_id = getattr(cv2.aruco, dictionary_name, None)
        if not isinstance(dictionary_id, int):
            raise ValueError(
                f'unknown ArUco dictionary: {dictionary_name}')
        self.dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
        parameters = (
            cv2.aruco.DetectorParameters_create()
            if hasattr(cv2.aruco, 'DetectorParameters_create')
            else cv2.aruco.DetectorParameters()
        )
        parameters.adaptiveThreshConstant = float(
            adaptive_thresh_constant)
        parameters.minOtsuStdDev = float(min_otsu_std_dev)
        parameters.maxErroneousBitsInBorderRate = float(
            max_erroneous_bits_in_border_rate)
        parameters.errorCorrectionRate = float(error_correction_rate)
        self.detector = (
            cv2.aruco.ArucoDetector(self.dictionary, parameters)
            if hasattr(cv2.aruco, 'ArucoDetector')
            else None
        )
        self.parameters = parameters
        self.marker_size_m = marker_size_m
        self.target_marker_id = target_marker_id
        self.min_side_px = min_side_px

    @staticmethod
    def _validate_sensitivity(
            adaptive_thresh_constant, min_otsu_std_dev,
            max_erroneous_bits_in_border_rate, error_correction_rate):
        checks = (
            ('adaptive_thresh_constant', adaptive_thresh_constant, 0.0, None),
            ('min_otsu_std_dev', min_otsu_std_dev, 0.0, None),
            (
                'max_erroneous_bits_in_border_rate',
                max_erroneous_bits_in_border_rate,
                0.0,
                1.0,
            ),
            ('error_correction_rate', error_correction_rate, 0.0, 1.0),
        )
        for name, value, lower, upper in checks:
            if not math.isfinite(value) or value < lower:
                raise ValueError(f'{name} must be finite and >= {lower}')
            if upper is not None and value > upper:
                raise ValueError(f'{name} must be <= {upper}')

    def detect(self, image, calibration):
        """Return the allowed marker nearest the image principal point."""
        return select_central(self.detect_all(image, calibration), calibration)

    def detect_all(self, image, calibration):
        """Return pose estimates for every allowed, sufficiently large marker."""
        if image is None or image.size == 0:
            return ()
        gray = (
            cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            if image.ndim == 3 else image
        )
        if self.detector is not None:
            corners, ids, _rejected = self.detector.detectMarkers(gray)
        else:
            corners, ids, _rejected = cv2.aruco.detectMarkers(
                gray,
                self.dictionary,
                parameters=self.parameters,
            )
        if ids is None:
            return ()

        detections = []
        for marker_corners, marker_id in zip(corners, ids.flatten()):
            marker_id = int(marker_id)
            if (self.target_marker_id >= 0
                    and marker_id != self.target_marker_id):
                continue
            points = np.asarray(
                marker_corners, dtype=np.float32).reshape(4, 2)
            sides = np.linalg.norm(
                points - np.roll(points, -1, axis=0), axis=1)
            side_px = float(np.mean(sides))
            if side_px < self.min_side_px:
                continue
            detection = self._estimate(
                marker_id, points, side_px, calibration)
            if detection is not None:
                detections.append(detection)
        return tuple(detections)

    def _estimate(self, marker_id, points, side_px, calibration):
        half = self.marker_size_m / 2.0
        object_points = np.array([
            [-half, half, 0.0],
            [half, half, 0.0],
            [half, -half, 0.0],
            [-half, -half, 0.0],
        ], dtype=np.float32)
        matrix = np.array([
            [calibration.focal_x_px, 0.0, calibration.center_x_px],
            [0.0, calibration.focal_y_px, calibration.center_y_px],
            [0.0, 0.0, 1.0],
        ], dtype=np.float64)
        distortion = np.asarray(
            calibration.distortion, dtype=np.float64)
        try:
            success, _rotation, translation = cv2.solvePnP(
                object_points,
                points,
                matrix,
                distortion,
                flags=cv2.SOLVEPNP_IPPE_SQUARE,
            )
        except cv2.error:
            return None
        if not success:
            return None
        xyz = translation.reshape(3)
        if xyz[2] <= 0 or not np.all(np.isfinite(xyz)):
            return None
        center_x = float(np.mean(points[:, 0]))
        center_y = float(np.mean(points[:, 1]))
        distance = math.hypot(float(xyz[0]), float(xyz[2]))
        bearing = math.atan2(
            calibration.center_x_px - center_x,
            calibration.focal_x_px,
        )
        return Detection(
            marker_id,
            distance,
            bearing,
            side_px,
            center_x,
            center_y,
        )
