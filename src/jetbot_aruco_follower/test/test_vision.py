"""Synthetic-image tests for central ArUco selection and pose."""

from pathlib import Path
import sys
import unittest

import cv2
import numpy as np

cv2.setNumThreads(1)

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from jetbot_aruco_follower.vision import (  # noqa: E402
    MarkerDetector,
    estimated_calibration,
)


def marker_image(marker_id, side):
    """Generate one DICT_4X4_50 marker across OpenCV versions."""
    dictionary = cv2.aruco.getPredefinedDictionary(
        cv2.aruco.DICT_4X4_50)
    if hasattr(cv2.aruco, 'generateImageMarker'):
        return cv2.aruco.generateImageMarker(
            dictionary, marker_id, side)
    return cv2.aruco.drawMarker(dictionary, marker_id, side)


def canvas_with(markers, width=640, height=480):
    """Place `(id, left, top, side)` markers on a white BGR canvas."""
    canvas = np.full((height, width, 3), 255, dtype=np.uint8)
    for marker_id, left, top, side in markers:
        marker = marker_image(marker_id, side)
        canvas[top:top + side, left:left + side] = cv2.cvtColor(
            marker, cv2.COLOR_GRAY2BGR)
    return canvas


class VisionTests(unittest.TestCase):
    """Validate selection using only generated camera frames."""

    def setUp(self):
        self.calibration = estimated_calibration(640, 480, 62.0)
        self.detector = MarkerDetector(
            0.10, target_marker_id=-1)

    def test_center_marker_wins_even_when_edge_marker_is_larger(self):
        image = canvas_with([
            (2, 30, 30, 130),
            (7, 285, 205, 70),
        ])
        detection = self.detector.detect(image, self.calibration)
        self.assertIsNotNone(detection)
        self.assertEqual(detection.marker_id, 7)
        self.assertAlmostEqual(detection.center_x_px, 320, delta=2)
        self.assertAlmostEqual(detection.center_y_px, 240, delta=2)

    def test_any_dictionary_id_can_be_selected(self):
        detection = self.detector.detect(
            canvas_with([(23, 270, 190, 100)]),
            self.calibration,
        )
        self.assertIsNotNone(detection)
        self.assertEqual(detection.marker_id, 23)

    def test_centered_marker_estimates_range(self):
        detection = self.detector.detect(
            canvas_with([(0, 270, 190, 100)]),
            self.calibration,
        )
        self.assertIsNotNone(detection)
        expected = self.calibration.focal_x_px * 0.10 / 100
        self.assertAlmostEqual(
            detection.distance_m, expected, delta=0.06)
        self.assertAlmostEqual(
            detection.bearing_rad, 0.0, delta=0.03)

    def test_left_and_right_bearing_signs(self):
        left = self.detector.detect(
            canvas_with([(1, 80, 190, 100)]),
            self.calibration,
        )
        right = self.detector.detect(
            canvas_with([(1, 460, 190, 100)]),
            self.calibration,
        )
        self.assertGreater(left.bearing_rad, 0)
        self.assertLess(right.bearing_rad, 0)

    def test_jpeg_artifacts_preserve_central_selection(self):
        image = canvas_with([
            (4, 40, 50, 120),
            (9, 280, 200, 80),
        ])
        ok, jpeg = cv2.imencode(
            '.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 25])
        self.assertTrue(ok)
        decoded = cv2.imdecode(jpeg, cv2.IMREAD_COLOR)
        detection = self.detector.detect(decoded, self.calibration)
        self.assertIsNotNone(detection)
        self.assertEqual(detection.marker_id, 9)

    def test_perspective_warp_is_detected(self):
        marker = marker_image(11, 160)
        source = np.float32([
            [0, 0], [159, 0], [159, 159], [0, 159],
        ])
        target = np.float32([
            [255, 155], [390, 170], [375, 310], [265, 295],
        ])
        transform = cv2.getPerspectiveTransform(source, target)
        gray = cv2.warpPerspective(
            marker,
            transform,
            (640, 480),
            borderValue=255,
        )
        image = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        detection = self.detector.detect(image, self.calibration)
        self.assertIsNotNone(detection)
        self.assertEqual(detection.marker_id, 11)

    def test_blank_and_subthreshold_marker_are_ignored(self):
        blank = np.full((480, 640, 3), 255, dtype=np.uint8)
        self.assertIsNone(
            self.detector.detect(blank, self.calibration))
        strict = MarkerDetector(
            0.10, target_marker_id=-1, min_side_px=40)
        tiny = canvas_with([(0, 305, 225, 30)])
        self.assertIsNone(strict.detect(tiny, self.calibration))

    def test_low_contrast_gray_marker_is_detected(self):
        image = canvas_with([(5, 250, 170, 120)])
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        low_contrast = np.where(gray < 128, 90, 150).astype(np.uint8)
        image = cv2.cvtColor(low_contrast, cv2.COLOR_GRAY2BGR)
        detector = MarkerDetector(
            0.12,
            min_side_px=10.0,
            adaptive_thresh_constant=3.0,
            min_otsu_std_dev=1.0,
            max_erroneous_bits_in_border_rate=0.45,
            error_correction_rate=0.8,
        )
        detection = detector.detect(image, self.calibration)
        self.assertIsNotNone(detection)
        self.assertEqual(detection.marker_id, 5)
        self.assertEqual(detector.parameters.minOtsuStdDev, 1.0)
        self.assertEqual(detector.parameters.adaptiveThreshConstant, 3.0)

    def test_invalid_sensitivity_is_rejected(self):
        with self.assertRaises(ValueError):
            MarkerDetector(0.10, min_otsu_std_dev=-1.0)
        with self.assertRaises(ValueError):
            MarkerDetector(0.10, error_correction_rate=1.5)


if __name__ == '__main__':
    unittest.main()
