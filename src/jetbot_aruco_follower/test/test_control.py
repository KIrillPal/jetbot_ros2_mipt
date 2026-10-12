"""Offline tests for following, stopping, and loss recovery."""

from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from jetbot_aruco_follower.control import (  # noqa: E402
    FollowConfig,
    FollowController,
    Target,
)


class FollowControllerTests(unittest.TestCase):
    """Exercise bounded motion and fail-safe controller states."""

    def setUp(self):
        self.controller = FollowController(FollowConfig())

    def test_no_camera_never_commands_motion(self):
        command = self.controller.command(
            None, 1.0, camera_ok=False)
        self.assertEqual(
            (command.linear_mps, command.angular_radps), (0.0, 0.0))
        self.assertEqual(command.state, 'CAMERA_STALE')

    def test_no_marker_turns_then_pauses(self):
        controller = FollowController(
            FollowConfig(), rng=_ScriptedRng((1.0, -1.0)))
        turn = controller.command(None, 0.0)
        self.assertEqual(turn.state, 'SEARCHING')
        self.assertEqual(turn.linear_mps, 0.0)
        self.assertAlmostEqual(turn.angular_radps, 0.6)
        self.assertEqual(
            controller.command(None, 2.49).state, 'SEARCHING')
        pause = controller.command(None, 2.5)
        self.assertEqual(pause.state, 'SEARCH_PAUSE')
        self.assertEqual(
            (pause.linear_mps, pause.angular_radps), (0.0, 0.0))
        self.assertEqual(
            controller.command(None, 2.99).state, 'SEARCH_PAUSE')
        next_turn = controller.command(None, 3.0)
        self.assertEqual(next_turn.state, 'SEARCHING')
        self.assertAlmostEqual(next_turn.angular_radps, -0.6)

    def test_approach_is_bounded(self):
        command = self.controller.command(Target(2.0, 0.2), 1.0)
        self.assertEqual(command.state, 'APPROACHING')
        self.assertGreater(command.linear_mps, 0)
        self.assertLessEqual(command.linear_mps, 0.08)
        self.assertLessEqual(abs(command.angular_radps), 0.6)

    def test_aligns_before_moving_forward(self):
        command = self.controller.command(Target(1.0, -0.6), 1.0)
        self.assertEqual(command.state, 'ALIGNING')
        self.assertEqual(command.linear_mps, 0.0)
        self.assertLess(command.angular_radps, 0)

    def test_stops_inside_acceptance_band(self):
        command = self.controller.command(Target(0.51, 0.0), 1.0)
        self.assertEqual(command.state, 'ARRIVED')
        self.assertEqual(
            (command.linear_mps, command.angular_radps), (0.0, 0.0))

    def test_lost_target_never_commands_motion(self):
        self.controller.command(Target(1.0, -0.2), 1.0)
        grace = self.controller.command(None, 1.5)
        missing = self.controller.command(None, 1.9)
        self.assertEqual(grace.state, 'TARGET_LOST_GRACE')
        self.assertEqual(missing.state, 'SEARCHING')
        self.assertEqual(missing.linear_mps, 0.0)
        self.assertNotEqual(missing.angular_radps, 0.0)

    def test_reset_restarts_search(self):
        self.controller.command(Target(1.0, -0.2), 1.0)
        self.controller.reset()
        command = self.controller.command(None, 2.0)
        self.assertEqual(command.state, 'SEARCHING')
        self.assertNotEqual(command.angular_radps, 0.0)

    def test_found_marker_cancels_search(self):
        self.controller.command(None, 0.0)
        command = self.controller.command(Target(1.0, 0.1), 0.2)
        self.assertEqual(command.state, 'APPROACHING')
        lost = self.controller.command(None, 0.3)
        self.assertEqual(lost.state, 'TARGET_LOST_GRACE')

    def test_invalid_config_is_rejected(self):
        with self.assertRaises(ValueError):
            FollowController(FollowConfig(stop_distance_m=0))
        with self.assertRaises(ValueError):
            FollowController(
                FollowConfig(max_angular_radps=float('nan')))


class _ScriptedRng:
    """Deterministic angle signs for search-cycle tests."""

    def __init__(self, signs):
        self._signs = list(signs)

    def uniform(self, low, high):
        return high

    def random(self):
        return 0.0 if self._signs.pop(0) > 0 else 1.0


if __name__ == '__main__':
    unittest.main()
