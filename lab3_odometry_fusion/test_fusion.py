import math
from pathlib import Path
import unittest
import yaml
from fusion_math import alignment, aligned, paired, diagonal, valid, History, diagnostic_level
from fusion_node import State


def row(**kwargs):
    return dict(dict(stamp_s=1., x=0., y=0., yaw=0., linear_x=0., linear_y=0., angular_z=0.), **kwargs)


class FusionTests(unittest.TestCase):
    def test_old_humble_byte_diagnostic_level(self):
        self.assertEqual(diagnostic_level(b'\x00'), 0)
        self.assertEqual(diagnostic_level(b'\x02'), 2)
        self.assertEqual(diagnostic_level(1), 1)

    def test_fixed_alignment_maps_initial_pose(self):
        wheel, laser = row(x=1., y=2., yaw=math.pi/2), row(x=3., y=-1., yaw=0.)
        transform = alignment(wheel, laser)
        first = aligned(laser, transform)
        self.assertAlmostEqual(first['x'], 1.)
        self.assertAlmostEqual(first['y'], 2.)
        self.assertAlmostEqual(first['yaw'], math.pi/2)
        moved = aligned(row(x=4., y=-1., yaw=.2), transform)
        self.assertAlmostEqual(moved['x'], 1.)
        self.assertAlmostEqual(moved['y'], 3.)
        self.assertAlmostEqual(moved['yaw'], math.pi/2+.2)

    def test_body_twist_and_stamp_not_rotated_or_restamped(self):
        laser = row(linear_x=.2, angular_z=.1, stamp_s=123.)
        result = aligned(laser, dict(x=5., y=8., yaw=2.))
        self.assertEqual(result['linear_x'], .2)
        self.assertEqual(result['angular_z'], .1)
        self.assertEqual(result['stamp_s'], 123.)

    def test_pairing_rejects_unsynchronized_samples(self):
        self.assertIsNone(paired([row(stamp_s=1.)], 2.))
        self.assertEqual(paired([row(stamp_s=1.), row(stamp_s=2.)], 2.05)['stamp_s'], 2.)

    def test_covariances_are_positive_and_diagonal(self):
        matrix = diagonal([.0025, .0025, 1e6, 1e6, 1e6, .0064])
        self.assertEqual(len(matrix), 36)
        self.assertTrue(all(matrix[i*7] > 0 for i in range(6)))
        self.assertTrue(all(v == 0 for i, v in enumerate(matrix) if i % 7))

    def test_no_nonfinite_state(self):
        self.assertFalse(valid(row(x=float('nan'))))

    def test_bounded_history_and_cursor_restart(self):
        now = [0.]
        history = History(lambda: now[0])
        for i in range(6010):
            now[0] += .2
            history.add(row(x=i))
        self.assertEqual(len(history.rows), 6000)
        self.assertEqual(history.state(6010, history.session)['points'], [])
        self.assertEqual(len(history.state(99999, 'old-server')['points']), 6000)
        now[0] += 2
        self.assertEqual(history.state()['status'], 'stale')

    def test_ekf_uses_nonduplicated_measurements_and_no_tf_or_control(self):
        cfg = yaml.safe_load((Path(__file__).parent/'params/ekf.yaml').read_text())['ekf_filter_node']['ros__parameters']
        self.assertFalse(cfg['publish_tf'])
        self.assertFalse(cfg['use_control'])
        self.assertEqual([i for i,v in enumerate(cfg['odom0_config']) if v], [6, 7, 11])
        self.assertEqual([i for i,v in enumerate(cfg['odom1_config']) if v], [0, 1, 5])
        self.assertTrue(cfg['smooth_lagged_data'])
        process = cfg['process_noise_covariance']
        self.assertEqual(len(process), 225)
        self.assertTrue(all(process[16*i] > 0 for i in range(15)))

    def test_fresh_filter_cannot_hide_stale_input(self):
        import time
        state = State()
        for source in state.sources.values():
            source.add(row())
        state.alignment['ready'] = True
        state.raw_lidar_at = state.scan_at = time.monotonic()
        self.assertTrue(state.snapshot()['input_health'])
        state.raw_lidar_at -= 3.
        result = state.snapshot()
        self.assertFalse(result['input_health'])
        self.assertEqual(result['sources']['fused']['status'], 'live')


if __name__ == '__main__':
    unittest.main()
