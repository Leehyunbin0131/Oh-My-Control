"""Live/stale state, pose estimates, and SSH dashboard controls."""

from pathlib import Path
import sys
import time
from types import SimpleNamespace as NS
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from sensor_monitor import CHANNELS, LiveChannel, camera_orientation, image_pixels
from terminal_view import RobotModel, Screen, rotation
from hardware_tui import Dashboard


class LiveStateTests(unittest.TestCase):
    def test_connected_then_stale_and_recovery(self):
        stream = LiveChannel()
        for i in range(5):
            stream.update(True, {}, i, 10 + i / 10)
        self.assertEqual(stream.snapshot(10.5, True, 10)['status'], 'LIVE')
        self.assertEqual(stream.snapshot(14, True, 10)['status'], 'STALE')
        stream.update(True, {}, 5, 14)
        self.assertEqual(stream.snapshot(14, True, 10)['status'], 'LIVE')

    def test_repeated_stamp_becomes_error(self):
        stream = LiveChannel()
        for i in range(5):
            stream.update(True, {}, 1, 10 + i / 10)
        self.assertEqual(stream.snapshot(10.5, True, 10)['status'], 'ERROR')

    def test_camera_yaw_drift_does_not_light_motion_indicator(self):
        stream = LiveChannel()
        stream.update(True, {'rpy_degrees': [0, 0, 0], 'axis_activity': [False] * 3}, 1, 1)
        stream.update(True, {'rpy_degrees': [0, 0, 20], 'axis_activity': [False] * 3}, 2, 2)
        self.assertEqual(stream.axes, [False] * 3)
        stream.update(True, {'axis_activity': [False, True, False]}, 3, 3)
        self.assertEqual(stream.axes, [False, True, False])


class RenderingTests(unittest.TestCase):
    def test_camera_optical_axes_upright_pose(self):
        rpy, yaw = camera_orientation([0, 0, 0], [0, -9.81, 0], 0, 0.01)
        np.testing.assert_allclose(rpy, [0, 0, 0], atol=1e-8)
        rpy, yaw = camera_orientation([0, -1, 0], [0, -9.81, 0], 0, 0.1)
        self.assertAlmostEqual(yaw, 0.1)

    def test_image_respects_padded_rows(self):
        message = NS(width=1, height=2, encoding='rgb8', step=4,
                     data=bytes([255, 0, 0, 99, 0, 255, 0, 99]))
        pixels = image_pixels(message)
        np.testing.assert_array_equal(pixels, [[[255, 0, 0]], [[0, 255, 0]]])

    def test_reference_model_has_real_geometry(self):
        model = RobotModel()
        self.assertGreater(len(model.lines), 100)
        self.assertTrue(0.25 < model.radius < 0.5)

    def test_compact_layout_and_keyboard_controls(self):
        channels = {key: LiveChannel().snapshot(time.monotonic(), True, time.monotonic()) for key in CHANNELS}
        monitor = NS(snapshot=lambda: {'channels': channels, 'elapsed': 0}, image=lambda key: None,
                     run_dir=Path('/tmp/test'))
        dashboard = Dashboard(RobotModel())
        for width, height in [(79, 24), (119, 42), (159, 48)]:
            screen = dashboard.render(monitor, width, height, 'none')
            output = screen.render(ansi=False)
            self.assertEqual(len(output.splitlines()), height)
            self.assertIn('NO LIVE SCAN', output)
            self.assertNotIn('\x1b', output)
        self.assertTrue(dashboard.key('2', monitor))
        self.assertEqual(dashboard.source, 'camera_imu')
        dashboard.key('c', monitor)
        self.assertEqual(dashboard.image_kind, 'depth')
        dashboard.key('+', monitor)
        self.assertEqual(dashboard.range, 8)
        self.assertFalse(dashboard.key('q', monitor))

    def test_relative_rotation_zero_is_identity(self):
        current = rotation([25, -10, 85])
        np.testing.assert_allclose(current.T @ current, np.eye(3), atol=1e-12)


if __name__ == '__main__':
    unittest.main()
