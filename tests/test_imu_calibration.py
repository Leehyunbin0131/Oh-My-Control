"""Static gyro calibration must not hide motion, corrupt samples or gravity."""
import json
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from imu_calibration import (FRAMES, G, analyze_capture, correction_profile,
                             json_write, subtract_gyro_bias)


class StaticCalibrationTests(unittest.TestCase):
    def setUp(self):
        rng = np.random.default_rng(21)
        times = np.arange(0, 30, 0.01)
        self.bias = np.array([0.003, -0.002, 0.001])
        gyro = self.bias + rng.normal(0, 0.00015, (len(times), 3))
        accel = [0, 0, -G] + rng.normal(0, 0.015, (len(times), 3))
        self.streams = {name: np.column_stack((times, times * 1e6, xyz, np.arange(len(times)),
                                               np.full(len(times), -1)))
                        for name, xyz in (('gyro', gyro), ('accel', accel))}
        self.meta = {'name': 'HFI-A9', 'identity': 'test_adapter', 'crc_errors': 0}

    def analyze(self):
        return analyze_capture(self.streams, 'imu', self.meta, 10, 10, 10)

    def test_independent_validation_reduces_residual_without_removing_gravity(self):
        result = self.analyze()
        self.assertEqual(result['status'], 'PASS', result)
        np.testing.assert_allclose(result['gyro_bias_rad_s'], self.bias, atol=2e-5)
        self.assertLess(result['validation_rms_after_rad_s'], result['validation_rms_before_rad_s'] / 5)
        profile = correction_profile('imu', self.meta, result, Path('/tmp/report.json'))
        self.assertFalse(profile['enabled_for_navigation'])
        self.assertFalse(profile['accelerometer_calibrated'])
        self.assertFalse(profile['orientation_output_corrected'])
        self.assertAlmostEqual(profile['gravity_reference_m_s2'][2], -G, delta=0.002)
        self.assertFalse(profile['validation']['temperature_range'])
        self.assertTrue(profile['additional_correction_recommended'])
        corrected = subtract_gyro_bias(self.bias, profile, identity='test_adapter',
                                       coordinate_frame=FRAMES['imu'])
        np.testing.assert_allclose(corrected, [0, 0, 0], atol=2e-5)

    def test_validation_data_is_not_used_to_fit_bias(self):
        self.streams['gyro'][2000:, 2] += 0.002
        result = self.analyze()
        self.assertEqual(result['status'], 'FAIL')
        np.testing.assert_allclose(result['gyro_bias_rad_s'], self.bias, atol=2e-5)
        self.assertIn('validation: residual gyro bias too large', result['reasons'])

    def test_already_good_zero_does_not_get_a_worse_additional_correction(self):
        # Tiny estimation noise changes sign in the independent window. Keep
        # the measured candidate as evidence, but select no extra subtraction.
        self.streams['gyro'][1000:2000, 2:5] = [0.00004, 0.00001, 0]
        self.streams['gyro'][2000:, 2:5] = [-0.00001, 0, 0]
        result = self.analyze()
        self.assertEqual(result['status'], 'PASS')
        profile = correction_profile('imu', self.meta, result, Path('/tmp/report.json'))
        self.assertFalse(profile['additional_correction_recommended'])
        self.assertEqual(profile['gyro_bias_rad_s'], [0, 0, 0])
        self.assertNotEqual(profile['estimated_gyro_bias_rad_s'], [0, 0, 0])

    def test_gyro_motion_rejected(self):
        self.streams['gyro'][1200:1400, 4] += 0.06
        self.assertEqual(self.analyze()['status'], 'FAIL')

    def test_pose_changed_between_windows_rejected(self):
        self.streams['accel'][2000:, 2] += 0.2
        result = self.analyze()
        self.assertEqual(result['status'], 'FAIL')
        self.assertIn('validation: posture changed between training and validation', result['reasons'])

    def test_nan_sample_and_missing_stream_rejected(self):
        self.streams['gyro'][1500, 3] = float('nan')
        self.assertEqual(self.analyze()['status'], 'FAIL')
        del self.streams['accel']
        self.assertEqual(self.analyze()['status'], 'FAIL')

    def test_repeated_timestamp_and_host_gap_rejected(self):
        self.streams['gyro'][1500, 1] = self.streams['gyro'][1499, 1]
        self.assertEqual(self.analyze()['status'], 'FAIL')
        self.setUp()
        self.streams['gyro'] = np.delete(self.streams['gyro'], np.arange(1400, 1440), axis=0)
        self.assertEqual(self.analyze()['status'], 'FAIL')

    def test_hfi_timestamp_wrap_passes_but_crc_corruption_fails(self):
        for samples in self.streams.values():
            samples[:, 1] = (samples[:, 1] + 2 ** 32 - 15_000_000) % 2 ** 32
        self.assertEqual(self.analyze()['status'], 'PASS')
        self.meta['crc_errors'] = 1
        result = self.analyze()
        self.assertEqual(result['status'], 'FAIL')
        with self.assertRaises(ValueError):
            correction_profile('imu', self.meta, result, Path('/tmp/report.json'))

    def test_camera_calibration_records_sdk_processing_and_device_identity(self):
        metadata = {'name': 'RealSense D435IF', 'serial': '123', 'sdk_version': '2.58.1',
                    'firmware': '5.17.3.10', 'motion_correction_enabled': 1,
                    'gyro': {'fps': 100, 'intrinsics': {'data': 'existing'}}, 'accel': {'fps': 100}}
        result = analyze_capture(self.streams, 'camera', metadata, 10, 10, 10)
        profile = correction_profile('camera', metadata, result, Path('/tmp/report.json'))
        self.assertEqual(profile['identity'], '123')
        self.assertEqual(profile['input_processing']['motion_correction_enabled'], 1)
        with self.assertRaises(ValueError):
            subtract_gyro_bias(self.bias, profile, identity='another_camera', coordinate_frame=FRAMES['camera'])
        with self.assertRaises(ValueError):
            subtract_gyro_bias(self.bias, profile, identity='123', coordinate_frame='base_link')
        self.streams['gyro'][2500:, 6] = 1
        self.assertEqual(analyze_capture(self.streams, 'camera', metadata, 10, 10, 10)['status'], 'FAIL')

    def test_failed_json_write_preserves_previous_profile(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'profile.json'
            json_write(path, {'good': True})
            with self.assertRaises(ValueError):
                json_write(path, {'bad': float('nan')})
            self.assertEqual(json.loads(path.read_text()), {'good': True})
            self.assertEqual(list(Path(directory).iterdir()), [path])


if __name__ == '__main__':
    unittest.main()
