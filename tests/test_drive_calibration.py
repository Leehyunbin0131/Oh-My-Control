"""Physical side mapping and forward/reverse polarity against simulated wiring."""

import copy
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
from drive_calibration import DriveCalibration
from motor_test import MotorClient, bench_test
from check_sensors import parse_args
from test_motor_check import MegaSimulator


def document(right_motor_reversed=False):
    return {'schema_version': 1, 'confirmed_by_user': True, 'front_reference': 'camera',
            'wheels': {
                'left': {'driver_channel': 'L', 'encoder_index': 1, 'encoder_pins': [50, 51],
                         'forward_command_sign': -1, 'forward_encoder_sign': -1},
                'right': {'driver_channel': 'R', 'encoder_index': 0, 'encoder_pins': [40, 41],
                          'forward_command_sign': -1 if right_motor_reversed else 1, 'forward_encoder_sign': 1}}}


class CalibrationTests(unittest.TestCase):
    def test_reversed_right_motor_keeps_raw_encoder_direction(self):
        calibration = DriveCalibration(document(right_motor_reversed=True))
        self.assertEqual(calibration.commands(180, 180), [-180, -180])
        self.assertEqual(calibration.commands(-180, -180), [180, 180])
        self.assertEqual(calibration.commands(-180, 180), [180, -180])
        self.assertEqual(calibration.commands(180, -180), [-180, 180])
        sample = {'counts': [102, -99], 'ticks_per_second': [1000, -900], 'ab': [2, 1], 'invalid': [0, 0]}
        self.assertEqual(calibration.wheel_data(sample)['right']['count'], 102)
        self.assertEqual(calibration.wheel_data(sample)['right']['ticks_per_second'], 1000)
        args = parse_args(['--motors', '--motor-bench-seconds', '0.2'])
        with MegaSimulator(swapped=True, protocol=8, encoder_signs=(1, -1)) as mega, tempfile.TemporaryDirectory() as folder:
            with MotorClient(mega.port, folder, 2, boot_delay=0, calibration=calibration) as client:
                result = bench_test(client, args)
                self.assertEqual(result['status'], 'PASS', result)
                self.assertEqual(result['pwm'], [-180, -180])
                self.assertTrue(result['stopped'])
                self.assertGreater(result['wheel_encoder_delta']['left'], 0)
                self.assertGreater(result['wheel_encoder_delta']['right'], 0)

    def test_forward_reverse_and_physical_side_mapping(self):
        calibration = DriveCalibration(document())
        self.assertEqual(calibration.commands(180, 180), [-180, 180])
        self.assertEqual(calibration.commands(-180, -180), [180, -180])
        self.assertEqual(calibration.commands(180, 0), [-180, 0])
        sample = {'counts': [102, -99], 'ticks_per_second': [1000, -900], 'ab': [2, 1], 'invalid': [0, 0]}
        wheels = calibration.wheel_data(sample)
        self.assertEqual(wheels['left']['count'], 99)
        self.assertEqual(wheels['right']['count'], 102)
        self.assertEqual(wheels['left']['ticks_per_second'], 900)
        self.assertEqual(wheels['right']['encoder_pins'], [40, 41])
        self.assertEqual(sample['counts'], [102, -99])  # raw diagnostic data stays intact

    def test_invalid_or_duplicate_channels_are_rejected(self):
        variants = []
        for field, value in [('driver_channel', 'R'), ('encoder_index', 0),
                             ('forward_command_sign', 0), ('forward_encoder_sign', True)]:
            bad = copy.deepcopy(document())
            bad['wheels']['left'][field] = value
            variants.append(bad)
        bad = document(); bad['confirmed_by_user'] = False; variants.append(bad)
        for bad in variants:
            with self.subTest(document=bad), self.assertRaises(ValueError):
                DriveCalibration(bad)

    def test_calibrated_cli_uses_physical_left_right_and_direction(self):
        with MegaSimulator(swapped=True) as mega, tempfile.TemporaryDirectory() as folder:
            config = Path(folder) / 'calibration.json'
            config.write_text(json.dumps(document()))
            result = subprocess.run([sys.executable, str(ROOT/'tools/check_sensors.py'),
                                     '--sensors', 'motors', '--motor-test', 'drive',
                                     '--drive-calibration', str(config), '--motor-port', mega.port,
                                     '--motor-pulse', '0.1', '--motor-min-ticks', '2',
                                     '--startup-timeout', '2', '--report-dir', folder],
                                    capture_output=True, text=True, timeout=15)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            report = json.loads(next(Path(folder).glob('*/summary.json')).read_text())['results']['motors']
            self.assertEqual([p['physical_wheel'] for p in report['pulses']], ['left', 'left', 'right', 'right'])
            self.assertTrue(all(p['direction'] * p['wheel_encoder_delta'] >= 2 for p in report['pulses']))
            runs = [c[1:3] for c in mega.commands if c[0] == 'RUN']
            self.assertEqual(runs, [['-180', '0'], ['180', '0'], ['0', '180'], ['0', '-180']])

    def test_bench_forward_and_wrong_encoder_sign_detection(self):
        args = parse_args(['--motors', '--motor-bench-seconds', '0.2'])
        for wrong_sign in (False, True):
            config = document()
            if wrong_sign:
                config['wheels']['left']['forward_encoder_sign'] = 1
            with self.subTest(wrong_sign=wrong_sign), MegaSimulator(swapped=True) as mega, tempfile.TemporaryDirectory() as folder:
                with MotorClient(mega.port, folder, 2, boot_delay=0, calibration=DriveCalibration(config)) as client:
                    result = bench_test(client, args)
                    self.assertEqual(result['status'], 'FAIL' if wrong_sign else 'PASS', result)
                    self.assertEqual(result['pwm'], [-180, 180])
                    self.assertTrue(result['stopped'])
                    self.assertEqual(result['wheel_encoder_delta']['left'] > 0, not wrong_sign)


if __name__ == '__main__':
    unittest.main()
