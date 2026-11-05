import copy
import math
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from motor_autotune import BOUNDS, Experiment, confirm, response_metrics, rollback_failed_channels, search
from motor_test import MotorError
from tune_motors import SpeedValidationError, validate_autotune
from drive_calibration import DriveCalibration
from test_drive_calibration import document
from test_motor_pid import pid_document, firmware_info


class AutotuneTests(unittest.TestCase):
    def test_measured_objective_penalizes_slow_overshooting_and_oscillating_responses(self):
        def samples(tau=0.09, ringing=0, sign=1):
            return [(i * 0.05, sign * (72 * (1 - math.exp(-i * 0.05 / tau)) +
                                      ringing * math.sin(i * 0.8)), sign * (480 + ringing * math.sin(i)))
                    for i in range(1, 53)]
        clean = response_metrics(samples(), 72)
        slow = response_metrics(samples(tau=0.5), 72)
        ringing = response_metrics(samples(ringing=8), 72)
        reverse = response_metrics(samples(sign=-1), -72)
        self.assertTrue(clean['acceptable'])
        self.assertLess(clean['score'], slow['score'])
        self.assertLess(clean['score'], ringing['score'])
        self.assertFalse(ringing['acceptable'])
        self.assertAlmostEqual(clean['score'], reverse['score'])
        self.assertGreater(clean['iae_rpm_seconds'], 0)

    def test_missing_start_gap_nonfinite_and_insufficient_data_rejected(self):
        points = [(i * .05, 72, 480) for i in range(1, 53)]
        for data in (points[:10], [(t + .2, v, p) for t, v, p in points],
                     points[:10] + points[15:], points[:20] + [(1.05, float('nan'), 480)] + points[21:]):
            with self.assertRaises(ValueError):
                response_metrics(data, 72)

    def test_search_adapts_to_feedback_and_preserves_original_mapping_and_feedforward(self):
        original = pid_document(DriveCalibration(document()))
        snapshot = copy.deepcopy(original)
        calls = []

        def evaluate(doc, sign, label, rpms):
            direction = 'forward' if sign > 0 else 'reverse'
            values = {}
            for name in ('left', 'right'):
                g = doc['wheels'][name][direction]
                for key, (low, high) in BOUNDS.items():
                    self.assertTrue(low <= g[key] <= high)
                goal_p = 2 if name == 'left' else .5
                goal_i = 4 if sign > 0 else 12
                cost = .1 + (g['kp'] - goal_p) ** 2 + .03 * (g['ki'] - goal_i) ** 2 + 100 * (g['kd'] - .015) ** 2
                values[name] = {'score': cost, 'acceptable': True}
            result = {'document': copy.deepcopy(doc), 'aggregate': values, 'label': label}
            calls.append(result)
            return result

        selected, stages = search(original, evaluate)
        self.assertEqual(original, snapshot)
        self.assertEqual(len(stages), 6)
        self.assertEqual(len(calls), 28)
        for name in ('left', 'right'):
            for direction in ('forward', 'reverse'):
                g = selected['wheels'][name][direction]
                self.assertEqual(g['kd'], .015)
                self.assertEqual(g['slope'], original['wheels'][name][direction]['slope'])
                self.assertEqual(g['offset'], original['wheels'][name][direction]['offset'])
        self.assertNotEqual(selected['wheels']['left']['forward']['kp'], selected['wheels']['right']['forward']['kp'])

    def test_confirmation_rejects_no_improvement_and_steady_error_regression(self):
        original = pid_document(DriveCalibration(document()))
        candidate = copy.deepcopy(original)
        for wheel in candidate['wheels'].values():
            for gains in wheel.values():
                gains['kp'] = 2

        def evaluate(doc, sign, label, rpms):
            tuned = 'candidate' in label
            # Left improves objective but doubles the allowed steady error.
            values = {'left': {'score': .2 if tuned else .5, 'acceptable': True,
                               'steady_rms_error_rpm': 1.5 if tuned else .3},
                      'right': {'score': .51 if tuned else .5, 'acceptable': True, 'steady_rms_error_rpm': .3}}
            return {'aggregate': values}

        selected, report = confirm(original, candidate, evaluate)
        self.assertEqual(selected, original)
        self.assertTrue(all(not r['accepted'] for direction in report.values() for r in direction.values()))

    def test_experiment_stops_on_telemetry_failure(self):
        calibration = DriveCalibration(document())
        calls = []
        client = SimpleNamespace(calibration=calibration,
                                 info=firmware_info(),
                                 latest={'armed': False, 'command_pwm': [0, 0], 'invalid': [0, 0]},
                                 command=lambda *args: calls.append(args))
        def fail(*args, **kwargs):
            raise MotorError('lost telemetry')
        client.collect = fail
        client.set_speed = lambda *args, **kwargs: calls.append(('SET_SPEED', *args))
        def stop(c):
            c.command('STOP')
            return []
        with tempfile.TemporaryDirectory() as folder:
            experiment = Experiment(client, folder, stop)
            with self.assertRaisesRegex(MotorError, 'lost telemetry'):
                experiment.run(pid_document(calibration), 1, 'failure', [30])
            self.assertEqual(calls[-1], ('STOP',))
            self.assertEqual(experiment.history, [])

    def test_final_speed_failure_restores_only_failed_wheel_direction(self):
        original = pid_document(DriveCalibration(document()))
        candidate = copy.deepcopy(original)
        candidate['wheels']['left']['forward']['ki'] = 3
        candidate['wheels']['left']['reverse']['ki'] = 6
        candidate['autotune'] = {'comparison': {'forward': {'left': {'accepted': True}}}}
        trial = {'target_rpm': [72, 36], 'tracking_difference_rpm': 1.02,
                 'metrics': {'left': {'target_rpm': 72, 'mean_error_rpm': 1.14, 'rms_error_rpm': 1.2},
                             'right': {'target_rpm': 36, 'mean_error_rpm': .12, 'rms_error_rpm': .3}}}
        selected = rollback_failed_channels(candidate, original, trial)
        self.assertEqual(selected['wheels']['left']['forward'], original['wheels']['left']['forward'])
        self.assertEqual(selected['wheels']['left']['reverse']['ki'], 6)
        self.assertFalse(selected['autotune']['comparison']['forward']['left']['accepted'])
        self.assertEqual(candidate['wheels']['left']['forward']['ki'], 3)
        with self.assertRaises(MotorError):
            rollback_failed_channels(selected, original, trial)

    def test_final_validation_retries_quality_failure_but_not_communication_failure(self):
        calibration = DriveCalibration(document())
        original = pid_document(calibration)
        candidate = copy.deepcopy(original)
        candidate['wheels']['left']['forward']['ki'] = 3
        candidate['autotune'] = {'comparison': {'forward': {'left': {'accepted': True}}}}
        trial = {'target_rpm': [72, 36], 'tracking_difference_rpm': 1.02,
                 'metrics': {'left': {'target_rpm': 72, 'mean_error_rpm': 1.14, 'rms_error_rpm': 1.2},
                             'right': {'target_rpm': 36, 'mean_error_rpm': .12, 'rms_error_rpm': .3}}}
        with tempfile.TemporaryDirectory() as folder:
            with patch('tune_motors.validate', side_effect=[SpeedValidationError(trial), []]) as validate:
                selected, checks = validate_autotune(None, candidate, original, calibration, Path(folder))
            self.assertEqual(validate.call_count, 2)
            self.assertEqual(selected['wheels']['left']['forward'], original['wheels']['left']['forward'])
        with tempfile.TemporaryDirectory() as folder:
            with patch('tune_motors.validate', side_effect=MotorError('lost telemetry')) as validate:
                with self.assertRaisesRegex(MotorError, 'lost telemetry'):
                    validate_autotune(None, candidate, original, calibration, Path(folder))
            self.assertEqual(validate.call_count, 1)


if __name__ == '__main__':
    unittest.main()
