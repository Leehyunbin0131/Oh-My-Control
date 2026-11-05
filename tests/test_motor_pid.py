import copy
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
from motor_pid import PIDSettings, fit_feedforward, control_defaults
from motor_test import parse_info, parse_telemetry, parse_control_timing, MotorError
from drive_calibration import DriveCalibration
from test_drive_calibration import document as direction_document


def pid_document(calibration):
    gains = {'kp': 1.0, 'ki': 8.0, 'kd': 0, 'slope': 6.3, 'offset': 35}
    doc = {**control_defaults(), 'validated': True, 'wheel_mapping': calibration.wheels,
           'validation_status': {'driving': True, 'braking': True, 'holding': False},
           'wheels': {name: {direction: dict(gains) for direction in ('forward', 'reverse')}
                      for name in ('left', 'right')}}
    return doc


def firmware_info(signs=(1, 1), features=15):
    return parse_info(('OMR1,I,8,40,41,50,51,20000,300,1500,250,1,1,30000,650,2652,1,10000,650,400000,1000,' +
                       ','.join(map(str, signs)) + f',0,0,1,{features}').split(','))



class PIDTests(unittest.TestCase):
    def test_hold_commissioning_requires_capability_and_separate_gains(self):
        calibration = DriveCalibration(direction_document())
        doc = pid_document(calibration)
        doc['hold'].update(enabled=True, max_duration_ms=0)
        settings = PIDSettings(doc, calibration)
        self.assertFalse(settings.document['validation_status']['holding'])
        calls = []
        client = SimpleNamespace(info=firmware_info(features=7),
                                 latest={'armed': False, 'command_pwm': [0, 0]},
                                 command=lambda *args: calls.append(args))
        with self.assertRaisesRegex(MotorError, '새 메가 펌웨어'):
            settings.configure(client)
        self.assertEqual(calls, [])
        client.info = firmware_info()
        settings.configure(client)
        self.assertEqual([c for c in calls if c[0] == 'HOLD_PI'],
                         [('HOLD_PI', 0, 8000, 10000), ('HOLD_PI', 1, 8000, 10000)])
        self.assertTrue(all(c[-2] == 0 for c in calls if c[0] == 'HOLD'))
        self.assertEqual([c for c in calls if c[0] == 'ZERO'],
                         [('ZERO', 0, 2000, 0, 150), ('ZERO', 1, 2000, 0, 150)])
        for key, value in [('velocity_kp', 0), ('velocity_ki', 51),
                           ('velocity_ki', float('nan')), ('max_duration_ms', 1)]:
            bad = copy.deepcopy(doc); bad['hold'][key] = value
            with self.subTest(key=key), self.assertRaises(ValueError):
                PIDSettings(bad, calibration)
        for feature in (0, 8, 23):
            with self.assertRaises(MotorError): firmware_info(features=feature)

    def test_strong_hold_requires_new_capability_before_sending_any_config(self):
        calibration = DriveCalibration(direction_document())
        doc = pid_document(calibration)
        doc['hold'].update(enabled=True, velocity_kp=8, velocity_ki=0, kp_rpm_per_count=.25,
                           position_kp_pwm_per_count=18,
                           pwm_limit=350, deadband_counts=1, max_duration_ms=0)
        settings = PIDSettings(doc, calibration)
        calls = []
        client = SimpleNamespace(info=firmware_info(features=15),
                                 latest={'armed':False,'command_pwm':[0,0]},
                                 command=lambda *args:calls.append(args))
        with self.assertRaisesRegex(MotorError,'강한 위치 유지'):
            settings.configure(client)
        self.assertEqual(calls,[])
        client.info=firmware_info(features=31)
        settings.configure(client)
        self.assertIn(('HOLD_PI',0,8000,0),calls)
        self.assertIn(('HOLD_POS',0,18000),calls)
        self.assertIn(('HOLD',0,1,25,500,350,1,100,0,1000),calls)
        self.assertIn(('ZERO',0,2000,0,150),calls)
        for key,value in [('velocity_kp',60.001),('pwm_limit',651),
                          ('position_kp_pwm_per_count',30.001),('position_kp_pwm_per_count',float('nan'))]:
            bad=copy.deepcopy(doc); bad['hold'][key]=value
            with self.assertRaises(ValueError): PIDSettings(bad,calibration)
        # Reusing the serial session with a legacy profile clears the extra P.
        legacy=PIDSettings(pid_document(calibration),calibration)
        calls.clear(); legacy.configure(client)
        self.assertIn(('HOLD_POS',0,0),calls)
        self.assertIn(('HOLD_POS',1,0),calls)

    def test_current_hold_profile_with_delayed_actuator_and_external_load(self):
        root=Path(__file__).resolve().parents[1]
        hold=json.loads((root/'config/motor_pid.json').read_text())['hold']
        with tempfile.TemporaryDirectory() as directory:
            binary=str(Path(directory)/'hold_delay')
            subprocess.run(['g++','-std=c++11','-O2','-fsanitize=undefined','-fno-sanitize-recover=all',
                            str(root/'tests/hold_delay_test.cpp'),'-o',binary],check=True)
            values=[round(hold['velocity_kp']*1000),round(hold['velocity_ki']*1000),
                    round(hold.get('position_kp_pwm_per_count',0)*1000),
                    round(hold['kp_rpm_per_count']*100),hold['pwm_limit'],hold['deadband_counts']]
            subprocess.run([binary,*map(str,values)],check=True,timeout=30)

    def test_ros_boundary_uses_physical_forward_and_measured_encoder_position(self):
        calibration = DriveCalibration(direction_document(right_motor_reversed=True))
        settings = PIDSettings(pid_document(calibration), calibration)
        self.assertEqual(settings.velocity_commands(math.pi, -math.pi), [-3000, 3000])
        with self.assertRaises(ValueError): settings.velocity_commands(101 * math.pi / 30, 0)
        sample = {'counts': [2652, -2652], 'command_pwm': [0, 0], 'target_rpm_centi': [0, 0],
                  'measured_rpm_centi': [-3000, 3000]}
        for value in settings.joint_states(sample).values():
            self.assertAlmostEqual(value['position'], 2 * math.pi)
            self.assertAlmostEqual(value['velocity'], math.pi)

    def test_reversed_right_motor_requires_matching_mcu_feedback_sign(self):
        calibration = DriveCalibration(direction_document(right_motor_reversed=True))
        doc = pid_document(calibration); doc['period_ms'] = 1
        settings = PIDSettings(doc, calibration)
        calls = []
        prefix = 'OMR1,I,6,40,41,50,51,20000,300,1500,250,1,1,30000,999,2652,1,12000,650,400000,1000'
        info = firmware_info((1, -1))
        self.assertEqual(info['pid_encoder_signs'], [1, -1])
        client = SimpleNamespace(info=info, latest={'armed': False, 'command_pwm': [0, 0]},
                                 command=lambda *args: calls.append(args))
        settings.configure(client)
        self.assertEqual([c[:3] for c in calls if c[0] == 'PID'], [('PID', 0, -1), ('PID', 0, 1), ('PID', 1, -1), ('PID', 1, 1)])
        self.assertEqual(settings.commands(7200, 3600), [-7200, -3600])
        sample = parse_telemetry('OMR1,T,100,1,2,0,0,0,0,-495,-480,1,0,4,1,1,-7200,-7200,-7150,7210'.split(','), 999)
        self.assertEqual(settings.wheel_speeds(sample)['right'], {'target_rpm': 72, 'rpm': 72.1, 'pwm': 480})
        calls.clear()
        client.info = firmware_info((1, 1))
        with self.assertRaisesRegex(MotorError, '엔코더 방향'):
            settings.configure(client)
        self.assertEqual(calls, [])
        with self.assertRaises(MotorError):
            parse_info((prefix + ',1,0').split(','))

    def test_mcu_closed_loop_load_change_and_faults(self):
        with tempfile.TemporaryDirectory() as folder:
            binary = str(Path(folder) / 'velocity_test')
            subprocess.run(['g++', '-std=c++11', '-Wall', '-Wextra', '-Werror', '-fsanitize=undefined',
                            str(ROOT / 'tests/velocity_core_test.cpp'), '-o', binary], check=True)
            subprocess.run([binary], check=True)

    def test_regression_and_direction_configuration(self):
        gains = fit_feedforward([(20, 160), (40, 280), (60, 400)])
        self.assertEqual(gains['slope'], 6)
        self.assertEqual(gains['offset'], 40)
        calibration = DriveCalibration(direction_document())
        settings = PIDSettings(pid_document(calibration), calibration)
        calls = []
        info = firmware_info()
        settings.configure(SimpleNamespace(info=info, latest={'armed': False, 'command_pwm': [0, 0]},
                                           command=lambda *args: calls.append(args)))
        self.assertEqual([c[:3] for c in calls if c[0] == 'PID'], [('PID', 0, -1), ('PID', 0, 1), ('PID', 1, 1), ('PID', 1, -1)])
        self.assertEqual(settings.commands(7200, 3600), [-7200, 3600])
        sample = parse_telemetry('OMR1,T,100,1,2,0,0,0,0,-495,480,1,0,4,1,1,-7200,7200,-7150,7210'.split(','), 999)
        speeds = settings.wheel_speeds(sample)
        self.assertEqual(speeds['left'], {'target_rpm': 72, 'rpm': 71.5, 'pwm': 495})
        self.assertEqual(speeds['right']['rpm'], 72.1)

    def test_invalid_calibration_and_unvalidated_file_are_rejected(self):
        calibration = DriveCalibration(direction_document())
        for field, value in [('validated', False), ('encoder_cpr', 663), ('period_ms', 100), ('max_rpm', float('nan'))]:
            data = pid_document(calibration); data[field] = value
            with self.subTest(field=field), self.assertRaises(ValueError):
                PIDSettings(data, calibration)
        for value in (-1, float('nan'), 21):
            data = pid_document(calibration); data['wheels']['left']['forward']['kp'] = value
            with self.assertRaises(ValueError): PIDSettings(data, calibration)
        data = pid_document(calibration); data['wheel_mapping'] = copy.deepcopy(data['wheel_mapping'])
        data['wheel_mapping']['left']['forward_command_sign'] = 1
        with self.assertRaises(ValueError): PIDSettings(data, calibration)

    def test_1khz_firmware_requires_matching_calibration_and_reports_actual_timing(self):
        calibration = DriveCalibration(direction_document())
        doc = pid_document(calibration)
        doc['period_ms'] = 50
        info = firmware_info()
        client = SimpleNamespace(info=info, latest={'armed': False, 'command_pwm': [0, 0]}, command=lambda *args: None)
        with self.assertRaises((ValueError, MotorError)):
            PIDSettings(doc, calibration).configure(client)
        doc['period_ms'] = 1
        PIDSettings(doc, calibration).configure(client)
        timing = parse_control_timing('OMR1,P,1000000,1000,900,0,996,1004,350,900,700,1020,400000,0'.split(','))
        self.assertEqual(timing['compute_max_us'], 350)
        self.assertEqual(timing['control_ticks'], 1000)
        with self.assertRaises(MotorError):
            parse_control_timing('OMR1,P,1000,1'.split(','))


if __name__ == '__main__':
    unittest.main()
