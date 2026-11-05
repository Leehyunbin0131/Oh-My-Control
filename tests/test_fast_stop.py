"""v8 continuous zero-speed braking and protocol tests; no physical hardware."""
import copy
from pathlib import Path
import subprocess
import tempfile
import time
import unittest
from unittest.mock import patch

import test_keyboard_control as keyboard_tests
from test_keyboard_control import FakeClient
from test_motor_check import MegaSimulator
from test_motor_pid import pid_document, firmware_info
from keyboard_control import Controller
from motor_pid import PIDSettings
from motor_test import MotorClient, MotorError
from drive_calibration import DriveCalibration
from test_drive_calibration import document

ROOT = Path(__file__).resolve().parents[1]


class FeedbackStopTests(unittest.TestCase):
    def test_production_firmware_parser_and_timer(self):
        with tempfile.TemporaryDirectory() as folder:
            binary = str(Path(folder) / 'firmware')
            subprocess.run(['g++', '-std=c++11', '-Wall', '-Wextra', '-Werror', '-fsanitize=address,undefined',
                            '-I', str(ROOT / 'tests/native'), str(ROOT / 'tests/firmware_protocol_test.cpp'),
                            '-o', binary], check=True)
            subprocess.run([binary], check=True)

    def test_pid_target_changes_do_not_stop_or_rearm(self):
        c = FakeClient()
        settings = PIDSettings(pid_document(c.calibration), c.calibration)
        controller = Controller(c, pid_settings=settings)
        helper = keyboard_tests.ControllerTests()
        for key, stamp, expected in [('w', 10, (-7200, 7200)), ('x', 10.02, (7200, -7200)),
                                     ('e', 10.04, (-7200, 3600))]:
            controller.keys([key], stamp)
            helper.step(controller, c, stamp)
            self.assertEqual(c.calls[-1], ('SET_SPEED', *expected, 300))
        controller.keys(['stop'], 10.06)
        helper.step(controller, c, 10.06)
        self.assertEqual(c.calls[-1], ('SET_SPEED', 0, 0, 300))
        for stamp in (10.1, 10.2, 10.4, 10.8, 11.1):
            helper.step(controller, c, stamp)
            self.assertEqual(c.calls[-1], ('SET_SPEED', 0, 0, 300))
        self.assertEqual(sum(call[0] == 'ARM' for call in c.calls), 1)
        self.assertFalse(any(call[0] in ('STOP', 'BRAKE', 'SPEED') for call in c.calls))

    def test_hold_commissioning_preserves_unverified_status_and_rejects_over_100(self):
        calibration = DriveCalibration(document())
        for field, value in [('max_rpm', 100.01), ('control_model', 'legacy_unsigned_pwm'), ('schema_version', 1)]:
            doc = pid_document(calibration); doc[field] = value
            with self.assertRaises(ValueError): PIDSettings(doc, calibration)
        doc = pid_document(calibration); doc['hold']['enabled'] = True
        settings = PIDSettings(doc, calibration)
        self.assertFalse(settings.document['validation_status']['holding'])
        missing = copy.deepcopy(doc); del missing['hold']['velocity_kp']
        with self.assertRaises(ValueError): PIDSettings(missing, calibration, require_validated=False)
        c = FakeClient(); c.info = firmware_info()
        with self.assertRaises(MotorError): Controller(c, pid_settings=PIDSettings(doc, calibration), rpm=101)

    def test_command_lease_expires_despite_ping_and_stop_preserves_fault(self):
        calibration = DriveCalibration(document())
        with MegaSimulator(protocol=8, swapped=True) as mega, tempfile.TemporaryDirectory() as folder:
            with MotorClient(mega.port, folder, boot_delay=0, calibration=calibration) as client:
                PIDSettings(pid_document(calibration), calibration).configure(client)
                client.command('ARM'); client.set_speed(-3000, 3000, wait=True)
                with self.assertRaisesRegex(MotorError, '유효기간'):
                    client.collect(.6, heartbeat=True)
                self.assertEqual(mega.fault, 9)
                client.command('STOP')
                self.assertEqual(mega.fault, 9)
                client.clear_fault()
                self.assertFalse(mega.armed)

    def test_zero_ack_precedes_standstill_and_cleanup_disables(self):
        calibration = DriveCalibration(document())
        with MegaSimulator(protocol=8, swapped=True) as mega, tempfile.TemporaryDirectory() as folder:
            with MotorClient(mega.port, folder, boot_delay=0, calibration=calibration) as client:
                PIDSettings(pid_document(calibration), calibration).configure(client)
                client.command('ARM'); client.set_speed(-3000, 3000, wait=True)
                client.collect(.15, target=(-3000, 3000))
                start = time.monotonic()
                client.controlled_stop()
                self.assertGreaterEqual(time.monotonic() - start, .09)
                self.assertTrue(mega.armed)
                self.assertEqual(client.control['wheel_states'], ['ZERO_SPEED', 'ZERO_SPEED'])
                for target in ((10001, 0), (0, -10001)):
                    with self.assertRaises(MotorError): client.set_speed(*target)
                self.assertFalse(any(command[0] == 'BRAKE' for command in mega.commands))
            deadline = time.monotonic() + .2
            while mega.armed and time.monotonic() < deadline: time.sleep(.005)
            self.assertFalse(mega.armed)


if __name__ == '__main__':
    unittest.main()
