"""Wheel command ownership, expiry and feedback; fake clock and PTY only."""

import math
from pathlib import Path
import sys
import tempfile
import time
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools'))

from drive_calibration import DriveCalibration
from motor_link import MotorClient, MotorError
from motor_pid import PIDSettings
from wheel_drive import WheelDrive
from test_drive_calibration import document
from test_motor_check import MegaSimulator
from test_motor_pid import firmware_info, pid_document


class Clock:
    def __init__(self, now=10.0):
        self.now = now

    def __call__(self):
        return self.now


class RecordingClient:
    """Record wire operations; reported measurements remain independent."""
    def __init__(self, clock, calibration):
        self.clock = clock
        self.calibration = calibration
        self.info = firmware_info(signs=(1, -1))
        self.events = []
        self.fail_speed = self.fail_stop = self.fail_arm = False
        self.on_arm = lambda: None
        self.refresh()

    def refresh(self):
        self.latest = {
            'stamp': round(self.clock() * 1000), 'received_at': self.clock(),
            'fault': 0, 'fault_reason': '', 'armed': True,
            'counts': [0, 0], 'invalid': [0, 0], 'command_pwm': [0, 0],
            'target_rpm_centi': [0, 0], 'measured_rpm_centi': [0, 0],
        }

    def command(self, operation, *values):
        self.events.append((self.clock(), operation, *values))
        if operation == 'ARM':
            if self.fail_arm:
                raise OSError('ARM write failed')
            self.on_arm()

    def send(self, operation, *values):
        self.events.append((self.clock(), operation, *values))
        if operation == 'STOP' and self.fail_stop:
            raise OSError('STOP write failed')

    def set_speed(self, left, right, lease_ms=300):
        self.events.append((self.clock(), 'SET_SPEED', left, right, lease_ms))
        if self.fail_speed:
            raise OSError('speed write failed')


class WheelDriveTests(unittest.TestCase):
    def fixture(self, now=10.0):
        clock = Clock(now)
        calibration = DriveCalibration(document(right_motor_reversed=True))
        config = pid_document(calibration)
        config['max_rpm'] = 95
        settings = PIDSettings(config, calibration)
        client = RecordingClient(clock, calibration)
        return clock, client, WheelDrive(client, settings, clock=clock)

    @staticmethod
    def at(clock, client, now):
        clock.now = now
        client.refresh()

    def test_physical_units_send_rate_and_measured_feedback(self):
        clock, client, drive = self.fixture()
        drive.set_target_rpm(12.34, -5.67)
        drive.tick()
        self.assertEqual(client.events, [
            (10.0, 'ARM'), (10.0, 'SET_SPEED', -1234, 567, 300),
        ])
        self.at(clock, client, 10.01)
        drive.set_target_rpm(20, -7)
        drive.tick()
        self.assertEqual(len(client.events), 2)
        self.at(clock, client, 10.02)
        drive.tick()
        self.assertEqual(client.events[-1], (10.02, 'SET_SPEED', -2000, 700, 290))
        self.at(clock, client, 10.04)
        drive.set_target_rpm(12.34, -5.67)
        drive.tick()
        self.assertEqual(client.events[-1], (10.04, 'SET_SPEED', -1234, 567, 300))

        # Telemetry deliberately differs from the latest requested target.
        client.latest.update(stamp=1234, counts=[-1326, -2652],
                             target_rpm_centi=[-1100, 600],
                             measured_rpm_centi=[-900, -450], command_pwm=[-200, 150])
        snapshot = drive.feedback()
        self.assertEqual(snapshot['stamp_ms'], 1234)
        self.assertTrue(snapshot['fresh'])
        left, right = snapshot['wheels']['left'], snapshot['wheels']['right']
        self.assertEqual((left['encoder_count'], right['encoder_count']), (2652, -1326))
        self.assertAlmostEqual(left['position_rad'], 2 * math.pi)
        self.assertAlmostEqual(right['position_rad'], -math.pi)
        self.assertEqual((left['target_rpm'], right['target_rpm']), (11, -6))
        self.assertEqual((left['measured_rpm'], right['measured_rpm']), (9, -4.5))
        self.assertEqual((left['pwm'], right['pwm']), (200, -150))
        self.assertEqual(drive.target_centi, (1234, -567))
        clock.now = 10.45
        self.assertFalse(drive.feedback()['fresh'])

    def test_input_expiry_does_not_renew_lease_and_zero_needs_fresh_intent(self):
        clock, client, drive = self.fixture(20.0)
        drive.set_target_rpm(0, 0)
        drive.tick()
        self.assertEqual(client.events, [])  # Zero alone never arms a disabled drive.
        drive.set_target_rpm(30, 30)
        drive.tick()
        for now in (20.02, 20.18, 20.2):
            self.at(clock, client, now)
            drive.tick()
        speed_events = [event for event in client.events if event[1] == 'SET_SPEED']
        self.assertEqual([event[-1] for event in speed_events], [300, 280, 120, 100])
        for event in speed_events:
            self.assertLessEqual(event[0] + event[-1] / 1000, 20.3 + 1e-9)
        event_count = len(client.events)
        for now in (20.201, 20.25, 20.299):
            self.at(clock, client, now)
            drive.tick()
        self.assertEqual(len(client.events), event_count)
        self.assertEqual(drive.command_at, 20.0)
        self.at(clock, client, 20.301)
        drive.tick()
        self.assertEqual(client.events[-1][1:], ('STOP',))
        self.assertTrue(drive.timed_out)
        self.assertFalse(drive.engaged)
        self.assertIsNone(drive.command_at)
        event_count = len(client.events)
        self.at(clock, client, 20.32)
        drive.tick()
        self.assertEqual(len(client.events), event_count)

        self.at(clock, client, 20.34)
        drive.set_target_rpm(30, 30)
        drive.tick()
        self.assertFalse(drive.timed_out)
        self.assertEqual(client.events[-2][1:], ('ARM',))
        self.at(clock, client, 20.35)
        drive.stop()
        self.assertEqual(client.events[-1][1:], ('SET_SPEED', 0, 0, 300))
        self.assertTrue(drive.engaged)
        for now in (20.4, 20.5):
            self.at(clock, client, now)
            drive.set_target_rpm(0, 0)
            drive.tick()
            self.assertEqual(client.events[-1][1:], ('SET_SPEED', 0, 0, 300))
        self.assertEqual(sum(event[1] == 'ARM' for event in client.events), 2)
        self.at(clock, client, 20.801)
        drive.tick()
        self.assertEqual(client.events[-1][1:], ('STOP',))

        # A delayed ARM acknowledgement must not revive an expired command.
        clock, client, drive = self.fixture(30.0)
        client.on_arm = lambda: self.at(clock, client, 30.301)
        drive.set_target_rpm(30, 30)
        drive.tick()
        self.assertEqual([event[1] for event in client.events], ['ARM', 'STOP'])
        self.assertTrue(drive.timed_out)

    def test_feedback_input_and_serial_failures_latch_until_new_session(self):
        cases = ('stale', 'missing', 'mcu_fault', 'nan', 'bool', 'over_limit',
                 'speed_write', 'speed_and_stop_write', 'arm_write')
        for case in cases:
            with self.subTest(case=case):
                clock, client, drive = self.fixture()
                if case == 'arm_write':
                    client.fail_arm = True
                    drive.set_target_rpm(30, 30)
                else:
                    drive.set_target_rpm(30, 30)
                    drive.tick()
                    self.at(clock, client, 10.02)
                action = drive.tick
                if case == 'stale':
                    client.latest['received_at'] = clock() - .401
                elif case == 'missing':
                    client.latest = None
                elif case == 'mcu_fault':
                    client.latest.update(fault=9, fault_reason='expired MCU lease')
                elif case in ('nan', 'bool', 'over_limit'):
                    invalid = {'nan': float('nan'), 'bool': True, 'over_limit': 95.01}[case]
                    action = lambda: drive.set_target_rpm(invalid, 30)
                elif case.startswith('speed'):
                    client.fail_speed = True
                    client.fail_stop = case == 'speed_and_stop_write'
                with self.assertRaises(MotorError):
                    action()
                self.assertEqual(client.events[-1][1:], ('STOP',))
                self.assertTrue(drive.error)
                self.assertFalse(drive.engaged)
                self.assertEqual(drive.target_centi, (0, 0))
                self.assertIsNone(drive.command_at)
                client.fail_speed = client.fail_stop = client.fail_arm = False
                self.at(clock, client, 10.03)
                event_count = len(client.events)
                for retry in (drive.tick, lambda: drive.set_target_rpm(30, 30)):
                    with self.assertRaises(MotorError):
                        retry()
                self.assertEqual(len(client.events), event_count)
                self.assertEqual(drive.feedback()['host_error'], drive.error)
                self.assertFalse(any(event[1] == 'CLEAR_FAULT' for event in client.events))

    def test_real_client_pty_forward_zero_resume_and_disable(self):
        calibration = DriveCalibration(document(right_motor_reversed=True))
        config = pid_document(calibration)
        config['hold']['enabled'] = True
        settings = PIDSettings(config, calibration)
        with MegaSimulator(swapped=True, protocol=8, encoder_signs=(1, -1)) as mega:
            with tempfile.TemporaryDirectory() as directory:
                with MotorClient(mega.port, directory, boot_delay=0,
                                 calibration=calibration) as client:
                    settings.configure(client)
                    drive = WheelDrive(client, settings)

                    def pump(target, seconds):
                        until = time.monotonic() + seconds
                        while time.monotonic() < until:
                            drive.set_target_rpm(*target)
                            drive.tick()
                            client.poll()
                            time.sleep(.003)
                        return drive.feedback()

                    forward = pump((30, 45), .16)
                    self.assertTrue(forward['fresh'])
                    self.assertTrue(forward['armed'])
                    for side, target in (('left', 30), ('right', 45)):
                        self.assertGreater(forward['wheels'][side]['encoder_count'], 0)
                        self.assertEqual(forward['wheels'][side]['target_rpm'], target)
                        self.assertEqual(forward['wheels'][side]['measured_rpm'], target)
                    drive.stop()
                    stopped = pump((0, 0), .14)
                    self.assertTrue(stopped['armed'])
                    self.assertEqual([w['target_rpm'] for w in stopped['wheels'].values()], [0, 0])
                    self.assertEqual([w['measured_rpm'] for w in stopped['wheels'].values()], [0, 0])
                    reverse = pump((-20, -10), .12)
                    self.assertEqual(reverse['wheels']['left']['measured_rpm'], -20)
                    self.assertEqual(reverse['wheels']['right']['measured_rpm'], -10)
                    self.assertEqual(sum(command[0] == 'ARM' for command in mega.commands), 1)
                    targets = [command[1:3] for command in mega.commands if command[0] == 'SET_SPEED']
                    self.assertIn(['-3000', '-4500'], targets)
                    self.assertIn(['0', '0'], targets)
                    self.assertIn(['2000', '1000'], targets)
                    drive.disable()
                    client.collect(.08)
                    self.assertFalse(drive.engaged)
                    self.assertFalse(client.latest['armed'])
                    self.assertEqual(client.latest['command_pwm'], [0, 0])
                    self.assertEqual(client.latest['fault'], 0)
                    self.assertEqual(mega.commands[-1][0], 'STOP')
                    self.assertFalse(any(command[0] in ('BRAKE', 'DRIVE', 'BENCH')
                                         for command in mega.commands))


if __name__ == '__main__':
    unittest.main()
