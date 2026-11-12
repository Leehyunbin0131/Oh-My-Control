"""Terminal input, keyboard timeout, steering signs and safe teleop cleanup."""

import json
import io
import os
from pathlib import Path
import pty
import select
import subprocess
import sys
import tempfile
import termios
import time
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
from keyboard_control import Controller, Keys, options, render
from drive_calibration import DriveCalibration
from motor_test import MotorError
from motor_pid import PIDSettings
from test_drive_calibration import document
from test_motor_check import MegaSimulator
from test_motor_pid import pid_document, firmware_info


class FakeClient:
    def __init__(self):
        self.calibration = DriveCalibration(document())
        self.info = firmware_info()
        self.info['max_drive_pwm'] = 250
        self.calls = []
        self.latest = {'stamp': 0, 'received_at': 10.0, 'fault': 0, 'fault_reason': '',
                       'command_pwm': [0, 0], 'armed': False, 'counts': [0, 0], 'invalid': [0, 0]}

    def command(self, operation, *values):
        self.calls.append((operation, *values))
        if operation in ('STOP', 'BRAKE'):
            self.latest.update(command_pwm=[0, 0], armed=False)
        elif operation == 'ARM':
            self.latest['armed'] = True
        elif operation == 'SET_SPEED':
            self.latest.update(pid_mode=True, target_rpm_centi=list(values[:2]))
        elif operation in ('BENCH', 'DRIVE'):
            self.latest['command_pwm'] = list(values[:2])

    send = command

    def set_speed(self, l, r, lease_ms=300, **kwargs):
        self.command('SET_SPEED', l, r, lease_ms)

    def sample(self, now, counts=None):
        self.latest = {**self.latest, 'stamp': round(now * 1000), 'received_at': now,
                       'counts': counts if counts is not None else self.latest['counts']}


class KeyTests(unittest.TestCase):
    def test_split_arrow_sequences_are_single_commands(self):
        keys = Keys()
        self.assertEqual(keys.feed(b'\x1b', 1), [])
        self.assertEqual(keys.feed(b'[', 1.01), [])
        self.assertEqual(keys.feed(b'A', 1.02), ['w'])
        self.assertEqual(keys.feed(b'\x1bOD\x1b[C\x1b[B', 2), ['a', 'd', 'x'])

    def test_escape_unknown_sequences_and_stop_keys(self):
        keys = Keys()
        self.assertEqual(keys.feed(b'\x1b', 1), [])
        self.assertEqual(keys.feed(b'', 1.11), ['stop'])
        self.assertEqual(keys.feed(b'[A', 1.2), ['stop'])
        self.assertEqual(keys.feed(b'\x1b[1;5D', 2), ['stop'])
        self.assertEqual(keys.feed(b'WASD xq', 3), ['w', 'a', 'stop', 'd', 'stop', 'x', 'q'])
        self.assertEqual(keys.feed(b'QWEASDZXC', 4), ['q', 'w', 'e', 'a', 'stop', 'd', 'z', 'x', 'c'])
        self.assertEqual(keys.feed(b'\x03\x04', 5), ['quit', 'quit'])

    def test_delayed_arrow_suffix_cannot_become_a_turn_command(self):
        keys = Keys()
        self.assertEqual(keys.feed(b'\x1b[', 1), [])
        self.assertEqual(keys.feed(b'', 1.11), ['stop'])
        self.assertEqual(keys.feed(b'', 1.12), [])
        self.assertEqual(keys.feed(b'A', 1.2), ['stop'])
        self.assertEqual(keys.feed(b'w', 1.3), ['w'])


class ControllerTests(unittest.TestCase):
    def test_pid_in_place_limit_preserves_selected_forward_speed(self):
        client = FakeClient()
        settings = PIDSettings(pid_document(client.calibration), client.calibration)
        controller = Controller(client, pid_settings=settings)
        for index, (key, expected) in enumerate([
            ('a', (-3000, 3000)), ('+', (-3000, 3000)),
            ('d', (3000, -3000)), ('w', (7400, 7400)),
        ]):
            now = 10 + index * .05
            controller.keys([key], now)
            self.step(controller, client, now)
            self.assertEqual(controller.active, expected)
        controller.keys(['-'] * 17 + ['a'], 10.2)
        self.step(controller, client, 10.2)
        self.assertEqual(controller.active, (-3000, 3000))
        controller.keys(['stop'], 10.25)
        self.step(controller, client, 10.25)
        self.assertEqual(client.calls[-1], ('SET_SPEED', 0, 0, 300))

    def test_pid_speed_limits_preserve_stop_and_reject_unstable_low_speed(self):
        client = FakeClient()
        client.info['pid_supported'] = True
        settings = PIDSettings(pid_document(client.calibration), client.calibration)
        controller = Controller(client, pid_settings=settings)
        controller.keys(['-'] * 100, 10)
        self.assertEqual(controller.rpm, 30)
        self.step(controller, client, 10)
        controller.keys(['+'] * 100, 10)
        self.assertEqual(controller.rpm, 100)
        self.step(controller, client, 10)
        self.assertEqual(client.calls, [])
        for rpm in (20, 101):
            with self.assertRaises(MotorError):
                Controller(client, pid_settings=settings, rpm=rpm)
        with patch('sys.stderr', new_callable=io.StringIO), self.assertRaises(SystemExit):
            options(['--rpm', '20'])

    def test_default_estimate_and_full_output_require_v3(self):
        self.assertTrue(options([]).pid)
        self.assertEqual(options([]).rpm, 72)
        self.assertIsNone(options([]).pwm)
        self.assertEqual(options([]).key_timeout, 0)
        self.assertEqual(options(['--key-timeout', '0.2']).key_timeout, 0.2)
        self.assertEqual(options(['--rpm-approx', '95']).pwm, 650)
        client = FakeClient()
        with self.assertRaisesRegex(MotorError, 'v8'):
            Controller(client, pwm=650)
        self.assertEqual(client.calls, [])
        client.info['max_drive_pwm'] = 999
        controller = Controller(client, pwm=650, key_timeout=0.6)
        controller.keys(['w'], 10)
        self.step(controller, client, 10)
        self.assertIn(('DRIVE', -650, 650, 30000), client.calls)
        client.latest['wheels'] = {'left': {'count': 2652, 'ticks_per_second': 5304}}
        self.assertIn('+120.0 RPM', render(controller))
        self.step(controller, client, 10.61, [200, -200])
        self.assertEqual(client.calls[-1], ('STOP',))

    def step(self, controller, client, now, counts=None):
        client.sample(now, counts)
        with patch('keyboard_control.time.monotonic', return_value=now):
            controller.step(now)

    def test_physical_forward_and_keyboard_timeout(self):
        client = FakeClient()
        controller = Controller(client, key_timeout=0.6)
        controller.keys(['w'], 10)
        self.step(controller, client, 10)
        self.assertIn(('BENCH', -180, 180, 30000), client.calls)
        self.step(controller, client, 10.3, [100, -100])
        self.assertIn(('PING',), client.calls)
        self.step(controller, client, 10.61, [200, -200])
        self.assertEqual(client.calls[-1], ('STOP',))
        self.assertEqual(controller.active, (0, 0))
        self.step(controller, client, 11)
        self.assertEqual(sum(c[0] == 'BENCH' for c in client.calls), 1)

    def test_stop_and_quit_dominate_buffered_movement(self):
        for stop in ('stop', 'quit'):
            with self.subTest(stop=stop):
                client = FakeClient(); controller = Controller(client)
                controller.keys(['w', stop, 'd'], 10)
                self.step(controller, client, 10)
                self.assertEqual(client.calls, [('STOP',)])
                self.assertIsNone(controller.direction)

    def test_turn_change_stops_before_reversing_a_wheel(self):
        client = FakeClient(); controller = Controller(client)
        controller.keys(['w'], 10)
        self.step(controller, client, 10)
        controller.keys(['a'], 10.1)
        self.step(controller, client, 10.1)
        self.assertEqual(client.calls[-1], ('STOP',))
        self.step(controller, client, 10.2)
        self.assertEqual(client.calls[-1], ('STOP',))
        self.step(controller, client, 10.31)
        self.assertEqual(controller.active, (-180, 180))
        self.assertIn(('BENCH', 180, 180, 30000), client.calls)

    def test_expired_firmware_segment_needs_fresh_keyboard_intent(self):
        for fresh in (False, True):
            with self.subTest(fresh=fresh):
                client = FakeClient(); controller = Controller(client, key_timeout=0.6)
                controller.keys(['w'], 10)
                self.step(controller, client, 10)
                client.latest.update(command_pwm=[0, 0], armed=False)
                if fresh:
                    controller.keys(['w'], 40.01)
                self.step(controller, client, 40.01, [30000, -30000])
                self.assertEqual(sum(c[0] == 'BENCH' for c in client.calls), 2 if fresh else 1)

    def test_speed_keys_do_not_renew_motion_lease(self):
        client = FakeClient(); controller = Controller(client, key_timeout=0.6)
        controller.keys(['w'], 10)
        self.step(controller, client, 10)
        controller.keys(['+'], 10.7)
        self.step(controller, client, 10.7)
        self.assertEqual(controller.pwm, 190)
        self.assertEqual(controller.active, (0, 0))

    def test_stale_feedback_and_encoder_stall_are_errors(self):
        client = FakeClient(); controller = Controller(client)
        controller.keys(['w'], 10)
        self.step(controller, client, 10)
        with self.assertRaisesRegex(MotorError, '수신 중단'):
            controller.step(10.41)
        for tick in range(1, 10):
            now = 10 + tick / 10
            controller.keys(['w'], now)
            self.step(controller, client, now)
        controller.keys(['w'], 11.1)
        with self.assertRaisesRegex(MotorError, '회전 부족'):
            self.step(controller, client, 11.1)

    def test_latched_key_survives_repeat_delay_until_s_and_never_restarts_after_stop(self):
        client = FakeClient()
        client.info['max_drive_pwm'] = 999
        controller = Controller(client, pwm=493)
        controller.keys(Keys().feed(b'w', 10), 10)
        self.step(controller, client, 10)
        # No repeat keys at all, including the old 0.2 s / 0.5 s gap.
        for now in (10.2, 10.5, 10.7, 11.0, 11.3):
            count = round((now - 10) * 600)
            self.step(controller, client, now, [count, -count])
            self.assertEqual(controller.active, (493, 493))
        self.assertEqual(sum(c[0] == 'DRIVE' for c in client.calls), 1)
        controller.keys(Keys().feed(b's', 11.31), 11.31)
        self.step(controller, client, 11.31)
        self.assertEqual(client.calls[-1], ('STOP',))
        self.assertIsNone(controller.direction)
        controller.keys(['+'], 11.4)
        self.step(controller, client, 12)
        self.assertEqual(sum(c[0] == 'DRIVE' for c in client.calls), 1)

    def test_eight_direction_keys_map_to_calibrated_outputs(self):
        expected = {'q': [-247, 493], 'w': [-493, 493], 'e': [-493, 247],
                    'a': [493, 493], 'd': [-493, -493],
                    'z': [247, -493], 'x': [493, -493], 'c': [493, -247]}
        for key, output in expected.items():
            with self.subTest(key=key):
                client = FakeClient(); client.info['max_drive_pwm'] = 999
                controller = Controller(client, pwm=493)
                controller.keys(Keys().feed(key.encode(), 10), 10)
                self.step(controller, client, 10)
                self.assertIn(('DRIVE', *output, 30000), client.calls)
                self.assertFalse(controller.quit)

    def test_latched_segment_continues_and_stop_at_boundary_prevents_restart(self):
        client = FakeClient(); controller = Controller(client)
        controller.keys(['w'], 10)
        self.step(controller, client, 10)
        client.latest.update(command_pwm=[0, 0], armed=False)
        self.step(controller, client, 40.01, [30000, -30000])
        self.assertEqual(sum(c[0] == 'BENCH' for c in client.calls), 2)
        client.latest.update(command_pwm=[0, 0], armed=False)
        controller.keys(Keys().feed(b's', 70.02), 70.02)
        self.step(controller, client, 70.02, [60000, -60000])
        self.step(controller, client, 70.5)
        self.assertEqual(sum(c[0] == 'BENCH' for c in client.calls), 2)

    def test_s_cancels_pending_direction_change(self):
        client = FakeClient(); controller = Controller(client)
        controller.keys(['w'], 10)
        self.step(controller, client, 10)
        controller.keys(['a'], 10.1)
        self.step(controller, client, 10.1)
        controller.keys(Keys().feed(b's', 10.15), 10.15)
        self.step(controller, client, 10.15)
        self.step(controller, client, 10.5)
        self.assertEqual(controller.active, (0, 0))
        self.assertEqual(sum(c[0] == 'BENCH' for c in client.calls), 1)


class TerminalIntegrationTests(unittest.TestCase):
    def session(self, folder, mega, default_speed=False, pid=False):
        config = Path(folder) / 'calibration.json'
        direction_data = document(right_motor_reversed=mega.encoder_signs[1] < 0)
        config.write_text(json.dumps(direction_data))
        master, slave = pty.openpty()
        saved = termios.tcgetattr(slave)
        speed_args = ['--pwm', '493'] if default_speed else ['--pwm', '180', '--key-timeout', '0.25']
        if pid:
            settings_path = Path(folder) / 'pid.json'
            pid_data = pid_document(DriveCalibration(direction_data))
            if mega.protocol >= 5:
                pid_data['period_ms'] = 1
            settings_path.write_text(json.dumps(pid_data))
            speed_args = ['--pid-config', str(settings_path)]
        process = subprocess.Popen([sys.executable, str(ROOT / 'tools/keyboard_control.py'), *speed_args,
                                    '--port', mega.port,
                                    '--calibration', str(config), '--report-dir', folder],
                                   stdin=slave, stdout=slave, stderr=slave, start_new_session=True)
        return NS(master=master, slave=slave, saved=saved, process=process, output=bytearray())

    def pump(self, session, seconds=0.03):
        if session.master >= 0 and select.select([session.master], [], [], seconds)[0]:
            try:
                session.output.extend(os.read(session.master, 65536))
            except OSError:
                pass
        else:
            time.sleep(0.005)

    def wait_for(self, session, predicate, timeout=3):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.pump(session)
            if predicate():
                return
        self.fail('Condition timed out: ' + session.output.decode(errors='replace')[-1200:])

    def cleanup(self, session):
        if session.process.poll() is None:
            session.process.kill()
            session.process.wait(timeout=2)
        if session.master >= 0:
            os.close(session.master)
        os.close(session.slave)

    def test_reversed_right_wiring_keyboard_pwm_and_pid_directions(self):
        for pid in (False, True):
            with self.subTest(pid=pid), MegaSimulator(swapped=True, protocol=8, encoder_signs=(1, -1)) as mega, tempfile.TemporaryDirectory() as folder:
                session = self.session(folder, mega, default_speed=not pid, pid=pid)
                try:
                    self.wait_for(session, lambda: b'READY / STOPPED' in session.output, 6)
                    magnitude, inner = (7200, 3600) if pid else (493, 247)
                    turn = 3000 if pid else magnitude
                    for key, expected in [('w', [-magnitude, -magnitude]), ('x', [magnitude, magnitude]),
                                          ('a', [turn, -turn]), ('d', [-turn, turn]),
                                          ('q', [-inner, -magnitude]), ('e', [-magnitude, -inner]),
                                          ('z', [inner, magnitude]), ('c', [magnitude, inner])]:
                        os.write(session.master, key.encode())
                        self.wait_for(session, lambda: (mega.targets if pid else mega.pwm) == expected)
                        if key == 'w':
                            deadline = time.monotonic() + 1.15
                            while time.monotonic() < deadline:
                                self.pump(session)
                                self.assertIsNone(session.process.poll(), session.output.decode(errors='replace'))
                    os.write(session.master, b's')
                    self.wait_for(session, lambda: mega.pwm == [0, 0])
                    os.write(session.master, b'\x04')
                    self.wait_for(session, lambda: session.process.poll() is not None)
                    self.assertEqual(session.process.returncode, 0, session.output.decode(errors='replace'))
                    self.assertEqual(termios.tcgetattr(session.slave), session.saved)
                finally:
                    self.cleanup(session)

    def test_real_tty_repeat_release_stop_and_restore(self):
        with MegaSimulator(swapped=True) as mega, tempfile.TemporaryDirectory() as folder:
            session = self.session(folder, mega)
            try:
                self.wait_for(session, lambda: b'READY / STOPPED' in session.output, 6)
                self.assertFalse(any(c[0] == 'BENCH' for c in mega.commands))
                os.write(session.master, b'\x1b[A')
                self.wait_for(session, lambda: mega.pwm == [-180, 180])
                for _ in range(8):
                    os.write(session.master, b'w')
                    self.pump(session, 0.04)
                    time.sleep(0.04)
                self.assertEqual(mega.pwm, [-180, 180])
                self.assertEqual(sum(c[0] == 'BENCH' for c in mega.commands), 1)
                self.wait_for(session, lambda: mega.pwm == [0, 0], 1)
                os.write(session.master, b'x')
                self.wait_for(session, lambda: mega.pwm == [180, -180])
                os.write(session.master, b' dw')  # STOP wins over buffered directions
                self.wait_for(session, lambda: mega.pwm == [0, 0], 1)
                os.write(session.master, b'\x04')
                self.wait_for(session, lambda: session.process.poll() is not None)
                self.assertEqual(session.process.returncode, 0, session.output.decode(errors='replace'))
                self.assertEqual(termios.tcgetattr(session.slave), session.saved)
                self.assertEqual(mega.commands[-1][0], 'STOP')
                record = json.loads(next(Path(folder).glob('teleop_*/summary.json')).read_text())
                self.assertIsNone(record['error'])
            finally:
                self.cleanup(session)

    def test_sigterm_while_moving_sends_stop(self):
        with MegaSimulator(swapped=True, protocol=8) as mega, tempfile.TemporaryDirectory() as folder:
            session = self.session(folder, mega, default_speed=True)
            try:
                self.wait_for(session, lambda: b'READY / STOPPED' in session.output, 6)
                os.write(session.master, b'd')
                self.wait_for(session, lambda: mega.pwm == [-493, -493])
                session.process.terminate()
                self.wait_for(session, lambda: session.process.poll() is not None)
                self.wait_for(session, lambda: mega.pwm == [0, 0])
                self.assertEqual(session.process.returncode, 130)
                self.assertEqual(termios.tcgetattr(session.slave), session.saved)
                self.assertEqual(mega.commands[-1][0], 'STOP')
            finally:
                self.cleanup(session)

    def test_terminal_disconnect_stops_motor(self):
        with MegaSimulator(swapped=True, protocol=8) as mega, tempfile.TemporaryDirectory() as folder:
            session = self.session(folder, mega, default_speed=True)
            try:
                self.wait_for(session, lambda: b'READY / STOPPED' in session.output, 6)
                os.write(session.master, b'w')
                self.wait_for(session, lambda: mega.pwm == [-493, 493])
                os.close(session.master)
                session.master = -1
                self.wait_for(session, lambda: mega.pwm == [0, 0], 1)
                self.wait_for(session, lambda: session.process.poll() is not None)
            finally:
                self.cleanup(session)

    def test_default_60_percent_output_and_v3_watchdog_cleanup(self):
        with MegaSimulator(swapped=True, protocol=8) as mega, tempfile.TemporaryDirectory() as folder:
            session = self.session(folder, mega, default_speed=True)
            try:
                self.wait_for(session, lambda: b'READY / STOPPED' in session.output, 6)
                self.assertIn(b'PWM 493/999', session.output)
                self.assertIn(b'LATCHED', session.output)
                self.assertFalse(any(c[0] == 'DRIVE' for c in mega.commands))
                os.write(session.master, b'w')
                self.wait_for(session, lambda: mega.pwm == [-493, 493])
                deadline = time.monotonic() + 0.7
                while time.monotonic() < deadline:
                    self.pump(session)
                    self.assertEqual(mega.pwm, [-493, 493])
                self.assertEqual(sum(c[0] == 'DRIVE' for c in mega.commands), 1)
                os.write(session.master, b'q')  # Q curves left; no longer quits.
                self.wait_for(session, lambda: mega.pwm == [-247, 493])
                self.assertIsNone(session.process.poll())
                os.write(session.master, b's')
                self.wait_for(session, lambda: mega.pwm == [0, 0], 1)
                os.write(session.master, b'x')
                self.wait_for(session, lambda: mega.pwm == [493, -493])
                session.process.kill()  # MCU heartbeat stops even without host cleanup
                session.process.wait(timeout=2)
                self.wait_for(session, lambda: mega.pwm == [0, 0], 1)
            finally:
                self.cleanup(session)

    def test_default_pid_targets_curve_stop_and_sigterm(self):
        with MegaSimulator(swapped=True, protocol=8) as mega, tempfile.TemporaryDirectory() as folder:
            session = self.session(folder, mega, pid=True)
            try:
                self.wait_for(session, lambda: b'READY / STOPPED' in session.output, 6)
                self.assertIn(b'PID target 72.0 RPM', session.output)
                self.assertEqual(len(mega.pid_gains), 4)
                self.assertFalse(any(c[0] == 'SPEED' for c in mega.commands))
                os.write(session.master, b'w')
                self.wait_for(session, lambda: mega.targets == [-7200, 7200])
                deadline = time.monotonic() + 1.2
                while time.monotonic() < deadline:
                    self.pump(session)
                    self.assertTrue(mega.pid_mode)
                os.write(session.master, b'q')
                self.wait_for(session, lambda: mega.targets == [-3600, 7200])
                os.write(session.master, b's')
                self.wait_for(session, lambda: mega.pwm == [0, 0])
                self.assertEqual(mega.targets, [0, 0])
                os.write(session.master, b'x')
                self.wait_for(session, lambda: mega.targets == [7200, -7200])
                session.process.terminate()
                self.wait_for(session, lambda: session.process.poll() is not None)
                self.wait_for(session, lambda: mega.pwm == [0, 0])
                self.assertEqual(session.process.returncode, 130)
                self.assertEqual(termios.tcgetattr(session.slave), session.saved)
            finally:
                self.cleanup(session)

    def test_1khz_pid_firmware_start_and_stop(self):
        with MegaSimulator(swapped=True, protocol=8) as mega, tempfile.TemporaryDirectory() as folder:
            session = self.session(folder, mega, pid=True)
            try:
                self.wait_for(session, lambda: b'READY / STOPPED' in session.output, 6)
                self.assertIn(b'MCU loop 1 ms (1000 Hz)', session.output)
                os.write(session.master, b'w')
                self.wait_for(session, lambda: mega.targets == [-7200, 7200])
                os.write(session.master, b's')
                self.wait_for(session, lambda: mega.pwm == [0, 0])
                os.write(session.master, b'\x04')
                self.wait_for(session, lambda: session.process.poll() is not None)
                self.assertEqual(session.process.returncode, 0)
                self.assertEqual(termios.tcgetattr(session.slave), session.saved)
            finally:
                self.cleanup(session)


if __name__ == '__main__':
    unittest.main()
