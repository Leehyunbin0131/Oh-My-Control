"""Motor fault, direction and cleanup checks without touching real hardware."""

from contextlib import redirect_stderr
import io
import os
from pathlib import Path
import pty
import select
import subprocess
import sys
import tempfile
import threading
import time
import tty
from types import SimpleNamespace as NS
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
from motor_test import (Decoder, MotorClient, MotorError, frame, parse_info,
                        signed_delta, observation_result, evaluate_pulse, drive_test, bench_test)
from check_sensors import parse_args
from sensor_monitor import SensorMonitor
from hardware_tui import Dashboard
from terminal_view import RobotModel


class MegaSimulator:
    """Independent PTY peer: commands are acknowledged and wheel counts simulated."""
    def __init__(self, link=1, frozen=False, corrupt_on_run=False, unknown=False, silence_on_run=False, swapped=False, protocol=8,
                 encoder_signs=(1, 1)):
        self.master, slave = pty.openpty()
        os.set_blocking(self.master, False)
        tty.setraw(slave)
        self.port = os.ttyname(slave)
        os.close(slave)
        self.link, self.frozen = link, frozen
        self.corrupt_on_run, self.unknown = corrupt_on_run, unknown
        self.silence_on_run = silence_on_run
        self.swapped = swapped
        self.protocol = protocol
        self.encoder_signs = encoder_signs  # Raw encoder direction per driver L/R command.
        self.stop_event = threading.Event()
        self.commands = []
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.sequence = 0
        self.counts, self.pwm = [0, 0], [0, 0]
        self.armed = False
        self.deadline = self.heartbeat = 0
        self.corrupted = False
        self.pid_mode = False
        self.targets = [0, 0]
        self.pid_gains = {}
        self.fault = 0
        self.streaming = False
        self.applied_sequence = 0
        self.zero_since = [0, 0]

    def __enter__(self):
        self.thread.start()
        return self

    def __exit__(self, *_):
        self.stop_event.set()
        self.thread.join(2)
        os.close(self.master)

    def send(self, body):
        try:
            os.write(self.master, frame(body))
        except OSError:
            pass  # no client currently has the slave open

    def run(self):
        decoder = Decoder()
        last_t = last_i = 0
        while not self.stop_event.wait(0.003):
            now = time.monotonic()
            if self.armed and (now >= self.deadline or now - self.heartbeat > 0.3):
                if self.streaming: self.fault = 9
                self.streaming = False
                self.pwm = [0, 0]
                self.armed = False
                self.pid_mode = False; self.targets = [0, 0]
            try:
                if select.select([self.master], [], [], 0)[0]:
                    incoming = os.read(self.master, 4096)
                    for fields in decoder.feed(incoming):
                        seq, command = int(fields[2]), fields[3]
                        self.sequence = seq
                        self.commands.append(fields[3:])
                        ok = True
                        if command == 'CLEAR_FAULT':
                            self.fault = 0
                        elif command in ('ZERO', 'HOLD', 'HOLD_PI'):
                            ok = self.protocol == 8 and not self.armed
                        elif command == 'BRAKE' and self.protocol == 7:
                            driven = any(self.pwm) and self.armed
                            self.pwm = [0, 0]
                            self.armed = False
                            self.pid_mode = False; self.targets = [0, 0]
                            self.send(f'OMR1,B,{seq},15000,0,0,14000,14000,3,3,{150 if driven else 0},{150 if driven else 0}')
                        elif command == 'STOP':
                            self.streaming = False
                            self.pwm = [0, 0]
                            self.armed = False
                            self.pid_mode = False; self.targets = [0, 0]
                        elif command == 'ARM':
                            ok = bool(self.link) and not self.fault
                            self.armed = ok
                            self.heartbeat = now
                            self.deadline = now + 10
                        elif command in ('RUN', 'BENCH', 'DRIVE'):
                            limit = (650 if self.protocol == 8 else 999) if command == 'DRIVE' and self.protocol >= 3 else 250
                            ok = self.armed and (command != 'DRIVE' or self.protocol >= 3) and all(abs(int(v)) <= limit for v in fields[4:6])
                            if ok:
                                self.pwm = [int(fields[4]), int(fields[5])]
                                self.pid_mode = False; self.targets = [0, 0]
                                self.deadline = now + int(fields[6]) / 1000
                        elif command == 'PID':
                            ok = self.protocol >= 4 and not self.armed
                            if ok:
                                self.pid_gains[(int(fields[4]), int(fields[5]))] = [int(v) / 1000 for v in fields[6:11]]
                        elif command in ('SPEED', 'SET_SPEED'):
                            targets = list(map(int, fields[4:6]))
                            ok = self.protocol >= 4 and self.armed and (any(targets) or command == 'SET_SPEED') and all(abs(v) <= (10000 if self.protocol == 8 else 12000) for v in targets)
                            ok = ok and all((i, 1 if t > 0 else -1) in self.pid_gains for i, t in enumerate(targets) if t)
                            if ok:
                                for i, target in enumerate(targets):
                                    if target != self.targets[i] or not self.pid_mode: self.zero_since[i] = now
                                self.applied_sequence = seq
                                self.streaming = command == 'SET_SPEED'
                                if self.streaming: self.heartbeat = now
                                self.targets = targets; self.pid_mode = True
                                self.pwm = []
                                for i, target in enumerate(targets):
                                    gains = self.pid_gains.get((i, 1 if target > 0 else -1), [0] * 5)
                                    self.pwm.append((1 if target > 0 else -1) * min(650, round(gains[4] + gains[3] * abs(target) / 100)) if target else 0)
                                self.deadline = now + int(fields[6]) / 1000
                        elif command == 'PING':
                            self.heartbeat = now
                        else:
                            ok = False
                        self.send(f'OMR1,A,{seq},{"OK" if ok else "ERR"},simulator')
            except OSError:
                continue
            if now - last_i >= 0.15:
                if self.unknown:
                    os.write(self.master, b'unrelated old sketch\n')
                else:
                    extra = (',650' if self.protocol == 8 else ',999') if self.protocol >= 3 else ''
                    if self.protocol >= 4:
                        extra += ',2652,' + ('1' if self.protocol >= 5 else '50') + (',10000,650' if self.protocol == 8 else ',12000,650')
                    if self.protocol >= 5:
                        extra += ',400000,1000'
                    if self.protocol >= 6:
                        extra += ',' + ','.join(map(str, self.encoder_signs))
                    if self.protocol == 7:
                        extra += ',150,60' if self.link == 1 else ',0,0'
                    if self.protocol == 8:
                        extra += ',0,0,1,15'
                    self.send(f'OMR1,I,{self.protocol},40,41,50,51,20000,300,1500,250,{self.link},1,30000{extra}')
                    self.send(f'OMR1,D,{int(now * 1000) & 0xffffffff},{(int(now * 1000) * 20) & 0xffffffff},0,0')
                last_i = now
            if not self.unknown and now - last_t >= 0.025:
                if not self.frozen:
                    drive_rates = [value * sign for value, sign in
                                   zip(self.targets if self.pid_mode else self.pwm, self.encoder_signs)]
                    rates = drive_rates[::-1] if self.swapped else drive_rates
                    if self.pid_mode:
                        self.counts = [(c + round(r / 100 * 2652 / 60 * (now - last_t))) & 0xffffffff for c, r in zip(self.counts, rates)]
                    else:
                        self.counts = [(count + (2 if pwm > 0 else -2 if pwm < 0 else 0)) & 0xffffffff
                                       for count, pwm in zip(self.counts, rates)]
                body = ','.join(map(str, ('OMR1', 'T', int(now * 1000) & 0xffffffff,
                                         *self.counts, 0, 0, 0, 0, *self.pwm,
                                         int(self.armed), self.fault, self.sequence, self.link)))
                if self.protocol >= 4:
                    measured = [target * sign for target, sign in zip(self.targets, self.encoder_signs)]
                    body += ',' + ','.join(map(str, (int(self.pid_mode), *self.targets, *measured)))
                if self.silence_on_run and any(c[0] == 'RUN' for c in self.commands):
                    pass
                elif self.corrupt_on_run and any(self.pwm) and not self.corrupted:
                    bad = bytearray(frame(body)); bad[-3] = ord('Z')
                    os.write(self.master, bad)
                    self.corrupted = True
                else:
                    self.send(body)
                    if self.protocol == 8:
                        states = [1 if n else 2 for n in self.targets] if self.pid_mode else [0, 0]
                        settled = [int(self.pid_mode and not n and now - start >= .1) for n, start in zip(self.targets, self.zero_since)]
                        mode = 3 if self.fault else 2 if self.pid_mode or any(self.pwm) else 1 if self.armed else 0
                        self.send(','.join(map(str, ('OMR1','Q',int(now*1000)&0xffffffff,self.applied_sequence,
                                                     int(now*1000)&0xffffffff,0,mode,*states,*settled,0,0,0,0,0,0))))
                last_t = now


class ProtocolTests(unittest.TestCase):
    def test_receive_diagnostics_and_stop_ack_survive_rejected_target_in_same_read(self):
        from types import SimpleNamespace
        from motor_test import parse_receive_diagnostic
        fields = 'OMR1,R,1234,256,42,3100,59,0,0,0'.split(',')
        self.assertEqual(parse_receive_diagnostic(fields)['max_queued'], 42)
        with self.assertRaises(MotorError):
            parse_receive_diagnostic('OMR1,R,1234,256,256,3100,59,0,0,0'.split(','))
        data = frame('OMR1,A,40,ERR,target') + frame('OMR1,A,41,OK,disabled') + frame(','.join(fields))
        client = MotorClient('unused', '.')
        client.serial = SimpleNamespace(in_waiting=len(data), read=lambda n: data)
        client.log = io.BytesIO()
        client.identified = True
        client.info = {'protocol': 8}
        client.latest = {'fault': 3}
        with self.assertRaisesRegex(MotorError, 'fault 3'):
            client.poll()
        self.assertEqual(client.acks[41], ['OK', 'disabled'])
        self.assertEqual(client.receive_diagnostic['frames'], 59)

    def test_check_vector_fragmentation_and_corruption(self):
        self.assertEqual(frame('123456789'), b'123456789*29B1\n')
        packet = frame('OMR1,I,2,40,41,50,51,20000,300,1500,250,1,1,30000')
        decoder = Decoder()
        decoded = []
        for b in b'garbage\n' + packet:
            decoded.extend(decoder.feed(bytes([b])))
        self.assertEqual(decoder.errors, 1)
        self.assertEqual(parse_info(decoded[0])['pins'], [[40, 41], [50, 51]])
        self.assertEqual(parse_info(decoded[0])['driver_link'], 'i2c')
        bad = bytearray(packet); bad[12] ^= 1
        self.assertEqual(len(decoder.feed(bad + packet)), 1)

    def test_count_rollover_and_no_movement_is_not_encoder_pass(self):
        self.assertEqual(signed_delta(2, 0xfffffffe), 4)
        self.assertEqual(signed_delta(0xfffffffe, 2), -4)
        samples = [{'counts': [0, 0], 'invalid': [0, 0]} for _ in range(10)]
        self.assertEqual(observation_result(samples)['status'], 'PASS')
        self.assertFalse(observation_result(samples)['motor_tested'])
        self.assertEqual(observation_result(samples, True)['status'], 'FAIL')

    def test_wrong_encoder_and_coasting_fail(self):
        before = {'counts': [0, 0], 'invalid': [0, 0]}
        samples = [{'counts': [0, 30], 'invalid': [0, 0], 'received_at': i / 20,
                    'command_pwm': [180, 0] if i < 4 else [0, 0], 'armed': i < 4}
                   for i in range(20)]
        self.assertEqual(evaluate_pulse(before, samples, 0, [180, 0], 8)['status'], 'FAIL')
        for i, sample in enumerate(samples):
            sample['counts'] = [i * 3, 0]
        self.assertIn('계속 움직임', evaluate_pulse(before, samples, 0, [180, 0], 8)['reason'])

    def test_parameter_limits_and_explicit_motion_selection(self):
        for argv in (['--motor-test', 'drive'], ['--motors', '--motor-pwm', '251'],
                     ['--motors', '--motor-pulse', 'nan'], ['--motors', '--motor-pulse', '2'],
                     ['--motors', '--motor-bench-seconds', '31'], ['--motors', '--motor-bench-seconds', 'nan']):
            with self.subTest(argv=argv), redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                parse_args(argv)
        self.assertNotIn('motors', parse_args([]).sensors)
        self.assertIn('motors', parse_args(['--motors']).sensors)


class SerialIntegrationTests(unittest.TestCase):
    def test_bench_both_motors_and_final_stop(self):
        args = parse_args(['--motors', '--motor-bench-seconds', '0.2'])
        with MegaSimulator() as mega, tempfile.TemporaryDirectory() as folder:
            with MotorClient(mega.port, folder, 2, boot_delay=0) as client:
                result = bench_test(client, args)
                self.assertEqual(result['status'], 'PASS', result)
                self.assertEqual(result['pwm'], [180, 180])
                self.assertTrue(result['stopped'])
            time.sleep(0.03)
            self.assertEqual(mega.commands[-1][0], 'STOP')

    def test_bench_encoder_stall_stops_before_deadline(self):
        args = parse_args(['--motors', '--motor-bench-seconds', '30'])
        with MegaSimulator(frozen=True) as mega, tempfile.TemporaryDirectory() as folder:
            start = time.monotonic()
            with self.assertRaisesRegex(MotorError, '회전이 멈췄'):
                with MotorClient(mega.port, folder, 2, boot_delay=0) as client:
                    bench_test(client, args)
            self.assertLess(time.monotonic() - start, 3)
            time.sleep(0.03)
            self.assertEqual(mega.commands[-1][0], 'STOP')

    def test_unknown_firmware_never_receives_commands(self):
        with MegaSimulator(unknown=True) as mega, tempfile.TemporaryDirectory() as folder:
            with self.assertRaisesRegex(MotorError, '펌웨어 응답 없음'):
                with MotorClient(mega.port, folder, 0.2, boot_delay=0):
                    pass
            self.assertEqual(mega.commands, [])

    def test_read_only_mode_has_no_arm_or_run(self):
        with MegaSimulator(link=0) as mega, tempfile.TemporaryDirectory() as folder:
            with MotorClient(mega.port, folder, 2, boot_delay=0) as client:
                result = observation_result(client.collect(0.15))
                self.assertEqual(result['status'], 'PASS')
                with self.assertRaisesRegex(MotorError, '배선 설정'):
                    drive_test(client, NS())
            time.sleep(0.03)
            self.assertTrue(mega.commands)
            self.assertTrue(all(command[0] == 'STOP' for command in mega.commands))

    def test_cli_four_pulses_and_stop_results(self):
        import json
        with MegaSimulator(swapped=True) as mega, tempfile.TemporaryDirectory() as folder:
            result = subprocess.run([sys.executable, str(ROOT / 'tools/check_sensors.py'),
                                     '--sensors', 'motors', '--motor-test', 'drive', '--raw-motors',
                                     '--motor-port', mega.port, '--motor-pulse', '0.1',
                                     '--motor-min-ticks', '2', '--startup-timeout', '2',
                                     '--report-dir', folder], capture_output=True, text=True, timeout=15)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            report = json.loads(next(Path(folder).glob('*/summary.json')).read_text())
            self.assertTrue(report['passed'])
            self.assertEqual(len(report['results']['motors']['pulses']), 4)
            runs = [command for command in mega.commands if command[0] == 'RUN']
            self.assertEqual([run[1:3] for run in runs], [['0', '180'], ['0', '-180'], ['180', '0'], ['-180', '0']])
            self.assertEqual(mega.commands[-1][0], 'STOP')

    def test_frozen_encoder_aborts_after_first_pulse(self):
        args = parse_args(['--motors', '--motor-pulse', '0.1'])
        with MegaSimulator(frozen=True) as mega, tempfile.TemporaryDirectory() as folder:
            with MotorClient(mega.port, folder, 2, boot_delay=0) as client:
                result = drive_test(client, args)
                self.assertEqual(result['status'], 'FAIL')
                self.assertEqual(len(result['pulses']), 1)
            time.sleep(0.03)
            self.assertEqual(mega.commands[-1][0], 'STOP')

    def test_corrupt_packet_during_motion_stops(self):
        args = parse_args(['--motors', '--motor-pulse', '0.2'])
        with MegaSimulator(corrupt_on_run=True) as mega, tempfile.TemporaryDirectory() as folder:
            with self.assertRaisesRegex(MotorError, 'CRC'):
                with MotorClient(mega.port, folder, 2, boot_delay=0) as client:
                    drive_test(client, args)
            time.sleep(0.03)
            self.assertTrue(mega.corrupted)
            self.assertEqual(mega.commands[-1][0], 'STOP')
            self.assertFalse(any(mega.pwm))

    def test_tui_motor_only_is_passive_until_key_and_space_stops(self):
        with MegaSimulator() as mega, tempfile.TemporaryDirectory() as folder:
            args = parse_args(['--sensors', 'motors', '--motor-test', 'drive', '--raw-motors', '--motor-pulse', '1',
                               '--motor-port', mega.port, '--report-dir', folder])
            monitor = SensorMonitor(args)
            dashboard = Dashboard(RobotModel())
            try:
                monitor.start()
                deadline = time.monotonic() + 5
                while time.monotonic() < deadline and monitor.snapshot()['channels']['motors']['status'] != 'LIVE':
                    time.sleep(0.03)
                self.assertEqual(monitor.snapshot()['channels']['motors']['status'], 'LIVE')
                self.assertEqual(len(monitor.threads), 1)
                self.assertFalse(any(c[0] == 'RUN' for c in mega.commands))
                dashboard.key('m', monitor)
                deadline = time.monotonic() + 5
                while time.monotonic() < deadline and not any(mega.pwm):
                    time.sleep(0.01)
                self.assertTrue(any(mega.pwm))
                dashboard.key(' ', monitor)
                deadline = time.monotonic() + 0.5
                while time.monotonic() < deadline and any(mega.pwm):
                    time.sleep(0.01)
                self.assertFalse(any(mega.pwm))
            finally:
                monitor.stop()
            self.assertFalse(any(t.is_alive() for t in monitor.threads))

    def test_telemetry_loss_aborts_and_sends_stop(self):
        args = parse_args(['--motors', '--motor-pulse', '0.8'])
        with MegaSimulator(silence_on_run=True) as mega, tempfile.TemporaryDirectory() as folder:
            with self.assertRaisesRegex(MotorError, '수신 중단'):
                with MotorClient(mega.port, folder, 2, boot_delay=0) as client:
                    drive_test(client, args)
            time.sleep(0.03)
            self.assertEqual(mega.commands[-1][0], 'STOP')
            self.assertFalse(any(mega.pwm))

    def test_cli_sigterm_stops_in_progress_motor(self):
        with MegaSimulator() as mega, tempfile.TemporaryDirectory() as folder:
            process = subprocess.Popen([sys.executable, str(ROOT / 'tools/check_sensors.py'),
                                        '--sensors', 'motors', '--motor-test', 'drive', '--raw-motors',
                                        '--motor-port', mega.port, '--report-dir', folder],
                                       stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
            try:
                deadline = time.monotonic() + 5
                while time.monotonic() < deadline and not any(mega.pwm):
                    time.sleep(0.01)
                self.assertTrue(any(mega.pwm))
                process.terminate()
                stdout, stderr = process.communicate(timeout=2)
                self.assertEqual(process.returncode, 130, stdout + stderr)
                self.assertFalse(any(mega.pwm))
                self.assertEqual(mega.commands[-1][0], 'STOP')
            finally:
                if process.poll() is None:
                    process.kill()
                    process.communicate()


class FirmwareLogicTests(unittest.TestCase):
    def test_real_firmware_core_quadrature_and_stop_deadlines(self):
        with tempfile.TemporaryDirectory() as folder:
            binary = str(Path(folder) / 'motor_core_test')
            subprocess.run(['g++', '-std=c++11', '-Wall', '-Wextra', '-Werror',
                            '-fsanitize=undefined', str(ROOT / 'tests/motor_core_test.cpp'), '-o', binary], check=True)
            subprocess.run([binary], check=True)


if __name__ == '__main__':
    unittest.main()
