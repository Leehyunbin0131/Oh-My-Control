#!/usr/bin/env python3
"""SSH terminal keyboard control using the calibrated Mega motor interface."""

import argparse
from collections import deque
from contextlib import contextmanager
from datetime import datetime
import json
import math
import os
from pathlib import Path
import select
import signal
import sys
import termios
import time
import tty

from drive_calibration import DEFAULT_CALIBRATION, DriveCalibration
from motor_test import DEFAULT_MOTOR_PORT, MotorClient, MotorError
from motor_pid import DEFAULT_PID_CONFIG, PIDSettings

WORKSPACE = Path(__file__).resolve().parents[1]
IN_PLACE_RPM_LIMIT = 50.0
DIRECTIONS = {
    'q': (0.5, 1), 'w': (1, 1), 'e': (1, 0.5),
    'a': (-1, 1), 'd': (1, -1),
    'z': (-0.5, -1), 'x': (-1, -1), 'c': (-1, -0.5),
}
NAMES = {
    'q': 'FORWARD LEFT', 'w': 'FORWARD', 'e': 'FORWARD RIGHT',
    'a': 'TURN LEFT', 'd': 'TURN RIGHT',
    'z': 'BACKWARD LEFT', 'x': 'BACKWARD', 'c': 'BACKWARD RIGHT',
}
# DFRobot KIT0085 / 28PA51G: 146 RPM unloaded at 12 V, 663 pulses
# per output revolution; our decoder counts all four quadrature edges.
NO_LOAD_RPM = 146.0
ENCODER_CPR = 663 * 4


class Keys:
    """Parse split SSH arrow sequences; never interpret their suffix as WASD."""
    def __init__(self):
        self.buffer = bytearray()
        self.escape_since = None
        self.escape_expired = False

    def feed(self, data, now):
        self.buffer.extend(data)
        result = []
        while self.buffer:
            if self.buffer[0] != 27:
                key = chr(self.buffer.pop(0)).lower()
                self.escape_since = None
                if key in DIRECTIONS or key in ('+', '-', '='):
                    result.append(key)
                elif key in ('\x03', '\x04'):
                    result.append('quit')
                else:
                    result.append('stop')
                continue
            self.escape_since = now if self.escape_since is None else self.escape_since
            if now - self.escape_since >= 0.1 and not self.escape_expired:
                self.escape_expired = True
                result.append('stop')
            if len(self.buffer) >= 2 and self.buffer[1] not in (ord('['), ord('O')):
                del self.buffer[:2]
                result.append('stop')
            elif len(self.buffer) >= 3:
                final = next((i for i in range(2, len(self.buffer)) if 64 <= self.buffer[i] <= 126), None)
                if final is not None:
                    sequence = bytes(self.buffer[:final + 1])
                    del self.buffer[:final + 1]
                    result.append('stop' if self.escape_expired else {
                        b'\x1b[A': 'w', b'\x1b[B': 'x', b'\x1b[C': 'd', b'\x1b[D': 'a',
                        b'\x1bOA': 'w', b'\x1bOB': 'x', b'\x1bOC': 'd', b'\x1bOD': 'a',
                    }.get(sequence, 'stop'))
                elif len(self.buffer) > 32:
                    # Keep the prefix so a delayed final A/D cannot become WASD.
                    del self.buffer[2:]
                    if not self.escape_expired:
                        result.append('stop')
                    self.escape_expired = True
                    break
                else:
                    break
            else:
                break
            self.escape_since = None
            self.escape_expired = False
        return result


class Controller:
    """Latched direction (or optional key timeout) with the MCU heartbeat."""
    def __init__(self, client, pwm=180, key_timeout=0, event=lambda value: None, segment_seconds=30,
                 *, pid_settings=None, rpm=72):
        if client.calibration is None or not client.info['link']:
            raise MotorError('방향 보정 설정과 모터 통신을 지원하는 메가 펌웨어가 필요합니다.')
        self.max_pwm = min(650, client.info.get('max_drive_pwm', 250))
        if pwm > self.max_pwm:
            raise MotorError('이 출력에는 PWM 상한 650을 지원하는 메가 v8 펌웨어가 필요합니다. ./mega_firmware.sh upload로 갱신하세요.')
        if (type(pwm) is not int or not 110 <= pwm <= self.max_pwm or
                not (key_timeout == 0 or 0.2 <= key_timeout <= 1.0) or not 0.1 <= segment_seconds <= 30):
            raise ValueError('허용 범위를 벗어난 키보드 조종 설정입니다.')
        self.client = client
        self.pwm = pwm
        self.pid_settings = pid_settings
        self.rpm = rpm
        if pid_settings and (not client.info.get('stream_supported') or not pid_settings.min_rpm <= rpm <= pid_settings.max_rpm):
            raise MotorError('PID 펌웨어 또는 목표 RPM 범위를 확인하세요.')
        self.key_timeout = key_timeout
        self.event = event
        self.segment_seconds = segment_seconds
        self.direction = None
        self.last_key = None
        self.active = (0, 0)
        self.output = [0, 0]
        self.started = self.last_ping = 0
        self.ready_after = 0
        self.engaged = False
        self.last_target_sent = 0
        self.force_stop = self.quit = False
        self.status = 'READY / STOPPED'
        self.history = deque(maxlen=200)
        self.baseline = None
        self.previous_stamp = None

    def keys(self, actions, now):
        # An emergency stop/quit wins over all direction bytes in the same read.
        if 'quit' in actions:
            self.quit = self.force_stop = True
            self.direction = None
        elif 'stop' in actions:
            self.force_stop = True
            self.direction = None
        else:
            for key in actions:
                if key in DIRECTIONS:
                    self.direction = key
                    self.last_key = now
                elif key in ('+', '='):
                    if self.pid_settings:
                        self.rpm = min(self.pid_settings.max_rpm, self.rpm + 2)
                    else:
                        self.pwm = min(self.max_pwm, self.pwm + 10)
                elif key == '-':
                    if self.pid_settings:
                        self.rpm = max(self.pid_settings.min_rpm, self.rpm - 2)
                    else:
                        self.pwm = max(110, self.pwm - 10)

    def stop(self, reason):
        if self.pid_settings and self.engaged:
            self.client.set_speed(0, 0)
            self.last_target_sent = time.monotonic()
            operation = 'SET_SPEED 0 0'
        else:
            self.client.command('STOP')
            operation = 'STOP'
        self.active = (0, 0)
        self.output = [0, 0]
        self.started = time.monotonic()
        self.history.clear()
        self.status = reason
        self.event({'event': 'stop', 'reason': reason, 'operation': operation})

    def _feedback(self, now):
        sample = self.client.latest
        if sample is None or now - sample['received_at'] > 0.4:
            raise MotorError('엔코더 수신 중단: 조종을 종료합니다.')
        if sample['fault']:
            raise MotorError(sample['fault_reason'])
        if not any(self.active) and not (self.pid_settings and self.engaged):
            return
        elapsed = now - self.started
        if not self.pid_settings and sample['command_pwm'] == [0, 0] and not sample['armed'] and elapsed >= self.segment_seconds - 0.1:
            # The firmware hard deadline is retained. A latched direction may
            # continue into another segment; timeout mode requires fresh input.
            self.active = (0, 0)
            self.output = [0, 0]
            self.history.clear()
            self.event({'event': 'segment_expired'})
            return
        matches = (sample.get('pid_mode') and sample.get('target_rpm_centi') == self.output
                   if self.pid_settings else sample['command_pwm'] == self.output)
        if elapsed > 0.2 and (not matches or not sample['armed']):
            raise MotorError('메가 출력 상태가 조종 명령과 다릅니다.')
        if sample['stamp'] == self.previous_stamp:
            return
        self.previous_stamp = sample['stamp']
        if any(n != p for n, p in zip(sample['invalid'], self.baseline['invalid'])):
            raise MotorError('엔코더 A/B 신호 오류: 조종을 종료합니다.')
        self.history.append(sample)
        while len(self.history) > 1 and now - self.history[1]['received_at'] >= 0.8:
            self.history.popleft()
        if elapsed >= 1 and self.history and now - self.history[0]['received_at'] >= 0.7:
            deltas = self.client.calibration.wheel_deltas(self.history[0], sample)
            for name, target in zip(('left', 'right'), self.active):
                if target and deltas[name] * (1 if target > 0 else -1) < 4:
                    raise MotorError(f'{name} 엔코더 회전 부족/방향 불일치: 조종을 종료합니다.')

    def _step_pid(self, now):
        rpm = min(self.rpm, IN_PLACE_RPM_LIMIT) if self.direction in ('a', 'd') else self.rpm
        desired = tuple(round(factor * rpm * 100) for factor in DIRECTIONS[self.direction]) if self.direction else (0, 0)
        changed = desired != self.active
        if not self.engaged and not any(desired):
            return
        if not self.engaged:
            self.client.command('ARM')
            self.engaged = True
        if changed:
            self.active = desired
            self.output = self.pid_settings.commands(*desired)
            self.baseline = self.client.latest
            self.started = now
            self.history.clear()
            self.previous_stamp = None
            self.status = NAMES[self.direction] if self.direction else 'ZERO SPEED / HOLD IF ENABLED'
            self.event({'event': 'target', 'physical_rpm': [n / 100 for n in desired]})
        if changed or now - self.last_target_sent >= .02:
            self.client.set_speed(*self.output)
            self.last_target_sent = now

    def step(self, now):
        if self.force_stop:
            self.stop('QUIT' if self.quit else 'STOP KEY')
            self.force_stop = False
            return
        self._feedback(now)
        if self.last_key is None or (self.key_timeout and now - self.last_key >= self.key_timeout):
            self.direction = None
        if self.pid_settings:
            self._step_pid(now)
            return
        else:
            # Raw PWM mode keeps the inner wheel above the driver's deadband.
            desired = tuple((1 if factor > 0 else -1) * max(110, int(abs(factor) * self.pwm + 0.5))
                            for factor in DIRECTIONS[self.direction]) if self.direction else (0, 0)
        if not any(desired):
            if any(self.active):
                self.stop('INPUT TIMEOUT / STOPPED')
            return
        if desired != self.active:
            if any(self.active):
                self.stop('CHANGING DIRECTION / STOPPED')
                self.ready_after = time.monotonic() + 0.2
                return
            if now < self.ready_after:
                return
            self.output = (self.pid_settings.commands(*desired) if self.pid_settings else
                           self.client.calibration.commands(*desired, limit=self.max_pwm))
            self.baseline = self.client.latest
            self.history.clear()
            self.previous_stamp = None
            self.client.command('ARM')
            self.started = time.monotonic()
            self.client.command('SPEED' if self.pid_settings else 'DRIVE' if self.pwm > 250 else 'BENCH',
                                *self.output, round(self.segment_seconds * 1000))
            self.active = desired
            self.last_ping = time.monotonic()
            self.status = NAMES[self.direction]
            self.event({'event': 'drive', 'control': 'pid' if self.pid_settings else 'pwm',
                        'physical_rpm' if self.pid_settings else 'physical_pwm':
                        [v / 100 for v in desired] if self.pid_settings else list(desired),
                        'raw_rpm_centi' if self.pid_settings else 'raw_pwm': self.output[:],
                        'direction': self.direction})
        if any(self.active) and time.monotonic() - self.last_ping >= 0.08:
            self.client.send('PING')
            self.last_ping = time.monotonic()


@contextmanager
def terminal():
    saved = termios.tcgetattr(sys.stdin.fileno())
    try:
        tty.setcbreak(sys.stdin.fileno())
        sys.stdout.write('\x1b[?1049h\x1b[?25l\x1b[2J')
        sys.stdout.flush()
        yield
    finally:
        termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, saved)
        sys.stdout.write('\x1b[0m\x1b[?25h\x1b[?1049l')
        sys.stdout.flush()


def render(controller):
    wheels = controller.client.latest.get('wheels', {})
    mode = ('LATCHED: tap once to move; S / Space to STOP' if controller.key_timeout == 0 else
            f'HOLD: repeat direction keys; input timeout {controller.key_timeout:.1f}s')
    lines = [
        'ROBOT KEYBOARD CONTROL / camera direction = forward',
        'Q: fwd left       W: forward       E: fwd right',
        'A: turn left      S: STOP          D: turn right',
        'Z: back left      X: backward      C: back right',
        'Space: STOP    Ctrl+C: stop and quit    +/-: ' + ('RPM' if controller.pid_settings else 'PWM'),
        mode,
        (f'PID target {controller.rpm:.1f} RPM ({controller.pid_settings.min_rpm}..{controller.pid_settings.max_rpm}) | MCU loop {controller.pid_settings.period_ms} ms ({1000 / controller.pid_settings.period_ms:g} Hz) | PWM limit 650/999' if controller.pid_settings else
         f'PWM {controller.pwm}/999 (110..{controller.max_pwm})'),
        ('Encoder feedback controls each wheel independently.' if controller.pid_settings else
         f'No-load estimate: {controller.pwm * NO_LOAD_RPM / 999:.1f} RPM (12 V); measured RPM below'),
        ('Arrows: forward/backward/turn. A/D in-place limit: '
         f'{IN_PLACE_RPM_LIMIT:g} RPM (or lower selected speed).' if controller.pid_settings else
         'Arrows: forward/backward/turn. A/D turns are in place.'),
        (f'State: {controller.status} | target L={controller.active[0]/100:+.1f} R={controller.active[1]/100:+.1f} RPM'
         if controller.pid_settings else
         f'State: {controller.status} | command L={controller.active[0]:+d} R={controller.active[1]:+d}'),
    ]
    if controller.pid_settings:
        if controller.pid_settings.hold['enabled']:
            verified = controller.pid_settings.document.get('validation_status', {}).get('holding') is True
            manual_confirmed = controller.pid_settings.document.get('hold_validation', {}).get('status') == 'user_confirmed_manual_hold'
            hold_status = '' if verified else (' / manual hold confirmed' if manual_confirmed else ' / user test pending')
            lines.append('Position HOLD enabled' + hold_status +
                         ' | S keeps zero target active; Ctrl+C releases output.')
        timing = getattr(controller.client, 'timing', None)
        if timing and timing.get('measured_control_hz') is not None:
            lines.append(f'Loop measured {timing["measured_control_hz"]:.1f} Hz | max compute {timing["compute_max_us"]} us | missed {timing["missed_ticks"]}')
        speeds = controller.pid_settings.wheel_speeds(controller.client.latest)
        for name in ('left', 'right'):
            wheel = speeds[name]
            state = getattr(controller.client, 'control', None)
            index = 0 if controller.client.calibration.wheels[name]['driver_channel'] == 'L' else 1
            label = state['wheel_states'][index] if state else 'WAIT'
            lines.append(f'{name.upper():5} {label:10} target {wheel["target_rpm"]:+6.1f} | '
                         f'actual {wheel["rpm"]:+6.1f} RPM | PWM {wheel["pwm"]:+4d}' +
                         (f' | hold error {abs(state["position_error"][index])} counts' if label == 'HOLD' else ''))
        return '\x1b[H' + '\n'.join('\x1b[2K' + line for line in lines) + '\x1b[J'
    lines.append('Diagnostic PWM: STOP disables output; measured overspeed guard at 100 RPM')
    for name in ('left', 'right'):
        wheel = wheels.get(name, {})
        lines.append(f'{name.upper():5} encoder {wheel.get("count", 0):+9d}  '
                     f'{wheel.get("ticks_per_second", 0):+8.0f} ticks/s  '
                     f'{wheel.get("ticks_per_second", 0) * 60 / ENCODER_CPR:+7.1f} RPM')
    return '\x1b[H' + '\n'.join('\x1b[2K' + line for line in lines) + '\x1b[J'


def options(argv=None):
    parser = argparse.ArgumentParser(description='SSH 키보드 조종: QWE/ASD/ZXC, 한 번 입력 후 유지, S 정지')
    parser.add_argument('--port', default=DEFAULT_MOTOR_PORT)
    speed = parser.add_mutually_exclusive_group()
    speed.add_argument('--rpm', type=float, help='엔코더 PID 목표 속도 (기본 72 RPM, 보정 범위 내)')
    speed.add_argument('--pwm', type=int, help='진단용 PWM 110..650; 실제 RPM 명령은 --rpm 사용')
    speed.add_argument('--rpm-approx', type=float, help='PID 없이 12V 무부하 사양으로 PWM 환산: 17..95 RPM')
    parser.add_argument('--clear-fault', action='store_true', help='원인 해소 후 메가 오류를 명시적으로 해제')
    parser.add_argument('--pid-config', type=Path, default=DEFAULT_PID_CONFIG)
    parser.add_argument('--key-timeout', type=float, default=0,
                        help='0: 한 번 입력 후 유지(기본). 0.2..1.0: 입력 중단 후 자동 정지 시간(초)')
    parser.add_argument('--calibration', type=Path, default=DEFAULT_CALIBRATION)
    parser.add_argument('--report-dir', type=Path, default=WORKSPACE / 'reports')
    args = parser.parse_args(argv)
    args.pid = args.pwm is None and args.rpm_approx is None
    if args.rpm_approx is not None:
        if not math.isfinite(args.rpm_approx) or not 17 <= args.rpm_approx <= 95:
            parser.error('--rpm-approx는 17..95 범위입니다.')
        args.pwm = round(999 * args.rpm_approx / NO_LOAD_RPM)
    if args.pwm is not None and not 110 <= args.pwm <= 650:
        parser.error('--pwm은 110..650 범위입니다.')
    if args.pid:
        args.rpm = 72 if args.rpm is None else args.rpm
        if not math.isfinite(args.rpm) or not 30 <= args.rpm <= 100:
            parser.error('--rpm은 30..100 RPM 범위이며 보정 파일의 최대값도 적용됩니다.')
    if not math.isfinite(args.key_timeout) or not (args.key_timeout == 0 or 0.2 <= args.key_timeout <= 1.0):
        parser.error('--key-timeout은 0(입력 유지) 또는 0.2..1.0초 범위입니다.')
    return args


def main(argv=None):
    args = options(argv)
    if not sys.stdin.isatty() or not sys.stdout.isatty():
        print('대화형 SSH 터미널이 필요합니다. ssh -t로 접속하세요.', file=sys.stderr)
        return 2
    try:
        calibration = DriveCalibration.load(args.calibration)
        pid_settings = PIDSettings.load(args.pid_config, calibration) if args.pid else None
        if pid_settings and args.rpm > pid_settings.max_rpm:
            raise ValueError(f'실측 보정한 목표 속도 범위는 {pid_settings.min_rpm}..{pid_settings.max_rpm} RPM입니다.')
    except (OSError, ValueError) as exc:
        print(f'모터 설정 오류: {exc}', file=sys.stderr)
        return 2
    report = args.report_dir.expanduser().resolve() / ('teleop_' + datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    report.mkdir(parents=True)
    outcome = {'started_at': datetime.now().astimezone().isoformat(), 'calibration': calibration.document,
               'key_timeout': args.key_timeout, 'initial_pwm': args.pwm,
               'control_mode': 'latched' if args.key_timeout == 0 else 'hold',
               'speed_control': 'pid' if args.pid else 'pwm', 'initial_rpm': args.rpm,
               'pid_config': pid_settings.document if pid_settings else None,
               'no_load_estimated_rpm': None if args.pid else args.pwm * NO_LOAD_RPM / 999,
               'encoder_counts_per_revolution': ENCODER_CPR, 'error': None}
    client = None
    code = 0

    def interrupt(*_):
        raise KeyboardInterrupt

    for signum in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP, signal.SIGTSTP):
        signal.signal(signum, interrupt)

    try:
        with (report / 'events.jsonl').open('w') as events, terminal():
            def event(value):
                events.write(json.dumps({'time': time.monotonic(), **value}, ensure_ascii=False) + '\n')
                events.flush()

            print('메가 연결 중… 잠시 기다리세요.', flush=True)
            with MotorClient(args.port, report, calibration=calibration, brake_on_exit=True) as client:
                if args.clear_fault:
                    client.clear_fault()
                if pid_settings:
                    pid_settings.configure(client)
                controller = Controller(client, args.pwm if args.pwm is not None else 180,
                                        args.key_timeout, event, pid_settings=pid_settings, rpm=args.rpm or 72)
                keys = Keys()
                # Discard keys typed during the USB reset/connection phase.
                termios.tcflush(sys.stdin.fileno(), termios.TCIFLUSH)
                next_render = 0
                while not controller.quit:
                    data = b''
                    if select.select([sys.stdin], [], [], 0.02)[0]:
                        data = os.read(sys.stdin.fileno(), 256)
                        if not data:
                            controller.keys(['quit'], time.monotonic())
                    controller.keys(keys.feed(data, time.monotonic()), time.monotonic())
                    # Process stop keys before doing any further blocking reads.
                    if controller.force_stop:
                        controller.step(time.monotonic())
                    if controller.quit:
                        break
                    # A direction key can use fresh cached telemetry; do not
                    # delay it behind a serial read waiting for the next sample.
                    if data:
                        controller.step(time.monotonic())
                    client.poll(block=False)
                    controller.step(time.monotonic())
                    if time.monotonic() >= next_render:
                        sys.stdout.write(render(controller))
                        sys.stdout.flush()
                        next_render = time.monotonic() + 0.1
    except KeyboardInterrupt:
        code = 130
        outcome['error'] = '사용자/터미널 신호로 중단'
    except Exception as exc:
        code = 1
        outcome['error'] = f'{type(exc).__name__}: {exc}'
    finally:
        outcome.update(finished_at=datetime.now().astimezone().isoformat(), exit_code=code,
                       last_sample=client.latest if client else None,
                       braking=list(client.brake_reports) if client else [])
        (report / 'summary.json').write_text(json.dumps(outcome, ensure_ascii=False, indent=2) + '\n')
    if outcome['error']:
        print(outcome['error'], file=sys.stderr)
    print(f'키보드 조종 종료. 로그: {report}')
    return code


if __name__ == '__main__':
    sys.exit(main())
