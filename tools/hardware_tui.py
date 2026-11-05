#!/usr/bin/env python3
"""SSH-friendly full-screen hardware dashboard. No display server is required."""

import argparse
from contextlib import contextmanager
import os
import select
import shutil
import signal
import sys
import termios
import time
import tty

import numpy as np

from check_sensors import parse_args as sensor_args
from sensor_monitor import SensorMonitor, CHANNELS
from terminal_view import (Screen, RobotModel, rotation, draw_robot, draw_image, draw_lidar,
                           BG, BLUE, TEXT, MUTED, GREEN, CYAN, RED, YELLOW)


def status_color(status):
    return GREEN if status == 'LIVE' else RED if status in ('ERROR', 'STALE', 'NO DATA') else YELLOW


def triple(values, digits=2):
    return ' / '.join(f'{value:+.{digits}f}' for value in values) if values is not None else '-- / -- / --'


class Dashboard:
    def __init__(self, model):
        self.model = model
        self.source = 'hfi_imu'
        self.image_kind = 'color'
        self.range = 4.0
        self.zero = {}
        self.message = 'Move/tilt a sensor to light up its R/P/Y motion indicators.'

    def render(self, monitor, width, height, color):
        screen = Screen(width, height, color)
        state = monitor.snapshot()
        channels = state['channels']
        live = sum(channel['status'] == 'LIVE' for channel in channels.values())
        active = sum(channel['status'] != 'DISABLED' for channel in channels.values())
        screen.text(1, 0, 'OH MY MOBILEROBOT / HARDWARE TEST', CYAN)
        screen.text(max(38, width - 30), 0, f'SSH  {live}/{active} LIVE  {state["elapsed"]:6.1f}s', GREEN if live == active else YELLOW)
        screen.text(1, 1, 'LIVE = receiving data | Motor PASS requires a completed drive test', MUTED, width - 2)
        card_width = width // len(CHANNELS)
        for i, (key, label) in enumerate(CHANNELS.items()):
            channel = channels[key]
            x = i * card_width + 1
            screen.text(x, 3, label, TEXT, card_width - 1)
            screen.text(x, 4, f'{channel["status"]:7} {channel["hz"]:5.1f}Hz', status_color(channel['status']), card_width - 1)
        panel_y = 6
        panel_height = max(7, height - 27)
        panel_height = min(panel_height, 25)
        widths = [width * 30 // 100, width * 36 // 100]
        widths.append(width - sum(widths))
        x1, x2, x3 = 0, widths[0], sum(widths[:2])
        screen.panel(x1, panel_y, widths[0], panel_height, 'ROBOT / ' + ('HFI-A9' if self.source == 'hfi_imu' else 'CAM IMU'))
        screen.panel(x2, panel_y, widths[1], panel_height, 'RGB CAMERA' if self.image_kind == 'color' else 'DEPTH / 0-4m')
        screen.panel(x3, panel_y, widths[2], panel_height, f'LIDAR XY / {self.range:g}m')
        selected = channels[self.source]
        rpy = selected['sample'].get('rpy_degrees')
        attitude = np.eye(3)
        if rpy is not None:
            current = rotation(rpy)
            self.zero.setdefault(self.source, current)
            attitude = self.zero[self.source].T @ current
        draw_robot(screen, (x1 + 1, panel_y + 1, widths[0] - 2, panel_height - 3), self.model, attitude)
        note = 'Relative pose / +X arrow' if selected['status'] == 'LIVE' else 'POSE NOT LIVE'
        screen.text(x1 + 2, panel_y + panel_height - 2, note, MUTED if selected['status'] == 'LIVE' else RED, widths[0] - 4)
        frame = monitor.image(self.image_kind)
        pixels = frame[1] if frame is not None and time.monotonic() - frame[0] < 2 and channels[self.image_kind]['status'] == 'LIVE' else None
        draw_image(screen, (x2 + 1, panel_y + 1, widths[1] - 2, panel_height - 3), pixels)
        image = channels[self.image_kind]['sample']
        info = f"{image.get('width', '--')}x{image.get('height', '--')}  {image.get('encoding', '')}"
        screen.text(x2 + 2, panel_y + panel_height - 2, info, MUTED, widths[1] - 4)
        lidar = channels['lidar']
        scan = lidar['sample'] if lidar['status'] == 'LIVE' else {}
        draw_lidar(screen, (x3 + 1, panel_y + 1, widths[2] - 2, panel_height - 3), scan, self.range)
        screen.text(x3 + 2, panel_y + panel_height - 2,
                    f"near {scan.get('min_m', '--')}m  rays {scan.get('valid_rays', 0)}", MUTED, widths[2] - 4)
        y = panel_y + panel_height + 1
        compact = height < 35
        for x, key, title in [(1, 'hfi_imu', 'HFI-A9 / fused orientation'),
                              (width // 2 + 1, 'camera_imu', 'CAMERA IMU / tilt + relative yaw')]:
            channel = channels[key]
            sample = channel['sample']
            span = width // 2 - 2
            screen.text(x, y, title, CYAN, span)
            screen.text(x, y + 1, 'RPY deg  ' + triple(sample.get('rpy_degrees'), 1), TEXT, span)
            if compact:
                continue
            camera = key == 'camera_imu'
            screen.text(x, y + 2, ('a m/s2   ' if camera else 'a g      ') + triple(sample.get('accel_m_s2' if camera else 'accel_g')), MUTED, span)
            screen.text(x, y + 3, ('w rad/s  ' if camera else 'gyro raw ') + triple(sample.get('gyro_rad_s' if camera else 'gyro_raw'), 3), MUTED, span)
            screen.text(x, y + 4, 'Motion ', TEXT)
            for i, label in enumerate('RPY'):
                active = channel['axes'][i]
                screen.text(x + 8 + i * 7, y + 4, f'[{label}:{"*" if active else "-"}]', GREEN if active else MUTED)
            screen.text(x, y + 5, f"samples {channel['count']}  errors {channel['errors']}", MUTED, span)
        motor_y = y + (3 if compact else 7)
        motor = channels['motors']
        sample = motor['sample']
        motor_state = state.get('motor', {})
        enabled = motor['status'] != 'DISABLED'
        title = 'MOTOR / ENCODER  [m] test  [Space/x] STOP' if enabled else 'MOTOR / ENCODER  (enable with --motors)'
        screen.text(1, motor_y, title, CYAN, width - 2)
        for i, side in enumerate(('left', 'right')):
            wheel = sample.get('wheels', {}).get(side)
            if wheel:
                pins = wheel['encoder_pins']
                label = f'{side.upper()} {pins[0]}/{pins[1]}'
                counts, rate, bits, errors = wheel['count'], wheel['ticks_per_second'], wheel['ab'], wheel['invalid']
            else:
                label = ('E1 40/41', 'E2 50/51')[i]
                counts = sample.get('counts', [0, 0])[i]
                rate = sample.get('ticks_per_second', [0, 0])[i]
                bits = sample.get('ab', [0, 0])[i]
                errors = sample.get('invalid', [0, 0])[i]
            receiving = motor['status'] == 'LIVE'
            spinner = '|/-\\'[(abs(counts) // 4) % 4] if receiving else '?'
            text = (f'{label} [{spinner}] ticks {counts:+8d}  {rate:+7.0f}/s  '
                    f'A:{(bits >> 1) & 1} B:{bits & 1}  bad:{errors}') if sample else f'{label}  -- waiting for encoder data --'
            control = sample.get('control')
            if control and wheel:
                index = 0 if wheel['driver_channel'] == 'L' else 1
                text += '  ' + control['wheel_states'][index]
            screen.text(1, motor_y + 1 + i, text, GREEN if receiving and abs(rate) > 1 else MUTED, width - 2)
        result = motor_state.get('result')
        phase = motor_state.get('phase', 'OBSERVE')
        motor_note = f'{result["status"]}: {result["reason"]}' if result else phase + ' | Motor function has not been verified.'
        if motor['problem']:
            motor_note = motor['problem']
        screen.text(1, motor_y + 3, motor_note, RED if motor['problem'] or (result and result['status'] != 'PASS') else YELLOW, width - 2)
        failures = [(CHANNELS[key], channel) for key, channel in channels.items()
                    if channel['status'] in ('ERROR', 'STALE', 'NO DATA')]
        for offset, (name, channel) in enumerate(failures[:max(0, height - motor_y - 8)]):
            screen.text(1, motor_y + 5 + offset, f"! {name}: {channel['problem'] or 'Sensor data stopped'}", RED, width - 2)
        screen.text(1, height - 3, self.message, YELLOW, width - 2)
        screen.text(1, height - 2, '1 HFI  2 Cam IMU  z Zero view  c RGB/Depth  +/- Range  s Save  r Retry  q Quit', CYAN, width - 2)
        screen.text(1, height - 1, 'LOG ' + str(monitor.run_dir), MUTED, width - 2)
        return screen

    def key(self, key, monitor):
        if key in ('q', 'Q', '\x04'):
            if hasattr(monitor, 'stop_motors'):
                monitor.stop_motors()
            return False
        if key in ('1', '2'):
            self.source = 'hfi_imu' if key == '1' else 'camera_imu'
            self.message = 'Camera heading is gyro-integrated and may drift.' if key == '2' else 'HFI-A9 fused attitude, displayed relative to the view reference.'
        elif key == 'c':
            self.image_kind = 'depth' if self.image_kind == 'color' else 'color'
        elif key == 'z':
            selected = monitor.snapshot()['channels'][self.source]
            if selected['status'] == 'LIVE' and 'rpy_degrees' in selected['sample']:
                self.zero[self.source] = rotation(selected['sample']['rpy_degrees'])
                self.message = 'View reference reset. Sensor calibration is unchanged.'
            else:
                self.message = 'A live IMU sample is required to reset the view.'
        elif key in ('+', '='):
            self.range = min(32, self.range * 2)
        elif key == '-':
            self.range = max(0.5, self.range / 2)
        elif key == 's':
            path = monitor.save(images=True)
            self.message = 'Saved status + available camera previews: ' + path.name
        elif key in (' ', 'x', 'X'):
            monitor.stop_motors()
            self.message = 'Motor STOP requested.'
        elif key == 'm':
            self.message = monitor.request_motor_test()
        elif key == 'r':
            monitor.stop()
            monitor.start()
            self.zero.clear()
            self.message = 'Sensor drivers restarted. Waiting for fresh data.'
        return True


@contextmanager
def terminal():
    old = termios.tcgetattr(sys.stdin.fileno())
    try:
        tty.setcbreak(sys.stdin.fileno())
        sys.stdout.write('\x1b[?1049h\x1b[?25l\x1b[2J')
        sys.stdout.flush()
        yield
    finally:
        termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, old)
        sys.stdout.write('\x1b[0m\x1b[?25h\x1b[?1049l')
        sys.stdout.flush()


def options(argv=None):
    parser = argparse.ArgumentParser(description='SSH 터미널용 센서·모터·엔코더 대시보드',
                                     epilog='모터 추가: --motors. 구동 허용: --motors --motor-test drive (화면에서 m 시작). '
                                            '모터만: --sensors motors. 공통 인자: ./check_sensors.sh --help')
    parser.add_argument('--color', choices=('auto', 'truecolor', '256', 'none'), default='auto')
    parser.add_argument('--once', action='store_true', help='센서를 읽고 화면 한 장을 출력한 뒤 종료')
    parser.add_argument('--seconds', type=float, default=25, help='--once 수집 시간 (기본: 25)')
    parser.add_argument('--width', type=int, default=120, help='--once 화면 너비')
    parser.add_argument('--height', type=int, default=42, help='--once 화면 높이')
    args, remaining = parser.parse_known_args(argv)
    hardware = sensor_args(remaining)
    if args.once and hardware.motor_test in ('drive', 'bench'):
        parser.error('--once 구동은 지원하지 않습니다. ./check_sensors.sh --sensors motors --motor-test drive를 사용하세요.')
    if args.seconds < 2 or not np.isfinite(args.seconds):
        parser.error('--seconds must be finite and >= 2')
    if args.width < 70 or args.height < 24:
        parser.error('--width >= 70, --height >= 24 required')
    if args.color == 'auto':
        args.color = 'truecolor' if os.environ.get('COLORTERM') in ('truecolor', '24bit') else '256'
    return args, hardware


def main(argv=None):
    args, hardware = options(argv)
    if not args.once and not (sys.stdin.isatty() and sys.stdout.isatty()):
        print('대화형 터미널이 필요합니다. SSH는 ssh -t 로 연결하거나 --once 를 사용하세요.', file=sys.stderr)
        return 2
    os.environ['ROS_DOMAIN_ID'] = str(hardware.domain_id)
    os.environ.pop('ROS_LOCALHOST_ONLY', None)
    os.environ.pop('ROS_STATIC_PEERS', None)
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'

    def interrupt(_signum, _frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, interrupt)
    signal.signal(signal.SIGHUP, interrupt)
    dashboard = Dashboard(RobotModel())
    if hardware.motor_test in ('drive', 'bench'):
        dashboard.message = 'Raise both wheels off the floor. m: start bounded motor test. Space/x: STOP.'
    monitor = SensorMonitor(hardware)
    saved = None
    result = 0
    try:
        monitor.start()
        if args.once:
            deadline = time.monotonic() + args.seconds
            while time.monotonic() < deadline:
                time.sleep(0.1)
            screen = dashboard.render(monitor, args.width, args.height, args.color)
            print(screen.render(ansi=args.color != 'none'))
            (monitor.run_dir / 'screen.txt').write_text(screen.render(ansi=False) + '\n')
            state = monitor.snapshot()
            result = 0 if all(v['status'] in ('LIVE', 'DISABLED') for v in state['channels'].values()) else 1
            if hardware.motor_test == 'encoders':
                result = 0 if result == 0 and (state['motor'].get('result') or {}).get('status') == 'PASS' else 1
        else:
            with terminal():
                while True:
                    size = shutil.get_terminal_size((120, 42))
                    if size.columns < 70 or size.lines < 24:
                        sys.stdout.write('\x1b[H\x1b[2JResize terminal to at least 70 columns x 24 rows. q: quit')
                    else:
                        screen = dashboard.render(monitor, min(220, size.columns - 1), min(70, size.lines), args.color)
                        sys.stdout.write('\x1b[H' + screen.render())
                    sys.stdout.flush()
                    if select.select([sys.stdin], [], [], 0.2)[0]:
                        keys = os.read(sys.stdin.fileno(), 32).decode('utf-8', errors='ignore')
                        if not keys or any(not dashboard.key(key, monitor) for key in keys):
                            break
        saved = monitor.save()
    except KeyboardInterrupt:
        result = 130
        if monitor.run_dir:
            saved = monitor.save()
    finally:
        monitor.stop()
    print(f'하드웨어 검사 종료. 결과: {saved or monitor.run_dir}')
    return result


if __name__ == '__main__':
    sys.exit(main())
