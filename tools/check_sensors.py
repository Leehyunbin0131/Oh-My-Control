#!/usr/bin/env python3
"""Read HFI-A9, RPLIDAR A2M12 and RealSense RGB/depth/IMU for a bounded interval."""

import argparse
from contextlib import contextmanager
from datetime import datetime
import json
import math
import os
from pathlib import Path
import shutil
import signal
import stat
import subprocess
import sys
import time

from hfi_a9 import A9Decoder
from motor_test import DEFAULT_MOTOR_PORT, check_motors
from drive_calibration import DEFAULT_CALIBRATION


WORKSPACE = Path(__file__).resolve().parents[1]
DEFAULT_IMU = '/dev/serial/by-id/usb-Silicon_Labs_HandsFree_IMU_USB_to_UART_Bridge_Controller_0001-if00-port0'
DEFAULT_LIDAR = '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_3915548a17a0164cb877f2ab2ade5994-if00-port0'
RSUSB_LIBRARY = Path.home() / '.local/opt/librealsense-2.58.1-rsusb/lib'


class StreamStats:
    """Require repeated valid samples, progressing timestamps and recent data."""

    def __init__(self):
        self.total = self.valid = self.stamp_errors = 0
        self.first = self.last = self.last_valid = self.previous_stamp = None
        self.latest_valid = False
        self.sample = {}

    def add(self, valid, sample, stamp, now, wrap32=False):
        self.total += 1
        self.first = now if self.first is None else self.first
        self.last = now
        if self.previous_stamp is not None:
            delta = stamp - self.previous_stamp
            progressing = 0 < (delta & 0xFFFFFFFF) < 0x80000000 if wrap32 else delta > 0
            if not progressing:
                self.stamp_errors += 1
        self.previous_stamp = stamp
        self.latest_valid = valid
        self.sample = sample
        if valid:
            self.valid += 1
            self.last_valid = now

    def summary(self, now, duration):
        span = 0.0 if self.first is None else self.last - self.first
        age = None if self.last_valid is None else now - self.last_valid
        hz = (self.total - 1) / span if span > 0 else 0.0
        reasons = []
        if self.valid < 3:
            reasons.append('정상 샘플 3개 미만')
        if self.valid < self.total * 0.9 or not self.latest_valid:
            reasons.append('유효하지 않은 데이터')
        if span < duration * 0.5:
            reasons.append('연속 수신 시간 부족')
        if age is None or age > min(2.0, duration / 2):
            reasons.append('데이터 수신 중단/시간 초과')
        if self.stamp_errors:
            reasons.append('센서 타임스탬프 반복/역행')
        return {
            'status': 'FAIL' if reasons else 'PASS', 'reasons': reasons,
            'messages': self.total, 'valid_messages': self.valid,
            'hz': round(hz, 2), 'span_seconds': round(span, 3),
            'last_valid_age_seconds': None if age is None else round(age, 3),
            'timestamp_errors': self.stamp_errors, 'sample': self.sample,
        }


def check_serial_port(port):
    path = Path(port)
    if not path.exists():
        raise RuntimeError(f'장치를 찾을 수 없습니다: {port} (USB 연결/전원 확인)')
    if not stat.S_ISCHR(path.stat().st_mode):
        raise RuntimeError(f'시리얼 장치 파일이 아닙니다: {port}')
    if not os.access(path, os.R_OK | os.W_OK):
        raise RuntimeError(f'시리얼 접근 권한이 없습니다: {port} (dialout 그룹 확인)')
    if shutil.which('fuser'):
        result = subprocess.run(['fuser', str(path.resolve())], capture_output=True,
                                text=True, timeout=3)
        if result.returncode == 0:
            raise RuntimeError(f'다른 프로세스가 사용 중입니다: {port}, PID {result.stdout.strip()}')


def check_imu(args, report_dir):
    import serial

    check_serial_port(args.imu_port)
    decoder = A9Decoder()
    streams = {name: StreamStats() for name in ('motion', 'attitude')}
    byte_count = 0
    with serial.Serial(args.imu_port, args.imu_baud, timeout=0.05, exclusive=True) as imu:
        imu.reset_input_buffer()
        deadline = time.monotonic() + args.duration
        with (report_dir / 'imu.bin').open('wb') as raw:
            while time.monotonic() < deadline:
                data = imu.read(max(1, min(4096, imu.in_waiting)))
                raw.write(data)
                byte_count += len(data)
                for frame in decoder.feed(data):
                    values = frame['values']
                    valid = all(math.isfinite(value) for value in values)
                    sample = {}
                    if valid and frame['kind'] == 'motion':
                        sample = {'gyro_raw': list(values[:3]), 'accel_g': list(values[3:6]),
                                  'mag_raw': list(values[6:9])}
                        valid = math.sqrt(sum(value * value for value in values[3:6])) > 1e-6
                    elif valid:
                        sample = {'rpy_degrees_raw': list(values)}
                    streams[frame['kind']].add(valid, sample, frame['stamp'],
                                               time.monotonic(), wrap32=True)
    now = time.monotonic()
    summaries = {name: stream.summary(now, args.duration) for name, stream in streams.items()}
    passed = all(stream['status'] == 'PASS' for stream in summaries.values())
    # Corrupt packets must remain visible even if enough other frames were readable.
    passed = passed and decoder.crc_errors == 0
    return {
        'status': 'PASS' if passed else 'FAIL', 'model': 'HFI-A9',
        'port': args.imu_port, 'baud': args.imu_baud, 'bytes': byte_count,
        'crc_errors': decoder.crc_errors, 'discarded_bytes': decoder.discarded_bytes,
        'streams': summaries, 'raw_log': str(report_dir / 'imu.bin'),
        'reason': 'CRC와 관성/자세 프레임 연속 수신 확인' if passed else
                  'IMU 데이터/CRC 검사 실패: 포트·전원·baud rate 및 streams 항목 확인',
    }


def validate_scan(message):
    if not (math.isfinite(message.range_min) and math.isfinite(message.range_max)
            and 0 <= message.range_min < message.range_max):
        return False, {'error': '잘못된 거리 범위'}
    ranges = [value for value in message.ranges if math.isfinite(value)
              and message.range_min <= value <= message.range_max and value > 0]
    return bool(ranges), {
        'rays': len(message.ranges), 'valid_rays': len(ranges),
        'min_m': round(min(ranges), 3) if ranges else None,
        'max_m': round(max(ranges), 3) if ranges else None,
    }


def validate_imu(message):
    gyro = [getattr(message.angular_velocity, axis) for axis in 'xyz']
    accel = [getattr(message.linear_acceleration, axis) for axis in 'xyz']
    finite = all(math.isfinite(value) for value in gyro + accel)
    available = (message.angular_velocity_covariance[0] != -1
                 and message.linear_acceleration_covariance[0] != -1)
    if not finite or not available:
        return False, {'error': '가속도/각속도 값이 없거나 유한하지 않음'}
    norm = math.sqrt(sum(value * value for value in accel))
    # D435i/IF has no fused orientation estimate. Only gyro + accel are required.
    return norm > 1e-6, {'gyro_rad_s': gyro, 'accel_m_s2': accel,
                         'accel_norm_m_s2': round(norm, 4)}


def validate_image(message, depth=False):
    import numpy as np

    sample = {'width': message.width, 'height': message.height, 'encoding': message.encoding}
    pixel_bytes = {'rgb8': 3, 'bgr8': 3, 'rgba8': 4, 'bgra8': 4, 'mono8': 1,
                   '16UC1': 2, '32FC1': 4}.get(message.encoding, 0)
    valid = (message.width > 0 and message.height > 0 and pixel_bytes > 0
             and message.step >= message.width * pixel_bytes
             and len(message.data) == message.step * message.height)
    if not valid:
        return False, {**sample, 'error': '영상 크기/버퍼/인코딩 불일치'}
    if not depth:
        return message.encoding in ('rgb8', 'bgr8', 'rgba8', 'bgra8', 'mono8'), sample
    if message.encoding not in ('16UC1', '32FC1'):
        return False, {**sample, 'error': '지원하지 않는 깊이 인코딩'}
    dtype = np.dtype(('>' if message.is_bigendian else '<') +
                     ('u2' if message.encoding == '16UC1' else 'f4'))
    pixels = np.ndarray((message.height, message.width), dtype=dtype,
                        buffer=message.data, strides=(message.step, dtype.itemsize))
    sampled = pixels[::8, ::8].astype(float)
    good = sampled[np.isfinite(sampled) & (sampled > 0)]
    meters = good * (0.001 if message.encoding == '16UC1' else 1.0)
    sample.update(valid_fraction=round(float(good.size / sampled.size), 3),
                  min_m=round(float(meters.min()), 3) if meters.size else None,
                  max_m=round(float(meters.max()), 3) if meters.size else None)
    return bool(good.size), sample


def driver_command(package, executable, namespace, parameters):
    from ament_index_python.packages import get_package_prefix, PackageNotFoundError

    try:
        path = Path(get_package_prefix(package)) / 'lib' / package / executable
    except PackageNotFoundError as exc:
        raise RuntimeError(f'ROS 2 드라이버 없음: {package}') from exc
    if not path.is_file():
        raise RuntimeError(f'드라이버 실행 파일 없음: {path}')
    command = [str(path), '--ros-args', '-r', f'__ns:={namespace}']
    for key, value in parameters.items():
        rendered = str(value).lower() if isinstance(value, bool) else str(value)
        command.extend(['-p', f'{key}:={rendered}'])
    return command


def stop_driver(process):
    for sig, timeout in ((signal.SIGINT, 5), (signal.SIGTERM, 2), (signal.SIGKILL, 2)):
        if process.poll() is not None:
            return
        try:
            os.killpg(process.pid, sig)
        except ProcessLookupError:
            return
        try:
            process.wait(timeout=timeout)
            return
        except subprocess.TimeoutExpired:
            continue
    raise RuntimeError(f'드라이버 종료 시간 초과: PID {process.pid}')


@contextmanager
def driver_process(command, environment, log_path):
    with log_path.open('w') as log:
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                   env=environment, start_new_session=True)
        try:
            yield process
        finally:
            stop_driver(process)


def ros_driver_spec(name, args, namespace):
    """Shared driver settings for the bounded check and the live GUI."""
    from sensor_msgs.msg import Image, Imu, LaserScan

    environment = os.environ.copy()
    if name == 'lidar':
        check_serial_port(args.lidar_port)
        command = driver_command('rplidar_ros', 'rplidar_composition', namespace, {
            'serial_port': args.lidar_port, 'serial_baudrate': args.lidar_baud,
            'frame_id': 'laser', 'inverted': False, 'angle_compensate': True,
            'scan_mode': 'Sensitivity', 'use_sim_time': False,
        })
        specs = {'scan': (f'{namespace}/scan', LaserScan, validate_scan)}
    else:
        library = Path(args.camera_library) if args.camera_library else RSUSB_LIBRARY
        if library.is_dir():
            environment['LD_LIBRARY_PATH'] = str(library) + ':' + environment.get('LD_LIBRARY_PATH', '')
        elif args.camera_library:
            raise RuntimeError(f'카메라 라이브러리 경로 없음: {library}')
        parameters = {
            'camera_name': 'camera', 'enable_color': True, 'enable_depth': True,
            'enable_infra1': False, 'enable_infra2': False,
            'enable_gyro': True, 'enable_accel': True, 'unite_imu_method': 2,
            'rgb_camera.color_profile': '640,480,15', 'rgb_camera.color_format': 'RGB8',
            'depth_module.depth_profile': '640,480,15', 'depth_module.depth_format': 'Z16',
            'align_depth.enable': False, 'pointcloud.enable': False, 'publish_tf': False,
            'wait_for_device_timeout': float(args.startup_timeout), 'use_sim_time': False,
        }
        if args.camera_serial:
            # Quote the serial so rcl's YAML parser preserves its string type.
            parameters['serial_no'] = json.dumps(args.camera_serial)
        command = driver_command('realsense2_camera', 'realsense2_camera_node', namespace, parameters)
        specs = {
            'color': (f'{namespace}/camera/color/image_raw', Image, validate_image),
            'depth': (f'{namespace}/camera/depth/image_rect_raw', Image,
                      lambda message: validate_image(message, depth=True)),
            'imu': (f'{namespace}/camera/imu', Imu, validate_imu),
        }
    return command, environment, specs


def check_ros_sensor(name, args, report_dir):
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.signals import SignalHandlerOptions

    namespace = f'/hardware_check_{os.getpid()}'
    log_path = report_dir / f'{name}.log'
    command, environment, specs = ros_driver_spec(name, args, namespace)

    context = Context()
    rclpy.init(args=[], context=context, domain_id=args.domain_id,
               signal_handler_options=SignalHandlerOptions.NO)
    node = None
    executor = SingleThreadedExecutor(context=context)
    streams = {key: StreamStats() for key in specs}
    problem = None
    measured = False
    try:
        node = rclpy.create_node(f'sensor_check_{name}_{os.getpid()}', context=context)
        executor.add_node(node)

        def callback(key, validator):
            def receive(message):
                valid, sample = validator(message)
                stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
                streams[key].add(valid and stamp > 0, sample, stamp, time.monotonic())
            return receive

        for key, (topic, message_type, validator) in specs.items():
            node.create_subscription(message_type, topic, callback(key, validator), qos_profile_sensor_data)
        with driver_process(command, environment, log_path) as process:
            deadline = time.monotonic() + args.startup_timeout
            while time.monotonic() < deadline:
                if process.poll() is not None:
                    problem = f'드라이버 조기 종료 (코드 {process.returncode})'
                    break
                executor.spin_once(timeout_sec=0.05)
                if all(stream.valid > 0 for stream in streams.values()):
                    break
            else:
                missing = [key for key, stream in streams.items() if stream.valid == 0]
                problem = f'시작 시간 초과: 정상 데이터 없음 ({", ".join(missing)})'
            if problem is None:
                measured = True
                streams = {key: StreamStats() for key in specs}
                deadline = time.monotonic() + args.duration
                while time.monotonic() < deadline:
                    if process.poll() is not None:
                        problem = f'측정 중 드라이버 종료 (코드 {process.returncode})'
                        break
                    executor.spin_once(timeout_sec=0.05)
            now = time.monotonic()
            summaries = {key: stream.summary(now, args.duration) for key, stream in streams.items()}
    finally:
        executor.shutdown(timeout_sec=1)
        if node is not None:
            node.destroy_node()
        context.try_shutdown()
    passed = measured and problem is None and all(item['status'] == 'PASS' for item in summaries.values())
    return {
        'status': 'PASS' if passed else 'FAIL',
        'reason': problem or ('실제 센서 데이터 연속 수신 확인' if passed else '데이터 품질/연속 수신 검사 실패'),
        'streams': summaries, 'topics': {key: spec[0] for key, spec in specs.items()},
        'driver_log': str(log_path), 'command': command,
    }


def positive_seconds(value):
    number = float(value)
    if not math.isfinite(number) or number < 2:
        raise argparse.ArgumentTypeError('2초 이상의 유한한 값을 입력하세요.')
    return number


def positive_integer(value):
    number = int(value)
    if number <= 0:
        raise argparse.ArgumentTypeError('양의 정수를 입력하세요.')
    return number


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description='IMU·라이다·카메라·모터/엔코더 하드웨어 검사')
    parser.add_argument('--sensors', '--devices', nargs='+', choices=('imu', 'lidar', 'camera', 'motors'),
                        default=['imu', 'lidar', 'camera'], help='검사 대상 (기본: imu lidar camera)')
    parser.add_argument('--motors', action='store_true', help='선택한 센서와 함께 메가 모터/엔코더도 검사')
    parser.add_argument('--motor-port', default=DEFAULT_MOTOR_PORT)
    parser.add_argument('--drive-calibration', type=Path, default=DEFAULT_CALIBRATION,
                        help='실제 좌우와 전진 부호 설정 JSON')
    parser.add_argument('--raw-motors', action='store_true', help='좌우/전진 보정 없이 드라이버 원시 채널 검사')
    parser.add_argument('--motor-test', choices=('observe', 'encoders', 'drive', 'bench'), default='observe',
                        help='observe: 수신 / encoders: 손 회전 / drive: 짧은 양방향 검사 / bench: 두 모터 연속 구동 (바퀴를 띄운 상태)')
    parser.add_argument('--motor-bench-seconds', type=float, default=30.0, help='bench 연속 구동 시간 0.1..30초 (기본: 30)')
    parser.add_argument('--motor-pwm', type=int, default=180, help='구동 검사 PWM 1..250 / v8 진단 출력 상한 650 (기본: 180)')
    parser.add_argument('--motor-pulse', type=float, default=0.8, help='모터별 한 방향 구동 시간 0.1..1.5초')
    parser.add_argument('--motor-min-ticks', type=positive_integer, default=8, help='회전 확인에 필요한 최소 엔코더 카운트')
    parser.add_argument('--motor-swap-channels', action=argparse.BooleanOptionalAction, default=True,
                        help='--raw-motors에서 사용: 40/41→R, 50/51→L (기본); 직접 대응은 --no-motor-swap-channels')
    parser.add_argument('--duration', type=positive_seconds, default=8.0, help='센서별 측정 시간, 초 (기본: 8)')
    parser.add_argument('--startup-timeout', type=positive_seconds, default=20.0,
                        help='ROS 드라이버별 시작 대기 제한, 초 (기본: 20)')
    parser.add_argument('--imu-port', default=DEFAULT_IMU)
    parser.add_argument('--imu-baud', type=positive_integer, default=921600)
    parser.add_argument('--lidar-port', default=DEFAULT_LIDAR)
    parser.add_argument('--lidar-baud', type=positive_integer, default=256000)
    parser.add_argument('--camera-serial', help='특정 RealSense 시리얼 번호')
    parser.add_argument('--camera-library', help='librealsense 공유 라이브러리 폴더 (기본: 이 시스템의 RSUSB 빌드)')
    parser.add_argument('--domain-id', type=int, choices=range(233), default=77, metavar='0..232',
                        help='검사용 ROS 도메인 (기본: 77, 로컬 통신)')
    parser.add_argument('--report-dir', type=Path, default=WORKSPACE / 'reports',
                        help='시간별 결과 폴더를 만들 상위 경로')
    args = parser.parse_args(argv)
    if args.motors and 'motors' not in args.sensors:
        args.sensors.append('motors')
    if args.motor_test != 'observe' and 'motors' not in args.sensors:
        parser.error('--motor-test는 --motors 또는 --sensors motors와 함께 사용하세요.')
    if not 1 <= args.motor_pwm <= 250:
        parser.error('--motor-pwm은 1..250 범위입니다.')
    if not math.isfinite(args.motor_pulse) or not 0.1 <= args.motor_pulse <= 1.5:
        parser.error('--motor-pulse는 0.1..1.5초 범위입니다.')
    if not math.isfinite(args.motor_bench_seconds) or not 0.1 <= args.motor_bench_seconds <= 30:
        parser.error('--motor-bench-seconds는 0.1..30초 범위입니다.')
    return args


def main(argv=None):
    args = parse_args(argv)
    # Apply only to this process and its children, never to the user's shell.
    os.environ['ROS_DOMAIN_ID'] = str(args.domain_id)
    os.environ.pop('ROS_LOCALHOST_ONLY', None)
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    os.environ.pop('ROS_STATIC_PEERS', None)
    run_dir = args.report_dir.expanduser().resolve() / datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    run_dir.mkdir(parents=True, exist_ok=False)
    report = {'started_at': datetime.now().astimezone().isoformat(),
              'duration_seconds': args.duration, 'domain_id': args.domain_id,
              'scope': 'hardware_check', 'motor_mode': args.motor_test,
              'results': {}, 'interrupted': False}

    def interrupt(_signum, _frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, interrupt)
    signal.signal(signal.SIGHUP, interrupt)
    current = None
    print(f'하드웨어 검사 | 수신 측정 {args.duration:g}초 | 로그: {run_dir}', flush=True)
    if args.motor_test in ('drive', 'bench'):
        print('모터 구동 검사: 바퀴를 지면에서 띄워야 합니다. Ctrl+C로 정지합니다.', flush=True)
    try:
        for current in dict.fromkeys(args.sensors):
            print(f'\n[검사] {current}', flush=True)
            try:
                if current == 'motors':
                    result = check_motors(args, run_dir)
                else:
                    result = check_imu(args, run_dir) if current == 'imu' else check_ros_sensor(current, args, run_dir)
            except Exception as exc:
                result = {'status': 'FAIL', 'reason': f'{type(exc).__name__}: {exc}'}
                log_path = run_dir / f'{current}.log'
                if log_path.exists():
                    result['driver_log'] = str(log_path)
            report['results'][current] = result
            print(f"[{result['status']}] {current}: {result['reason']}", flush=True)
            for key, stream in result.get('streams', {}).items():
                print(f"  {key}: {stream['status']} / {stream['messages']}개 / {stream['hz']:.1f} Hz"
                      + (f" / {', '.join(stream['reasons'])}" if stream['reasons'] else ''), flush=True)
    except KeyboardInterrupt:
        report['interrupted'] = True
        if current is not None:
            report['results'][current] = {'status': 'INTERRUPTED', 'reason': '사용자 중단'}
        print('\n검사를 중단했습니다.', flush=True)
    finally:
        for name in args.sensors:
            report['results'].setdefault(name, {'status': 'NOT_RUN', 'reason': '검사 미실행'})
        report['finished_at'] = datetime.now().astimezone().isoformat()
        report['passed'] = not report['interrupted'] and all(
            result['status'] == 'PASS' for result in report['results'].values())
        (run_dir / 'summary.json').write_text(json.dumps(report, ensure_ascii=False, indent=2,
                                                       allow_nan=False) + '\n')
        print(f"\n종합: {'PASS' if report['passed'] else 'FAIL'} | 결과: {run_dir / 'summary.json'}", flush=True)
    return 130 if report['interrupted'] else (0 if report['passed'] else 1)


if __name__ == '__main__':
    sys.exit(main())
