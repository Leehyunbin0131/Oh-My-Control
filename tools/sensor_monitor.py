"""Live sensor backend shared by the SSH terminal dashboard and its tests."""

from collections import deque
from datetime import datetime
import json
import math
import os
from pathlib import Path
import threading
import time

import numpy as np

from check_sensors import driver_process, ros_driver_spec
from serial_port import check_serial_port
from hfi_a9 import A9Decoder
from motor_link import MotorClient, MotorCancelled
from motor_test import drive_test, bench_test, observation_result
from drive_calibration import calibration_for


CHANNELS = {
    'hfi_imu': 'HFI-A9 IMU', 'lidar': 'RPLIDAR A2M12',
    'color': 'RealSense RGB', 'depth': 'RealSense depth', 'camera_imu': 'RealSense IMU',
    'motors': 'Motor / encoders',
}


class LiveChannel:
    def __init__(self):
        self.times = deque(maxlen=1000)
        self.count = self.errors = 0
        self.last = self.stamp = None
        self.sample = {}
        self.problem = ''
        self.axes = [False, False, False]
        self.baseline = None

    def update(self, valid, sample, stamp, now, wrap32=False):
        if self.stamp is not None:
            delta = stamp - self.stamp
            valid = valid and (0 < (delta & 0xFFFFFFFF) < 0x80000000 if wrap32 else delta > 0)
        self.stamp = stamp
        if not valid:
            self.errors += 1
            self.problem = 'Invalid data or repeated timestamp'
            return
        self.problem = ''
        self.sample = sample
        self.last = now
        self.count += 1
        self.times.append(now)
        if 'axis_activity' in sample:
            self.axes = [old or new for old, new in zip(self.axes, sample['axis_activity'])]
        elif 'rpy_degrees' in sample:
            if self.baseline is None:
                self.baseline = sample['rpy_degrees'][:]
            for i, angle in enumerate(sample['rpy_degrees']):
                difference = abs((angle - self.baseline[i] + 180) % 360 - 180)
                self.axes[i] |= difference >= 3.0

    def snapshot(self, now, running, started):
        recent = [t for t in self.times if now - t < 3]
        hz = (len(recent) - 1) / (recent[-1] - recent[0]) if len(recent) > 1 and recent[-1] > recent[0] else 0
        age = now - self.last if self.last is not None else None
        if not running:
            status = 'STOPPED'
        elif self.problem:
            status = 'ERROR'
        elif age is not None and age > 2:
            status = 'STALE'
        elif self.count >= 3:
            status = 'LIVE'
        else:
            status = 'WAIT' if now - started < 25 else 'NO DATA'
        return {'status': status, 'hz': round(hz, 1), 'count': self.count,
                'age': None if age is None else round(age, 2), 'errors': self.errors,
                'problem': self.problem, 'sample': self.sample, 'axes': self.axes[:]}


def camera_orientation(gyro, accel, previous_yaw, dt):
    """Display estimate only: optical axes -> forward/left/up; yaw is relative."""
    ax, ay, az = accel[2], -accel[0], -accel[1]
    roll = math.atan2(ay, az)
    pitch = math.atan2(-ax, math.hypot(ay, az))
    # Euler yaw rate from body-frame gyro. Near vertical, heading is undefined.
    wy, wz = -gyro[0], -gyro[1]
    yaw_rate = (math.sin(roll) * wy + math.cos(roll) * wz) / math.cos(pitch) if abs(math.cos(pitch)) > 0.1 else 0
    yaw = previous_yaw + yaw_rate * dt
    return [math.degrees(roll), math.degrees(pitch), math.degrees(yaw)], yaw


def image_pixels(message, depth=False):
    """Small RGB preview; original dimensions remain in the sample metadata."""
    stride_y, stride_x = max(1, message.height // 120), max(1, message.width // 160)
    if not depth:
        pixels = np.ndarray((message.height, message.width, 3), dtype=np.uint8,
                            buffer=message.data, strides=(message.step, 3, 1))
        pixels = pixels[::stride_y, ::stride_x].copy()
        return pixels if message.encoding == 'rgb8' else pixels[:, :, ::-1].copy()
    dtype = np.dtype(('>' if message.is_bigendian else '<') + ('u2' if message.encoding == '16UC1' else 'f4'))
    data = np.ndarray((message.height, message.width), dtype=dtype, buffer=message.data,
                      strides=(message.step, dtype.itemsize))[::stride_y, ::stride_x].astype(float)
    meters = data * (0.001 if message.encoding == '16UC1' else 1)
    good = np.isfinite(meters) & (meters > 0)
    t = np.clip(np.nan_to_num(meters, nan=0, posinf=4) / 4, 0, 1)
    pixels = np.stack([np.clip(1.5 - abs(4 * t - offset), 0, 1) for offset in (3, 2, 1)], axis=-1)
    pixels[~good] = 0
    return (pixels * 255).astype(np.uint8)


class SensorMonitor:
    def __init__(self, args):
        self.args = args
        self.lock = threading.Lock()
        self.channels = {key: LiveChannel() for key in CHANNELS}
        self.images = {}
        self.stop_event = threading.Event()
        self.threads = []
        self.running = False
        self.started = 0
        self.run_dir = None
        self.motor_request = threading.Event()
        self.motor_cancel = threading.Event()
        self.motor_state = {'phase': 'OBSERVE', 'result': None, 'enabled': False}
        groups = set(args.sensors)
        self.enabled = {key for key in CHANNELS if
                        {'hfi_imu': 'imu', 'color': 'camera', 'depth': 'camera', 'camera_imu': 'camera'}.get(key, key) in groups}

    def start(self):
        if any(thread.is_alive() for thread in self.threads):
            raise RuntimeError('Previous sensor workers are still stopping')
        self.channels = {key: LiveChannel() for key in CHANNELS}
        self.images = {}
        self.stop_event.clear()
        self.motor_request.clear()
        self.motor_cancel.clear()
        self.motor_state = {'phase': 'OBSERVE', 'result': None, 'enabled': 'motors' in self.enabled}
        self.started = time.monotonic()
        self.running = True
        self.run_dir = self.args.report_dir / ('tui_' + datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
        self.run_dir.mkdir(parents=True)
        workers = [('hfi_imu', self._imu), ('lidar', lambda: self._ros('lidar')),
                   ('camera', lambda: self._ros('camera')), ('motors', self._motors)]
        self.threads = [threading.Thread(target=self._guard, args=(name, worker), daemon=True)
                        for name, worker in workers if ('color' if name == 'camera' else name) in self.enabled]
        for thread in self.threads:
            thread.start()

    def _guard(self, name, worker):
        try:
            worker()
        except Exception as exc:
            error = f'{type(exc).__name__}: {exc}'
            self.error(name, error)
            if name == 'motors':
                result = {'status': 'FAIL', 'reason': error}
                with self.lock:
                    self.motor_state.update(phase='ERROR', result=result)
                (self.run_dir / 'motors_failure.json').write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n')
            with (self.run_dir / 'errors.log').open('a') as log:
                log.write(f'{datetime.now().isoformat()} {name}: {error}\n')

    def error(self, name, reason):
        with self.lock:
            keys = ('color', 'depth', 'camera_imu') if name == 'camera' else (name,)
            for key in keys:
                self.channels[key].problem = reason

    def update(self, key, valid, sample, stamp, now, wrap32=False):
        with self.lock:
            self.channels[key].update(valid, sample, stamp, now, wrap32)

    def snapshot(self):
        with self.lock:
            channels = {key: value.snapshot(time.monotonic(), self.running, self.started)
                        for key, value in self.channels.items()}
            for key in channels:
                if key not in self.enabled:
                    channels[key]['status'] = 'DISABLED'
            return {'running': self.running, 'elapsed': round(time.monotonic() - self.started, 1),
                    'log_directory': str(self.run_dir),
                    'motor': dict(self.motor_state), 'channels': channels}

    def request_motor_test(self):
        with self.lock:
            if 'motors' not in self.enabled or self.args.motor_test not in ('drive', 'bench'):
                return 'Restart with --motors --motor-test drive or bench to enable the motor test.'
            if self.channels['motors'].snapshot(time.monotonic(), self.running, self.started)['status'] != 'LIVE':
                return 'Live Mega firmware telemetry is required.'
            if self.motor_state['phase'] not in ('OBSERVE', 'DONE', 'STOPPED'):
                return 'Motor test is already running.'
            self.motor_cancel.clear()
            self.motor_state.update(phase='QUEUED', result=None)
            self.motor_request.set()
            return 'Motor test started. Space / x: STOP.'

    def stop_motors(self):
        self.motor_request.clear()
        self.motor_cancel.set()

    def _motors(self):
        def sample(value):
            self.update('motors', not value['fault'], value, value['stamp'], time.monotonic(), wrap32=True)
            if value['fault']:
                self.error('motors', value['fault_reason'])

        def progress(message):
            with self.lock:
                self.motor_state['phase'] = message

        calibration = calibration_for(self.args)
        with MotorClient(self.args.motor_port, self.run_dir, self.args.startup_timeout, sample, calibration=calibration) as client:
            with self.lock:
                self.motor_state['firmware'] = client.info
                self.motor_state['drive_calibration'] = calibration.document if calibration else None
            while not self.stop_event.is_set():
                if self.motor_request.is_set():
                    self.motor_request.clear()
                    try:
                        test = bench_test if self.args.motor_test == 'bench' else drive_test
                        result = test(client, self.args,
                                            cancelled=lambda: self.motor_cancel.is_set() or self.stop_event.is_set(),
                                            progress=progress)
                    except MotorCancelled:
                        result = {'status': 'INTERRUPTED', 'reason': '사용자 중단'}
                    with self.lock:
                        self.motor_state.update(phase='DONE', result=result)
                    (self.run_dir / ('motors_' + datetime.now().strftime('%H%M%S_%f') + '.json')).write_text(
                        json.dumps(result, ensure_ascii=False, indent=2) + '\n')
                if self.motor_cancel.is_set():
                    client.command('STOP')
                    self.motor_cancel.clear()
                    progress('STOPPED')
                client.poll()
                if self.args.motor_test == 'encoders':
                    with self.lock:
                        self.motor_state['result'] = observation_result(list(client.samples), True, self.args.motor_min_ticks)

    def image(self, key):
        with self.lock:
            return self.images.get(key)

    def _imu(self):
        import serial

        check_serial_port(self.args.imu_port)
        decoder = A9Decoder()
        motion, motion_time, previous_motion_stamp = None, 0, None
        with serial.Serial(self.args.imu_port, self.args.imu_baud, timeout=0.05, exclusive=True) as imu:
            imu.reset_input_buffer()
            while not self.stop_event.is_set():
                data = imu.read(max(1, min(4096, imu.in_waiting)))
                previous_crc = decoder.crc_errors
                frames = decoder.feed(data)
                if decoder.crc_errors != previous_crc:
                    self.error('hfi_imu', 'Serial CRC error')
                    with self.lock:
                        self.channels['hfi_imu'].errors += decoder.crc_errors - previous_crc
                for frame in frames:
                    now = time.monotonic()
                    values = frame['values']
                    if not all(math.isfinite(value) for value in values):
                        self.error('hfi_imu', 'Invalid IMU value')
                        continue
                    if frame['kind'] == 'motion':
                        progressing = previous_motion_stamp is None or 0 < ((frame['stamp'] - previous_motion_stamp) & 0xFFFFFFFF) < 0x80000000
                        previous_motion_stamp = frame['stamp']
                        if progressing and math.sqrt(sum(v * v for v in values[3:6])) > 1e-6:
                            motion, motion_time = values, now
                        continue
                    if motion is None or now - motion_time > 0.15:
                        self.error('hfi_imu', 'No fresh acceleration/gyro frame')
                        continue
                    sample = {'rpy_degrees': [values[0], -values[1], -values[2]],
                              'gyro_raw': list(motion[:3]), 'accel_g': list(motion[3:6]),
                              'crc_errors': decoder.crc_errors, 'orientation_source': 'HFI-A9 fused'}
                    self.update('hfi_imu', True, sample, frame['stamp'], now, wrap32=True)

    def _ros(self, name):
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import qos_profile_sensor_data
        from rclpy.signals import SignalHandlerOptions

        namespace = f'/hardware_tui_{os.getpid()}'
        command, environment, specs = ros_driver_spec(name, self.args, namespace)
        context = Context()
        rclpy.init(args=[], context=context, domain_id=self.args.domain_id,
                   signal_handler_options=SignalHandlerOptions.NO)
        executor = SingleThreadedExecutor(context=context)
        node = None
        yaw, previous_camera_stamp = 0.0, None
        try:
            node = rclpy.create_node(f'tui_{name}_{os.getpid()}', context=context)
            executor.add_node(node)

            def callback(key, validator):
                def receive(message):
                    nonlocal yaw, previous_camera_stamp
                    valid, sample = validator(message)
                    stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
                    valid = valid and stamp > 0
                    channel = {'scan': 'lidar', 'imu': 'camera_imu'}.get(key, key)
                    if valid and key == 'scan':
                        sample['points'] = [[round(r * math.cos(message.angle_min + i * message.angle_increment), 3),
                                             round(r * math.sin(message.angle_min + i * message.angle_increment), 3)]
                                            for i, r in enumerate(message.ranges)
                                            if math.isfinite(r) and message.range_min <= r <= message.range_max and r > 0]
                    elif valid and key == 'imu':
                        dt = 0 if previous_camera_stamp is None else (stamp - previous_camera_stamp) / 1e9
                        if not 0 < dt < 0.1:
                            dt = 0
                        previous_camera_stamp = stamp
                        rpy, yaw = camera_orientation(sample['gyro_rad_s'], sample['accel_m_s2'], yaw, dt)
                        gx, gy, gz = sample['gyro_rad_s']
                        sample.update(rpy_degrees=rpy, orientation_source='accel tilt + relative gyro yaw',
                                      axis_activity=[abs(value) > 0.12 for value in (gz, -gx, -gy)])
                    elif valid and key in ('color', 'depth'):
                        pixels = image_pixels(message, depth=key == 'depth')
                        with self.lock:
                            self.images[key] = (time.monotonic(), pixels)
                    self.update(channel, valid, sample, stamp, time.monotonic())
                return receive

            for key, (topic, message_type, validator) in specs.items():
                node.create_subscription(message_type, topic, callback(key, validator), qos_profile_sensor_data)
            with driver_process(command, environment, self.run_dir / f'{name}.log') as process:
                start = time.monotonic()
                while not self.stop_event.is_set():
                    if process.poll() is not None:
                        raise RuntimeError(f'Driver exited ({process.returncode}); see {name}.log')
                    executor.spin_once(timeout_sec=0.05)
                    if time.monotonic() - start > self.args.startup_timeout:
                        keys = ('lidar',) if name == 'lidar' else ('color', 'depth', 'camera_imu')
                        with self.lock:
                            for key in keys:
                                if self.channels[key].count == 0:
                                    self.channels[key].problem = f'No data; check {name}.log / USB / power'
        finally:
            executor.shutdown(timeout_sec=1)
            if node is not None:
                node.destroy_node()
            context.try_shutdown()

    def stop(self):
        self.stop_motors()
        self.stop_event.set()
        # Workers receive the stop request together; all use bounded IO/cleanup.
        deadline = time.monotonic() + 12
        for thread in self.threads:
            thread.join(timeout=max(0, deadline - time.monotonic()))
        alive = [thread.name for thread in self.threads if thread.is_alive()]
        self.running = False
        if alive:
            raise RuntimeError('Sensor workers did not stop: ' + ', '.join(alive))

    def save(self, images=False):
        stamp = datetime.now().strftime('%H%M%S_%f')
        path = self.run_dir / f'status_{stamp}.json'
        path.write_text(json.dumps(self.snapshot(), ensure_ascii=False, indent=2, allow_nan=False) + '\n')
        if images:
            from PIL import Image
            for key in ('color', 'depth'):
                frame = self.image(key)
                if frame is not None and time.monotonic() - frame[0] < 2:
                    Image.fromarray(frame[1]).save(self.run_dir / f'{key}_{stamp}.png')
        return path
