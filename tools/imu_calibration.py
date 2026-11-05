#!/usr/bin/env python3
"""Stationary residual gyro-bias calibration, without device or navigation writes.

HFI-A9: manufacturer's packet XYZ; gyro goes directly into angular_velocity in
the reference driver, hence rad/s. Acceleration remains packet XYZ, g -> m/s².
RealSense: SDK motion XYZ, with the SDK's existing motion correction preserved.
Neither frame is assumed to be base_link. One pose cannot identify accel bias.
"""

import argparse
from concurrent.futures import ThreadPoolExecutor
import csv
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import threading
import time

import numpy as np

from check_sensors import DEFAULT_IMU, check_serial_port
from hfi_a9 import A9Decoder

WORKSPACE = Path(__file__).resolve().parents[1]
SDK = Path.home() / '.local/opt/librealsense-2.58.1-rsusb'
PROFILE = WORKSPACE / 'config/imu_calibration.json'
G = 9.80665
FRAMES = {'imu': 'hfi_a9_packet_xyz', 'camera': 'realsense_sdk_motion_xyz'}
# Engineering screening limits for this static capture, not manufacturer specs.
LIMITS = {
    'min_sample_fraction': 0.80, 'max_host_gap_s': 0.25,
    'max_gyro_mean_norm_rad_s': 0.05, 'max_gyro_axis_std_rad_s': 0.005,
    'max_gyro_deviation_rad_s': 0.04, 'max_accel_axis_std_m_s2': 0.12,
    'max_accel_deviation_m_s2': 0.8, 'max_gyro_block_drift_rad_s': 0.001,
    'max_accel_block_drift_m_s2': 0.12,
    'max_validation_bias_axis_rad_s': 0.001,
    'min_validation_mean_bias_reduction': 0.20,
}


def json_write(path, value):
    """Atomic replacement: failed captures must not truncate a previous profile."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(mode='w', dir=path.parent, delete=False) as f:
        name = f.name
        try:
            json.dump(value, f, ensure_ascii=False, indent=2, allow_nan=False)
            f.write('\n')
        except BaseException:
            os.unlink(name)
            raise
    os.replace(name, path)


def build_camera_reader(sdk):
    source = WORKSPACE / 'tools/capture_realsense_imu.cpp'
    binary = WORKSPACE / 'build/imu_calibration/capture_realsense_imu'
    binary.parent.mkdir(parents=True, exist_ok=True)
    if not (sdk / 'include/librealsense2/rs.hpp').is_file():
        raise RuntimeError(f'RealSense SDK headers missing: {sdk}')
    if not binary.exists() or binary.stat().st_mtime < source.stat().st_mtime:
        subprocess.run(['g++', '-std=c++14', '-O2', '-Wall', '-Wextra', '-pthread',
                        str(source), '-isystem', str(sdk / 'include'), '-L' + str(sdk / 'lib'),
                        '-Wl,-rpath,' + str(sdk / 'lib'), '-lrealsense2', '-o', str(binary)],
                       check=True, timeout=90)
    return binary


def capture_camera(args, directory, stop, binary):
    total = args.warmup + args.duration + args.verify
    environment = os.environ.copy()
    environment['LD_LIBRARY_PATH'] = str(args.sdk / 'lib') + ':' + environment.get('LD_LIBRARY_PATH', '')
    with (directory / 'camera.log').open('w') as log:
        process = subprocess.Popen([str(binary), str(total), str(directory / 'camera.csv'),
                                    str(directory / 'camera_device.json'), args.camera_serial or ''],
                                   stdout=log, stderr=subprocess.STDOUT, env=environment)
        deadline = time.monotonic() + total + 30
        try:
            while process.poll() is None:
                if stop.wait(0.2):
                    raise RuntimeError('capture cancelled')
                if time.monotonic() > deadline:
                    raise RuntimeError('camera capture timed out')
            if process.returncode:
                raise RuntimeError(f'camera capture failed; see {directory / "camera.log"}')
        finally:
            if process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=5)
    return json.loads((directory / 'camera_device.json').read_text())


def capture_hfi(args, directory, stop):
    import serial
    check_serial_port(args.imu_port)
    decoder = A9Decoder()
    counts = {key: 0 for key in ('gyro', 'accel', 'mag', 'attitude')}
    total = args.warmup + args.duration + args.verify
    first = None
    began = time.monotonic()
    with serial.Serial(args.imu_port, args.imu_baud, timeout=0.05, exclusive=True) as port, \
            (directory / 'imu.bin').open('wb') as raw, (directory / 'imu.csv').open('w') as output:
        port.reset_input_buffer()
        writer = csv.writer(output)
        writer.writerow(('stream', 'host_s', 'stamp', 'frame_number', 'timestamp_domain', 'x', 'y', 'z'))
        while first is None or time.monotonic() - first < total:
            if stop.is_set():
                raise RuntimeError('capture cancelled')
            if first is None and time.monotonic() - began > 10:
                raise RuntimeError('HFI-A9 has no valid motion frames')
            data = port.read(max(1, min(4096, port.in_waiting)))
            raw.write(data)
            for frame in decoder.feed(data):
                now = time.monotonic()
                values = frame['values']
                if frame['kind'] == 'motion':
                    first = now if first is None else first
                    samples = {'gyro': values[:3], 'accel': [v * G for v in values[3:6]],
                               'mag': values[6:9]}
                else:
                    samples = {'attitude': values}
                for kind, xyz in samples.items():
                    counts[kind] += 1
                    writer.writerow((kind, now, frame['stamp'], counts[kind], 'hfi_uint32', *xyz))
    meta = {'name': 'HFI-A9', 'port': args.imu_port, 'identity': Path(args.imu_port).name,
            'identity_kind': 'USB UART adapter ID; not a unique IMU module serial',
            'baud': args.imu_baud, 'crc_errors': decoder.crc_errors,
            'discarded_bytes': decoder.discarded_bytes, 'frames': counts,
            'coordinate_frame': FRAMES['imu'], 'device_calibration_written': False,
            'gyro_units_source': 'HANDS-FREE/handsfree_ros_imu scripts/hfi_a9_ros.py angular_velocity assignment'}
    json_write(directory / 'imu_device.json', meta)
    return meta


def read_capture(path):
    streams = {}
    with Path(path).open() as f:
        for row in csv.DictReader(f):
            streams.setdefault(row['stream'], []).append([
                float(row['host_s']), float(row['stamp']), float(row['x']),
                float(row['y']), float(row['z']), float(row['frame_number']),
                -1.0 if row['timestamp_domain'] == 'hfi_uint32' else float(row['timestamp_domain'])])
    return {key: np.asarray(value, dtype=float) for key, value in streams.items()}


def stats(samples):
    values = samples[:, 2:5]
    mean = values.mean(axis=0)
    return {
        'samples': len(samples), 'span_s': float(samples[-1, 0] - samples[0, 0]),
        'mean': mean.tolist(), 'std': values.std(axis=0, ddof=1).tolist(),
        'sample_covariance': np.cov(values, rowvar=False).tolist(),
        'mean_vector_norm': float(np.linalg.norm(mean)),
        'peak_deviation_norm': float(np.max(np.linalg.norm(values - mean, axis=1))),
        'rms_vector_norm': float(np.sqrt(np.mean(np.sum(values * values, axis=1)))),
    }


def stream_health(samples, name, seconds, expected_hz, wrap32):
    problems = []
    if len(samples) < max(100, seconds * expected_hz * LIMITS['min_sample_fraction']):
        problems.append(f'{name}: too few samples')
    if len(samples) < 2 or not np.isfinite(samples).all():
        return problems + [f'{name}: missing/nonfinite samples']
    span = samples[-1, 0] - samples[0, 0]
    if span < seconds * 0.95:
        problems.append(f'{name}: incomplete time coverage')
    host_delta = np.diff(samples[:, 0])
    if np.any(host_delta < 0) or host_delta.max() > LIMITS['max_host_gap_s']:
        problems.append(f'{name}: host timestamp gap/regression')
    delta = np.diff(samples[:, 1])
    if wrap32:
        delta %= 2 ** 32
        invalid = (delta <= 0) | (delta >= 2 ** 31)
    else:
        invalid = delta <= 0
    if np.any(invalid):
        problems.append(f'{name}: device timestamp repeated/regressed')
    if np.any(np.diff(samples[:, 5]) <= 0):
        problems.append(f'{name}: frame number repeated/regressed')
    if len(np.unique(samples[:, 6])) != 1:
        problems.append(f'{name}: timestamp domain changed')
    return problems


def block_drift(samples, block_seconds=5):
    elapsed = samples[:, 0] - samples[0, 0]
    blocks = [samples[(elapsed >= start) & (elapsed < start + block_seconds), 2:5]
              for start in np.arange(0, elapsed[-1] - block_seconds + 0.1, block_seconds)]
    means = np.array([block.mean(axis=0) for block in blocks if len(block)])
    return float(np.max(np.ptp(means, axis=0))) if len(means) >= 2 else 0.0


def analyze_capture(streams, sensor, metadata, warmup, duration, verify):
    reasons = []
    if any(name not in streams for name in ('gyro', 'accel')):
        return {'status': 'FAIL', 'reasons': ['missing gyro or accel stream']}
    beginning = min(streams[name][0, 0] for name in ('gyro', 'accel'))
    result = {'status': 'FAIL', 'reasons': reasons, 'phases': {}}
    for phase, start, length in (('calibration', warmup, duration),
                                 ('validation', warmup + duration, verify)):
        phase_stats = {}
        selected = {}
        for name in ('gyro', 'accel'):
            data = streams[name]
            data = data[(data[:, 0] >= beginning + start) &
                        (data[:, 0] < beginning + start + length)]
            hz = 100.0 if sensor == 'imu' else float(metadata[name]['fps'])
            problems = stream_health(data, name, length, hz, sensor == 'imu')
            reasons.extend(f'{phase}: {problem}' for problem in problems)
            if len(data) < 2 or not np.isfinite(data).all():
                continue
            selected[name] = data
            phase_stats[name] = stats(data)
            phase_stats[name]['max_block_mean_change'] = block_drift(data)
        result['phases'][phase] = phase_stats
        if len(selected) != 2:
            continue
        gyro, accel = phase_stats['gyro'], phase_stats['accel']
        for bad, message in (
            (gyro['mean_vector_norm'] > LIMITS['max_gyro_mean_norm_rad_s'], 'gyro mean too high for static capture'),
            (max(gyro['std']) > LIMITS['max_gyro_axis_std_rad_s'], 'gyro vibration/rotation'),
            (gyro['peak_deviation_norm'] > LIMITS['max_gyro_deviation_rad_s'], 'gyro motion/transient'),
            (max(accel['std']) > LIMITS['max_accel_axis_std_m_s2'], 'accel vibration/motion'),
            (accel['peak_deviation_norm'] > LIMITS['max_accel_deviation_m_s2'], 'accel motion/transient'),
            (not 8.0 <= accel['mean_vector_norm'] <= 12.0, 'gravity magnitude outside screening range'),
            (gyro['max_block_mean_change'] > LIMITS['max_gyro_block_drift_rad_s'], 'gyro drift between 5 s blocks'),
            (accel['max_block_mean_change'] > LIMITS['max_accel_block_drift_m_s2'], 'accel posture changed'),
        ):
            if bad:
                reasons.append(f'{phase}: {message}')
    if sensor == 'imu' and metadata.get('crc_errors', 0):
        reasons.append('HFI CRC errors')
    if any(len(result['phases'][key]) != 2 for key in ('calibration', 'validation')):
        return result
    training = result['phases']['calibration']
    validation = result['phases']['validation']
    bias = np.array(training['gyro']['mean'])
    residual = np.array(validation['gyro']['mean']) - bias
    if np.max(np.abs(residual)) > LIMITS['max_validation_bias_axis_rad_s']:
        reasons.append('validation: residual gyro bias too large')
    if np.max(np.abs(np.array(validation['accel']['mean']) - training['accel']['mean'])) > LIMITS['max_accel_block_drift_m_s2']:
        reasons.append('validation: posture changed between training and validation')
    raw_rms = validation['gyro']['rms_vector_norm']
    # Translation leaves the centered covariance unchanged.
    corrected_rms = math.sqrt(float(np.sum(residual ** 2)) +
                              sum(np.square(validation['gyro']['std'])) *
                              (validation['gyro']['samples'] - 1) / validation['gyro']['samples'])
    before_norm = float(np.linalg.norm(validation['gyro']['mean']))
    after_norm = float(np.linalg.norm(residual))
    reduction = 1 - after_norm / before_norm if before_norm else 0.0
    recommend = reduction >= LIMITS['min_validation_mean_bias_reduction']
    result.update(
        status='FAIL' if reasons else 'PASS', gyro_bias_rad_s=bias.tolist(),
        validation_mean_before_rad_s=validation['gyro']['mean'],
        validation_mean_after_rad_s=residual.tolist(),
        validation_rms_before_rad_s=raw_rms, validation_rms_after_rad_s=corrected_rms,
        gravity_reference_m_s2=training['accel']['mean'],
        gravity_norm_error_percent=100 * (training['accel']['mean_vector_norm'] / G - 1),
        additional_correction_recommended=recommend,
        validation_mean_bias_reduction=reduction,
        selected_gyro_bias_rad_s=bias.tolist() if recommend else [0.0, 0.0, 0.0],
    )
    return result


def correction_profile(sensor, metadata, result, report):
    if result['status'] != 'PASS':
        raise ValueError('cannot save a failed calibration')
    return {
        'model': metadata['name'],
        'identity': metadata['identity'] if sensor == 'imu' else metadata['serial'],
        'coordinate_frame': FRAMES[sensor], 'gyro_units': 'rad/s',
        'method': 'stationary_residual_gyro_bias',
        'gyro_bias_rad_s': result['selected_gyro_bias_rad_s'],
        'estimated_gyro_bias_rad_s': result['gyro_bias_rad_s'],
        'additional_correction_recommended': result['additional_correction_recommended'],
        'correction_decision': 'subtract_estimated_bias' if result['additional_correction_recommended']
                               else 'keep_existing_output; no verified improvement from additional subtraction',
        'correction': 'gyro_corrected = gyro_input - gyro_bias_rad_s (same sensor axes)',
        'input_processing': 'HFI packet values' if sensor == 'imu' else {
            'sdk': metadata['sdk_version'], 'firmware': metadata['firmware'],
            'motion_correction_enabled': metadata['motion_correction_enabled'],
            'gyro_intrinsics': metadata['gyro']['intrinsics']},
        'gravity_reference_m_s2': result['gravity_reference_m_s2'],
        'gyro_sample_covariance_rad2_s2': result['phases']['calibration']['gyro']['sample_covariance'],
        'validation': {'stationary': True, 'independent_following_window': True,
                       'candidate_mean_residual_rad_s': result['validation_mean_after_rad_s'],
                       'selected_mean_residual_rad_s': result['validation_mean_after_rad_s']
                           if result['additional_correction_recommended'] else result['validation_mean_before_rad_s'],
                       'rms_before_rad_s': result['validation_rms_before_rad_s'],
                       'candidate_rms_after_rad_s': result['validation_rms_after_rad_s'],
                       'selected_rms_after_rad_s': result['validation_rms_after_rad_s']
                           if result['additional_correction_recommended'] else result['validation_rms_before_rad_s'],
                       'min_mean_bias_reduction_for_selection': LIMITS['min_validation_mean_bias_reduction'],
                       'dynamic': False, 'temperature_range': False, 'power_cycle': False},
        'accelerometer_calibrated': False, 'magnetometer_calibrated': False,
        'mounting_transform_calibrated': False, 'orientation_output_corrected': False,
        'device_calibration_written': False, 'enabled_for_navigation': False,
        'report': str(report.relative_to(WORKSPACE)) if report.is_relative_to(WORKSPACE) else str(report),
    }


def subtract_gyro_bias(values, profile, *, identity, coordinate_frame):
    """Explicit future adapter helper; never auto-applied by existing drivers.

    The caller must additionally match RealSense input_processing (SDK intrinsic
    correction) and avoid applying a second calibration to already corrected data.
    """
    if identity != profile['identity'] or coordinate_frame != profile['coordinate_frame']:
        raise ValueError('IMU identity/coordinate frame mismatch')
    if profile['gyro_units'] != 'rad/s' or not profile['validation']['stationary']:
        raise ValueError('unvalidated gyro calibration or wrong units')
    xyz = np.asarray(values, dtype=float)
    bias = np.asarray(profile['gyro_bias_rad_s'], dtype=float)
    if xyz.shape != (3,) or bias.shape != (3,) or not np.isfinite(xyz).all() or not np.isfinite(bias).all():
        raise ValueError('expected three finite gyro/bias values')
    return (xyz - bias).tolist()


def seconds(value):
    number = float(value)
    if not math.isfinite(number) or not 10 <= number <= 600:
        raise argparse.ArgumentTypeError('10..600초를 입력하세요.')
    return number


def main(argv=None):
    parser = argparse.ArgumentParser(description='외장/카메라 IMU 정지 자이로 보정. 주행 적용·센서 EEPROM 기록 없음.')
    parser.add_argument('--sensors', nargs='+', choices=('imu', 'camera'), default=['imu', 'camera'])
    parser.add_argument('--stationary', action='store_true', required=True,
                        help='로봇이 정지하고 흔들리지 않는 상태임을 확인')
    parser.add_argument('--warmup', type=seconds, default=60.0)
    parser.add_argument('--duration', type=seconds, default=60.0, help='보정용 측정 시간')
    parser.add_argument('--verify', type=seconds, default=30.0, help='별도 검증용 측정 시간')
    parser.add_argument('--imu-port', default=DEFAULT_IMU)
    parser.add_argument('--imu-baud', type=int, default=921600)
    parser.add_argument('--camera-serial')
    parser.add_argument('--sdk', type=Path, default=SDK)
    parser.add_argument('--profile', type=Path, default=PROFILE)
    parser.add_argument('--report-dir', type=Path, default=WORKSPACE / 'reports')
    args = parser.parse_args(argv)
    directory = args.report_dir / datetime.now().strftime('imu_calibration_%Y%m%d_%H%M%S_%f')
    directory.mkdir(parents=True)
    summary = {'started_at': datetime.now(timezone.utc).isoformat(), 'status': 'FAIL',
               'warmup_s': args.warmup, 'calibration_s': args.duration, 'validation_s': args.verify,
               'limits': LIMITS, 'sensors': {}, 'profile_saved': False,
               'scope': 'stationary gyro only; no device calibration writes or navigation changes'}
    stop = threading.Event()
    try:
        binary = build_camera_reader(args.sdk) if 'camera' in args.sensors else None
        print(f'결과 폴더: {directory}', flush=True)
        print(f'정지 유지: 안정화 {args.warmup:g}초 + 측정 {args.duration:g}초 + 검증 {args.verify:g}초', flush=True)
        with ThreadPoolExecutor(max_workers=2) as executor:
            tasks = {name: executor.submit(capture_hfi, args, directory, stop) if name == 'imu'
                     else executor.submit(capture_camera, args, directory, stop, binary)
                     for name in dict.fromkeys(args.sensors)}
            try:
                last_progress = began = time.monotonic()
                while not all(task.done() for task in tasks.values()):
                    now = time.monotonic()
                    if now - last_progress >= 10:
                        print(f'수집 중: {now - began:.0f}초 경과', flush=True)
                        last_progress = now
                    stop.wait(0.2)
            except BaseException:
                stop.set()
                raise
            profiles = {}
            for name, task in tasks.items():
                try:
                    metadata = task.result()
                    result = analyze_capture(read_capture(directory / f'{name}.csv'), name, metadata,
                                             args.warmup, args.duration, args.verify)
                    summary['sensors'][name] = {'device': metadata, **result}
                    if result['status'] == 'PASS':
                        profiles[name] = correction_profile(name, metadata, result, directory / 'summary.json')
                    print(f'{name}: {result["status"]}; {result["reasons"]}', flush=True)
                    if 'gyro_bias_rad_s' in result:
                        print(f'  bias rad/s: {result["gyro_bias_rad_s"]}\n'
                              f'  후보 보정의 검증 잔차 rad/s: {result["validation_mean_after_rad_s"]}\n'
                              f'  추가 보정 권장: {result["additional_correction_recommended"]}', flush=True)
                except Exception as exc:
                    summary['sensors'][name] = {'status': 'FAIL', 'reasons': [str(exc)]}
            # Commit the selected set together. Keep previous good calibration on any failure.
            if len(profiles) == len(tasks):
                previous = json.loads(args.profile.read_text()) if args.profile.exists() else {'schema': 1, 'sensors': {}}
                if previous.get('schema') != 1:
                    raise RuntimeError('unsupported existing calibration profile schema')
                previous['sensors'].update(profiles)
                previous.update(updated_at=datetime.now(timezone.utc).isoformat(), enabled_for_navigation=False)
                json_write(args.profile, previous)
                summary.update(status='PASS', profile_saved=True, profile=str(args.profile))
    except KeyboardInterrupt:
        stop.set()
        summary['reason'] = 'user interrupted'
        return_code = 130
    except Exception as exc:
        summary['reason'] = str(exc)
        return_code = 1
    else:
        return_code = 0 if summary['status'] == 'PASS' else 1
    finally:
        stop.set()
        summary['completed_at'] = datetime.now(timezone.utc).isoformat()
        summary['sha256'] = {str(path.relative_to(directory)): hashlib.sha256(path.read_bytes()).hexdigest()
                             for path in directory.iterdir() if path.is_file() and path.name != 'summary.json'}
        summary['source_sha256'] = {str(path.relative_to(WORKSPACE)): hashlib.sha256(path.read_bytes()).hexdigest()
                                    for path in (Path(__file__), WORKSPACE / 'tools/capture_realsense_imu.cpp',
                                                 WORKSPACE / 'tools/hfi_a9.py')}
        json_write(directory / 'summary.json', summary)
    print(f'{summary["status"]}: {directory / "summary.json"}', flush=True)
    if summary.get('reason'):
        print(summary['reason'], file=sys.stderr)
    if summary['profile_saved']:
        print(f'보정값 저장: {args.profile} (주행 적용 꺼짐)', flush=True)
    return return_code


if __name__ == '__main__':
    sys.exit(main())
