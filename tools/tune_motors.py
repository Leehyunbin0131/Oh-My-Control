#!/usr/bin/env python3
"""Measure unloaded motor response, fit PI + feedforward, validate and save."""

import argparse
import copy
from datetime import datetime
import json
from pathlib import Path
import shutil
import signal
import statistics
import time

from drive_calibration import DriveCalibration, DEFAULT_CALIBRATION
from motor_test import MotorClient, MotorError, DEFAULT_MOTOR_PORT
from motor_pid import PIDSettings, DEFAULT_PID_CONFIG, COUNTS_PER_REV, fit_feedforward, control_defaults

WORKSPACE = Path(__file__).resolve().parents[1]


class SpeedValidationError(MotorError):
    def __init__(self, trial):
        super().__init__('PID 속도 오차 기준을 초과했습니다. 기존 보정 파일은 교체하지 않습니다.')
        self.trial = trial


def save(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n')


def stopped(client):
    before = client.latest
    began = time.monotonic()
    streamed = client.stream_started
    brake_samples = client.controlled_stop(timeout=1.5) if streamed else []
    brake_seconds = time.monotonic() - began
    client.command('STOP')
    samples = client.collect(0.6)
    if client.latest['armed'] or any(client.latest['command_pwm']):
        raise MotorError('출력 해제 명령 후 출력이 0이 아닙니다.')
    if len(samples) < 5 or any(abs(d) > 2 for d in client.calibration.wheel_deltas(samples[-5], samples[-1]).values()):
        raise MotorError('정지 후 바퀴 회전이 계속됩니다.')
    client.last_stop_report = None
    if streamed:
        trace = [before, *brake_samples, *samples]
        reverse = {'left': 0, 'right': 0}
        for a, b in zip(trace, trace[1:]):
            for name, spec in client.calibration.wheels.items():
                channel = 0 if spec['driver_channel'] == 'L' else 1
                direction = before['target_rpm_centi'][channel] * spec['forward_command_sign']
                change = client.calibration.wheel_deltas(a, b)[name]
                if direction:
                    reverse[name] += max(0, -change * (1 if direction > 0 else -1))
        delta = client.calibration.wheel_deltas(before, samples[-1])
        client.last_stop_report = {'host_zero_to_settled_seconds': brake_seconds, 'wheel_delta_counts': delta,
                                  'reverse_counts': reverse, 'ground_distance_measured': False,
                                  'pass': max(reverse.values()) <= 4}
        if not client.last_stop_report['pass']:
            raise MotorError('피드백 정지 중 역방향 이동이 4 counts를 초과했습니다. 제동 계수를 재검토하세요.')
    return brake_samples + samples


def characterize(client, report):
    trials = []
    for direction in (1, -1):
        for pwm in (180, 300, 400, 493):
            before = client.latest
            started = time.monotonic()
            raw = client.calibration.commands(direction * pwm, direction * pwm, limit=999)
            client.command('ARM')
            client.command('BENCH' if pwm <= 250 else 'DRIVE', *raw, 2400)

            def check(sample):
                if sample['invalid'] != before['invalid']:
                    raise MotorError('엔코더 A/B 전이 오류')
                if time.monotonic() - started > 0.6:
                    if any(d * direction < 10 for d in client.calibration.wheel_deltas(before, sample).values()):
                        raise MotorError('엔코더 회전 부족 또는 방향 불일치')

            samples = client.collect(2.1, heartbeat=True, sample_check=check)
            stop_samples = stopped(client)
            rpm = {name: statistics.mean(s['wheels'][name]['ticks_per_second'] * 60 / COUNTS_PER_REV for s in samples[-15:])
                   for name in ('left', 'right')}
            trials.append({'direction': direction, 'pwm': pwm, 'rpm': rpm, 'samples': samples, 'stopped': stop_samples})
            save(report / 'characterization.json', trials)
            print(f'측정 PWM {direction * pwm:+d}: L={rpm["left"]:.2f}, R={rpm["right"]:.2f} RPM', flush=True)
    return trials


def candidate(trials, calibration, report, *, period_ms=1):
    if sorted((t['direction'], t['pwm']) for t in trials) != sorted((d, p) for d in (-1, 1) for p in (180, 300, 400, 493)):
        raise ValueError('정·역회전 출력 단계별 실측 자료가 모두 필요합니다.')
    doc = {**control_defaults(), 'period_ms': period_ms,
           'pwm_limit': 650, 'min_rpm': 30, 'max_rpm': 100, 'default_rpm': 72, 'wheel_mapping': calibration.wheels,
           'evidence_directory': str(report), 'conditions': 'Both wheels raised, no external load',
           'method': 'Per-wheel/direction measured feedforward + conservative PI; derivative gain zero', 'wheels': {}}
    for name in ('left', 'right'):
        doc['wheels'][name] = {}
        for label, sign in (('forward', 1), ('reverse', -1)):
            selected = [t for t in trials if t['direction'] == sign]
            if any(t['rpm'][name] * sign <= 0 for t in selected):
                raise ValueError('실측 회전 방향이 보정 설정과 다릅니다.')
            doc['wheels'][name][label] = fit_feedforward([(t['rpm'][name], t['pwm']) for t in selected])
    return doc


def validate(client, settings, report, targets=None):
    trial_doc = copy.deepcopy(settings.document)
    if 'hold' in trial_doc:
        trial_doc['hold']['enabled'] = False
    PIDSettings(trial_doc, settings.calibration, require_validated=False).configure(client)
    trials = []
    if targets is None:
        targets = [(n, n) for n in (30, 50, 72, settings.max_rpm, -30, -50, -72, -settings.max_rpm)]
        targets += [(-50, 50), (50, -50), (36, 72), (72, 36), (-36, -72), (-72, -36),
                    (15, 30), (30, 15), (-15, -30), (-30, -15)]
    for target in targets:
        raw = settings.commands(*(round(v * 100) for v in target))
        before = client.latest
        client.command('ARM')
        client.set_speed(*raw, wait=True)
        start = time.monotonic()

        def check(sample):
            if sample['invalid'] != before['invalid']:
                raise MotorError('PID 검증 중 엔코더 A/B 오류')
            if time.monotonic() - start > 0.2 and (not sample.get('pid_mode') or sample.get('target_rpm_centi') != raw):
                raise MotorError('메가의 PID 목표 속도가 요청과 다릅니다.')

        samples = client.collect(3.5, target=raw, sample_check=check)
        stop_samples = stopped(client)
        tail = samples[-20:]
        if len(tail) < 15:
            raise MotorError('PID 정상 수신 샘플 부족')
        metrics = {}
        for name, goal in zip(('left', 'right'), target):
            actual = [settings.wheel_speeds(s)[name]['rpm'] for s in tail]
            mean = statistics.mean(actual)
            rms = statistics.mean((v - goal) ** 2 for v in actual) ** 0.5
            metrics[name] = {'target_rpm': goal, 'mean_rpm': mean, 'mean_error_rpm': mean - goal,
                             'rms_error_rpm': rms, 'mean_pwm': statistics.mean(settings.wheel_speeds(s)[name]['pwm'] for s in tail)}
        passed = all(abs(m['mean_error_rpm']) <= max(1, abs(m['target_rpm']) * 0.015) and
                     m['rms_error_rpm'] <= max(2, abs(m['target_rpm']) * 0.05) for m in metrics.values())
        mismatch = abs((metrics['left']['mean_rpm'] - metrics['right']['mean_rpm']) - (target[0] - target[1]))
        passed = passed and mismatch <= 1.5
        trials.append({'target_rpm': target, 'metrics': metrics, 'tracking_difference_rpm': mismatch,
                       'pass': passed, 'samples': samples, 'stopped': stop_samples,
                       'braking': client.last_stop_report})
        save(report / 'validation.json', trials)
        print(f'PID 목표 {target}: L={metrics["left"]["mean_rpm"]:.2f}, R={metrics["right"]["mean_rpm"]:.2f} RPM '
              f'{"PASS" if passed else "FAIL"}', flush=True)
        if not passed:
            raise SpeedValidationError(trials[-1])
    return trials


def validate_autotune(client, doc, original, calibration, report, *, failed_trial=None):
    """Veto a candidate per wheel/direction, then rerun the complete motion set.

    Only measured speed-quality failures permit fallback. Communication,
    encoder, firmware and stop failures abort without retrying motion.
    """
    from motor_autotune import rollback_failed_channels
    for attempt in range(5):  # Four independently tuned wheel/direction pairs.
        if failed_trial is not None:
            doc = rollback_failed_channels(doc, original, failed_trial)
            save(report / 'final_validation_rejections.json', doc['autotune']['final_validation_rejections'])
            save(report / 'confirmation.json', doc['autotune']['comparison'])
            print('AUTO 최종 속도 검증 탈락 방향은 기존 계수로 복구하고 다시 검증합니다.', flush=True)
        folder = report / 'final_validation' / f'attempt_{attempt}'
        folder.mkdir(parents=True)
        save(folder / 'candidate_pid.json', doc)
        try:
            checks = validate(client, PIDSettings(doc, calibration, require_validated=False), folder)
        except SpeedValidationError as exc:
            failed_trial = exc.trial
            continue
        save(report / 'validation.json', checks)
        return doc, checks
    raise MotorError('PID 오토튠의 최종 검증을 통과하지 못했습니다. 기존 보정 파일을 유지합니다.')


def main():
    parser = argparse.ArgumentParser(description='바퀴를 띄운 상태에서 모터 PI/PID 보정과 실측 검증 (실제 구동)')
    parser.add_argument('--max-rpm', type=int, default=95, choices=range(72, 101), help='보정할 상한 RPM, 기본 95 / 절대 상한 100')
    parser.add_argument('--clear-fault', action='store_true', help='원인 해소 후 메가 오류 명시적 해제')
    parser.add_argument('--port', default=DEFAULT_MOTOR_PORT)
    parser.add_argument('--calibration', type=Path, default=DEFAULT_CALIBRATION)
    parser.add_argument('--output', type=Path, default=DEFAULT_PID_CONFIG)
    parser.add_argument('--report-dir', type=Path, default=WORKSPACE / 'reports')
    parser.add_argument('--characterization', type=Path, help='같은 배선에서 이미 측정한 출력/RPM 자료를 재사용')
    parser.add_argument('--autotune', action='store_true', help='실제 응답으로 Kp/Ki/Kd를 반복 탐색하고 기존 값과 비교 (약 6분)')
    parser.add_argument('--pid-config', type=Path, default=DEFAULT_PID_CONFIG, help='오토튠 시작점인 검증된 보정 파일')
    args = parser.parse_args()
    if args.autotune and args.characterization:
        parser.error('--autotune과 --characterization은 함께 사용할 수 없습니다.')
    report = args.report_dir / ('pid_tune_' + datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    report.mkdir(parents=True)
    result = {'status': 'FAIL', 'report': str(report), 'error': None}

    def interrupt(*_):
        raise KeyboardInterrupt

    for signum in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(signum, interrupt)
    code = 1
    try:
        calibration = DriveCalibration.load(args.calibration)
        initial = PIDSettings.load(args.pid_config, calibration).document if args.autotune else None
        existing = initial
        if existing is None and args.output.exists():
            try:
                existing = PIDSettings.load(args.output, calibration, require_validated=False).document
            except Exception:
                existing = None
        with MotorClient(args.port, report, calibration=calibration) as client:
            result['firmware'] = client.info
            if not client.info.get('stream_supported'):
                raise MotorError('메가 v8 펌웨어가 필요합니다.')
            if args.clear_fault:
                client.clear_fault()
            if client.latest['fault']:
                raise MotorError(client.latest['fault_reason'] + ' (원인 해소 후 --clear-fault)')
            if args.autotune:
                from motor_autotune import autotune
                doc = autotune(client, initial, report, stopped)
            else:
                trials = json.loads(args.characterization.read_text()) if args.characterization else characterize(client, report)
                if args.characterization:
                    save(report / 'characterization.json', trials)
                doc = candidate(trials, calibration, report, period_ms=client.info['pid_period_ms'])
            doc['max_rpm'] = min(doc['max_rpm'], args.max_rpm)
            save(report / 'candidate_pid.json', doc)
            settings = PIDSettings(doc, calibration, require_validated=False)
            if args.autotune:
                save(report / 'confirmation_before_validation.json', doc['autotune']['comparison'])
                doc, checks = validate_autotune(client, doc, initial, calibration, report)
            else:
                checks = validate(client, settings, report)
            result['final_timing'] = client.timing
            result['receive_diagnostic'] = client.receive_diagnostic
            result['disabled_verified'] = not client.latest['armed'] and client.latest['command_pwm'] == [0, 0]
            save(report / 'timing.json', list(client.timing_samples))
            save(report / 'receive_diagnostics.json', list(client.receive_samples))
            if not result['disabled_verified']:
                raise MotorError('보정 후 출력 해제를 확인하지 못했습니다.')
            if client.receive_diagnostic and any(client.receive_diagnostic[k] for k in ('frame_errors', 'crc_errors', 'sequence_errors')):
                raise MotorError('보정 중 명령 수신 오류가 기록되었습니다.')
        doc['validated'] = True
        prev_status = (existing or {}).get('validation_status', {})
        doc['validation_status'] = {
            'driving': True,
            'braking': True,
            'holding': bool(prev_status.get('holding', False)),
            'ground': bool(prev_status.get('ground', False)),
        }
        if existing and 'hold' in existing:
            doc['hold'] = copy.deepcopy(existing['hold'])
        else:
            doc['hold']['enabled'] = False
        if existing and 'hold_validation' in existing:
            doc['hold_validation'] = copy.deepcopy(existing['hold_validation'])
        doc['conditions'] = (existing or {}).get(
            'conditions',
            'v8 unloaded driving and zero-speed braking; hold and ground validation pending',
        )
        doc['validated_at'] = datetime.now().astimezone().isoformat()
        doc['validation'] = [{k: v for k, v in t.items() if k not in ('samples', 'stopped')} for t in checks]
        PIDSettings(doc, calibration)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        if args.output.exists():
            shutil.copy2(args.output, report / 'previous_pid.json')
        temporary = args.output.with_suffix('.tmp')
        save(temporary, doc)
        temporary.replace(args.output)
        save(report / 'calibrated_pid.json', doc)
        result.update(status='PASS', output=str(args.output), validation=doc['validation'])
        code = 0
    except KeyboardInterrupt:
        result['error'] = '사용자 중단'; code = 130
    except Exception as exc:
        result['error'] = f'{type(exc).__name__}: {exc}'
    save(report / 'summary.json', result)
    print(json.dumps({k: v for k, v in result.items() if k != 'validation'}, ensure_ascii=False), flush=True)
    return code


if __name__ == '__main__':
    raise SystemExit(main())
