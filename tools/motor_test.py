"""Bounded motor diagnostics using the shared Mega connection.

The protocol names remain available here for existing callers. Control tools
import motor_link directly; this module owns diagnostic test procedures only.
"""

from collections import deque
import json
from pathlib import Path
import time

from drive_calibration import calibration_for
from motor_link import (
    DEFAULT_MOTOR_PORT, FAULTS, MAX_RPM, CONTROL_MODEL, WHEEL_STATES, LINKS,
    MotorError, MotorCancelled, signed_delta, frame, Decoder, parse_info,
    parse_brake_report, parse_control_state, parse_control_timing,
    parse_receive_diagnostic, parse_telemetry, MotorClient,
)


def observation_result(samples, require_motion=False, minimum_ticks=8):
    reasons = []
    if len(samples) < 3:
        reasons.append('정상 텔레메트리 3개 미만')
    encoders = {}
    for i, name in enumerate(('left', 'right')):
        calibrated = bool(samples and 'wheels' in samples[0])
        counts = [s['wheels'][name]['count'] if calibrated else s['counts'][i] for s in samples]
        span = max(counts) - min(counts) if counts else 0
        invalid_values = [s['wheels'][name]['invalid'] if calibrated else s['invalid'][i] for s in samples]
        invalid = (invalid_values[-1] - invalid_values[0]) & 0xffffffff if samples else 0
        name = name if calibrated else f'encoder_{i + 1}'
        if invalid:
            reasons.append(f'{name}: A/B 불가능한 전이 {invalid}회')
        seen_motion = span >= minimum_ticks
        if require_motion and not seen_motion:
            reasons.append(f'{name}: 회전 미관측 (손으로 바퀴를 돌려주세요)')
        encoders[name] = {'count_span': span, 'invalid_transitions': invalid,
                          'motion_observed': seen_motion,
                          'last_count': counts[-1] if counts else None}
    return {'status': 'FAIL' if reasons else 'PASS', 'encoders': encoders,
            'reason': '; '.join(reasons) if reasons else
                      ('양쪽 엔코더 카운트 변화 확인' if require_motion else
                       '메가 텔레메트리 수신 확인 (모터 구동·엔코더 회전 판정은 미실행)'),
            'scope': 'encoder_movement' if require_motion else 'telemetry_only',
            'motor_tested': False, 'messages': len(samples)}


def evaluate_pulse(before, samples, encoder, expected_pwm, minimum_ticks):
    if not samples:
        return {'status': 'FAIL', 'reason': '펄스 동안 수신 없음'}
    after = samples[-1]
    deltas = [after['counts'][i] - before['counts'][i] for i in range(2)]
    invalid = [(after['invalid'][i] - before['invalid'][i]) & 0xffffffff for i in range(2)]
    reasons = []
    if abs(deltas[encoder]) < minimum_ticks:
        reasons.append('해당 엔코더 회전 부족/모터 정지')
    if abs(deltas[1 - encoder]) > 4:
        reasons.append('반대쪽 엔코더도 움직임: 채널 매핑/배선 확인')
    if any(invalid):
        reasons.append('A/B 신호 전이 오류: 배선/샘플링 확인')
    if not any(s['command_pwm'] == expected_pwm and s['armed'] for s in samples):
        reasons.append('메가 구동 명령 상태 미확인')
    if after['command_pwm'] != [0, 0] or after['armed']:
        reasons.append('메가 출력 정지 미확인')
    # The final 250 ms must be stationary, so coasting/stuck commands do not PASS.
    tail = [s for s in samples if after['received_at'] - s['received_at'] <= 0.25]
    if len(tail) < 3 or any(max(s['counts'][i] for s in tail) - min(s['counts'][i] for s in tail) > 2 for i in range(2)):
        reasons.append('정지 후 엔코더가 계속 움직임/정지 수신 부족')
    return {'status': 'FAIL' if reasons else 'PASS', 'reason': '; '.join(reasons) or '구동·엔코더 변화·정지 확인',
            'encoder_delta': deltas, 'invalid_transitions': invalid}


def drive_test(client, args, *, cancelled=lambda: False, progress=lambda message: None):
    if not client.info['link']:
        raise MotorError('모터 배선 설정이 없습니다. mega_firmware.sh의 --mode를 실제 배선에 맞게 지정해 업로드하세요.')
    results = []
    signs = {}
    try:
        client.command('STOP')
        client.collect(0.6, cancelled=cancelled)
        for index, label in enumerate(('left', 'right')):
            spec = client.calibration.wheels[label] if client.calibration else None
            encoder = spec['encoder_index'] if spec else index
            channel = (0 if spec['driver_channel'] == 'L' else 1) if spec else (1 - encoder if args.motor_swap_channels else encoder)
            for direction in (1, -1):
                if cancelled():
                    raise MotorCancelled('모터 검사 중단')
                progress(f'{label} {"+" if direction > 0 else "-"} / PWM {args.motor_pwm} / {args.motor_pulse:g}s')
                before = client.latest
                output = [0, 0]
                output[channel] = direction * args.motor_pwm * (spec['forward_command_sign'] if spec else 1)
                client.command('ARM')
                client.command('RUN', *output, round(args.motor_pulse * 1000))
                # Include samples received while waiting for the RUN ACK.
                samples = [s for s in client.samples if s['received_at'] > before['received_at']]
                samples.extend(client.collect(args.motor_pulse + 0.1, heartbeat=True, cancelled=cancelled))
                client.command('STOP')
                samples.extend(client.collect(0.65, cancelled=cancelled))
                result = evaluate_pulse(before, samples, encoder, output, args.motor_min_ticks)
                result.update(encoder=label, driver_channel='L' if channel == 0 else 'R',
                              direction=direction, pwm=args.motor_pwm, pulse_seconds=args.motor_pulse,
                              physical_wheel=label if spec else None, encoder_index=encoder,
                              raw_command_pwm=output)
                if 'encoder_delta' in result:
                    delta = result['encoder_delta'][encoder]
                    if spec:
                        result['wheel_encoder_delta'] = delta * spec['forward_encoder_sign']
                        if direction * result['wheel_encoder_delta'] < args.motor_min_ticks:
                            result.update(status='FAIL', reason=result['reason'] + '; 보정된 엔코더의 전진/후진 부호 불일치')
                    if direction > 0:
                        signs[encoder] = delta
                    elif signs[encoder] * delta >= 0:
                        result.update(status='FAIL', reason=result['reason'] + '; 정·역회전 카운트 부호가 반전되지 않음')
                results.append(result)
                if result['status'] != 'PASS':
                    return {'status': 'FAIL', 'reason': f'{label}: {result["reason"]}',
                            'scope': 'powered_motor_encoder', 'motor_tested': True, 'pulses': results}
        return {'status': 'PASS', 'reason': '양쪽 모터의 양방향 회전·엔코더 방향 반전·정지 확인',
                'scope': 'powered_motor_encoder', 'motor_tested': True, 'pulses': results}
    finally:
        client.send('STOP')


def bench_test(client, args, *, cancelled=lambda: False, progress=lambda message: None):
    """Both motors at low duty for up to 30 s; abort if either encoder stalls."""
    if not client.info['link']:
        raise MotorError('구동 가능한 I2C/RS-232 펌웨어가 필요합니다.')
    seconds = args.motor_bench_seconds
    output = client.calibration.commands(args.motor_pwm, args.motor_pwm) if client.calibration else [args.motor_pwm, args.motor_pwm]
    samples = []
    active = []
    window = deque()
    last_progress = -3
    try:
        client.command('STOP')
        client.collect(0.6, cancelled=cancelled)
        before = client.latest

        def check(sample):
            nonlocal last_progress
            if any(signed_delta(n, p) != 0 for n, p in zip(sample['invalid'], before['invalid'])):
                raise MotorError('엔코더 A/B 전이 오류: 장시간 구동 중단')
            if sample['command_pwm'] == output and sample['armed']:
                active.append(sample)
                elapsed = signed_delta(sample['stamp'], active[0]['stamp']) / 1000
                window.append(sample)
                while len(window) > 1 and signed_delta(sample['stamp'], window[1]['stamp']) >= 800:
                    window.popleft()
                if elapsed >= 1 and signed_delta(sample['stamp'], window[0]['stamp']) >= 750:
                    for i, label in enumerate(('left', 'right')):
                        if abs(sample['counts'][i] - window[0]['counts'][i]) < 4:
                            raise MotorError(f'{label} 엔코더 회전이 멈췄습니다. 장시간 구동 중단')
                    if client.calibration:
                        if any(delta < 4 for delta in client.calibration.wheel_deltas(window[0], sample).values()):
                            raise MotorError('전진 구동 중 보정된 엔코더 방향 불일치')
                if elapsed - last_progress >= 3:
                    if client.calibration:
                        delta = client.calibration.wheel_deltas(before, sample)
                        progress(f'{elapsed:4.1f}/{seconds:g}s / LEFT={delta["left"]:+d}, RIGHT={delta["right"]:+d}')
                    else:
                        delta = [n - p for n, p in zip(sample['counts'], before['counts'])]
                        progress(f'{elapsed:4.1f}/{seconds:g}s / E1={delta[0]:+d}, E2={delta[1]:+d}')
                    last_progress = elapsed
            elif active and signed_delta(sample['stamp'], active[0]['stamp']) / 1000 < seconds - 0.15:
                raise MotorError('요청 시간보다 일찍 구동이 정지했습니다.')

        progress(f'두 모터 동시 구동: PWM {args.motor_pwm}/999, {seconds:g}초')
        client.command('ARM')
        client.command('BENCH', *output, round(seconds * 1000))
        samples = [s for s in client.samples if s['received_at'] > before['received_at']]
        for sample in samples:
            check(sample)
        samples.extend(client.collect(seconds + 0.1, heartbeat=True, cancelled=cancelled, sample_check=check))
        client.command('STOP')
        samples.extend(client.collect(0.8, cancelled=cancelled))
        after = samples[-1]
        delta = [n - p for n, p in zip(after['counts'], before['counts'])]
        invalid = [signed_delta(n, p) for n, p in zip(after['invalid'], before['invalid'])]
        span = signed_delta(active[-1]['stamp'], active[0]['stamp']) / 1000 if len(active) > 1 else 0
        tail = [s for s in samples if after['received_at'] - s['received_at'] <= 0.25]
        stopped = (len(tail) >= 3 and after['command_pwm'] == [0, 0] and not after['armed'] and
                   all(max(s['counts'][i] for s in tail) - min(s['counts'][i] for s in tail) <= 2 for i in range(2)))
        reasons = []
        if span < seconds - 0.15:
            reasons.append('연속 구동 시간 부족')
        if any(abs(d) < args.motor_min_ticks for d in delta):
            reasons.append('엔코더 회전 부족')
        if any(invalid):
            reasons.append('A/B 신호 오류')
        wheel_deltas = client.calibration.wheel_deltas(before, after) if client.calibration else None
        if wheel_deltas and any(delta < args.motor_min_ticks for delta in wheel_deltas.values()):
            reasons.append('보정된 엔코더의 전진 방향 불일치')
        if not stopped:
            reasons.append('구동 종료 후 정지 미확인')
        result = {'status': 'FAIL' if reasons else 'PASS',
                  'reason': '; '.join(reasons) or '양쪽 모터 연속 구동·엔코더 수신·정지 확인',
                  'scope': 'motor_bench_run', 'motor_tested': True,
                  'requested_seconds': seconds, 'command_sample_span_seconds': round(span, 3),
                  'pwm': output, 'encoder_delta': delta, 'invalid_transitions': invalid,
                  'wheel_encoder_delta': wheel_deltas, 'stopped': stopped, 'messages': len(samples)}
        (client.report_dir / 'motors_bench_samples.json').write_text(json.dumps(samples, indent=2) + '\n')
        return result
    finally:
        client.send('STOP')


def check_motors(args, report_dir):
    calibration = calibration_for(args)
    with MotorClient(args.motor_port, report_dir, args.startup_timeout, calibration=calibration) as client:
        if args.motor_test in ('drive', 'bench'):
            test = bench_test if args.motor_test == 'bench' else drive_test
            result = test(client, args, progress=lambda text: print('  ' + text, flush=True))
        else:
            if args.motor_test == 'encoders':
                print(f'  {args.duration:g}초 동안 양쪽 바퀴를 손으로 돌려주세요.', flush=True)
            samples = client.collect(args.duration)
            result = observation_result(samples, args.motor_test == 'encoders', args.motor_min_ticks)
        result.update(port=args.motor_port, firmware=client.info, crc_errors=client.decoder.errors,
                      sampling_diagnostics=client.diagnostics,
                      drive_calibration=calibration.document if calibration else None)
        (Path(report_dir) / 'motors.json').write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n')
        return result
