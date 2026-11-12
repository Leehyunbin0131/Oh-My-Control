"""Validated wheel speed calibration and MCU PID configuration (protocol 8)."""

import json
import math
from pathlib import Path

from motor_link import MotorError, CONTROL_MODEL, MAX_RPM

DEFAULT_PID_CONFIG = Path(__file__).resolve().parents[1] / 'config/motor_pid.json'
COUNTS_PER_REV = 2652
GAIN_NAMES = ('kp', 'ki', 'kd', 'slope', 'offset')
GAIN_LIMITS = {'kp': (0, 20), 'ki': (0, 50), 'kd': (0, 2), 'slope': (0.001, 20), 'offset': (0, 200)}


def control_defaults():
    """Commissioning defaults. Driving calibration starts with HOLD disabled."""
    return {'schema_version': 2, 'control_model': CONTROL_MODEL, 'validated': False,
            'encoder_cpr': COUNTS_PER_REV, 'period_ms': 1, 'pwm_limit': 650,
            'min_rpm': 30, 'max_rpm': MAX_RPM, 'default_rpm': 72,
            'validation_status': {'driving': False, 'braking': False, 'holding': False},
            'zero_control': {name: {'kp': 2.0, 'ki': 0.0, 'pwm_limit': 150} for name in ('left', 'right')},
            'hold': {'enabled': False, 'kp_rpm_per_count': .25, 'max_rpm': 5,
                     'velocity_kp': 8.0, 'velocity_ki': 10.0,
                     'pwm_limit': 120, 'deadband_counts': 2, 'max_error_counts': 100,
                     'max_duration_ms': 10000, 'saturation_ms': 1000}}


class PIDSettings:
    def __init__(self, document, calibration, *, require_validated=True):
        if not isinstance(document, dict) or document.get('schema_version') != 2 or document.get('control_model') != CONTROL_MODEL:
            raise ValueError('v8 signed_pwm_v1 / schema 2 PID 설정이 필요합니다. 구형 검증값은 새 제어기에 적용할 수 없습니다.')
        if require_validated and document.get('validated') is not True:
            reason = document.get('invalidation_reason', '')
            raise ValueError('실측 검증을 완료한 PID 보정 파일이 필요합니다. ' +
                             (reason + ' ' if reason else '') +
                             '모터 전원을 켜고 바퀴를 띄운 뒤 ./tune_motors.sh로 새 제어기를 검증하세요.')
        if document.get('encoder_cpr') != COUNTS_PER_REV or document.get('period_ms') != 1 or document.get('pwm_limit') != 650:
            raise ValueError('PID 엔코더/주기/출력 한도가 메가 설정과 다릅니다.')
        if document.get('wheel_mapping') != calibration.wheels:
            raise ValueError('PID 보정 당시의 모터·엔코더 배선과 현재 방향 설정이 다릅니다.')
        self.document = document
        self.calibration = calibration
        self.period_ms = document['period_ms']
        self.min_rpm = 30  # Curves use half speed; 10 RPM was unstable on this driver.
        self.max_rpm = document.get('max_rpm')
        if type(self.max_rpm) not in (int, float) or not math.isfinite(self.max_rpm) or not self.min_rpm <= self.max_rpm <= MAX_RPM:
            raise ValueError('PID 최대 RPM 설정 오류')
        wheels = document.get('wheels', {})
        if not isinstance(wheels, dict) or set(wheels) != {'left', 'right'}:
            raise ValueError('PID 좌우 바퀴 보정값이 필요합니다.')
        for name, spec in calibration.wheels.items():
            expected_index = 1 if spec['driver_channel'] == 'L' else 0
            if spec['encoder_index'] != expected_index:
                raise ValueError('PID 펌웨어의 드라이버-엔코더 대응과 설정이 일치하지 않습니다.')
            directions = wheels[name]
            if not isinstance(directions, dict) or set(directions) != {'forward', 'reverse'}:
                raise ValueError('PID 정·역회전 보정값이 필요합니다.')
            for gains in directions.values():
                for key, (lo, hi) in GAIN_LIMITS.items():
                    value = gains.get(key) if isinstance(gains, dict) else None
                    if type(value) not in (int, float) or not math.isfinite(value) or not lo <= value <= hi:
                        raise ValueError(f'잘못된 PID 계수: {name}/{key}')

        validation = document.get('validation_status', {})
        if require_validated and not all(validation.get(k) is True for k in ('driving', 'braking')):
            raise ValueError('v8 주행·제동 실측 검증이 필요합니다.')
        self.zero = document.get('zero_control', {})
        for name in ('left', 'right'):
            params = self.zero.get(name, {})
            for key, lo, hi in (('kp', .001, 20), ('ki', 0, 50), ('pwm_limit', 1, 150)):
                value = params.get(key)
                if type(value) not in (int, float) or not math.isfinite(value) or not lo <= value <= hi:
                    raise ValueError(f'0 RPM 제어 설정 오류: {name}/{key}')
            if type(params['pwm_limit']) is not int:
                raise ValueError('제동 PWM 한도는 정수여야 합니다.')
        self.hold = document.get('hold', {})
        if type(self.hold.get('enabled')) is not bool:
            raise ValueError('위치 유지 enabled 설정이 필요합니다.')
        limits = (('kp_rpm_per_count', .01, 10), ('max_rpm', .01, 5), ('pwm_limit', 1, 650),
                  ('deadband_counts', 0, 20), ('max_error_counts', 1, 2652),
                  ('max_duration_ms', 0, 60000), ('saturation_ms', 100, 5000))
        for key, lo, hi in limits:
            value = self.hold.get(key)
            if type(value) not in (int, float) or not math.isfinite(value) or not lo <= value <= hi:
                raise ValueError(f'위치 유지 한도 설정 오류: {key}')
            if key not in ('kp_rpm_per_count', 'max_rpm') and type(value) is not int:
                raise ValueError(f'위치 유지 설정은 정수여야 합니다: {key}')
        if self.hold['max_error_counts'] <= self.hold['deadband_counts']:
            raise ValueError('최대 위치 이탈은 데드밴드보다 커야 합니다.')
        if 0 < self.hold['max_duration_ms'] < 100:
            raise ValueError('위치 유지 시간은 0(명령 유효기간으로 관리) 또는 100..60000 ms입니다.')
        if self.hold['enabled']:
            # Commissioning is allowed without claiming on-hardware validation.
            for key, lo, hi in (('velocity_kp', .001, 60), ('velocity_ki', 0, 50)):
                value = self.hold.get(key)
                if type(value) not in (int, float) or not math.isfinite(value) or not lo <= value <= hi:
                    raise ValueError(f'위치 유지 속도 PI 설정 오류: {key}')
        position_kp = self.hold.get('position_kp_pwm_per_count', 0)
        if (type(position_kp) not in (int, float) or not math.isfinite(position_kp)
                or not 0 <= position_kp <= 30):
            raise ValueError('위치 유지 직접 P 설정 오류: position_kp_pwm_per_count')

    @classmethod
    def load(cls, path, calibration, **kwargs):
        return cls(json.loads(Path(path).read_text()), calibration, **kwargs)

    def configure(self, client):
        if (not client.info.get('stream_supported') or client.info.get('control_model') != CONTROL_MODEL or
                client.info.get('max_rpm_centi') != 10000 or not client.info.get('pid_supported') or client.info.get('encoder_cpr') != COUNTS_PER_REV or
                client.info.get('pid_period_ms') != self.period_ms or client.info.get('max_pid_pwm') != 650):
            raise MotorError(f'보정 파일의 {self.period_ms} ms 주기와 메가 PID 펌웨어 설정이 일치해야 합니다.')
        if self.hold['enabled'] and not client.info.get('hold_servo_supported'):
            raise MotorError('정지 유지용 새 메가 펌웨어가 필요합니다. 모터 전원을 끄고 ./mega_firmware.sh upload로 갱신하세요.')
        if (self.hold['enabled'] and (self.hold['velocity_kp'] > 20 or self.hold['pwm_limit'] > 150
                                     or self.hold.get('position_kp_pwm_per_count', 0) > 0)
                and not client.info.get('extended_hold_supported')):
            raise MotorError('강한 위치 유지용 새 메가 펌웨어가 필요합니다. 모터 전원을 끄고 ./mega_firmware.sh upload로 갱신하세요.')
        expected_signs = [None, None]
        for spec in self.calibration.wheels.values():
            expected_signs[0 if spec['driver_channel'] == 'L' else 1] = (
                spec['forward_encoder_sign'] * spec['forward_command_sign'])
        if client.info.get('pid_encoder_signs', [1, 1]) != expected_signs:
            raise MotorError('메가의 PID 엔코더 방향과 현재 모터 배선이 다릅니다. 방향 수정 펌웨어를 업로드하세요.')
        if client.latest.get('fault'):
            raise MotorError('메가 오류가 남아 있습니다. 원인 확인 후 --clear-fault로 명시적으로 해제하세요.')
        if client.latest['armed'] or any(client.latest['command_pwm']):
            raise MotorError('PID 보정값은 정지 상태에서만 적용할 수 있습니다.')
        for name, spec in self.calibration.wheels.items():
            channel = 0 if spec['driver_channel'] == 'L' else 1
            for direction, sign in (('forward', 1), ('reverse', -1)):
                gains = self.document['wheels'][name][direction]
                encoded = [round(gains[key] * 1000) for key in GAIN_NAMES]
                client.command('PID', channel, sign * spec['forward_command_sign'], *encoded)
            z = self.zero[name]
            client.command('ZERO', channel, round(z['kp'] * 1000), round(z['ki'] * 1000), z['pwm_limit'])
            h = self.hold
            if h['enabled']:
                client.command('HOLD_PI', channel, round(h['velocity_kp'] * 1000), round(h['velocity_ki'] * 1000))
            if client.info.get('extended_hold_supported'):
                # Also clear a prior direct position term for a profile that
                # omits it; disarm alone intentionally preserves other gains.
                client.command('HOLD_POS', channel, round(h.get('position_kp_pwm_per_count', 0) * 1000))
            client.command('HOLD', channel, int(h['enabled']), round(h['kp_rpm_per_count'] * 100),
                           round(h['max_rpm'] * 100), h['pwm_limit'], h['deadband_counts'],
                           h['max_error_counts'], h['max_duration_ms'], h['saturation_ms'])

    def commands(self, left_centi, right_centi):
        return self.calibration.commands(left_centi, right_centi, limit=round(self.max_rpm * 100))

    def velocity_commands(self, left_rad_s, right_rad_s):
        """ROS hardware boundary: physical forward rad/s -> driver centi-RPM."""
        values = (left_rad_s, right_rad_s)
        if any(type(v) not in (int, float) or not math.isfinite(v) or
               abs(v) > self.max_rpm * math.pi / 30 for v in values):
            raise ValueError('바퀴 속도는 설정 상한(최대 100 RPM) 안의 유한한 rad/s여야 합니다.')
        return self.commands(*(round(v * 3000 / math.pi) for v in values))

    def joint_states(self, sample):
        """Measured encoder position/velocity, never command-derived odometry."""
        speeds = self.wheel_speeds(sample)
        return {name: {'position': sample['counts'][spec['encoder_index']] * spec['forward_encoder_sign'] *
                                  2 * math.pi / COUNTS_PER_REV,
                       'velocity': speeds[name]['rpm'] * math.pi / 30}
                for name, spec in self.calibration.wheels.items()}

    def wheel_speeds(self, sample):
        targets = sample.get('target_rpm_centi', [0, 0])
        measured = sample.get('measured_rpm_centi', [0, 0])
        return {name: {'target_rpm': targets[0 if spec['driver_channel'] == 'L' else 1] * spec['forward_command_sign'] / 100,
                       'rpm': measured[0 if spec['driver_channel'] == 'L' else 1] * spec['forward_encoder_sign'] / 100,
                       'pwm': sample['command_pwm'][0 if spec['driver_channel'] == 'L' else 1] * spec['forward_command_sign']}
                for name, spec in self.calibration.wheels.items()}


def fit_feedforward(points):
    """Fit PWM = slope * |RPM| + offset from independent steady measurements."""
    if len(points) < 3:
        raise ValueError('출력 단계가 3개 이상 필요합니다.')
    x = [abs(p[0]) for p in points]
    y = [p[1] for p in points]
    mx, my = sum(x) / len(x), sum(y) / len(y)
    denominator = sum((v - mx) ** 2 for v in x)
    if denominator < 100:
        raise ValueError('속도 측정 범위가 부족합니다.')
    slope = sum((a - mx) * (b - my) for a, b in zip(x, y)) / denominator
    offset = my - slope * mx
    if not 0.1 <= slope <= 20 or not -10 <= offset <= 200:
        raise ValueError('측정한 PWM/RPM 관계가 예상 범위를 벗어납니다.')
    offset = max(0, offset)
    residual = math.sqrt(sum((slope * a + offset - b) ** 2 for a, b in zip(x, y)) / len(x))
    if residual > 15:
        raise ValueError('출력과 속도의 관계가 불안정합니다. 기계적 간섭/전원을 점검하세요.')
    # Conservative PI: 0.75 s correction horizon, 0.12 s nominal motor response.
    # Measured feedforward supplies most output; derivative remains zero to
    # avoid amplifying encoder quantization. Hardware validation follows the fit.
    return {'kp': round(slope * 0.12 / 0.75, 3), 'ki': round(slope / 0.75, 3), 'kd': 0.0,
            'slope': round(slope, 3), 'offset': round(offset, 3)}
