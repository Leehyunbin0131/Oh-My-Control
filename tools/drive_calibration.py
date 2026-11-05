"""Physical wheel sides and forward signs, separate from raw MCU channel order."""

import json
from pathlib import Path

DEFAULT_CALIBRATION = Path(__file__).resolve().parents[1] / 'config/drive_calibration.json'
ENCODER_PINS = ([40, 41], [50, 51])


class DriveCalibration:
    def __init__(self, document):
        if not isinstance(document, dict) or document.get('schema_version') != 1 or document.get('confirmed_by_user') is not True:
            raise ValueError('사용자 확인을 마친 방향 보정 설정(version 1)이 필요합니다.')
        if not isinstance(document.get('front_reference'), str) or not document['front_reference'].strip():
            raise ValueError('로봇 전진 방향 기준이 없습니다.')
        wheels = document.get('wheels', {})
        if not isinstance(wheels, dict) or set(wheels) != {'left', 'right'}:
            raise ValueError('left/right 바퀴 설정이 모두 필요합니다.')
        for spec in wheels.values():
            if not isinstance(spec, dict):
                raise ValueError('잘못된 바퀴 설정입니다.')
            index = spec.get('encoder_index')
            if type(index) is not int or index not in (0, 1) or spec.get('encoder_pins') != ENCODER_PINS[index]:
                raise ValueError('엔코더 순서와 핀 설정이 일치하지 않습니다.')
            if spec.get('driver_channel') not in ('L', 'R'):
                raise ValueError('드라이버 채널은 L 또는 R입니다.')
            for field in ('forward_command_sign', 'forward_encoder_sign'):
                if type(spec.get(field)) is not int or spec[field] not in (-1, 1):
                    raise ValueError('모터와 엔코더 방향 부호는 -1 또는 +1입니다.')
        if len({s['driver_channel'] for s in wheels.values()}) != 2 or len({s['encoder_index'] for s in wheels.values()}) != 2:
            raise ValueError('두 바퀴에 같은 모터/엔코더를 지정할 수 없습니다.')
        self.document = document
        self.wheels = wheels

    @classmethod
    def load(cls, path):
        try:
            return cls(json.loads(Path(path).read_text()))
        except FileNotFoundError as exc:
            raise ValueError(f'방향 설정이 없습니다: {path}. 원시 채널 검사에는 --raw-motors를 사용하세요.') from exc

    def commands(self, left, right, *, limit=250):
        output = [0, 0]  # wire protocol order is driver L, driver R
        for name, pwm in (('left', left), ('right', right)):
            if type(pwm) is not int or not -limit <= pwm <= limit:
                raise ValueError(f'PWM은 {-limit}..{limit} 정수 범위입니다.')
            spec = self.wheels[name]
            output[0 if spec['driver_channel'] == 'L' else 1] = pwm * spec['forward_command_sign']
        return output

    def wheel_data(self, sample):
        result = {}
        for name, spec in self.wheels.items():
            index = spec['encoder_index']
            result[name] = {
                'encoder_pins': spec['encoder_pins'], 'driver_channel': spec['driver_channel'],
                'count': sample['counts'][index] * spec['forward_encoder_sign'],
                'ticks_per_second': sample['ticks_per_second'][index] * spec['forward_encoder_sign'],
                'raw_count': sample['counts'][index], 'ab': sample['ab'][index],
                'invalid': sample['invalid'][index],
            }
        return result

    def wheel_deltas(self, before, after):
        return {name: (after['counts'][spec['encoder_index']] - before['counts'][spec['encoder_index']]) *
                      spec['forward_encoder_sign'] for name, spec in self.wheels.items()}


def calibration_for(args):
    return None if args.raw_motors else DriveCalibration.load(args.drive_calibration)
