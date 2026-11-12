"""Shared Mega serial protocol and connection for control and diagnostics.

Wire commands use driver channel order; DriveCalibration and PIDSettings
translate physical wheel directions and units at the host boundary.
Importing this module does not open a serial port or start a ROS node.
"""

import binascii
from collections import deque
import json
from pathlib import Path
import time

from serial_port import check_serial_port


DEFAULT_MOTOR_PORT = '/dev/serial/by-id/usb-Arduino__www.arduino.cc__0042_24235313336351619002-if00'
FAULTS = {0: '', 1: '호스트 통신 watchdog 정지', 2: '모터드라이버 통신 실패', 3: '잘못된 명령/순서',
          4: 'PID 엔코더 회전 중단', 5: 'PID 방향 오류/과속', 6: 'PID 엔코더 A/B 전이 오류',
          7: 'PID 제어 주기 지연', 8: 'PID 출력 한계에서 목표 속도 도달 실패',
          9: '목표 속도 유효기간 만료', 10: '위치 유지 한도 초과', 11: '100 RPM 과속 감시 정지'}
MAX_RPM = 100
CONTROL_MODEL = 'signed_pwm_v1'
WHEEL_STATES = ('DISABLED', 'TRACK', 'ZERO_SPEED', 'HOLD')
LINKS = {0: 'encoder_only', 1: 'i2c', 2: 'rs232_serial1'}


class MotorError(RuntimeError):
    pass


class MotorCancelled(MotorError):
    pass


def signed_delta(new, old):
    return ((new - old + 0x80000000) & 0xffffffff) - 0x80000000


def frame(body):
    encoded = body.encode('ascii')
    return encoded + f'*{binascii.crc_hqx(encoded, 0xffff):04X}\n'.encode('ascii')


class Decoder:
    def __init__(self):
        self.buffer = bytearray()
        self.errors = 0

    def feed(self, data):
        self.buffer.extend(data)
        result = []
        while b'\n' in self.buffer:
            raw, _, rest = self.buffer.partition(b'\n')
            self.buffer = bytearray(rest)
            try:
                body, crc = raw.rstrip(b'\r').rsplit(b'*', 1)
                if len(raw) > 256 or len(crc) != 4 or int(crc, 16) != binascii.crc_hqx(body, 0xffff):
                    raise ValueError('checksum')
                fields = body.decode('ascii').split(',')
                if fields[0] != 'OMR1':
                    raise ValueError('identity')
                result.append(fields)
            except (ValueError, UnicodeError):
                self.errors += 1
        if len(self.buffer) > 256:
            self.buffer.clear()
            self.errors += 1
        return result


def parse_info(fields):
    if len(fields) not in (14, 15, 19, 21, 23, 25, 27) or fields[:2] != ['OMR1', 'I']:
        raise MotorError('펌웨어 식별 프레임 형식 오류')
    values = [int(value) for value in fields[2:]]
    version = values[0]
    if ((version, len(fields)) not in ((2, 14), (3, 15), (4, 19), (5, 21), (6, 23), (7, 25), (8, 27)) or
            values[1:5] != [40, 41, 50, 51] or values[5:9] != [20000, 300, 1500, 250] or
            values[11] != 30000 or (version >= 3 and values[12] != (650 if version == 8 else 999)) or
            (version == 4 and values[13:] != [2652, 50, 12000, 650]) or
            (version >= 5 and (values[13:17] != [2652, 1, 10000 if version == 8 else 12000, 650] or
                               values[17] not in (0, 100000, 400000) or values[18] not in (50, 500, 1000))) or
            (version >= 6 and any(sign not in (-1, 1) for sign in values[19:21])) or
            (version == 7 and (values[21:23] not in ([0, 0], [150, 60]) or
                              (values[21] and (values[9] != 1 or values[17] != 400000)))) or
            (version == 8 and (values[21:24] != [0, 0, 1] or values[24] not in (7, 15, 31)))):
        raise MotorError('펌웨어 버전·엔코더 핀·구동 제한이 이 검사 코드와 다릅니다.')
    if values[9] not in LINKS or not 1 <= values[10] <= 127:
        raise MotorError('잘못된 모터드라이버 설정')
    return {'protocol': version, 'pins': [[40, 41], [50, 51]], 'sample_hz': values[5],
            'watchdog_ms': values[6], 'max_pulse_ms': values[7], 'max_pwm': values[8],
            'link': values[9], 'driver_link': LINKS[values[9]], 'i2c_address': values[10],
            'max_bench_ms': values[11], 'max_drive_pwm': values[12] if version >= 3 else 250,
            'pid_supported': version >= 4,
            'encoder_cpr': values[13] if version >= 4 else None,
            'pid_period_ms': values[14] if version >= 4 else None,
            'max_rpm_centi': values[15] if version >= 4 else None,
            'max_pid_pwm': values[16] if version >= 4 else None,
            'driver_i2c_hz': values[17] if version >= 5 else None,
            'max_driver_output_hz': values[18] if version >= 5 else None,
            'pid_encoder_signs': values[19:21] if version >= 6 else [1, 1],
            'brake_supported': version >= 7 and values[21] > 0,
            'max_brake_pwm': values[21] if version >= 7 else 0,
            'max_brake_ms': values[22] if version >= 7 else 0,
            'stream_supported': version == 8, 'hold_supported': version == 8,
            'hold_servo_supported': version == 8 and bool(values[24] & 8),
            'extended_hold_supported': version == 8 and bool(values[24] & 16),
            'control_model': CONTROL_MODEL if version == 8 else 'legacy_unsigned_pwm'}


def parse_brake_report(fields):
    if len(fields) != 12 or fields[:2] != ['OMR1', 'B']:
        raise MotorError('제동 결과 프레임 형식 오류')
    v = [int(x) for x in fields[2:]]
    if (not 1 <= v[0] <= 0xffffffff or not 0 <= v[1] <= 1000000 or
            any(not -0x80000000 <= n <= 0x7fffffff for n in v[2:4]) or
            any(not 0 <= n <= v[1] for n in v[4:6]) or
            any(not 1 <= n <= 8 for n in v[6:8]) or any(not 0 <= n <= 150 for n in v[8:10])):
        raise MotorError('제동 결과 값 범위 오류')
    return {'sequence': v[0], 'elapsed_us': v[1], 'raw_encoder_delta_driver_order': v[2:4],
            'wheel_finish_us': v[4:6], 'wheel_reason': v[6:8], 'peak_pwm': v[8:10]}


def parse_control_state(fields):
    if len(fields) != 17 or fields[:2] != ['OMR1', 'Q']:
        raise MotorError('v8 제어 상태 프레임 형식 오류')
    v = list(map(int, fields[2:]))
    if (any(not 0 <= v[i] <= 0xffffffff for i in (0, 1, 2, 3, 11, 12)) or
            any(not 0 <= v[i] <= 3 for i in (4, 5, 6)) or any(v[i] not in (0, 1) for i in (7, 8, 9, 10)) or
            any(not -0x80000000 <= n <= 0x7fffffff for n in v[13:15])):
        raise MotorError('v8 제어 상태 값 범위 오류')
    return {'stamp': v[0], 'applied_sequence': v[1], 'applied_tick': v[2], 'target_age_ms': v[3],
            'mode': ('DISABLED', 'ARMED', 'ACTIVE', 'FAULT')[v[4]],
            'wheel_states': [WHEEL_STATES[n] for n in v[5:7]], 'settled': list(map(bool, v[7:9])),
            'limited': list(map(bool, v[9:11])), 'hold_reference': v[11:13], 'position_error': v[13:15]}


def parse_control_timing(fields):
    names = ('stamp_us', 'control_ticks', 'active_updates', 'missed_ticks', 'period_min_us',
             'period_max_us', 'compute_max_us', 'driver_writes', 'driver_max_us', 'driver_gap_max_us',
             'driver_i2c_hz', 'tx_dropped')
    if len(fields) != 14 or fields[:2] != ['OMR1', 'P']:
        raise MotorError('PID 주기 진단 프레임 형식 오류')
    values = [int(v) for v in fields[2:]]
    if any(not 0 <= v <= 0xffffffff for v in values) or values[10] not in (0, 100000, 400000):
        raise MotorError('PID 주기 진단 값 범위 오류')
    return dict(zip(names, values))


def parse_receive_diagnostic(fields):
    names = ('stamp', 'buffer_size', 'max_queued', 'max_service_gap_us', 'frames',
             'frame_errors', 'crc_errors', 'sequence_errors')
    if len(fields) != 10 or fields[:2] != ['OMR1', 'R']:
        raise MotorError('명령 수신 진단 프레임 형식 오류')
    values = list(map(int, fields[2:]))
    if (any(not 0 <= n <= 0xffffffff for n in values) or
            not 1 <= values[1] <= 256 or values[2] >= values[1]):
        raise MotorError('명령 수신 진단 값 범위 오류')
    return dict(zip(names, values))


def parse_telemetry(fields, max_pwm=250):
    if len(fields) not in (15, 20) or fields[:2] != ['OMR1', 'T']:
        raise MotorError('엔코더 텔레메트리 형식 오류')
    v = [int(value) for value in fields[2:]]
    if any(not 0 <= v[i] <= 0xffffffff for i in (0, 1, 2, 5, 6, 11)):
        raise MotorError('엔코더 타임스탬프/카운트 범위 오류')
    if not all(0 <= v[i] <= 3 for i in (3, 4)) or v[9] not in (0, 1) or v[10] not in FAULTS or v[12] not in LINKS:
        raise MotorError('엔코더 상태 범위 오류')
    if any(abs(v[i]) > max_pwm for i in (7, 8)):
        raise MotorError('허용 범위를 넘는 모터 명령 상태')
    extra = {}
    if len(fields) == 20:
        if v[13] not in (0, 1) or any(abs(n) > 12000 for n in v[14:16]) or any(abs(n) > 30000 for n in v[16:18]):
            raise MotorError('PID 속도 텔레메트리 범위 오류')
        extra = {'pid_mode': bool(v[13]), 'target_rpm_centi': v[14:16], 'measured_rpm_centi': v[16:18]}
    return {'stamp': v[0], 'raw_counts': v[1:3], 'ab': v[3:5], 'invalid': v[5:7],
            'command_pwm': v[7:9], 'armed': bool(v[9]), 'fault': v[10],
            'fault_reason': FAULTS[v[10]], 'sequence': v[11], 'link': v[12], **extra}


class MotorClient:
    """Single-owner serial connection. A TUI worker owns all reads and writes."""
    def __init__(self, port, report_dir, startup_timeout=6, on_sample=None, serial_factory=None, boot_delay=2.0, calibration=None,
                 brake_on_exit=False):
        self.port = port
        self.report_dir = Path(report_dir)
        self.startup_timeout = startup_timeout
        self.on_sample = on_sample or (lambda sample: None)
        self.serial_factory = serial_factory
        self.boot_delay = boot_delay
        self.calibration = calibration
        self.brake_on_exit = brake_on_exit
        self.serial = self.log = None
        self.command_log = None
        self.decoder = Decoder()
        self.info = self.latest = None
        self.diagnostics = None
        self.identified = False
        self.sequence = 0
        self.acks = {}
        self.last_received = 0
        self.counts = [0, 0]
        self.samples = deque(maxlen=10000)
        self.control = None
        self.stream_started = False
        self.timing = None
        self.timing_samples = deque(maxlen=2000)
        self.receive_diagnostic = None
        self.receive_samples = deque(maxlen=2000)
        self.brake_reports = deque(maxlen=100)

    def __enter__(self):
        import serial

        if self.serial_factory is None:
            check_serial_port(self.port)
        self.log = (self.report_dir / 'motors_serial.log').open('ab')
        self.command_log = (self.report_dir / 'motors_commands.jsonl').open('a')
        try:
            self.serial = (self.serial_factory or serial.Serial)(
                self.port, 115200, timeout=0.02, write_timeout=0.15, exclusive=True)
            # Mega's USB bridge can deliver pre-reset bytes before its bootloader
            # finishes. Drain them, then require a fresh identity after startup.
            settle_until = time.monotonic() + self.boot_delay
            while time.monotonic() < settle_until:
                self.serial.read(max(1, min(2048, self.serial.in_waiting)))
            self.serial.reset_input_buffer()
            self.decoder = Decoder()
            deadline = time.monotonic() + self.startup_timeout
            # Opening USB serial can reset Mega. Listen for OUR identity first;
            # never send guessed commands to unknown, previously installed code.
            while time.monotonic() < deadline:
                self.poll()
                if self.info is not None and self.latest is not None:
                    self.identified = True
                    self.sequence = self.latest['sequence']
                    self.command('STOP')
                    return self
            raise MotorError('전용 메가 펌웨어 응답 없음: firmware/mega_motor_test를 먼저 업로드하세요.')
        except BaseException:
            self.close()
            raise

    def __exit__(self, exc_type, *_):
        try:
            if (self.brake_on_exit and exc_type in (None, KeyboardInterrupt) and self.stream_started and
                    self.latest and not self.latest['fault'] and time.monotonic() - self.last_received <= 0.4):
                try:
                    self.controlled_stop(timeout=1.0)
                except Exception:
                    pass  # close always attempts unconditional disable
        finally:
            self.close()

    def close(self):
        try:
            if self.serial is not None and self.identified:
                try:
                    # Best effort even when a read/protocol failure prevents an ACK.
                    self.send('STOP')
                except Exception:
                    pass
        finally:
            if self.serial is not None:
                self.serial.close()
                self.serial = None
            if self.log is not None:
                self.log.close()
                self.log = None
            if self.command_log is not None:
                self.command_log.close()
                self.command_log = None

    def poll(self, *, block=True):
        available = min(2048, self.serial.in_waiting)
        data = self.serial.read(max(1, available) if block else available)
        if data:
            self.log.write(data)
        errors = self.decoder.errors
        rejected = None
        for fields in self.decoder.feed(data):
            try:
                if self.info is None and fields[1] != 'I':
                    continue  # A USB open can start in the middle of the periodic telemetry stream.
                if fields[1] == 'I':
                    info = parse_info(fields)
                    if self.info is not None and self.info != info:
                        raise MotorError('실행 도중 펌웨어 설정 변경/재부팅')
                    self.info = info
                elif fields[1] == 'T':
                    sample = parse_telemetry(fields, self.info['max_drive_pwm'] if self.info else 250)
                    previous = self.latest
                    dt = None if previous is None else signed_delta(sample['stamp'], previous['stamp'])
                    if dt is not None and dt <= 0:
                        raise MotorError('메가 타임스탬프 반복/역행 또는 재부팅')
                    delta = [0, 0] if previous is None else [signed_delta(n, p) for n, p in
                                                                           zip(sample['raw_counts'], previous['raw_counts'])]
                    self.counts = [count + d for count, d in zip(self.counts, delta)]
                    sample['counts'] = self.counts[:]
                    sample['ticks_per_second'] = [d * 1000 / dt if dt else 0.0 for d in delta]
                    sample['received_at'] = time.monotonic()
                    if self.calibration is not None:
                        sample['wheels'] = self.calibration.wheel_data(sample)
                    if self.info and self.info['protocol'] == 8 and any(abs(v) > 10000 for v in sample.get('target_rpm_centi', [])):
                        raise MotorError('100 RPM 목표 상한을 넘는 펌웨어 상태')
                    sample['control'] = self.control
                    self.latest = sample
                    self.last_received = sample['received_at']
                    self.samples.append(sample)
                    self.on_sample(sample)
                elif fields[1] == 'P':
                    timing = parse_control_timing(fields)
                    timing['received_at'] = time.monotonic()
                    if not self.info or self.info['protocol'] < 5:
                        raise MotorError('지원하지 않는 펌웨어의 PID 주기 진단')
                    if self.timing is not None:
                        elapsed = signed_delta(timing['stamp_us'], self.timing['stamp_us'])
                        ticks = signed_delta(timing['control_ticks'], self.timing['control_ticks'])
                        if elapsed <= 0 or ticks <= 0:
                            raise MotorError('PID 하드웨어 타이머 진행 중단')
                        timing['measured_control_hz'] = ticks * 1000000 / elapsed
                        timing['measured_host_control_hz'] = ticks / (timing['received_at'] - self.timing['received_at'])
                        timing['measured_driver_hz'] = signed_delta(timing['driver_writes'], self.timing['driver_writes']) * 1000000 / elapsed
                    self.timing = timing
                    self.timing_samples.append(timing)
                elif fields[1] == 'Q':
                    if not self.info or self.info['protocol'] != 8:
                        raise MotorError('지원하지 않는 펌웨어의 v8 상태')
                    self.control = {**parse_control_state(fields), 'received_at': time.monotonic()}
                elif fields[1] == 'R':
                    if not self.info or self.info['protocol'] != 8:
                        raise MotorError('지원하지 않는 펌웨어의 수신 진단')
                    self.receive_diagnostic = {**parse_receive_diagnostic(fields), 'received_at': time.monotonic()}
                    self.receive_samples.append(self.receive_diagnostic)
                elif fields[1] == 'B':
                    if not self.info or self.info['protocol'] != 7:
                        raise MotorError('지원하지 않는 펌웨어의 제동 결과')
                    report = parse_brake_report(fields)
                    report['received_at'] = time.monotonic()
                    self.brake_reports.append(report)
                elif fields[1] == 'A' and len(fields) == 5:
                    self.acks[int(fields[2])] = fields[3:]
                    if self.identified and fields[3] != 'OK':
                        rejected = f'메가 명령 거부: {fields[4]}'
                        if fields[4] == 'hold_limit':
                            rejected += ' (fault 10: 정지 유지 위치 이탈 또는 연속 출력 포화 한도)'
                    while len(self.acks) > 32:
                        del self.acks[next(iter(self.acks))]
                elif fields[1] == 'D' and len(fields) == 6:
                    stamp, ticks, left, right = map(int, fields[2:])
                    if not 0 <= stamp <= 0xffffffff or not 0 <= ticks <= 0xffffffff or not 0 <= left <= 3 or not 0 <= right <= 3:
                        raise MotorError('엔코더 샘플링 진단 프레임 오류')
                    frequency = None
                    if self.diagnostics is not None:
                        elapsed = signed_delta(stamp, self.diagnostics['stamp'])
                        count = signed_delta(ticks, self.diagnostics['sample_ticks'])
                        if elapsed <= 0 or count <= 0:
                            raise MotorError('메가 엔코더 타이머가 진행하지 않습니다.')
                        frequency = count * 1000 / elapsed
                        if not 18000 <= frequency <= 22000:
                            raise MotorError(f'메가 엔코더 샘플링 속도 이상: {frequency:.0f} Hz')
                    self.diagnostics = {'stamp': stamp, 'sample_ticks': ticks,
                                        'digital_ab': [left, right], 'measured_sample_hz': frequency}
                else:
                    raise MotorError('알 수 없는 메가 응답')
            except (ValueError, IndexError) as exc:
                raise MotorError(f'잘못된 메가 응답: {exc}') from exc
        if self.identified and self.decoder.errors != errors:
            raise MotorError('메가 USB 패킷 CRC/프레임 오류')
        if rejected:
            # Preserve telemetry/STOP ACKs already in this same read, including
            # the fault explaining why a later target was rejected.
            raise MotorError(rejected + (f' (fault {self.latest["fault"]})' if self.latest and self.latest['fault'] else ''))
        if self.identified and time.monotonic() - self.last_received > 0.4:
            raise MotorError('엔코더 수신 중단 (0.4초); 모터 정지')

    def send(self, operation, *values):
        if not self.identified:
            raise MotorError('식별되지 않은 펌웨어에 명령을 보낼 수 없습니다.')
        if operation in ('ARM', 'RUN', 'BENCH', 'DRIVE', 'SPEED', 'SET_SPEED') and not self.info.get('stream_supported'):
            raise MotorError('100 RPM 제한과 연속 제어를 지원하는 메가 v8 펌웨어가 필요합니다.')
        if operation in ('SPEED', 'SET_SPEED'):
            if (len(values) != 3 or any(type(n) is not int for n in values) or
                    any(abs(n) > 10000 for n in values[:2]) or not 100 <= values[2] <= (300 if operation == 'SET_SPEED' else 30000)):
                raise MotorError('속도 명령은 ±100 RPM 및 허용 유효기간 안이어야 합니다.')
        if operation in ('RUN', 'BENCH', 'DRIVE') and (len(values) != 3 or
                any(type(n) is not int for n in values) or any(abs(n) > (650 if operation == 'DRIVE' else 250) for n in values[:2])):
            raise MotorError('진단 PWM 상한 초과')
        if operation == 'BRAKE':
            raise MotorError('BRAKE는 제거되었습니다. 정상 정지는 SET_SPEED 0 0을 사용하세요.')
        if operation == 'STOP':
            self.stream_started = False
        self.sequence = (self.sequence + 1) & 0xffffffff or 1
        encoded = frame(','.join(map(str, ('OMR1', 'C', self.sequence, operation, *values))))
        if self.serial.write(encoded) != len(encoded):
            raise MotorError('모터 명령 전송 중단')
        if self.command_log is not None:
            self.command_log.write(json.dumps({'sent_at': time.monotonic(), 'sequence': self.sequence,
                                               'operation': operation, 'values': values, 'bytes': len(encoded)}) + '\n')
        return self.sequence

    def command(self, operation, *values):
        seq = self.send(operation, *values)
        deadline = time.monotonic() + 0.25
        while time.monotonic() < deadline:
            self.poll()
            if seq in self.acks:
                status, reason = self.acks.pop(seq)
                if status != 'OK':
                    raise MotorError(f'{operation} 거부: {reason}')
                return seq
        raise MotorError(f'{operation} 응답 시간 초과')

    def set_speed(self, left_centi, right_centi, lease_ms=300, *, wait=False):
        if not self.info.get('stream_supported'):
            raise MotorError('연속 속도 제어에는 메가 v8이 필요합니다.')
        seq = (self.command if wait else self.send)('SET_SPEED', left_centi, right_centi, lease_ms)
        self.stream_started = True
        return seq

    def clear_fault(self):
        if not self.info.get('stream_supported'):
            raise MotorError('오류 해제에는 메가 v8 펌웨어가 필요합니다.')
        self.command('STOP')
        until = time.monotonic() + .12
        while time.monotonic() < until:
            self.poll()
            if self.latest['armed'] or any(self.latest['command_pwm']):
                raise MotorError('오류 해제 전에 출력 해제를 확인할 수 없습니다.')
        self.command('CLEAR_FAULT')
        self.collect(0.12)

    def controlled_stop(self, timeout=1.0):
        """Zero servo while this bounded operation owns intent; ACK is not standstill."""
        if not self.stream_started:
            self.command('STOP')
            return []
        began = time.monotonic()
        first = self.set_speed(0, 0)
        next_update = began + .02
        samples = []
        previous = self.latest
        try:
            while time.monotonic() - began < timeout:
                if time.monotonic() >= next_update:
                    self.set_speed(0, 0)
                    next_update = time.monotonic() + .02
                self.poll()
                if self.latest['fault']:
                    raise MotorError(self.latest['fault_reason'])
                if self.latest is not previous:
                    samples.append(self.latest); previous = self.latest
                state = self.control
                if (state and state['received_at'] >= began and signed_delta(state['applied_sequence'], first) >= 0 and
                        state['mode'] == 'ACTIVE' and all(state['settled']) and
                        self.latest.get('target_rpm_centi') == [0, 0] and
                        all(abs(v) <= 100 for v in self.latest.get('measured_rpm_centi', [30000, 30000]))):
                    return samples
            raise MotorError('0 RPM 정지 확인 시간 초과')
        except BaseException:
            self.send('STOP')
            raise

    def collect(self, seconds, *, heartbeat=False, target=None, cancelled=lambda: False, sample_check=lambda sample: None):
        start = time.monotonic()
        next_ping = start
        collected = []
        previous = self.latest
        while time.monotonic() - start < seconds:
            if cancelled():
                raise MotorCancelled('모터 검사 중단')
            if time.monotonic() >= next_ping:
                if target is not None:
                    self.set_speed(*target)
                elif heartbeat:
                    self.send('PING')
                next_ping = time.monotonic() + (0.02 if target is not None else 0.08)
            self.poll()
            if self.latest is not previous:
                collected.append(self.latest)
                previous = self.latest
                if self.latest['fault']:
                    raise MotorError(self.latest['fault_reason'])
                sample_check(self.latest)
        return collected
