"""Host wheel commands with one owner for timing, units and measured feedback.

The caller opens MotorClient, applies PIDSettings, and polls the connection.
This layer sends physical left/right RPM; it does not run another motor PID.
All calls belong to the same serial-owning thread.
"""

import math
import time

from motor_link import MotorError


class WheelDrive:
    SEND_PERIOD = 0.02
    COMMAND_TIMEOUT = 0.3
    FEEDBACK_TIMEOUT = 0.4

    def __init__(self, client, settings, *, clock=None):
        if client.calibration is None or not client.info.get('stream_supported'):
            raise MotorError('보정된 메가 v8 연결이 필요합니다.')
        if client.calibration.wheels != settings.calibration.wheels:
            raise MotorError('연결과 PID 설정의 바퀴 방향이 다릅니다.')
        self.client = client
        self.settings = settings
        self.clock = clock or (lambda: time.monotonic())
        self.target_centi = (0, 0)
        self.output = [0, 0]
        self.command_at = None
        self.last_sent_at = None
        self.last_sent_target = None
        self.engaged = False
        self.timed_out = False
        self.error = None

    def set_target_rpm(self, left, right):
        """Accept fresh intent in physical wheel order; zero is a braking target.

        A received command expires after 300 ms. Calling tick never refreshes
        its creation time. A latched keyboard operator deliberately supplies
        fresh intent on each iteration while that direction remains selected.
        """
        if self.error:
            raise MotorError(self.error)
        if any(type(v) not in (int, float) or not math.isfinite(v) or
               abs(v) > self.settings.max_rpm for v in (left, right)):
            self._fail('목표 RPM은 보정 상한 안의 유한한 숫자여야 합니다.')
        target = (round(left * 100), round(right * 100))
        self.output = self.settings.commands(*target)
        self.target_centi = target
        self.command_at = self.clock()
        self.timed_out = False

    def _fail(self, message):
        self.error = message
        try:
            self.disable()
        except Exception:
            pass  # The MCU's lease still expires if serial output is unavailable.
        raise MotorError(message)

    def check_feedback(self):
        """Reject missing/stale feedback and latched MCU faults before sending."""
        if self.error:
            raise MotorError(self.error)
        sample = self.client.latest
        age = None if sample is None else self.clock() - sample['received_at']
        if age is None or not 0 <= age <= self.FEEDBACK_TIMEOUT:
            self._fail('엔코더 수신 중단: 구동 출력을 해제합니다.')
        if sample['fault']:
            self._fail(sample['fault_reason'] or f'메가 오류 {sample["fault"]}')

    def tick(self, *, force=False):
        """Send at 50 Hz while the original command and feedback remain fresh."""
        self.check_feedback()
        now = self.clock()
        if self.command_at is None:
            return
        if now - self.command_at >= self.COMMAND_TIMEOUT:
            self.timed_out = True
            self.disable()
            return
        if not self.engaged:
            if not any(self.target_centi):
                return
            try:
                self.client.command('ARM')
            except Exception as exc:
                self._fail(f'메가 구동 준비 실패: {exc}')
            self.engaged = True
            # ARM waits for an ACK: do not send intent that expired in that wait.
            self.check_feedback()
            now = self.clock()
            if now - self.command_at >= self.COMMAND_TIMEOUT:
                self.timed_out = True
                self.disable()
                return

        # Do not keep extending an old command's MCU lease with each 50 Hz tick.
        # Firmware accepts leases >=100 ms, so leave the last lease to expire
        # when less than that remains. A new input starts a new 300 ms window.
        lease_ms = min(300, int((self.COMMAND_TIMEOUT - (now - self.command_at)) * 1000 + 1e-6))
        if lease_ms < 100:
            return
        if (force or self.last_sent_target is None or self.last_sent_at is None or
                now - self.last_sent_at >= self.SEND_PERIOD - 1e-9):
            try:
                self.client.set_speed(*self.output, lease_ms=lease_ms)
            except Exception as exc:
                self._fail(f'메가 속도 전송 실패: {exc}')
            self.last_sent_at = now
            self.last_sent_target = self.target_centi

    def stop(self):
        """Apply a zero target immediately; continuing HOLD needs fresh intent."""
        self.set_target_rpm(0, 0)
        if self.engaged:
            self.tick(force=True)
        else:
            self.disable()

    def disable(self):
        """Release output and forget intent. This never clears an MCU fault."""
        self.target_centi = (0, 0)
        self.output = [0, 0]
        self.command_at = None
        self.last_sent_target = None
        self.engaged = False
        self.client.send('STOP')

    def feedback(self):
        """Return reported wheel targets and measurements, never command odometry."""
        sample = self.client.latest
        if sample is None:
            return None
        if 'target_rpm_centi' not in sample or 'measured_rpm_centi' not in sample:
            raise MotorError('측정 속도가 포함된 메가 텔레메트리가 필요합니다.')
        now = self.clock()
        age = now - sample['received_at']
        speeds = self.settings.wheel_speeds(sample)
        joints = self.settings.joint_states(sample)
        wheels = {}
        for name, spec in self.settings.calibration.wheels.items():
            wheels[name] = {
                'encoder_count': sample['counts'][spec['encoder_index']] * spec['forward_encoder_sign'],
                'position_rad': joints[name]['position'],
                'target_rpm': speeds[name]['target_rpm'],
                'measured_rpm': speeds[name]['rpm'],
                'pwm': speeds[name]['pwm'],
            }
        return {
            'stamp_ms': sample['stamp'], 'received_at': sample['received_at'],
            'age_s': age, 'fresh': 0 <= age <= self.FEEDBACK_TIMEOUT,
            'armed': sample['armed'], 'fault': sample['fault'],
            'fault_reason': sample['fault_reason'], 'host_error': self.error,
            'command_timed_out': self.timed_out, 'wheels': wheels,
        }
