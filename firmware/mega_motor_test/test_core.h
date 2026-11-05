#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#ifdef __AVR__
#include <util/crc16.h>
#endif

// NT-S-DCDM1210 E is 100 us per PWM step. Bypass its ramp for both
// starting and stopping so keyboard taps are not consumed by acceleration.
struct DriverOutput {
  int8_t delaySetting = -1;
  bool write(bool (*send)(const char*), int16_t left, int16_t right) {
    const uint8_t delay = 0;
    bool configured = true;
    if (delaySetting != delay) {
      configured = send("<0E0>");
      delaySetting = configured ? delay : -1;
    }
    // A failed configuration must never prevent attempting zero output.
    if (!configured && (left || right)) return false;
    char command[] = "<0L1000R1000>";
    const uint16_t l = left < 0 ? -left : left, r = right < 0 ? -right : right;
    command[3] = left < 0 ? '0' : '1'; command[8] = right < 0 ? '0' : '1';
    command[4] += l / 100; command[5] += (l / 10) % 10; command[6] += l % 10;
    command[9] += r / 100; command[10] += (r / 10) % 10; command[11] += r % 10;
    const bool written = send(command);
    if (!written) delaySetting = -1;
    return configured && written;
  }
};

// Also compiled by the host regression tests; no Arduino dependency here.
inline uint16_t frameCrc(const char* data, size_t size) {
  uint16_t crc = 0xffff;
  for (size_t i = 0; i < size; ++i) {
#ifdef __AVR__
    crc = _crc_xmodem_update(crc, static_cast<uint8_t>(data[i]));
#else
    crc ^= static_cast<uint16_t>(static_cast<uint8_t>(data[i])) << 8;
    for (uint8_t bit = 0; bit < 8; ++bit)
      crc = crc & 0x8000 ? (crc << 1) ^ 0x1021 : crc << 1;
#endif
  }
  return crc;
}

struct Quadrature {
  // Unsigned arithmetic gives defined rollover; the host unwraps to signed deltas.
  volatile uint32_t count = 0, invalid = 0;
  volatile uint8_t ab = 0;
  void sample(uint8_t next) {
    static const int8_t steps[16] = {
      0, 1, -1, 0, -1, 0, 0, 1, 1, 0, 0, -1, 0, -1, 1, 0
    };
    const uint8_t old = ab;
    if (next == old) return;
    if ((old ^ next) == 3) ++invalid;
    count += steps[(old << 2) | next];
    ab = next;
  }
};

enum Fault : uint8_t { OK = 0, WATCHDOG = 1, DRIVER_IO = 2, BAD_COMMAND = 3,
                       ENCODER_STALL = 4, SPEED_DIRECTION = 5, ENCODER_INVALID = 6,
                       CONTROL_TIMING = 7, SPEED_UNREACHABLE = 8,
                       COMMAND_EXPIRED = 9, HOLD_LIMIT = 10, OVERSPEED = 11 };
struct MotionGuard {
  static const uint16_t WATCHDOG_MS = 300, MAX_PULSE_MS = 1500;
  static const uint16_t MAX_BENCH_MS = 30000;
  static const int16_t MAX_PWM = 250;
  static const int16_t MAX_DRIVE_PWM = 650, MAX_RPM_CENTI = 10000;
  bool armed = false, moving = false;
  bool speedMode = false, streaming = false;
  int16_t left = 0, right = 0;
  uint8_t fault = OK;
  uint32_t heartbeat = 0, started = 0, targetAt = 0;
  uint16_t duration = 0, lease = 0;

  void stop(uint8_t reason = OK) {
    armed = moving = false;
    speedMode = streaming = false;
    left = right = 0;
    if (reason != OK && fault == OK) fault = reason;
  }
  bool clearFault() {
    if (armed || moving) return false;
    fault = OK;
    return true;
  }
  bool arm(uint32_t now) {
    if (fault || armed || moving) return false;
    armed = true;
    heartbeat = now;
    return true;
  }
  bool run(int16_t l, int16_t r, uint16_t ms, uint32_t now) {
    // Exactly one motor per pulse; a repeated RUN cannot extend its deadline.
    return begin(l, r, ms, now, false);
  }
  bool bench(int16_t l, int16_t r, uint16_t ms, uint32_t now) {
    // Explicit bench command; both wheels raised, with the same watchdog.
    return begin(l, r, ms, now, true);
  }
  bool drive(int16_t l, int16_t r, uint16_t ms, uint32_t now) {
    return begin(l, r, ms, now, true, MAX_DRIVE_PWM);
  }
  bool speed(int16_t l, int16_t r, uint16_t ms, uint32_t now) {
    if (!armed || moving || fault || (!l && !r) || l < -MAX_RPM_CENTI || l > MAX_RPM_CENTI ||
        r < -MAX_RPM_CENTI || r > MAX_RPM_CENTI || ms < 100 || ms > MAX_BENCH_MS) return false;
    moving = speedMode = true;
    left = right = 0;
    duration = ms; started = heartbeat = now;
    return true;
  }
  bool setSpeed(int16_t l, int16_t r, uint16_t ms, uint32_t now) {
    if (!armed || fault || (moving && !streaming) || l < -MAX_RPM_CENTI || l > MAX_RPM_CENTI ||
        r < -MAX_RPM_CENTI || r > MAX_RPM_CENTI || ms < 100 || ms > WATCHDOG_MS) return false;
    moving = speedMode = streaming = true;
    targetAt = heartbeat = now;
    lease = ms;
    return true;
  }
  bool begin(int16_t l, int16_t r, uint16_t ms, uint32_t now, bool both, int16_t limit = MAX_PWM) {
    if (!armed || moving || fault || (both ? (!l || !r) : (l == 0) == (r == 0)) ||
        l < -limit || l > limit || r < -limit || r > limit ||
        ms < 100 || ms > (both ? MAX_BENCH_MS : MAX_PULSE_MS)) return false;
    moving = true;
    speedMode = false;
    left = l; right = r; duration = ms; started = heartbeat = now;
    return true;
  }
  bool tick(uint32_t now) {
    if (streaming && static_cast<uint32_t>(now - targetAt) >= lease) {
      stop(COMMAND_EXPIRED); return true;
    }
    if (armed && static_cast<uint32_t>(now - heartbeat) >= WATCHDOG_MS) {
      stop(WATCHDOG); return true;
    }
    if (moving && !streaming && static_cast<uint32_t>(now - started) >= duration) {
      stop(); return true;
    }
    return false;
  }
};
