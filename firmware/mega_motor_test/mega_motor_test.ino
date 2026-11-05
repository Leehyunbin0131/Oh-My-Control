// Mega 2560 encoder acquisition and bounded NT-S-DCDM1210 bench tests.
#include <Arduino.h>
#include <Wire.h>
#include <util/atomic.h>
#include "config.h"
#include "test_core.h"
#include "velocity_core.h"

#if !defined(__AVR_ATmega2560__)
#error This sketch requires an Arduino Mega 2560.
#endif
#if MOTOR_LINK < 0 || MOTOR_LINK > 2
#error MOTOR_LINK must be 0, 1 or 2.
#endif
#if SERIAL_RX_BUFFER_SIZE != 256
#error Build sketch AND Arduino core with SERIAL_RX_BUFFER_SIZE=256 (mega_firmware.sh).
#endif

Quadrature encL, encR;
volatile uint32_t sampleTicks = 0;
MotionGuard motion;
VelocityControl velocity(PID_ENCODER_SIGN_L, PID_ENCODER_SIGN_R);
uint32_t lastSequence = 0, lastTelemetry = 0, lastInfo = 0, lastStop = 0;
char inputLine[120];
uint8_t inputLength = 0;
bool inputOverflow = false;
volatile bool controlEnabled = false, pendingTarget = false, rawEnabled = false;
volatile uint32_t requestedSequence = 0, appliedSequence = 0, appliedTick = 0, lastEncoderErrorTick = 0;
volatile int16_t requestedRpm[2] = {0, 0};
volatile uint8_t controlFault = OK;
volatile uint32_t controlTicks = 0, activeUpdates = 0, missedTicks = 0;
volatile uint32_t previousControlUs = 0;
volatile uint16_t controlMinUs = 65535, controlMaxUs = 0, computeMaxUs = 0;
uint32_t driverClock = 0, driverWrites = 0, driverLastUs = 0, driverGapMaxUs = 0;
uint32_t lastOutputTick = 0, txDropped = 0;
uint16_t driverMaxUs = 0, outputPeriodUs = 1000;
char txQueue[512];
uint16_t txHead = 0, txTail = 0;
uint32_t rxLastServiceUs = 0, rxMaxGapUs = 0, rxFrames = 0;
uint32_t rxFrameErrors = 0, rxCrcErrors = 0, rxSequenceErrors = 0;
uint16_t rxMaxQueued = 0;
uint8_t pendingInfo = 0;
bool pendingControlState = false;
void serviceDriver();

// D40=PG1, D41=PG0; D50=PB3, D51=PB2. Both phases sampled together.
ISR(TIMER5_COMPA_vect) {
  ++sampleTicks;
  encL.sample(PING & 3);
  encR.sample((PINB >> 2) & 3);
}

// Timer1 runs the actual controller, independently of UART/I2C work in loop().
// Mask its own interrupt before enabling nesting so the 20 kHz encoder ISR
// remains serviceable throughout fixed-point PID calculation.
ISR(TIMER1_COMPA_vect) {
  TIMSK1 &= ~_BV(OCIE1A);
  const uint32_t entered = micros();
  const uint32_t elapsed = entered - previousControlUs;
  previousControlUs = entered;
  if (controlTicks) {
    if (elapsed < controlMinUs) controlMinUs = elapsed;
    if (elapsed > controlMaxUs) controlMaxUs = elapsed > 65535 ? 65535 : elapsed;
    if (elapsed > 1500 || elapsed < 500) {
      ++missedTicks;
      if (controlEnabled || rawEnabled) controlFault = CONTROL_TIMING;
    }
  }
  const uint32_t tick = ++controlTicks;
  sei();
  uint32_t counts[2], invalid[2];
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    counts[0] = encR.count; counts[1] = encL.count;
    invalid[0] = encR.invalid; invalid[1] = encL.invalid;
  }
  for (uint8_t i = 0; i < 2; ++i)
    if (invalid[i] != velocity.lastInvalid[i]) lastEncoderErrorTick = tick;
  if (!controlEnabled || controlFault) {
    if (velocity.active) velocity.stop();
    const uint8_t fault = velocity.update(counts, invalid, tick, rawEnabled);
    if (fault) controlFault = fault;
  } else {
    if (pendingTarget) {
      if (!velocity.active) velocity.start(requestedRpm[0], requestedRpm[1], counts, invalid, tick - 1);
      else velocity.setTargets(requestedRpm[0], requestedRpm[1], tick);
      appliedSequence = requestedSequence; appliedTick = tick;
      pendingTarget = false;
    }
    const uint8_t fault = velocity.update(counts, invalid, tick);
    ++activeUpdates;
    if (fault) { controlFault = fault; velocity.stop(); }
  }
  const uint32_t runtime = micros() - entered;
  if (runtime > computeMaxUs) computeMaxUs = runtime > 65535 ? 65535 : runtime;
  if (runtime >= 900 && (controlEnabled || rawEnabled)) { controlFault = CONTROL_TIMING; velocity.stop(); }
  cli();
  TIMSK1 |= _BV(OCIE1A);
}

void drainSerial() {
  uint8_t budget = 24;
  while (budget-- && txTail != txHead && Serial.availableForWrite()) {
    Serial.write(txQueue[txTail]);
    txTail = (txTail + 1) & 511;
  }
}

void sendFrame(const char* body) {
  const uint16_t size = strlen(body);
  if (((txTail - txHead - 1) & 511) < size + 6) { ++txDropped; return; }
  const uint16_t crc = frameCrc(body, size);
  for (uint16_t i = 0; i < size; ++i) { txQueue[txHead] = body[i]; txHead = (txHead + 1) & 511; }
  txQueue[txHead] = '*'; txHead = (txHead + 1) & 511;
  for (int8_t shift = 12; shift >= 0; shift -= 4) {
    const uint8_t digit = (crc >> shift) & 15;
    txQueue[txHead] = digit < 10 ? '0' + digit : 'A' + digit - 10;
    txHead = (txHead + 1) & 511;
  }
  txQueue[txHead] = '\n'; txHead = (txHead + 1) & 511;
}

// Formatting must not perform blocking I2C work. The scheduler sends only one
// diagnostic frame per loop, with command reception and driver service between.
struct Message {
  char body[220];
  uint16_t length;
  bool valid = true;
  explicit Message(const char* prefix) { strcpy(body, prefix); length = strlen(body); }
  void append(const char* value) {
    const uint8_t n = strlen(value);
    if (static_cast<size_t>(length) + n + 2 > sizeof(body)) { valid = false; return; }
    body[length++] = ','; strcpy(body + length, value); length += n;
    drainSerial();
  }
  void number(int32_t value) { char text[12]; ltoa(value, text, 10); append(text); }
  void positive(uint32_t value) { char text[11]; ultoa(value, text, 10); append(text); }
  void send() { if (valid) sendFrame(body); else ++txDropped; }
};

bool driverCommand(const char* command) {
#if MOTOR_LINK == 0
  return false;
#else
#if MOTOR_LINK == 1
  Wire.clearWireTimeoutFlag();
  Wire.beginTransmission(MOTOR_I2C_ADDRESS);
  Wire.write(command);
  const uint8_t result = Wire.endTransmission();
  return result == 0 && !Wire.getWireTimeoutFlag();
#else
  if (Serial1.availableForWrite() < static_cast<int>(strlen(command))) return false;
  Serial1.print(command);
  // UART transmission is not a driver acknowledgement. Encoder response is
  // required to pass a powered test; no claim of driver feedback is made.
  return true;
#endif
#endif
}

DriverOutput driverOutput;
bool writeDriver(int16_t left, int16_t right) {
#if MOTOR_LINK == 0
  return left == 0 && right == 0;
#else
  return driverOutput.write(driverCommand, left, right);
#endif
}

void stopMotors(uint8_t reason = OK) {
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    if (reason == OK && controlFault) reason = controlFault;
    controlEnabled = rawEnabled = false; pendingTarget = false; controlFault = OK;
  }
  motion.stop(reason);
  if (!writeDriver(0, 0)) {
    if (motion.fault == OK) motion.fault = DRIVER_IO;
#if MOTOR_LINK == 1
    // If fast-mode transmission failed, also try zero at the original bus rate.
    Wire.setClock(100000); driverClock = 100000; outputPeriodUs = 2000;
    driverOutput.delaySetting = -1;
    writeDriver(0, 0);
#endif
  }
  lastStop = millis();
  lastOutputTick = 0;
}

void serviceDriver() {
  if (motion.tick(millis())) { stopMotors(motion.fault); return; }
  if (controlFault) { stopMotors(controlFault); return; }
  if (!motion.speedMode || pendingTarget) return;
  uint32_t tick;
  int16_t l, r;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { tick = controlTicks; l = velocity.output[0]; r = velocity.output[1]; }
  if (tick == lastOutputTick) return;
  const uint32_t now = micros();
  if (outputPeriodUs > 1000 && static_cast<uint32_t>(now - driverLastUs) < outputPeriodUs) return;
  if (lastOutputTick && now - driverLastUs > driverGapMaxUs) driverGapMaxUs = now - driverLastUs;
  driverLastUs = now;
  const bool written = writeDriver(l, r);
  const uint32_t duration = micros() - now;
  if (duration > driverMaxUs) driverMaxUs = duration > 65535 ? 65535 : duration;
  ++driverWrites;
  lastOutputTick = tick;
  if (!written) { stopMotors(DRIVER_IO); return; }
  motion.left = l; motion.right = r;
}

void sendIdentity() {
  Message info("OMR1,I,8,40,41,50,51");
  const uint16_t fields[] = {SAMPLE_HZ, MotionGuard::WATCHDOG_MS, MotionGuard::MAX_PULSE_MS,
                            MotionGuard::MAX_PWM, MOTOR_LINK, MOTOR_I2C_ADDRESS, MotionGuard::MAX_BENCH_MS,
                            MotionGuard::MAX_DRIVE_PWM, VelocityControl::COUNTS_PER_REV,
                            VelocityControl::PERIOD_MS, VelocityControl::MAX_RPM_CENTI, VelocityControl::MAX_PWM};
  for (uint8_t i = 0; i < sizeof(fields) / sizeof(fields[0]); ++i) info.positive(fields[i]);
  info.positive(driverClock); info.positive(1000000UL / outputPeriodUs);
  info.number(PID_ENCODER_SIGN_L); info.number(PID_ENCODER_SIGN_R);
  info.positive(0); info.positive(0); // old BRAKE unsupported
  info.positive(1); info.positive(7); // signed_pwm_v1; streaming | zero servo | conditional hold
  info.send();
}

void sendEncoderDiagnostic() {
  uint32_t ticks;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { ticks = sampleTicks; }
  const uint8_t left = (digitalRead(ENCODER_L_A) << 1) | digitalRead(ENCODER_L_B);
  const uint8_t right = (digitalRead(ENCODER_R_A) << 1) | digitalRead(ENCODER_R_B);
  Message diagnostic("OMR1,D");
  diagnostic.positive(millis()); diagnostic.positive(ticks); diagnostic.positive(left); diagnostic.positive(right); diagnostic.send();
}

void sendTiming() {
  uint32_t ct, updates, misses;
  uint16_t minimum, maximum, runtime;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    ct = controlTicks; updates = activeUpdates; misses = missedTicks;
    minimum = controlMinUs; maximum = controlMaxUs; runtime = computeMaxUs;
  }
  Message timing("OMR1,P");
  timing.positive(micros()); timing.positive(ct); timing.positive(updates); timing.positive(misses);
  timing.positive(minimum == 65535 ? 0 : minimum); timing.positive(maximum); timing.positive(runtime);
  timing.positive(driverWrites); timing.positive(driverMaxUs); timing.positive(driverGapMaxUs);
  timing.positive(driverClock); timing.positive(txDropped); timing.send();
}

void sendReceiveDiagnostic() {
  Message m("OMR1,R");
  m.positive(millis()); m.positive(SERIAL_RX_BUFFER_SIZE); m.positive(rxMaxQueued);
  m.positive(rxMaxGapUs); m.positive(rxFrames); m.positive(rxFrameErrors);
  m.positive(rxCrcErrors); m.positive(rxSequenceErrors); m.send();
}

void sendInfo() {
  sendIdentity(); sendEncoderDiagnostic(); sendTiming(); sendReceiveDiagnostic();
}

void sendTelemetry() {
  uint32_t l, r, li, ri;
  uint8_t la, ra;
  int16_t targets[2], measured[2];
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    l = encL.count; r = encR.count; li = encL.invalid; ri = encR.invalid;
    la = encL.ab; ra = encR.ab;
    for (uint8_t i = 0; i < 2; ++i) { targets[i] = velocity.target[i]; measured[i] = velocity.rawMeasuredCenti(i); }
  }
  Message message("OMR1,T");
  message.positive(millis()); message.positive(l); message.positive(r);
  message.positive(la); message.positive(ra); message.positive(li); message.positive(ri);
  message.number(motion.left); message.number(motion.right); message.positive(motion.armed ? 1 : 0);
  message.positive(motion.fault); message.positive(lastSequence); message.positive(MOTOR_LINK);
  message.positive(motion.speedMode ? 1 : 0); message.number(targets[0]); message.number(targets[1]);
  message.number(measured[0]); message.number(measured[1]); message.send();
}

void acknowledge(uint32_t seq, bool accepted, const char* reason) {
  char body[90];
  snprintf(body, sizeof(body), "OMR1,A,%lu,%s,%s", static_cast<unsigned long>(seq), accepted ? "OK" : "ERR", reason);
  sendFrame(body);
}

bool parseNumber(const char* token, long minimum, long maximum, long& out) {
  if (!token || !*token) return false;
  const char* digit = token + (*token == '-' ? 1 : 0);
  if (!*digit) return false;
  for (; *digit; ++digit) if (*digit < '0' || *digit > '9') return false;
  char* end;
  out = strtol(token, &end, 10);
  return !*end && out >= minimum && out <= maximum;
}

void sendControlState() {
  uint32_t seq, tick, refs[2];
  int32_t errors[2];
  uint8_t states[2], settled[2], limits[2];
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    seq = appliedSequence; tick = appliedTick;
    for (uint8_t i = 0; i < 2; ++i) {
      refs[i] = velocity.holdReference[i]; errors[i] = velocity.positionError[i];
      states[i] = velocity.state[i]; settled[i] = velocity.settled[i]; limits[i] = velocity.limited[i];
    }
  }
  Message m("OMR1,Q");
  m.positive(millis()); m.positive(seq); m.positive(tick);
  m.positive(motion.streaming ? static_cast<uint32_t>(millis() - motion.targetAt) : 0);
  m.positive(motion.fault ? 3 : motion.moving ? 2 : motion.armed ? 1 : 0);
  for (uint8_t i = 0; i < 2; ++i) m.positive(states[i]);
  for (uint8_t i = 0; i < 2; ++i) m.positive(settled[i]);
  for (uint8_t i = 0; i < 2; ++i) m.positive(limits[i]);
  for (uint8_t i = 0; i < 2; ++i) m.positive(refs[i]);
  for (uint8_t i = 0; i < 2; ++i) m.number(errors[i]);
  m.send();
}

void parseCommand(char* line) {
  ++rxFrames;
  char* star = strrchr(line, '*');
  if (!star || strlen(star + 1) != 4) { ++rxFrameErrors; stopMotors(BAD_COMMAND); return; }
  char* end;
  for (const char* c = star + 1; *c; ++c)
    if (!((*c >= '0' && *c <= '9') || (*c >= 'A' && *c <= 'F') || (*c >= 'a' && *c <= 'f'))) {
      ++rxFrameErrors; stopMotors(BAD_COMMAND); return;
    }
  const unsigned long checksum = strtoul(star + 1, &end, 16);
  if (*end || checksum != frameCrc(line, star - line)) { ++rxCrcErrors; stopMotors(BAD_COMMAND); return; }
  *star = 0;
  if (star == line || star[-1] == ',' || strstr(line, ",,")) { ++rxFrameErrors; stopMotors(BAD_COMMAND); return; }
  char* fields[18];
  uint8_t size = 0;
  char* cursor;
  for (char* part = strtok_r(line, ",", &cursor); part && size < 18;
       part = strtok_r(nullptr, ",", &cursor)) fields[size++] = part;
  if (size < 4 || strcmp(fields[0], "OMR1") || strcmp(fields[1], "C")) {
    ++rxFrameErrors; stopMotors(BAD_COMMAND); return;
  }
  const size_t seqLength = strlen(fields[2]);
  if (!seqLength || seqLength > 10 || (seqLength == 10 && strcmp(fields[2], "4294967295") > 0)) {
    ++rxFrameErrors; stopMotors(BAD_COMMAND); return;
  }
  for (const char* c = fields[2]; *c; ++c)
    if (*c < '0' || *c > '9') { ++rxFrameErrors; stopMotors(BAD_COMMAND); return; }
  const uint32_t seq = strtoul(fields[2], &end, 10);
  if (*end || !seq) { ++rxFrameErrors; stopMotors(BAD_COMMAND); return; }
  const bool newer = static_cast<uint32_t>(seq - lastSequence) < 0x80000000UL && seq != lastSequence;
  // STOP is always honoured, including duplicate sequence numbers.
  if (size == 4 && !strcmp(fields[3], "STOP")) {
    if (newer) lastSequence = seq;
    stopMotors();
    acknowledge(seq, true, "disabled");
    return;
  }
  if (!newer) { ++rxSequenceErrors; stopMotors(BAD_COMMAND); acknowledge(seq, false, "sequence"); return; }
  lastSequence = seq;
  bool accepted = false;
  const char* reason = "command";
  if (size == 4 && !strcmp(fields[3], "BRAKE")) {
    reason = "unsupported_brake";
  } else if (size == 4 && !strcmp(fields[3], "CLEAR_FAULT")) {
    uint32_t tick, invalidTick;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { tick = controlTicks; invalidTick = lastEncoderErrorTick; }
    accepted = !motion.armed && !motion.moving && !controlFault && !velocity.active &&
               static_cast<uint32_t>(tick - invalidTick) >= 100 && writeDriver(0, 0) && motion.clearFault();
    reason = "clear_fault";
  } else if (size == 4 && !strcmp(fields[3], "ARM")) {
    accepted = MOTOR_LINK != 0 && motion.arm(millis());
    reason = MOTOR_LINK == 0 ? "driver_not_configured" : "arm";
  } else if (size == 4 && !strcmp(fields[3], "PING")) {
    if (motion.armed) motion.heartbeat = millis();
    accepted = true; reason = "heartbeat";
  } else if (size == 11 && !strcmp(fields[3], "PID")) {
    long channel, direction, kp, ki, kd, slope, offset;
    accepted = !motion.armed && !motion.moving && motion.fault == OK &&
               parseNumber(fields[4], 0, 1, channel) && parseNumber(fields[5], -1, 1, direction) &&
               parseNumber(fields[6], 0, 20000, kp) && parseNumber(fields[7], 0, 50000, ki) &&
               parseNumber(fields[8], 0, 2000, kd) && parseNumber(fields[9], 1, 20000, slope) &&
               parseNumber(fields[10], 0, 200000, offset);
    if (accepted) {
      // This command is only accepted disarmed. The timer's idle estimator
      // does not touch gains; keep encoder/PID timer interrupts enabled while
      // converting the gains to fixed-point.
      accepted = velocity.configureScaled(channel, direction, kp, ki, kd, slope, offset);
    }
    reason = "pid_config";
  } else if (size == 8 && !strcmp(fields[3], "ZERO")) {
    long c, kp, ki, pwm;
    accepted = !motion.armed && !motion.fault && parseNumber(fields[4], 0, 1, c) &&
      parseNumber(fields[5], 1, 20000, kp) && parseNumber(fields[6], 0, 50000, ki) &&
      parseNumber(fields[7], 1, 150, pwm) && velocity.configureZero(c, kp, ki, pwm);
    reason = "zero_config";
  } else if (size == 13 && !strcmp(fields[3], "HOLD")) {
    long v[9];
    accepted = !motion.armed && !motion.fault;
    for (uint8_t i = 0; i < 9 && accepted; ++i) accepted = parseNumber(fields[4+i], 0, 60000, v[i]);
    if (accepted) accepted = v[0] <= 1 && v[1] <= 1 &&
      velocity.configureHold(v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8]);
    reason = "hold_config";
  } else if (size == 7 && (!strcmp(fields[3], "SPEED") || !strcmp(fields[3], "SET_SPEED"))) {
    long l, r, ms;
    const bool stream = !strcmp(fields[3], "SET_SPEED");
    accepted = parseNumber(fields[4], -10000, 10000, l) &&
               parseNumber(fields[5], -10000, 10000, r) && parseNumber(fields[6], 100, stream ? 300 : 30000, ms) &&
               velocity.ready(l, r) && (stream ? motion.setSpeed(l, r, ms, millis()) : motion.speed(l, r, ms, millis()));
    if (accepted) {
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        requestedRpm[0] = l; requestedRpm[1] = r; requestedSequence = seq;
        pendingTarget = true; controlEnabled = true; rawEnabled = false;
      }
    }
    reason = stream ? "target" : "speed";
  } else if (size == 7 && (!strcmp(fields[3], "RUN") || !strcmp(fields[3], "BENCH") || !strcmp(fields[3], "DRIVE"))) {
    const bool drive = !strcmp(fields[3], "DRIVE");
    const bool bench = drive || !strcmp(fields[3], "BENCH");
    const int16_t limit = drive ? MotionGuard::MAX_DRIVE_PWM : MotionGuard::MAX_PWM;
    long l, r, ms;
    accepted = parseNumber(fields[4], -limit, limit, l) &&
               parseNumber(fields[5], -limit, limit, r) &&
               parseNumber(fields[6], 100, bench ? 30000 : 1500, ms) &&
               (drive ? motion.drive(l, r, ms, millis()) :
                bench ? motion.bench(l, r, ms, millis()) : motion.run(l, r, ms, millis()));
    if (accepted) {
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { controlEnabled = false; pendingTarget = false; }
      if (!writeDriver(l, r)) { stopMotors(DRIVER_IO); accepted = false; }
      else rawEnabled = true;
    }
    reason = "run";
  }
  if (!accepted && motion.fault == OK) stopMotors(BAD_COMMAND);
  acknowledge(seq, accepted, reason);
}

void setup() {
  const uint8_t pins[] = {ENCODER_L_A, ENCODER_L_B, ENCODER_R_A, ENCODER_R_B};
  for (uint8_t pin : pins)
    pinMode(pin, INPUT_PULLUP);
  encL.ab = PING & 3;
  encR.ab = (PINB >> 2) & 3;
  // Timer5 CTC, prescaler 8, 20 kHz. Pins 44/45/46 PWM and Servo Timer5
  // are unavailable while this standalone test firmware is running.
  TCCR5A = 0; TCCR5B = 0; TCNT5 = 0;
  OCR5A = F_CPU / 8 / SAMPLE_HZ - 1;
  TCCR5B = _BV(WGM52) | _BV(CS51);
  TIMSK5 = _BV(OCIE5A);
#if MOTOR_LINK == 1
  Wire.begin();
  Wire.setClock(400000); driverClock = 400000;
  Wire.setWireTimeout(2500, true);
#elif MOTOR_LINK == 2
  Serial1.begin(MOTOR_RS232_BAUD);
  outputPeriodUs = 20000;
#endif
  stopMotors();
  Serial.begin(USB_BAUD);
  // Timer1 CTC at exactly F_CPU / (8 * 2000) = 1000 Hz on a 16 MHz Mega.
  TCCR1A = 0; TCCR1B = 0; TCNT1 = 0;
  OCR1A = F_CPU / 8 / 1000 - 1;
  TCCR1B = _BV(WGM12) | _BV(CS11);
  TIMSK1 = _BV(OCIE1A);
  sendInfo();
  rxLastServiceUs = micros();
}

void serviceReceive() {
  const uint32_t now = micros();
  const uint32_t gap = now - rxLastServiceUs;
  if (gap > rxMaxGapUs) rxMaxGapUs = gap;
  rxLastServiceUs = now;
  const uint16_t queued = Serial.available();
  if (queued > rxMaxQueued) rxMaxQueued = queued;
  // At least three full 50 Hz targets fit in one batch. Keep the budget bounded
  // so malformed floods cannot starve driver service or the command deadline.
  for (uint8_t n = 0; n < 128 && Serial.available(); ++n) {
    const char c = Serial.read();
    if (c == '\n') {
      if (inputOverflow) { ++rxFrameErrors; stopMotors(BAD_COMMAND); }
      else if (inputLength) { inputLine[inputLength] = 0; parseCommand(inputLine); }
      inputLength = 0; inputOverflow = false;
    } else if (c != '\r') {
      if (inputLength < sizeof(inputLine) - 1) inputLine[inputLength++] = c;
      else inputOverflow = true;
    }
  }
}

void loop() {
  uint32_t now = millis();
  if (motion.tick(now)) stopMotors(motion.fault);
  serviceReceive();
  serviceDriver();
  drainSerial();
  // Retry zero periodically; never resume a command after a bus failure.
  if (!motion.moving && static_cast<uint32_t>(now - lastStop) >= 200) {
    if (!writeDriver(0, 0) && motion.fault == OK) motion.fault = DRIVER_IO;
    lastStop = now;
  }
  serviceReceive();
  now = millis();
  if (static_cast<uint32_t>(now - lastInfo) >= 1000 && !pendingInfo) {
    pendingInfo = 4; lastInfo = now;
  }
  if (pendingControlState) { sendControlState(); pendingControlState = false; }
  else if (static_cast<uint32_t>(now - lastTelemetry) >= 50) {
    sendTelemetry(); pendingControlState = true; lastTelemetry = now;
  } else if (pendingInfo) {
    switch (pendingInfo--) {
      case 4: sendIdentity(); break;
      case 3: sendEncoderDiagnostic(); break;
      case 2: sendTiming(); break;
      case 1: sendReceiveDiagnostic(); break;
    }
  }
  serviceReceive();
  serviceDriver();
  drainSerial();
}
