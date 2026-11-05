#pragma once

// Confirmed by the user: I2C, DIP switches 1..4 all ON (address 0x01).
// 0 = encoder only, 1 = I2C (Mega SDA20/SCL21), 2 = Serial1 (TX18/RX19).
// Serial1 requires a TTL <-> RS-232 level converter, not a direct connection.
#ifndef MOTOR_LINK
#define MOTOR_LINK 1
#endif
#ifndef MOTOR_I2C_ADDRESS
#define MOTOR_I2C_ADDRESS 0x01
#endif
#define MOTOR_RS232_BAUD 9600

// Legacy L/R variable names mean first/second encoder in the wire protocol.
// Physical right: first encoder -> driver R/M1. Motor leads were reversed:
// negative PWM = forward, but the unchanged encoder still increases forward.
// Physical left: second encoder -> driver L/M2, negative PWM/counts = forward.
// Host config/drive_calibration.json performs side/polarity normalization.
// Raw encoder sign per positive driver command, in driver L/R order.
constexpr int8_t PID_ENCODER_SIGN_L = 1, PID_ENCODER_SIGN_R = -1;
constexpr uint8_t ENCODER_L_A = 40, ENCODER_L_B = 41;
constexpr uint8_t ENCODER_R_A = 50, ENCODER_R_B = 51;
constexpr uint32_t USB_BAUD = 115200;
constexpr uint16_t SAMPLE_HZ = 20000;
