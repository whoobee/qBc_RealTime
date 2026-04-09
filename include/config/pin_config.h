#pragma once
// =============================================================================
// pin_config.h — Teensy 4.1 pin assignments for qB Companion robot
// =============================================================================
// Derived from EasyEDA schematic Sheet_1 REV 1.0 (2026-03-29)
// All physical wiring is defined HERE. Change only this file when rewiring.
//
// Teensy 4.1 hardware UARTs used:
//   Serial1 — Raspberry Pi     (TX=1,  RX=0)
//   Serial2 — ST3215 servos    (TX=8,  RX=7)
//   Serial3 — DDSM210 Motor R  (TX=14, RX=15)
//   Serial5 — DDSM210 Motor L  (TX=20, RX=21)
//   Serial6 — YDLidar GS2      (TX=24, RX=25)
//
// I2C bus:
//   Wire  (SDA=18, SCL=19) — BNO055 IMU + 3× VL53L0X TOF sensors
// =============================================================================

// --- UART Serial Port Assignments ---
#define SERIAL_PI            Serial1   // RPi ↔ Teensy (COBS / PacketSerial)
#define SERIAL_SERVOS        Serial2   // ST3215 servo bus
#define SERIAL_MOTOR_RIGHT   Serial3   // DDSM210 — dedicated port per motor
#define SERIAL_MOTOR_LEFT    Serial5   // DDSM210
#define SERIAL_LIDAR         Serial6   // YDLidar GS2
#define SERIAL_DEBUG         Serial    // USB serial for debug logging

// --- I2C Bus ---
#define I2C_BUS              Wire      // BNO055 + 3× VL53L0X share this bus

// --- TOF Sensor XSHUT (shutdown / enable) Pins ---
// ⚠  TODO: Verify these against actual wiring — not visible in schematic.
// All XSHUT pins must be OUTPUT.  LOW = sensor held in reset, HIGH = active.
// Sensors are powered up one-by-one for I2C address reassignment.
#define PIN_TOF_XSHUT_LEFT   30    // TOF-L1 XSHUT
#define PIN_TOF_XSHUT_BACK   31    // TOF-B1 XSHUT
#define PIN_TOF_XSHUT_RIGHT  32    // TOF-R1 XSHUT

// --- Battery Monitor ADC Pins ---
// ⚠  TODO: No ADC voltage-divider circuit visible in schematic REV 1.0.
//    Uncomment and set when hardware is added.
// #define PIN_BATTERY_VOLTAGE  A0
// #define PIN_BATTERY_CURRENT  A1
#define BATTERY_MONITOR_ENABLED  0   // Set to 1 when ADC hardware is present

// --- Debug Logging ---
#define DEBUG_COMM_ENABLED   1        // Set to 0 to disable comm debug output
#define DEBUG_BAUDRATE       115200   // USB Serial baud rate

// --- Optional: Status LED ---
#define PIN_STATUS_LED       13   // Teensy built-in LED
