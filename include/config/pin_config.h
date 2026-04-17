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
//   Serial7 — YDLidar GS2      (TX=29, RX=28)  — 921600 baud, 8N1
//
// I2C buses (validated in qB_Test_TOF):
//   Wire   (SDA=18,  SCL=19)  — MPU6050 IMU at 0x68
//   Wire2  (SDA2=25, SCL2=24) — VL53L0X TOF at 0x29 (single sensor, XSHUT = pin 32)
// =============================================================================

// --- UART Serial Port Assignments ---
#define SERIAL_PI            Serial1   // RPi ↔ Teensy (COBS / PacketSerial)
#define SERIAL_SERVOS        Serial2   // ST3215 servo bus
#define SERIAL_MOTOR_RIGHT   Serial3   // DDSM210 — dedicated port per motor
#define SERIAL_MOTOR_LEFT    Serial5   // DDSM210
#define SERIAL_LIDAR         Serial7   // YDLidar GS2 (pins 28/29, 921600 baud)
#define SERIAL_DEBUG         Serial    // USB serial for debug logging

// --- I2C Buses ---
// IMU and TOF are on SEPARATE buses — MPU6050 on Wire, VL53L0X on Wire2.
#define I2C_BUS_IMU          Wire      // MPU6050
#define I2C_BUS_TOF          Wire2     // VL53L0X (DFRobot fork hardcodes Wire2)

// --- TOF Sensor XSHUT (shutdown / enable) Pins ---
// The array driver supports up to 3 sensors on Wire2 with per-sensor XSHUT
// control for sequential I2C address assignment. Set a slot to
// TOF_XSHUT_NOT_WIRED (0xFF) to disable that sensor slot — it will be skipped
// at init and readAll() will report 0 mm for that index.
//
// Current HW (qB_Test_TOF, 2026-04): only one sensor wired, on pin 32.
// Assigned to the RIGHT slot to match the legacy schematic convention.
#define TOF_XSHUT_NOT_WIRED  0xFF

#define PIN_TOF_XSHUT_LEFT   TOF_XSHUT_NOT_WIRED   // TOF-L (not wired yet)
#define PIN_TOF_XSHUT_RIGHT  32                    // TOF-R (wired)
#define PIN_TOF_XSHUT_BACK   TOF_XSHUT_NOT_WIRED   // TOF-B (not wired yet)

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
