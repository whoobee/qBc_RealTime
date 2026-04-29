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
//   Serial4 — DDSM210 Motor L  (TX=17, RX=16)
//   Serial7 — YDLidar GS2      (TX=29, RX=28)  — 921600 baud, 8N1
//                              (LIDAR currently disabled — see feature_config.h)
//
// I2C buses:
//   Wire   (SDA=18,  SCL=19)  — MPU6050 IMU at 0x68
//   Wire2  (SDA2=25, SCL2=24) — VL53L0X TOF array (front XSHUT = pin 32,
//                                back sensor has no XSHUT — always on at 0x29)
// =============================================================================

// --- UART Serial Port Assignments ---
#define SERIAL_PI            Serial1   // RPi ↔ Teensy (COBS / PacketSerial)
#define SERIAL_SERVOS        Serial2   // ST3215 servo bus
#define SERIAL_MOTOR_RIGHT   Serial3   // DDSM210 — dedicated port per motor
#define SERIAL_MOTOR_LEFT    Serial4   // DDSM210
#define SERIAL_LIDAR         Serial7   // YDLidar GS2 (pins 28/29, 921600 baud)
#define SERIAL_DEBUG         Serial    // USB serial for debug logging

// --- I2C Buses ---
// IMU and TOF are on SEPARATE buses — MPU6050 on Wire, VL53L0X on Wire2.
#define I2C_BUS_IMU          Wire      // MPU6050
#define I2C_BUS_TOF          Wire2     // VL53L0X (DFRobot fork hardcodes Wire2)

// --- TOF Sensor XSHUT (shutdown / enable) Pins ---
// The array driver supports up to 4 sensors on Wire2 with per-sensor XSHUT
// control for sequential I2C address assignment.
//
// Two sentinels are recognised by the driver:
//   TOF_XSHUT_NOT_WIRED (0xFF) — slot empty; skipped at init, readAll() = 0 mm
//   TOF_XSHUT_NONE      (0xFE) — sensor present but XSHUT not wired to Teensy
//                                (powered up at 0x29 from boot). The driver
//                                must reassign its address FIRST, while every
//                                XSHUT-controlled sensor is still held in reset.
//
// Current HW (2026-04-27):
//   LEFT  — VL53L0X with XSHUT on pin 30
//   RIGHT — VL53L0X with XSHUT on pin 32
//   FRONT — VL53L0X with XSHUT on pin 31 (was originally wired as the "back"
//           slot; sensor physically relocated to the front of the robot)
//   BACK  — VL53L0X with NO XSHUT line, always live at 0x29 from boot
//           (new sensor — driver must reassign its address FIRST)
#define TOF_XSHUT_NOT_WIRED  0xFF
#define TOF_XSHUT_NONE       0xFE

#define PIN_TOF_XSHUT_LEFT   30                    // TOF-L
#define PIN_TOF_XSHUT_RIGHT  32                    // TOF-R
#define PIN_TOF_XSHUT_FRONT  31                    // TOF-F (legacy "back" XSHUT)
#define PIN_TOF_XSHUT_BACK   TOF_XSHUT_NONE        // TOF-B (no XSHUT — always on)

// --- Battery Monitor ADC Pins ---
// ⚠  TODO: No ADC voltage-divider circuit visible in schematic REV 1.0.
//    Uncomment and set when hardware is added.
// #define PIN_BATTERY_VOLTAGE  A0
// #define PIN_BATTERY_CURRENT  A1
#define BATTERY_MONITOR_ENABLED  0   // Set to 1 when ADC hardware is present

// --- Debug Logging ---
// Per-channel debug switches live in debug_config.h. SERIAL_DEBUG (above) and
// DEBUG_BAUDRATE define the physical port; the channel flags decide what gets
// printed. Included here so any TU pulling pin_config.h gets them transitively.
#include "config/debug_config.h"

// --- Optional: Status LED ---
#define PIN_STATUS_LED       13   // Teensy built-in LED
