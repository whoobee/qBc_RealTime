#pragma once
// =============================================================================
// feature_config.h — Selective hardware module enable/disable
// =============================================================================
// Set each flag to 1 (enabled) or 0 (disabled).
// Disabled modules are not initialised and their tasks become no-ops.
//
// FEATURE_SAFETY_ENABLED = 0 bypasses ALL safety checks — the safety manager
// always reports "all clear" so the robot will obey commands unconditionally.
// ⚠  FOR BENCH TESTING ONLY.  Re-enable before any untethered operation.
// =============================================================================

// --- Hardware Modules ---
#define FEATURE_MOTORS_ENABLED    0   // DDSM210 wheel motors
#define FEATURE_SERVOS_ENABLED    1   // ST3215 servo bus
#define FEATURE_IMU_ENABLED       0   // BNO055 IMU
#define FEATURE_TOF_ENABLED       0   // VL53L0X TOF array
#define FEATURE_LIDAR_ENABLED     0   // YDLidar GS2

// --- Safety ---
#define FEATURE_SAFETY_ENABLED    0   // Master safety bypass (0 = no safety checks)

// --- Communication ---
#define FEATURE_PI_COMM_ENABLED   1   // Pi serial link (COBS/PacketSerial)
