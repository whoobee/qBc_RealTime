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
#define FEATURE_MOTORS_ENABLED    1   // DDSM210 wheel motors
#define FEATURE_SERVOS_ENABLED    1   // ST3215 servo bus
#define FEATURE_IMU_ENABLED       1   // BNO055 IMU
#define FEATURE_TOF_ENABLED       1   // VL53L0X TOF array master switch
#define FEATURE_LIDAR_ENABLED     0   // YDLidar GS2 (HW removed — driver kept, disabled)

// --- Per-slot TOF enables ---
// FEATURE_TOF_ENABLED above must also be 1 for any of these to take effect.
// Disabled slots are skipped at init (treated like an unwired slot) and their
// readings always report 0 mm. Use to bring up sensors one at a time during
// hardware bring-up without rewiring.
#define FEATURE_TOF_LEFT_ENABLED  1   // pin 30 XSHUT — disabled pending shell aperture redesign
#define FEATURE_TOF_RIGHT_ENABLED 1   // pin 32 XSHUT — disabled pending shell aperture redesign
#define FEATURE_TOF_FRONT_ENABLED 1   // pin 31 XSHUT — carrier hardware-dead, re-enable when replaced
#define FEATURE_TOF_BACK_ENABLED  1   // no XSHUT (always-on, Pololu carrier provides 47kΩ pull-up)

// --- Safety ---
#define FEATURE_SAFETY_ENABLED    0   // Master safety bypass (0 = no safety checks)

// --- Communication ---
#define FEATURE_PI_COMM_ENABLED   1   // Pi serial link (COBS/PacketSerial)
