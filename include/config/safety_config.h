#pragma once
// =============================================================================
// safety_config.h — Safety thresholds and servo position limits
// =============================================================================
// All tunable safety constants live here.
// Safety logic is enforced in:
//   - task_motion_control (obstacle / tilt / pickup detection)
//   - scservo_driver       (position clamping per joint)
//   - task_monitor         (battery, temperature, watchdog)
// =============================================================================

// --- Obstacle / Perception Thresholds (mm) ---
#define OBSTACLE_CRITICAL_MM    200u   // Hard stop — no override possible
#define OBSTACLE_WARNING_MM     500u   // Clamp drive speed to 30%
#define OBSTACLE_CAUTION_MM     800u   // Publish warning only, no speed limit

// --- IMU Tilt / Pickup Thresholds (degrees / m/s²) ---
#define TILT_THRESHOLD_DEG      35.0f  // abs(roll) or abs(pitch) > this → stop
#define FREEFALL_ACCEL_MS2       2.0f  // accelZ < this → detected pickup/freefall

// --- Pi Heartbeat Watchdog ---
#define PI_WATCHDOG_TIMEOUT_MS  2000u  // No heartbeat for this long → safe idle

// --- Battery Thresholds ---
#define BATTERY_LOW_VOLTAGE_V   10.5f  // 3S LiPo low cell threshold
#define BATTERY_CRITICAL_V       9.9f  // Hard stop

// --- Motor Temperature Thresholds (°C) ---
#define MOTOR_TEMP_WARNING_C    70
#define MOTOR_TEMP_CRITICAL_C   80

// --- Speed Clamp Factor (applied when in WARNING zone) ---
#define OBSTACLE_WARNING_SPEED_FACTOR  0.30f

// --- ST3215 Servo Position Limits (SMS_STS units, 0–4096) ---
// Range: 0–4096, step ≈ 0.088°/unit.  Values from qB calibration JSON.
// ⚠  Never send a position outside [MIN, MAX] for each joint.

#define SERVO_NECK_MIN        889    // −90°
#define SERVO_NECK_MAX       2937    // +90°

#define SERVO_EAR_L_MIN       800    // −90°
#define SERVO_EAR_L_MAX      2848    // +90°

#define SERVO_EAR_R_MIN      1292    // −90°
#define SERVO_EAR_R_MAX      3340    // +90°

#define SERVO_LEG_FL_MIN     1536    // −45°
#define SERVO_LEG_FL_MAX     2560    // +45°

#define SERVO_LEG_FR_MIN     1955    // −45°
#define SERVO_LEG_FR_MAX     2979    // +45°

#define SERVO_LEG_BL_MIN     1731    // −45°
#define SERVO_LEG_BL_MAX     2755    // +45°

#define SERVO_LEG_BR_MIN     1619    // −45°
#define SERVO_LEG_BR_MAX     2643    // +45°
