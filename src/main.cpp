// =============================================================================
// main.cpp — qB Companion Teensy 4.1 Firmware Entry Point
// =============================================================================
// Initialises shared state, registers HeliOS timer tasks, runs scheduler.
// HeliOS is a cooperative kernel — each task callback must return quickly.
// =============================================================================

#include <Arduino.h>
#include <Wire.h>
#include <HeliOS_Arduino.h>

#include "config/feature_config.h"
#include "config/device_config.h"
#include "system/shared_state.h"
#include "debug/debug_comm.h"

#include "tasks/task_motor_control.h"
#include "tasks/task_state_estimation.h"
#include "tasks/task_perception.h"
#include "tasks/task_monitor.h"
#include "tasks/task_motion_control.h"
#include "tasks/task_comm.h"

// =============================================================================
void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 2000) {}
    Serial.println(F("qB Companion — Teensy 4.1 HeliOS Firmware"));

    // ---- Print active feature flags ----
    Serial.println(F("Features:"));
    Serial.print(F("  Motors="));  Serial.print(FEATURE_MOTORS_ENABLED);
    Serial.print(F("  Servos="));  Serial.print(FEATURE_SERVOS_ENABLED);
    Serial.print(F("  IMU="));     Serial.print(FEATURE_IMU_ENABLED);
    Serial.print(F("  TOF="));     Serial.print(FEATURE_TOF_ENABLED);
    Serial.print(F("  LIDAR="));   Serial.print(FEATURE_LIDAR_ENABLED);
    Serial.print(F("  Safety="));  Serial.print(FEATURE_SAFETY_ENABLED);
    Serial.print(F("  PiComm=")); Serial.println(FEATURE_PI_COMM_ENABLED);

#if !FEATURE_SAFETY_ENABLED
    Serial.println(F("*** SAFETY BYPASSED — BENCH TEST MODE ***"));
#endif

    debug_comm_init();

    // ---- I2C buses ----
    // MPU6050 IMU → Wire (SDA=18, SCL=19)
    // VL53L0X TOF → Wire2 (SDA2=25, SCL2=24) via DFRobot fork
#if FEATURE_IMU_ENABLED
    Wire.begin();
    Wire.setClock(MPU6050_I2C_CLOCK_HZ);
#endif
#if FEATURE_TOF_ENABLED
    Wire2.begin();
    Wire2.setClock(TOF_I2C_CLOCK_HZ);
#endif

    shared_state_init();
    xHeliOSSetup();

    Serial.println(F("Initialising tasks..."));
    task_motor_control_init();
    task_state_estimation_init();
    task_perception_init();
    task_monitor_init();
    task_motion_control_init();
    task_comm_init();

    Serial.println(F("All tasks registered.  Starting HeliOS scheduler."));
}

// =============================================================================
void loop() {
    xHeliOSLoop();
}
