#pragma once
// =============================================================================
// task_motor_control.h — Layer 1: DDSM210 + ST3215 hardware actuation
// =============================================================================
// Owns both motor UARTs.  Receives MotorCommand / ServoCommand from shared
// state, drives hardware, and publishes feedback.
// Rate: 50 Hz (HeliOS timer callback)
// =============================================================================

void task_motor_control_init();
