#pragma once
// =============================================================================
// task_state_estimation.h — Layer 2b: IMU + wheel encoder → odometry
// =============================================================================
// Reads BNO055 at 200 Hz, integrates with wheel encoder deltas to produce
// a stable odometry estimate (x, y, heading).
// Rate: 200 Hz (HeliOS timer callback)
// =============================================================================

void task_state_estimation_init();
