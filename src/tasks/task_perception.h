#pragma once
// =============================================================================
// task_perception.h — Layer 2a: TOF + LIDAR → perception curtain
// =============================================================================
// Fuses 3× VL53L0X + YDLidar GS2 into a unified PerceptionData struct.
// Rate: 20 Hz (HeliOS timer callback)
// =============================================================================

void task_perception_init();
