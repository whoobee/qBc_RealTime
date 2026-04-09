#pragma once
// =============================================================================
// task_motion_control.h — Layer 4: Priority arbitration + safety enforcement
// =============================================================================
// Receives Pi commands from g_rxCommandBuf, applies safety checks,
// then forwards safe commands to Motor Control via shared state.
//
// Priority stack (highest wins):
//   P1 — Monitor override   (overtemp, low battery → hard stop)
//   P2 — Self-preservation  (obstacle, tilt, freefall → hard stop)
//   P3 — Pi commands        (velocity / position targets)
//   P4 — Idle defaults      (hold position, zero velocity)
//
// Rate: 50 Hz (HeliOS timer callback)
// =============================================================================

void task_motion_control_init();
