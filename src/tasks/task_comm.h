#pragma once
// =============================================================================
// task_comm.h — Layer 5: Serial communication with Raspberry Pi
// =============================================================================
// Two HeliOS timer tasks:
//   CommRX — polls PacketSerial.update() at 1 kHz
//   CommTX — sends responses + periodic telemetry at 20 Hz
// =============================================================================

void task_comm_init();
