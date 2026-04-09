#include "system/shared_state.h"

// =============================================================================
// Global shared data
// =============================================================================
IMUData         g_imuData         = {};
OdometryData    g_odomData        = {};
PerceptionData  g_perceptionData  = {};
MonitorData     g_monitorData     = {};
DDSM210Status   g_motorStatus[2]  = {};
ServoStatus     g_servoStatus[7]  = {};

uint8_t         g_safetyBits      = 0;

RingBuffer<RequestPacket,  16> g_rxCommandBuf;
RingBuffer<ResponsePacket, 32> g_txResponseBuf;
RingBuffer<ServoCommand,   10> g_servoCmdBuf;

MotorCommand    g_motorCmd        = {};
bool            g_motorCmdPending = false;

// =============================================================================
void shared_state_init() {
    // Zero-init is already done by static initialization.
    // Seed the Pi heartbeat so watchdog doesn't fire on startup.
    g_monitorData.last_pi_heartbeat_ms = millis();
}
