#include "tasks/task_comm.h"
#include <HeliOS_Arduino.h>
#include "system/shared_state.h"
#include "comm/serial_transport.h"
#include "comm/appl_protocol.h"
#include "config/pin_config.h"
#include "config/device_config.h"
#include "config/feature_config.h"
#include "debug/debug_comm.h"

#if FEATURE_PI_COMM_ENABLED

static SerialTransport s_transport;

// =============================================================================
// COMM_RX callback — 1 kHz (1 000 µs)
// =============================================================================
static void commRxCallback(TaskId_t id_) {
    debug_comm_rx_tick(SERIAL_PI);
    s_transport.update();
}

// =============================================================================
// COMM_TX callback — 20 Hz (50 000 µs)
// =============================================================================
static void commTxCallback(TaskId_t id_) {
    // ---- 1. Drain the response buffer (command replies) ----
    ResponsePacket rsp;
    while (g_txResponseBuf.pop(rsp)) {
        debug_print_response(rsp);
        s_transport.send(rsp);
    }

    // ---- 2. Periodic telemetry push (only for enabled modules) ----
#if FEATURE_IMU_ENABLED
    s_transport.send(appl_make_telemetry(DEV_SYSTEM, PARAM_ODOM_X,       g_odomData.x_mm));
    s_transport.send(appl_make_telemetry(DEV_SYSTEM, PARAM_ODOM_Y,       g_odomData.y_mm));
    s_transport.send(appl_make_telemetry(DEV_SYSTEM, PARAM_ODOM_HEADING, g_odomData.heading_deg));

    s_transport.send(appl_make_telemetry(DEV_IMU, PARAM_ORIENTATION_ROLL,  g_imuData.roll_deg));
    s_transport.send(appl_make_telemetry(DEV_IMU, PARAM_ORIENTATION_PITCH, g_imuData.pitch_deg));
    s_transport.send(appl_make_telemetry(DEV_IMU, PARAM_ORIENTATION_YAW,   g_imuData.yaw_deg));
#endif

#if FEATURE_TOF_ENABLED
    s_transport.send(appl_make_telemetry(DEV_TOF_LEFT,  PARAM_DISTANCE_MM, (float)g_perceptionData.tof_left_mm));
    s_transport.send(appl_make_telemetry(DEV_TOF_RIGHT, PARAM_DISTANCE_MM, (float)g_perceptionData.tof_right_mm));
    s_transport.send(appl_make_telemetry(DEV_TOF_BACK,  PARAM_DISTANCE_MM, (float)g_perceptionData.tof_back_mm));
#endif

    s_transport.send(appl_make_telemetry(DEV_BATTERY, PARAM_VOLTAGE, g_monitorData.battery_voltage));
    s_transport.send(appl_make_telemetry(DEV_SYSTEM, PARAM_SAFETY_STATUS, (float)g_safetyBits));
}

#endif // FEATURE_PI_COMM_ENABLED

// =============================================================================
void task_comm_init() {
#if !FEATURE_PI_COMM_ENABLED
    return;
#else
    s_transport.begin(&SERIAL_PI, PI_LINK_BAUDRATE);

    TaskId_t rxId = xTaskAdd("CommRX", &commRxCallback);
    xTaskWait(rxId);
    xTaskSetTimer(rxId, 1000);     // 1 kHz

    TaskId_t txId = xTaskAdd("CommTX", &commTxCallback);
    xTaskWait(txId);
    xTaskSetTimer(txId, 50000);    // 20 Hz
#endif
}
