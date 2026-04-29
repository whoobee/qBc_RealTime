#include "tasks/task_comm.h"
#include <HeliOS_Arduino.h>
#include <math.h>
#include "system/shared_state.h"
#include "comm/serial_transport.h"
#include "comm/appl_protocol.h"
#include "config/pin_config.h"
#include "config/device_config.h"
#include "config/feature_config.h"
#include "debug/debug_comm.h"

// Sensor → telemetry-float helpers.
//
// Wire-level convention: invalid / low-confidence sensor samples are sent
// as IEEE-754 NaN. The Pi-side bridge converts NaN → JSON `null`, so every
// downstream consumer (navigation, behavior trees, UI) can branch on
// "missing reading" cleanly without inventing magic-number sentinels.
//
// 65535 matches TOFFilter::TOF_MAX_MM (the "fully-decayed, no confident
// target" output of the per-slot median/EMA/hold/decay pipeline).
static constexpr uint16_t TOF_NO_TARGET_SENTINEL = 65535;

static inline float tof_to_telem(uint16_t mm, bool sensor_ok) {
    if (!sensor_ok) return NAN;
    // 0 = slot not configured at boot; 65535 = decayed-out no-target.
    if (mm == 0 || mm >= TOF_NO_TARGET_SENTINEL) return NAN;
    return (float)mm;
}

static inline float imu_to_telem(float v, bool sensor_ok) {
    return sensor_ok ? v : NAN;
}

static inline float lidar_to_telem(uint16_t mm, bool sensor_ok) {
    if (!sensor_ok) return NAN;
    if (mm == 0) return NAN;   // 0 = no return in this sector
    return (float)mm;
}

#if FEATURE_PI_COMM_ENABLED

static SerialTransport s_transport;

// Round-robin counters (each TX tick advances one step)
static uint8_t s_servoRR = 0;                  // 0..6 — one servo per tick
static uint8_t s_motorRR = 0;                  // 0..1 — one motor per tick
static uint8_t s_lidarRRBase = 0;              // 0..30 step 6 — 6 bins per tick
static constexpr uint8_t LIDAR_BINS_PER_TICK = 6;   // 36 bins / 6 = 300 ms full refresh at 20 Hz

// Map round-robin index to APPL device ID for servos / motors
static constexpr uint8_t SERVO_DEV_IDS[7] = {
    DEV_SERVO_NECK, DEV_SERVO_EAR_L, DEV_SERVO_EAR_R,
    DEV_SERVO_LEG_FL, DEV_SERVO_LEG_FR, DEV_SERVO_LEG_BL, DEV_SERVO_LEG_BR,
};
static constexpr uint8_t MOTOR_DEV_IDS[2] = { DEV_WHEEL_LEFT, DEV_WHEEL_RIGHT };

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
#if DEBUG_REQUESTS_ENABLED
        debug_print_response(rsp);
#endif
        s_transport.send(rsp);
    }

    // ---- 2. Periodic telemetry push (only for enabled modules) ----
#if FEATURE_IMU_ENABLED
    {
        // IMU validity gate: low confidence / not present → emit NaN for every
        // IMU and odometry-derived field so consumers can detect cleanly.
        const bool imu_ok = g_monitorData.sensor_ok_imu
                            && g_imuData.timestamp_ms != 0;

        s_transport.send(appl_make_telemetry(DEV_SYSTEM, PARAM_ODOM_X,       imu_to_telem(g_odomData.x_mm,       imu_ok)));
        s_transport.send(appl_make_telemetry(DEV_SYSTEM, PARAM_ODOM_Y,       imu_to_telem(g_odomData.y_mm,       imu_ok)));
        s_transport.send(appl_make_telemetry(DEV_SYSTEM, PARAM_ODOM_HEADING, imu_to_telem(g_odomData.heading_deg, imu_ok)));

        s_transport.send(appl_make_telemetry(DEV_IMU, PARAM_ORIENTATION_ROLL,  imu_to_telem(g_imuData.roll_deg,  imu_ok)));
        s_transport.send(appl_make_telemetry(DEV_IMU, PARAM_ORIENTATION_PITCH, imu_to_telem(g_imuData.pitch_deg, imu_ok)));
        s_transport.send(appl_make_telemetry(DEV_IMU, PARAM_ORIENTATION_YAW,   imu_to_telem(g_imuData.yaw_deg,   imu_ok)));

        // Quaternion + linear accel — terminated on ACCEL_Z so the Pi can flush as a group
        s_transport.send(appl_make_telemetry(DEV_IMU, PARAM_QUATERNION_W, imu_to_telem(g_imuData.qw,      imu_ok)));
        s_transport.send(appl_make_telemetry(DEV_IMU, PARAM_QUATERNION_X, imu_to_telem(g_imuData.qx,      imu_ok)));
        s_transport.send(appl_make_telemetry(DEV_IMU, PARAM_QUATERNION_Y, imu_to_telem(g_imuData.qy,      imu_ok)));
        s_transport.send(appl_make_telemetry(DEV_IMU, PARAM_QUATERNION_Z, imu_to_telem(g_imuData.qz,      imu_ok)));
        s_transport.send(appl_make_telemetry(DEV_IMU, PARAM_ACCEL_X,      imu_to_telem(g_imuData.accel_x, imu_ok)));
        s_transport.send(appl_make_telemetry(DEV_IMU, PARAM_ACCEL_Y,      imu_to_telem(g_imuData.accel_y, imu_ok)));
        s_transport.send(appl_make_telemetry(DEV_IMU, PARAM_ACCEL_Z,      imu_to_telem(g_imuData.accel_z, imu_ok)));
    }
#endif

#if FEATURE_TOF_ENABLED
    // Order matters: BACK must be sent LAST so the Pi bridge uses it as the
    // group terminator to flush the aggregated MQTT message.
    s_transport.send(appl_make_telemetry(DEV_TOF_LEFT,  PARAM_DISTANCE_MM,
        tof_to_telem(g_perceptionData.tof_left_mm,  g_monitorData.sensor_ok_tof[0])));
    s_transport.send(appl_make_telemetry(DEV_TOF_RIGHT, PARAM_DISTANCE_MM,
        tof_to_telem(g_perceptionData.tof_right_mm, g_monitorData.sensor_ok_tof[1])));
    s_transport.send(appl_make_telemetry(DEV_TOF_FRONT, PARAM_DISTANCE_MM,
        tof_to_telem(g_perceptionData.tof_front_mm, g_monitorData.sensor_ok_tof[2])));
    s_transport.send(appl_make_telemetry(DEV_TOF_BACK,  PARAM_DISTANCE_MM,
        tof_to_telem(g_perceptionData.tof_back_mm,  g_monitorData.sensor_ok_tof[3])));
#endif

#if FEATURE_LIDAR_ENABLED
    // Stream a slice of the 36-bin polar histogram each tick.
    // Bridge accumulates all 36 params and publishes when the final bin arrives.
    {
        const bool lidar_ok = g_monitorData.sensor_ok_lidar;
        for (uint8_t k = 0; k < LIDAR_BINS_PER_TICK; k++) {
            uint8_t bin = (s_lidarRRBase + k) % 36;
            s_transport.send(appl_make_telemetry(
                DEV_LIDAR,
                (uint8_t)(PARAM_LIDAR_BIN_0 + bin),
                lidar_to_telem(g_perceptionData.lidar_bins[bin], lidar_ok)));
        }
        s_lidarRRBase = (s_lidarRRBase + LIDAR_BINS_PER_TICK) % 36;
    }
#endif

#if FEATURE_SERVOS_ENABLED
    // Round-robin one servo per tick (7 servos -> full refresh every 350 ms).
    // Termination marker: the LOAD param is sent last in the per-servo group so
    // the Pi bridge knows when to emit the MQTT message.
    {
        uint8_t idx = s_servoRR;
        uint8_t dev = SERVO_DEV_IDS[idx];
        const ServoStatus& s = g_servoStatus[idx];
        s_transport.send(appl_make_telemetry(dev, PARAM_POSITION,    (float)s.position));
        s_transport.send(appl_make_telemetry(dev, PARAM_VELOCITY,    (float)s.speed));
        s_transport.send(appl_make_telemetry(dev, PARAM_TEMPERATURE, (float)s.temperature));
        s_transport.send(appl_make_telemetry(dev, PARAM_VOLTAGE,     (float)s.voltage));
        s_transport.send(appl_make_telemetry(dev, PARAM_CURRENT,     (float)s.current));
        s_transport.send(appl_make_telemetry(dev, PARAM_LOAD,        (float)s.load));   // group terminator
        s_servoRR = (s_servoRR + 1) % 7;
    }
#endif

#if FEATURE_MOTORS_ENABLED
    // Round-robin one motor per tick (2 motors -> full refresh every 100 ms).
    // Termination marker: FAULT_CODE sent last.
    {
        uint8_t idx = s_motorRR;
        uint8_t dev = MOTOR_DEV_IDS[idx];
        const DDSM210Status& m = g_motorStatus[idx];
        s_transport.send(appl_make_telemetry(dev, PARAM_VELOCITY,    m.speed_rpm));
        s_transport.send(appl_make_telemetry(dev, PARAM_POSITION,    (float)m.position));
        s_transport.send(appl_make_telemetry(dev, PARAM_CURRENT,     m.current_a));
        s_transport.send(appl_make_telemetry(dev, PARAM_TEMPERATURE, (float)m.temperature_c));
        s_transport.send(appl_make_telemetry(dev, PARAM_FAULT_CODE,  (float)m.fault_code));   // group terminator
        s_motorRR = (s_motorRR + 1) % 2;
    }
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
