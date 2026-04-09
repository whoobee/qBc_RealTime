#include "tasks/task_state_estimation.h"
#include <HeliOS_Arduino.h>
#include "system/shared_state.h"
#include "config/feature_config.h"

#if FEATURE_IMU_ENABLED
#include "drivers/bno055_imu.h"
#include <cmath>

static constexpr float WHEEL_CIRCUMF_MM = 204.2f;
static constexpr float MM_PER_REV       = WHEEL_CIRCUMF_MM;

static BNO055IMU s_imu;
static bool      s_imuOk = false;

static void stateEstimationCallback(TaskId_t id_) {
    if (!s_imuOk) return;

    static IMUReading reading = {};

    if (s_imu.read(reading)) {
        g_imuData.roll_deg  = reading.roll;
        g_imuData.pitch_deg = reading.pitch;
        g_imuData.yaw_deg   = reading.yaw;
        g_imuData.qw = reading.qw;
        g_imuData.qx = reading.qx;
        g_imuData.qy = reading.qy;
        g_imuData.qz = reading.qz;
        g_imuData.accel_x = reading.accel_x;
        g_imuData.accel_y = reading.accel_y;
        g_imuData.accel_z = reading.accel_z;
        g_imuData.timestamp_ms = millis();
    }

    // ---- Integrate odometry from wheel velocities ----
    float vL = g_motorStatus[0].speed_rpm;
    float vR = g_motorStatus[1].speed_rpm;

    float vL_mms = (vL / 60.0f) * MM_PER_REV;
    float vR_mms = (vR / 60.0f) * MM_PER_REV;

    float dt_s = 0.005f;
    float v_linear = (vL_mms + vR_mms) * 0.5f;

    float heading_rad = reading.yaw * (PI / 180.0f);

    g_odomData.x_mm       += v_linear * cosf(heading_rad) * dt_s;
    g_odomData.y_mm       += v_linear * sinf(heading_rad) * dt_s;
    g_odomData.heading_deg = reading.yaw;
    g_odomData.vel_left_rpm  = vL;
    g_odomData.vel_right_rpm = vR;
    g_odomData.timestamp_ms  = millis();
}
#endif // FEATURE_IMU_ENABLED

// =============================================================================
void task_state_estimation_init() {
#if !FEATURE_IMU_ENABLED
    return;
#else
    for (int attempt = 0; attempt < 5 && !s_imuOk; attempt++) {
        s_imuOk = s_imu.begin();
        if (!s_imuOk) delay(200);
    }
    g_monitorData.sensor_ok_imu = s_imuOk;

    TaskId_t id = xTaskAdd("StateEst", &stateEstimationCallback);
    xTaskWait(id);
    xTaskSetTimer(id, 5000);   // 200 Hz
#endif
}
