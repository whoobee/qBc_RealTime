#include "tasks/task_motion_control.h"
#include <HeliOS_Arduino.h>
#include "system/shared_state.h"
#include "safety/safety_manager.h"
#include "comm/appl_protocol.h"
#include "config/safety_config.h"
#include "config/device_config.h"

// =============================================================================
// Per-servo configured acceleration (set via CMD_CONFIGURE, used by CMD_DRIVE)
// =============================================================================
static uint8_t s_servoAccel[7] = {0};

// =============================================================================
// Helper: dispatch a single request packet, build a response
// =============================================================================
static ResponsePacket handleRequest(const RequestPacket& req,
                                    const SafetyManager& safety) {
    ResponsePacket rsp = appl_make_response(req, ERR_OK);

    uint8_t dev = req.device_id;
    uint8_t par = req.parameter;
    uint8_t cmd = req.command;
    bool    isWrite = (req.rw_flag == RW_WRITE);

    // ---- Heartbeat (special case: updates watchdog timer) ----
    if (par == PARAM_HEARTBEAT && dev == DEV_SYSTEM) {
        g_monitorData.last_pi_heartbeat_ms = millis();
        return rsp;
    }

    // ---- Safety status read ----
    if (cmd == CMD_READ && par == PARAM_SAFETY_STATUS) {
        appl_pack_uint32(safety.currentBits(), rsp.value);
        return rsp;
    }

    // ---- CONFIGURE commands ----
    if (cmd == CMD_CONFIGURE && isWrite) {
        if (dev >= DEV_SERVO_NECK && dev <= DEV_SERVO_LEG_BR) {
            uint8_t idx = dev - DEV_SERVO_NECK;
            if (par == PARAM_ACCELERATION) {
                s_servoAccel[idx] = (uint8_t)appl_unpack_uint32(req.value);
                return rsp;
            }
            rsp.error = ERR_UNKNOWN_PARAM;
            return rsp;
        }
        rsp.error = ERR_UNKNOWN_DEVICE;
        return rsp;
    }

    // ---- DRIVE commands ----
    if (cmd == CMD_DRIVE && isWrite) {
        // Block all drive commands during hard stop
        if (safety.isHardStop()) {
            rsp.error = ERR_SAFETY_BLOCK;
            return rsp;
        }

        float val = appl_unpack_float(req.value);

        // --- Wheel motors ---
        if (dev == DEV_WHEEL_LEFT || dev == DEV_WHEEL_RIGHT ||
            dev == GRP_ALL_WHEELS) {

            float scaled = val * safety.speedScaleFactor();
            MotorCommand mcmd = {};

            if (dev == GRP_ALL_WHEELS) {
                mcmd.left_rpm  = scaled;
                mcmd.right_rpm = scaled;
            } else {
                if (dev == DEV_WHEEL_LEFT)  mcmd.left_rpm  = scaled;
                if (dev == DEV_WHEEL_RIGHT) mcmd.right_rpm = scaled;
            }
            g_motorCmd = mcmd;
            g_motorCmdPending = true;
            return rsp;
        }

        // --- Servos ---
        if (dev >= DEV_SERVO_NECK && dev <= DEV_SERVO_LEG_BR) {
            ServoCommand scmd;
            scmd.servo_appl_id = dev;
            if (par == PARAM_POSITION) {
                // value[4] = [position:uint16_LE][speed:uint16_LE]
                uint16_t pos, spd;
                memcpy(&pos, req.value, 2);
                memcpy(&spd, req.value + 2, 2);
                scmd.position     = pos;
                scmd.speed        = (spd > 0) ? spd : 500;
                scmd.acceleration = s_servoAccel[dev - DEV_SERVO_NECK];
            } else if (par == PARAM_VELOCITY) {
                scmd.position = 0;
                scmd.speed = (uint16_t)val;
                scmd.acceleration = 0;
            } else {
                rsp.error = ERR_UNKNOWN_PARAM;
                return rsp;
            }
            g_servoCmdBuf.push(scmd);
            return rsp;
        }

        rsp.error = ERR_UNKNOWN_DEVICE;
        return rsp;
    }

    // ---- READ commands ----
    if (cmd == CMD_READ) {
        // Wheel motor status
        if (dev == DEV_WHEEL_LEFT || dev == DEV_WHEEL_RIGHT) {
            uint8_t idx = (dev == DEV_WHEEL_LEFT) ? 0 : 1;
            float v = 0;
            switch (par) {
                case PARAM_VELOCITY:    v = g_motorStatus[idx].speed_rpm; break;
                case PARAM_CURRENT:     v = g_motorStatus[idx].current_a; break;
                case PARAM_TEMPERATURE: v = (float)g_motorStatus[idx].temperature_c; break;
                case PARAM_FAULT_CODE:  v = (float)g_motorStatus[idx].fault_code; break;
                default: rsp.error = ERR_UNKNOWN_PARAM; break;
            }
            appl_pack_float(v, rsp.value);
            return rsp;
        }

        // Servo status
        if (dev >= DEV_SERVO_NECK && dev <= DEV_SERVO_LEG_BR) {
            uint8_t idx = dev - DEV_SERVO_NECK;
            if (idx >= 7) { rsp.error = ERR_UNKNOWN_DEVICE; return rsp; }
            float v = 0;
            switch (par) {
                case PARAM_POSITION:    v = (float)g_servoStatus[idx].position; break;
                case PARAM_VELOCITY:    v = (float)g_servoStatus[idx].speed; break;
                case PARAM_LOAD:        v = (float)g_servoStatus[idx].load; break;
                case PARAM_TEMPERATURE: v = (float)g_servoStatus[idx].temperature; break;
                case PARAM_VOLTAGE:     v = (float)g_servoStatus[idx].voltage; break;
                case PARAM_CURRENT:     v = (float)g_servoStatus[idx].current; break;
                default: rsp.error = ERR_UNKNOWN_PARAM; break;
            }
            appl_pack_float(v, rsp.value);
            return rsp;
        }

        // IMU
        if (dev == DEV_IMU) {
            float v = 0;
            switch (par) {
                case PARAM_ORIENTATION_ROLL:  v = g_imuData.roll_deg; break;
                case PARAM_ORIENTATION_PITCH: v = g_imuData.pitch_deg; break;
                case PARAM_ORIENTATION_YAW:   v = g_imuData.yaw_deg; break;
                case PARAM_QUATERNION_W:      v = g_imuData.qw; break;
                case PARAM_QUATERNION_X:      v = g_imuData.qx; break;
                case PARAM_QUATERNION_Y:      v = g_imuData.qy; break;
                case PARAM_QUATERNION_Z:      v = g_imuData.qz; break;
                case PARAM_ACCEL_X:           v = g_imuData.accel_x; break;
                case PARAM_ACCEL_Y:           v = g_imuData.accel_y; break;
                case PARAM_ACCEL_Z:           v = g_imuData.accel_z; break;
                default: rsp.error = ERR_UNKNOWN_PARAM; break;
            }
            appl_pack_float(v, rsp.value);
            return rsp;
        }

        // TOF sensors
        if (dev >= DEV_TOF_LEFT && dev <= DEV_TOF_BACK && par == PARAM_DISTANCE_MM) {
            float d = 0;
            if (dev == DEV_TOF_LEFT)  d = (float)g_perceptionData.tof_left_mm;
            if (dev == DEV_TOF_RIGHT) d = (float)g_perceptionData.tof_right_mm;
            if (dev == DEV_TOF_BACK)  d = (float)g_perceptionData.tof_back_mm;
            appl_pack_float(d, rsp.value);
            return rsp;
        }

        // Battery
        if (dev == DEV_BATTERY) {
            float v = 0;
            if (par == PARAM_VOLTAGE) v = g_monitorData.battery_voltage;
            if (par == PARAM_CURRENT) v = g_monitorData.battery_current;
            appl_pack_float(v, rsp.value);
            return rsp;
        }

        // Odometry
        if (dev == DEV_SYSTEM && (par >= PARAM_ODOM_X && par <= PARAM_ODOM_HEADING)) {
            float v = 0;
            if (par == PARAM_ODOM_X)       v = g_odomData.x_mm;
            if (par == PARAM_ODOM_Y)       v = g_odomData.y_mm;
            if (par == PARAM_ODOM_HEADING) v = g_odomData.heading_deg;
            appl_pack_float(v, rsp.value);
            return rsp;
        }

        rsp.error = ERR_UNKNOWN_DEVICE;
        return rsp;
    }

    rsp.error = ERR_UNKNOWN_CMD;
    return rsp;
}

// =============================================================================
// File-static safety manager
// =============================================================================
static SafetyManager s_safety;

// =============================================================================
// HeliOS timer callback — 50 Hz (20 000 µs)
// =============================================================================
static void motionControlCallback(TaskId_t id_) {
    // ---- Evaluate safety using current shared state ----
    s_safety.evaluate(g_perceptionData, g_imuData, g_monitorData);

    // If hard stop, force wheels to zero immediately
    if (s_safety.isHardStop()) {
        g_motorCmd = { 0.0f, 0.0f };
        g_motorCmdPending = true;
    }

    // ---- Process incoming Pi commands ----
    RequestPacket req;
    while (g_rxCommandBuf.pop(req)) {
        ResponsePacket rsp = handleRequest(req, s_safety);
        g_txResponseBuf.push(rsp);
    }
}

// =============================================================================
void task_motion_control_init() {
    TaskId_t id = xTaskAdd("MotionCtrl", &motionControlCallback);
    xTaskWait(id);
    xTaskSetTimer(id, 20000);   // 50 Hz
}
