#pragma once
// =============================================================================
// appl_protocol.h — Application-layer protocol (PLATFORM AGNOSTIC)
// =============================================================================
// This header defines the request/response packet schema for the
// Raspberry Pi (master) ↔ Teensy (slave) communication link.
//
// *** NO Arduino / platform includes. Only <stdint.h> + <string.h>. ***
// This file compiles identically on Teensy (ARM g++) and RPi (g++ / Python ctypes).
//
// Transport: PacketSerial (COBS) handles framing — this layer only
// defines the byte layout INSIDE each COBS frame.
// =============================================================================

#include <stdint.h>
#include <string.h>

// ---- Packet type (direction) ------------------------------------------------
enum ApplPktType : uint8_t {
    PKT_REQUEST   = 0x01,  // Pi  → Teensy
    PKT_RESPONSE  = 0x02,  // Teensy → Pi  (reply to a request)
    PKT_TELEMETRY = 0x03,  // Teensy → Pi  (unsolicited periodic push)
};

// ---- Command types ----------------------------------------------------------
enum ApplCommand : uint8_t {
    CMD_DRIVE     = 0x01,  // Control actuators (velocity, position, torque)
    CMD_CONFIGURE = 0x02,  // Set / get device configuration parameters
    CMD_READ      = 0x03,  // Read sensor / actuator status
};

// ---- Read / Write flag ------------------------------------------------------
enum ApplRWFlag : uint8_t {
    RW_READ  = 0x00,
    RW_WRITE = 0x01,
};

// ---- Device IDs (individual + groups) ---------------------------------------
// Namespace layout:
//   0x00       System / global
//   0x10-0x1F  Wheel motors  (DDSM210)
//   0x20-0x2F  Servos        (ST3215)
//   0x30-0x3F  TOF sensors   (VL53L0X)
//   0x40-0x4F  IMU           (BNO055)
//   0x50-0x5F  LIDAR         (YDLidar GS2)
//   0x60-0x6F  Battery / power
//   0xE0-0xEF  Group IDs
//   0xFF       Broadcast
enum ApplDeviceID : uint8_t {
    // ---- Individual devices ----
    DEV_SYSTEM          = 0x00,

    // Wheel motors
    DEV_WHEEL_LEFT      = 0x10,
    DEV_WHEEL_RIGHT     = 0x11,

    // ST3215 Servos
    DEV_SERVO_NECK      = 0x20,
    DEV_SERVO_EAR_L     = 0x21,
    DEV_SERVO_EAR_R     = 0x22,
    DEV_SERVO_LEG_FL    = 0x23,
    DEV_SERVO_LEG_FR    = 0x24,
    DEV_SERVO_LEG_BL    = 0x25,
    DEV_SERVO_LEG_BR    = 0x26,

    // TOF sensors
    DEV_TOF_LEFT        = 0x30,
    DEV_TOF_RIGHT       = 0x31,
    DEV_TOF_BACK        = 0x32,

    // IMU
    DEV_IMU             = 0x40,

    // LIDAR
    DEV_LIDAR           = 0x50,

    // Battery
    DEV_BATTERY         = 0x60,

    // ---- Group IDs ----
    GRP_ALL_WHEELS      = 0xE0,
    GRP_ALL_SERVOS      = 0xE1,
    GRP_ALL_TOFS        = 0xE2,
    GRP_ALL_SENSORS     = 0xE3,
    GRP_ALL             = 0xFF,
};

// ---- Parameters (shared across command types) -------------------------------
// A parameter means "which property" to read / write / drive.
// Context depends on the device:
//   e.g. PARAM_POSITION on a servo = target angle,
//        PARAM_POSITION on a motor = encoder position.
enum ApplParam : uint8_t {
    // Drive / status parameters (actuators + sensors)
    PARAM_VELOCITY          = 0x01,  // RPM or rad/s (float)
    PARAM_POSITION          = 0x02,  // Degrees or encoder ticks (float)
    PARAM_TORQUE_ENABLE     = 0x03,  // 0 / 1 (int32)
    PARAM_ACCELERATION      = 0x04,  // units vary per device (float)
    PARAM_SPEED             = 0x05,  // Servo speed register (uint16 inside int32)

    // Sensor read parameters
    PARAM_TEMPERATURE       = 0x10,  // °C (float)
    PARAM_VOLTAGE           = 0x11,  // Volts (float)
    PARAM_CURRENT           = 0x12,  // Amps (float)
    PARAM_LOAD              = 0x13,  // 0-100 % (float)
    PARAM_DISTANCE_MM       = 0x14,  // mm (float)
    PARAM_ORIENTATION_ROLL  = 0x15,  // degrees (float)
    PARAM_ORIENTATION_PITCH = 0x16,
    PARAM_ORIENTATION_YAW   = 0x17,
    PARAM_QUATERNION_W      = 0x18,
    PARAM_QUATERNION_X      = 0x19,
    PARAM_QUATERNION_Y      = 0x1A,
    PARAM_QUATERNION_Z      = 0x1B,
    PARAM_ACCEL_X           = 0x1C,
    PARAM_ACCEL_Y           = 0x1D,
    PARAM_ACCEL_Z           = 0x1E,

    // Odometry
    PARAM_ODOM_X            = 0x20,
    PARAM_ODOM_Y            = 0x21,
    PARAM_ODOM_HEADING      = 0x22,

    // Configuration parameters
    PARAM_POS_LIMIT_MIN     = 0x30,
    PARAM_POS_LIMIT_MAX     = 0x31,
    PARAM_VEL_LIMIT         = 0x32,
    PARAM_TORQUE_LIMIT      = 0x33,
    PARAM_CONTROL_MODE      = 0x34,  // 0=speed, 1=position, 2=open-loop
    PARAM_OBSTACLE_THRESH   = 0x35,

    // System
    PARAM_SAFETY_STATUS     = 0x40,  // Bitmask of SAFETY_* bits
    PARAM_HEARTBEAT         = 0x41,  // Pi → Teensy keepalive
    PARAM_FAULT_CODE        = 0x42,

    // Lidar polar histogram — 36 bins × 10° covering 0..360° CCW from front.
    // Encoded as param = PARAM_LIDAR_BIN_0 + bin_index, value = min distance (mm, float).
    // Bin i covers angles [i*10, i*10+10).
    PARAM_LIDAR_BIN_0       = 0x80,
    PARAM_LIDAR_BIN_35      = 0xA3,  // inclusive upper bound
};

// ---- Error codes (byte 10 in ResponsePacket) --------------------------------
enum ApplError : uint8_t {
    ERR_OK              = 0x00,
    ERR_UNKNOWN_CMD     = 0x01,
    ERR_UNKNOWN_DEVICE  = 0x02,
    ERR_UNKNOWN_PARAM   = 0x03,
    ERR_DEVICE_FAULT    = 0x04,  // Underlying hardware reported an error
    ERR_SAFETY_BLOCK    = 0x05,  // Command rejected by safety layer
    ERR_TIMEOUT         = 0x06,  // Device did not respond in time
    ERR_INVALID_VALUE   = 0x07,  // Value out of allowed range
    ERR_NOT_READY       = 0x08,  // Device not initialised yet
};

// ---- Packet structures (packed, no padding) ---------------------------------
// RequestPacket:  10 bytes   |  ResponsePacket: 11 bytes
#pragma pack(push, 1)

struct RequestPacket {
    uint8_t  pkt_type;    // PKT_REQUEST (0x01)
    uint8_t  sequence;    // 0–254 request ID;  0xFF reserved for telemetry
    uint8_t  command;     // ApplCommand
    uint8_t  device_id;   // ApplDeviceID
    uint8_t  rw_flag;     // ApplRWFlag
    uint8_t  parameter;   // ApplParam
    uint8_t  value[4];    // Payload for WRITE (little-endian float or int32)
};
// sizeof(RequestPacket) == 10

struct ResponsePacket {
    uint8_t  pkt_type;    // PKT_RESPONSE (0x02) or PKT_TELEMETRY (0x03)
    uint8_t  sequence;    // Echo of request sequence (or 0xFF for telemetry)
    uint8_t  command;     // Echo
    uint8_t  device_id;   // Echo
    uint8_t  rw_flag;     // Echo
    uint8_t  parameter;   // Echo
    uint8_t  value[4];    // Returned value (READ) or echo (WRITE)
    uint8_t  error;       // ApplError — isolated from device faults
};
// sizeof(ResponsePacket) == 11

#pragma pack(pop)

// Telemetry sequence marker
#define SEQ_TELEMETRY  0xFF

// ---- Value packing helpers (little-endian, works on both ARM targets) -------
inline void appl_pack_float(float f, uint8_t dst[4]) {
    memcpy(dst, &f, 4);
}
inline float appl_unpack_float(const uint8_t src[4]) {
    float f;
    memcpy(&f, src, 4);
    return f;
}
inline void appl_pack_int32(int32_t v, uint8_t dst[4]) {
    memcpy(dst, &v, 4);
}
inline int32_t appl_unpack_int32(const uint8_t src[4]) {
    int32_t v;
    memcpy(&v, src, 4);
    return v;
}
inline void appl_pack_uint32(uint32_t v, uint8_t dst[4]) {
    memcpy(dst, &v, 4);
}
inline uint32_t appl_unpack_uint32(const uint8_t src[4]) {
    uint32_t v;
    memcpy(&v, src, 4);
    return v;
}

// ---- Quick response builder -------------------------------------------------
inline ResponsePacket appl_make_response(const RequestPacket& req,
                                         ApplError err = ERR_OK) {
    ResponsePacket rsp;
    rsp.pkt_type  = PKT_RESPONSE;
    rsp.sequence  = req.sequence;
    rsp.command   = req.command;
    rsp.device_id = req.device_id;
    rsp.rw_flag   = req.rw_flag;
    rsp.parameter = req.parameter;
    memset(rsp.value, 0, 4);
    rsp.error     = err;
    return rsp;
}

inline ResponsePacket appl_make_telemetry(uint8_t device_id,
                                          uint8_t parameter,
                                          float   val) {
    ResponsePacket rsp;
    rsp.pkt_type  = PKT_TELEMETRY;
    rsp.sequence  = SEQ_TELEMETRY;
    rsp.command   = CMD_READ;
    rsp.device_id = device_id;
    rsp.rw_flag   = RW_READ;
    rsp.parameter = parameter;
    appl_pack_float(val, rsp.value);
    rsp.error     = ERR_OK;
    return rsp;
}
