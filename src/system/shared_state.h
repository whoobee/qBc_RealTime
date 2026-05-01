#pragma once
// =============================================================================
// shared_state.h — Shared data structures + lightweight ring buffers
// =============================================================================
// HeliOS is cooperative (non-preemptive), so NO mutexes are needed.
// Only one task runs at a time — shared globals are safe to read/write.
// =============================================================================

#include <Arduino.h>
#include "comm/appl_protocol.h"

// =============================================================================
// Simple ring buffer (no locking needed — cooperative scheduler)
// =============================================================================
template <typename T, uint8_t N>
struct RingBuffer {
    T       items[N];
    uint8_t head = 0;
    uint8_t tail = 0;

    bool push(const T& item) {
        uint8_t next = (head + 1) % N;
        if (next == tail) return false;   // full — drop
        items[head] = item;
        head = next;
        return true;
    }

    bool pop(T& item) {
        if (head == tail) return false;   // empty
        item = items[tail];
        tail = (tail + 1) % N;
        return true;
    }

    bool peek(const T*& item) const {
        if (head == tail) return false;
        item = &items[tail];
        return true;
    }

    // Always succeeds — overwrites oldest entry if full.
    void overwrite(const T& item) {
        items[head] = item;
        uint8_t next = (head + 1) % N;
        if (next == tail) tail = (tail + 1) % N;   // drop oldest
        head = next;
    }

    bool    empty()   const { return head == tail; }
    uint8_t count()   const { return (head - tail + N) % N; }
};

// =============================================================================
// Shared data structures
// =============================================================================

struct OdometryData {
    float x_mm;
    float y_mm;
    float heading_deg;
    float vel_left_rpm;
    float vel_right_rpm;
    uint32_t timestamp_ms;
};

struct PerceptionData {
    uint16_t tof_left_mm;
    uint16_t tof_right_mm;
    uint16_t tof_front_mm;
    uint16_t tof_back_mm;
    uint16_t lidar_min_front_mm;
    uint16_t lidar_min_left_mm;
    uint16_t lidar_min_right_mm;
    uint16_t lidar_min_back_mm;
    // Full 360 deg polar histogram, 10 deg resolution.
    // bin[i] holds the min distance (mm) seen in sector [i*10, i*10+10), CCW from front.
    // 0 = no valid return in that sector.
    uint16_t lidar_bins[36];
    uint32_t timestamp_ms;
};

struct IMUData {
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
    float qw, qx, qy, qz;
    float accel_x, accel_y, accel_z;
    uint32_t timestamp_ms;
};

enum MotorCmdMode : uint8_t {
    MOTOR_CMD_VELOCITY = 0,   // value = RPM (signed)
    MOTOR_CMD_POSITION = 1,   // value = delta degrees of wheel rotation (signed)
};

struct MotorCommand {
    MotorCmdMode mode;
    float left_value;
    float right_value;
};

struct ServoCommand {
    uint8_t  servo_appl_id;
    uint16_t position;      // SMS_STS units, 0–4096
    uint16_t speed;
    uint8_t  acceleration;
};

struct MonitorData {
    float battery_voltage;
    float battery_current;
    uint8_t motor_temp_left;
    uint8_t motor_temp_right;
    bool sensor_ok_imu;
    bool sensor_ok_lidar;
    bool sensor_ok_tof[4];
    uint32_t last_pi_heartbeat_ms;
};

struct DDSM210Status {
    float speed_rpm;
    float current_a;
    uint8_t temperature_c;
    uint8_t fault_code;
    int32_t mileage;
    int16_t position;
};

struct ServoStatus {
    int16_t position;
    int16_t speed;
    int16_t load;
    uint8_t voltage;
    uint8_t temperature;
    int16_t current;
};

// =============================================================================
// Safety bitmask (replaces FreeRTOS event group)
// =============================================================================
#define SAFETY_OBSTACLE_BIT   (1 << 0)
#define SAFETY_TILT_BIT       (1 << 1)
#define SAFETY_PICKUP_BIT     (1 << 2)
#define SAFETY_LOWBATT_BIT    (1 << 3)
#define SAFETY_WATCHDOG_BIT   (1 << 4)
#define SAFETY_OVERTEMP_BIT   (1 << 5)

#define SAFETY_ANY_CRITICAL   (SAFETY_OBSTACLE_BIT | SAFETY_TILT_BIT | \
                               SAFETY_PICKUP_BIT | SAFETY_LOWBATT_BIT | \
                               SAFETY_WATCHDOG_BIT | SAFETY_OVERTEMP_BIT)

// =============================================================================
// Global shared state — safe to access directly (cooperative scheduling)
// =============================================================================

// Latest sensor data (written by sensor tasks, read by others)
extern IMUData          g_imuData;
extern OdometryData     g_odomData;
extern PerceptionData   g_perceptionData;
extern MonitorData      g_monitorData;
extern DDSM210Status    g_motorStatus[2];   // [0]=left, [1]=right
extern ServoStatus      g_servoStatus[7];

// Safety bitmask (written by safety_manager, read by motion control + comm)
extern uint8_t          g_safetyBits;

// Command / response ring buffers
extern RingBuffer<RequestPacket,  16> g_rxCommandBuf;    // CommRX → MotionCtrl
extern RingBuffer<ResponsePacket, 32> g_txResponseBuf;   // any task → CommTX
extern RingBuffer<ServoCommand,   10> g_servoCmdBuf;     // MotionCtrl → MotorCtrl

// Motor command (single value, overwrite)
extern MotorCommand g_motorCmd;
extern bool         g_motorCmdPending;

// =============================================================================
void shared_state_init();
