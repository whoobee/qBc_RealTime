#include "safety/safety_manager.h"
#include "config/safety_config.h"
#include "config/feature_config.h"
#include "config/pin_config.h"
#include <cmath>

SafetyManager::SafetyManager()
    : _speedScale(1.0f), _hardStop(false), _bits(0), _prevBits(0xFF) {}

void SafetyManager::evaluate(const PerceptionData& perc,
                             const IMUData& imu,
                             const MonitorData& mon) {
    _bits       = 0;
    _hardStop   = false;
    _speedScale = 1.0f;

#if !FEATURE_SAFETY_ENABLED
    // Safety bypassed — all clear
    g_safetyBits = 0;
    return;
#else
    checkObstacles(perc);
    checkTilt(imu);
    checkPickup(imu);
    checkBattery(mon);
    checkWatchdog(mon);
    checkTemperature(mon);

    g_safetyBits = _bits;
    _hardStop = (_bits & SAFETY_ANY_CRITICAL) != 0;

#if DEBUG_COMM_ENABLED
    // Log only on state change to avoid flooding
    uint8_t entered = _bits & ~_prevBits;
    uint8_t cleared = _prevBits & ~_bits;
    if (entered || cleared) {
        if (entered & SAFETY_OBSTACLE_BIT) {
            SERIAL_DEBUG.print(F("[SAFETY] OBSTACLE  tofL="));
            SERIAL_DEBUG.print(perc.tof_left_mm);
            SERIAL_DEBUG.print(F(" tofR="));
            SERIAL_DEBUG.print(perc.tof_right_mm);
            SERIAL_DEBUG.print(F(" tofF="));
            SERIAL_DEBUG.print(perc.tof_front_mm);
            SERIAL_DEBUG.print(F(" tofB="));
            SERIAL_DEBUG.print(perc.tof_back_mm);
            SERIAL_DEBUG.print(F(" lidarF="));
            SERIAL_DEBUG.print(perc.lidar_min_front_mm);
            SERIAL_DEBUG.print(F(" lidarL="));
            SERIAL_DEBUG.print(perc.lidar_min_left_mm);
            SERIAL_DEBUG.print(F(" lidarR="));
            SERIAL_DEBUG.print(perc.lidar_min_right_mm);
            SERIAL_DEBUG.print(F(" lidarB="));
            SERIAL_DEBUG.println(perc.lidar_min_back_mm);
        }
        if (entered & SAFETY_TILT_BIT) {
            SERIAL_DEBUG.print(F("[SAFETY] TILT  roll="));
            SERIAL_DEBUG.print(imu.roll_deg);
            SERIAL_DEBUG.print(F(" pitch="));
            SERIAL_DEBUG.println(imu.pitch_deg);
        }
        if (entered & SAFETY_PICKUP_BIT) {
            SERIAL_DEBUG.print(F("[SAFETY] PICKUP  accelZ="));
            SERIAL_DEBUG.println(imu.accel_z);
        }
        if (entered & SAFETY_LOWBATT_BIT) {
            SERIAL_DEBUG.print(F("[SAFETY] LOW_BATTERY  V="));
            SERIAL_DEBUG.println(mon.battery_voltage);
        }
        if (entered & SAFETY_WATCHDOG_BIT) {
            SERIAL_DEBUG.print(F("[SAFETY] WATCHDOG  elapsed="));
            SERIAL_DEBUG.println(millis() - mon.last_pi_heartbeat_ms);
        }
        if (entered & SAFETY_OVERTEMP_BIT) {
            SERIAL_DEBUG.print(F("[SAFETY] OVERTEMP  motorL="));
            SERIAL_DEBUG.print(mon.motor_temp_left);
            SERIAL_DEBUG.print(F(" motorR="));
            SERIAL_DEBUG.println(mon.motor_temp_right);
        }
        if (cleared) {
            SERIAL_DEBUG.print(F("[SAFETY] CLEARED bits=0x"));
            SERIAL_DEBUG.println(cleared, HEX);
        }
    }
    _prevBits = _bits;
#endif
#endif
}

void SafetyManager::checkObstacles(const PerceptionData& perc) {
    uint16_t min_mm = UINT16_MAX;

    uint16_t tof_vals[] = { perc.tof_left_mm, perc.tof_right_mm,
                            perc.tof_front_mm, perc.tof_back_mm };
    for (int i = 0; i < 4; i++) {
        if (tof_vals[i] > 0 && tof_vals[i] < min_mm) min_mm = tof_vals[i];
    }

    uint16_t lidar_vals[] = { perc.lidar_min_front_mm, perc.lidar_min_left_mm,
                              perc.lidar_min_right_mm, perc.lidar_min_back_mm };
    for (int i = 0; i < 4; i++) {
        if (lidar_vals[i] > 0 && lidar_vals[i] < min_mm) min_mm = lidar_vals[i];
    }

    if (min_mm <= OBSTACLE_CRITICAL_MM) {
        _bits |= SAFETY_OBSTACLE_BIT;
        _speedScale = 0.0f;
    } else if (min_mm <= OBSTACLE_WARNING_MM) {
        _speedScale = OBSTACLE_WARNING_SPEED_FACTOR;
    }
}

void SafetyManager::checkTilt(const IMUData& imu) {
    if (fabsf(imu.roll_deg) > TILT_THRESHOLD_DEG ||
        fabsf(imu.pitch_deg) > TILT_THRESHOLD_DEG) {
        _bits |= SAFETY_TILT_BIT;
        _speedScale = 0.0f;
    }
}

void SafetyManager::checkPickup(const IMUData& imu) {
    if (imu.accel_z < -FREEFALL_ACCEL_MS2) {
        _bits |= SAFETY_PICKUP_BIT;
        _speedScale = 0.0f;
    }
}

void SafetyManager::checkBattery(const MonitorData& mon) {
    if (mon.battery_voltage > 0.0f && mon.battery_voltage < BATTERY_LOW_VOLTAGE_V) {
        _bits |= SAFETY_LOWBATT_BIT;
        if (mon.battery_voltage < BATTERY_CRITICAL_V) {
            _speedScale = 0.0f;
        }
    }
}

void SafetyManager::checkWatchdog(const MonitorData& mon) {
    uint32_t elapsed = millis() - mon.last_pi_heartbeat_ms;
    if (elapsed > PI_WATCHDOG_TIMEOUT_MS) {
        _bits |= SAFETY_WATCHDOG_BIT;
        _speedScale = 0.0f;
    }
}

void SafetyManager::checkTemperature(const MonitorData& mon) {
    if (mon.motor_temp_left  >= MOTOR_TEMP_CRITICAL_C ||
        mon.motor_temp_right >= MOTOR_TEMP_CRITICAL_C) {
        _bits |= SAFETY_OVERTEMP_BIT;
        _speedScale = 0.0f;
    }
}
