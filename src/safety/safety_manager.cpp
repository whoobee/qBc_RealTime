#include "safety/safety_manager.h"
#include "config/safety_config.h"
#include "config/feature_config.h"
#include <cmath>

SafetyManager::SafetyManager()
    : _speedScale(1.0f), _hardStop(false), _bits(0) {}

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
#endif
}

void SafetyManager::checkObstacles(const PerceptionData& perc) {
    uint16_t min_mm = UINT16_MAX;

    uint16_t tof_vals[] = { perc.tof_left_mm, perc.tof_right_mm, perc.tof_back_mm };
    for (int i = 0; i < 3; i++) {
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
