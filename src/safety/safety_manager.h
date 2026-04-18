#pragma once
// =============================================================================
// safety_manager.h — Centralised safety evaluation
// =============================================================================
// Reads perception, IMU, and monitor data; writes g_safetyBits.
// Called from motion control task at 50 Hz.
// =============================================================================

#include <Arduino.h>
#include "system/shared_state.h"

class SafetyManager {
public:
    SafetyManager();

    // Evaluate all safety conditions, update g_safetyBits.
    void evaluate(const PerceptionData& perc,
                  const IMUData& imu,
                  const MonitorData& mon);

    float speedScaleFactor() const { return _speedScale; }
    bool  isHardStop()       const { return _hardStop; }
    uint8_t currentBits()    const { return _bits; }

private:
    void checkObstacles(const PerceptionData& perc);
    void checkTilt(const IMUData& imu);
    void checkPickup(const IMUData& imu);
    void checkBattery(const MonitorData& mon);
    void checkWatchdog(const MonitorData& mon);
    void checkTemperature(const MonitorData& mon);

    float   _speedScale;
    bool    _hardStop;
    uint8_t _bits;
    uint8_t _prevBits;
};
