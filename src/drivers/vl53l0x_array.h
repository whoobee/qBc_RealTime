#pragma once
// =============================================================================
// vl53l0x_array.h — up to 4 VL53L0X TOF sensors on shared Wire2 bus
// =============================================================================
// Uses the local-fork DFRobot_VL53L0X library (lib/DFRobot_VL53L0X/), which is
// hardcoded to Wire2 (Teensy SDA2=25, SCL2=24).
//
// Init sequence:
//   1. All slots with a valid XSHUT pin → XSHUT LOW (sensors held in reset).
//      Any TOF_XSHUT_NONE slot stays awake at the default 0x29.
//   2. Reassign every TOF_XSHUT_NONE slot's I2C address FIRST. This is safe
//      because all XSHUT-controlled sensors are still in reset, so only one
//      sensor is alive at 0x29 at a time. (In practice we expect at most
//      one no-XSHUT sensor on the bus; multiple would collide at 0x29.)
//   3. For each XSHUT-controlled slot: XSHUT HIGH → wait >1.2 ms →
//      sensor.begin(addr) → the DFRobot lib reassigns the sensor away from
//      0x29 so it no longer collides when the next slot is woken.
//
// Slot sentinels (defined in pin_config.h):
//   TOF_XSHUT_NOT_WIRED — slot empty, skipped at init, readAll() = 0 mm
//   TOF_XSHUT_NONE      — sensor present but XSHUT not wired (always on)
// =============================================================================

#include <Arduino.h>
#include <DFRobot_VL53L0X.h>

class VL53L0XArray {
public:
    static constexpr uint8_t SENSOR_COUNT = 4;

    enum Sensor : uint8_t { LEFT = 0, RIGHT = 1, FRONT = 2, BACK = 3 };

    VL53L0XArray();

    // Perform XSHUT reset, sequential address assignment, and continuous-mode
    // start for all configured sensors. Wire2.begin() must be called BEFORE
    // this (from main.cpp). Returns the number of sensors that initialised.
    uint8_t begin();

    // Single-sensor range read (mm). Returns 0 if slot not wired / not ready.
    uint16_t readRange(Sensor idx);

    // Read all 3 slots into a caller-supplied array. Unwired slots → 0.
    void readAll(uint16_t distances_mm[SENSOR_COUNT]);

    bool isReady(Sensor idx) const {
        return idx < SENSOR_COUNT && _ok[idx];
    }

    // True if the slot has a valid XSHUT pin (i.e. is expected to be wired).
    bool isConfigured(Sensor idx) const;

private:
    DFRobot_VL53L0X _tof[SENSOR_COUNT];
    uint8_t         _xshutPins[SENSOR_COUNT];
    uint8_t         _addresses[SENSOR_COUNT];   // 7-bit, unique per slot
    bool            _ok[SENSOR_COUNT];
};
