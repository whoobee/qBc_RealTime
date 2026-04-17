#pragma once
// =============================================================================
// vl53l0x_array.h — 1..3 VL53L0X TOF sensors on shared Wire2 bus
// =============================================================================
// Uses the local-fork DFRobot_VL53L0X library (lib/DFRobot_VL53L0X/), which is
// hardcoded to Wire2 (Teensy SDA2=25, SCL2=24). For multi-sensor setups the
// per-sensor XSHUT pin gives us a deterministic power-on sequence so each
// sensor can be assigned a unique I2C address before the next is woken.
//
// Init sequence (per sensor slot that has a valid XSHUT pin):
//   1. All XSHUT pins LOW (all configured sensors held in reset)
//   2. For slot i: XSHUT[i] HIGH → wait >1.2 ms → sensor.begin(addr[i])
//      → the DFRobot lib reassigns the sensor to `addr[i]` so it no longer
//      collides with others at 0x29 when we wake the next slot.
//
// Slots whose XSHUT pin is TOF_XSHUT_NOT_WIRED (0xFF) are skipped at init
// and readAll() will report 0 mm for that index.
// =============================================================================

#include <Arduino.h>
#include <DFRobot_VL53L0X.h>

class VL53L0XArray {
public:
    static constexpr uint8_t SENSOR_COUNT = 3;

    enum Sensor : uint8_t { LEFT = 0, RIGHT = 1, BACK = 2 };

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
