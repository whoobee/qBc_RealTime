#pragma once
// =============================================================================
// vl53l0x_array.h — 3× VL53L0X TOF sensor array on shared I2C bus
// =============================================================================
// Init sequence:
//   1. All XSHUT pins LOW (all sensors off)
//   2. For each sensor: XSHUT HIGH → begin(new_addr) → reassigned
//   3. After init, all three run at unique I2C addresses
// =============================================================================

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>

class VL53L0XArray {
public:
    static constexpr uint8_t SENSOR_COUNT = 3;

    // Sensor indices
    enum Sensor : uint8_t { LEFT = 0, RIGHT = 1, BACK = 2 };

    VL53L0XArray();

    // Perform the power-up + address-assignment sequence.
    // Returns the number of sensors that initialised successfully (0–3).
    uint8_t begin();

    // Single-shot ranging on one sensor.  Returns distance in mm.
    // Returns 0 on error or out-of-range.
    uint16_t readRange(Sensor idx);

    // Read all three sensors into the supplied array (mm).
    void readAll(uint16_t distances_mm[SENSOR_COUNT]);

    // True if sensor initialised OK
    bool isReady(Sensor idx) const { return _ok[idx]; }

private:
    Adafruit_VL53L0X _lox[SENSOR_COUNT];
    bool             _ok[SENSOR_COUNT];
    uint8_t          _xshutPins[SENSOR_COUNT];
    uint8_t          _addresses[SENSOR_COUNT];
};
