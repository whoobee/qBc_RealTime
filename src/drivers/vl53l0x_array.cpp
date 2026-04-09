#include "drivers/vl53l0x_array.h"
#include "config/pin_config.h"
#include "config/device_config.h"

VL53L0XArray::VL53L0XArray()
    : _ok{false, false, false}
    , _xshutPins{PIN_TOF_XSHUT_LEFT, PIN_TOF_XSHUT_RIGHT, PIN_TOF_XSHUT_BACK}
    , _addresses{TOF_ADDR_LEFT, TOF_ADDR_RIGHT, TOF_ADDR_BACK}
{}

// =============================================================================
// Power-up sequence: one sensor at a time, assign unique I2C addresses
// =============================================================================
uint8_t VL53L0XArray::begin() {
    // Step 1 — All XSHUT LOW → all sensors held in reset
    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
        pinMode(_xshutPins[i], OUTPUT);
        digitalWrite(_xshutPins[i], LOW);
    }
    delay(10);

    uint8_t count = 0;

    // Step 2 — Bring up each sensor one at a time
    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
        // Release this sensor from reset
        digitalWrite(_xshutPins[i], HIGH);
        delay(10);  // VL53L0X boot time

        // Sensor comes up at default address (0x29).
        // begin() talks to 0x29 then reprograms to _addresses[i].
        if (_lox[i].begin(_addresses[i], false, &Wire)) {
            _ok[i] = true;
            count++;
        } else {
            _ok[i] = false;
        }
    }

    return count;
}

// =============================================================================
uint16_t VL53L0XArray::readRange(Sensor idx) {
    if (!_ok[idx]) return 0;

    VL53L0X_RangingMeasurementData_t measure;
    _lox[idx].rangingTest(&measure, false);

    // RangeStatus 4 = "phase failure" (out of range / no target)
    if (measure.RangeStatus == 4) return 0;
    return measure.RangeMilliMeter;
}

void VL53L0XArray::readAll(uint16_t distances_mm[SENSOR_COUNT]) {
    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
        distances_mm[i] = readRange(static_cast<Sensor>(i));
    }
}
