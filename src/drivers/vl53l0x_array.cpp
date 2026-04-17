#include "drivers/vl53l0x_array.h"
#include "config/pin_config.h"
#include "config/device_config.h"

// Per-slot target 7-bit I2C addresses. Unique so sensors don't collide on
// Wire2 once all are awake. 0x29 is the default — we avoid it for clarity.
// The DFRobot lib passes whatever value you give straight to the sensor's
// SLAVE_DEVICE_ADDRESS register (7-bit, masked by 0x7F internally).
static constexpr uint8_t SLOT_ADDR_LEFT  = 0x30;
static constexpr uint8_t SLOT_ADDR_RIGHT = 0x31;
static constexpr uint8_t SLOT_ADDR_BACK  = 0x32;

VL53L0XArray::VL53L0XArray() {
    _xshutPins[LEFT]  = PIN_TOF_XSHUT_LEFT;
    _xshutPins[RIGHT] = PIN_TOF_XSHUT_RIGHT;
    _xshutPins[BACK]  = PIN_TOF_XSHUT_BACK;

    _addresses[LEFT]  = SLOT_ADDR_LEFT;
    _addresses[RIGHT] = SLOT_ADDR_RIGHT;
    _addresses[BACK]  = SLOT_ADDR_BACK;

    for (uint8_t i = 0; i < SENSOR_COUNT; i++) _ok[i] = false;
}

bool VL53L0XArray::isConfigured(Sensor idx) const {
    if (idx >= SENSOR_COUNT) return false;
    return _xshutPins[idx] != TOF_XSHUT_NOT_WIRED;
}

// =============================================================================
// Power-up sequence: one sensor at a time, assign unique I2C addresses.
// Skips slots whose XSHUT pin is TOF_XSHUT_NOT_WIRED (0xFF).
// =============================================================================
uint8_t VL53L0XArray::begin() {
    // 1) Drive ALL configured XSHUT pins LOW → every sensor held in reset.
    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
        if (_xshutPins[i] == TOF_XSHUT_NOT_WIRED) continue;
        pinMode(_xshutPins[i], OUTPUT);
        digitalWriteFast(_xshutPins[i], LOW);
    }
    delay(10);

    // 2) Wake each configured sensor in sequence, reassign its I2C address.
    //    The DFRobot lib's begin() internally starts at 0x29 then writes the
    //    new address to SLAVE_DEVICE_ADDRESS — so only one sensor can be
    //    awake at 0x29 at a time.
    uint8_t okCount = 0;
    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
        if (_xshutPins[i] == TOF_XSHUT_NOT_WIRED) continue;

        digitalWriteFast(_xshutPins[i], HIGH);
        delay(10);    // VL53L0X boot time from XSHUT rising edge is ~1.2 ms

        _tof[i].begin(_addresses[i]);
        _tof[i].setMode(DFRobot_VL53L0X::eContinuous,
                        DFRobot_VL53L0X::eHigh);
        _tof[i].start();
        _ok[i] = true;    // DFRobot begin() is void — assume ok after XSHUT
        okCount++;
    }
    return okCount;
}

// =============================================================================
uint16_t VL53L0XArray::readRange(Sensor idx) {
    if (idx >= SENSOR_COUNT || !_ok[idx]) return 0;
    float d = _tof[idx].getDistance();
    if (d < 0.0f) return 0;
    if (d > 65535.0f) return 65535;
    return (uint16_t)d;
}

void VL53L0XArray::readAll(uint16_t distances_mm[SENSOR_COUNT]) {
    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
        distances_mm[i] = _ok[i] ? readRange((Sensor)i) : 0;
    }
}
