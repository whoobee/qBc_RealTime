#include "drivers/vl53l0x_array.h"
#include "config/pin_config.h"
#include "config/device_config.h"
#include "config/feature_config.h"
#include <Wire.h>

// VL53L0X MODEL_ID register (0xC0) reads back 0xEE on every healthy chip.
// Used as a post-begin sanity check to catch silent address-assignment failures.
// VL53L0X register addresses are 8-bit on the standard I2C interface, so we send
// only the low byte — matching the DFRobot library's own readByteData() format.
static constexpr uint8_t REG_MODEL_ID       = 0xC0;
static constexpr uint8_t EXPECTED_MODEL_ID  = 0xEE;

static const char* slotName(uint8_t i) {
    switch (i) {
        case 0: return "LEFT";
        case 1: return "RIGHT";
        case 2: return "FRONT";
        case 3: return "BACK";
        default: return "?";
    }
}

// Raw Wire2 read of a 1-byte register from `addr`. Returns 0xFF if the device
// did not ACK or no bytes were returned — the same sentinel a missing sensor
// would yield, which is harmless because we only compare against 0xEE.
static uint8_t probeRegister(uint8_t addr, uint8_t reg) {
    Wire2.beginTransmission(addr);
    Wire2.write(reg);
    if (Wire2.endTransmission(false) != 0) return 0xFF;
    if (Wire2.requestFrom((int)addr, 1) != 1) return 0xFF;
    return (uint8_t)Wire2.read();
}

// Per-slot target 7-bit I2C addresses. Unique so sensors don't collide on
// Wire2 once all are awake. 0x29 is the default — we avoid it for clarity.
// The DFRobot lib passes whatever value you give straight to the sensor's
// SLAVE_DEVICE_ADDRESS register (7-bit, masked by 0x7F internally).
static constexpr uint8_t SLOT_ADDR_LEFT  = 0x30;
static constexpr uint8_t SLOT_ADDR_RIGHT = 0x31;
static constexpr uint8_t SLOT_ADDR_BACK  = 0x32;
static constexpr uint8_t SLOT_ADDR_FRONT = 0x33;

VL53L0XArray::VL53L0XArray() {
    // A disabled slot is treated as TOF_XSHUT_NOT_WIRED so the rest of the
    // init / read path skips it and reports 0 mm — no other code paths need
    // to care about per-slot feature flags.
    _xshutPins[LEFT]  = FEATURE_TOF_LEFT_ENABLED  ? PIN_TOF_XSHUT_LEFT  : TOF_XSHUT_NOT_WIRED;
    _xshutPins[RIGHT] = FEATURE_TOF_RIGHT_ENABLED ? PIN_TOF_XSHUT_RIGHT : TOF_XSHUT_NOT_WIRED;
    _xshutPins[FRONT] = FEATURE_TOF_FRONT_ENABLED ? PIN_TOF_XSHUT_FRONT : TOF_XSHUT_NOT_WIRED;
    _xshutPins[BACK]  = FEATURE_TOF_BACK_ENABLED  ? PIN_TOF_XSHUT_BACK  : TOF_XSHUT_NOT_WIRED;

    _addresses[LEFT]  = SLOT_ADDR_LEFT;
    _addresses[RIGHT] = SLOT_ADDR_RIGHT;
    _addresses[FRONT] = SLOT_ADDR_FRONT;
    _addresses[BACK]  = SLOT_ADDR_BACK;

    for (uint8_t i = 0; i < SENSOR_COUNT; i++) _ok[i] = false;
}

bool VL53L0XArray::isConfigured(Sensor idx) const {
    if (idx >= SENSOR_COUNT) return false;
    return _xshutPins[idx] != TOF_XSHUT_NOT_WIRED;
}

// =============================================================================
// Bring up a single slot: call DFRobot begin() then verify that the sensor is
// actually responding at the assigned address by reading MODEL_ID. Retries
// once if the first attempt fails (covers occasional missed XSHUT timing).
// Logs each step over USB serial so init failures are visible.
// =============================================================================
bool VL53L0XArray::initSlot(uint8_t i) {
    const uint8_t addr = _addresses[i];
    for (uint8_t attempt = 1; attempt <= 2; attempt++) {
        _tof[i].begin(addr);
        _tof[i].setMode(DFRobot_VL53L0X::eContinuous,
                        DFRobot_VL53L0X::eHigh);
        _tof[i].start();

        uint8_t modelId = probeRegister(addr, REG_MODEL_ID);
        Serial.print(F("[TOF] "));
        Serial.print(slotName(i));
        Serial.print(F(" addr=0x"));
        Serial.print(addr, HEX);
        Serial.print(F(" attempt="));
        Serial.print(attempt);
        Serial.print(F(" model_id=0x"));
        Serial.println(modelId, HEX);

        if (modelId == EXPECTED_MODEL_ID) return true;
        delay(20);
    }
    Serial.print(F("[TOF] "));
    Serial.print(slotName(i));
    Serial.println(F(" — INIT FAILED (no MODEL_ID)"));
    return false;
}

// =============================================================================
// Power-up sequence: one sensor at a time, assign unique I2C addresses.
//   - TOF_XSHUT_NOT_WIRED slots: skipped entirely.
//   - TOF_XSHUT_NONE slots: reassigned FIRST (only sensor awake at 0x29 while
//     all XSHUT-controlled sensors are still held in reset).
//   - XSHUT-controlled slots: woken one at a time, reassigned in sequence.
// =============================================================================
uint8_t VL53L0XArray::begin() {
    Serial.println(F("[TOF] Initialising VL53L0X array on Wire2"));

    // 1) Drive every XSHUT-controlled slot LOW → those sensors held in reset.
    //    Slots marked TOF_XSHUT_NONE have no Teensy-driven XSHUT line and stay
    //    awake at 0x29 from boot — we will reassign their address first.
    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
        uint8_t pin = _xshutPins[i];
        if (pin == TOF_XSHUT_NOT_WIRED || pin == TOF_XSHUT_NONE) continue;
        pinMode(pin, OUTPUT);
        digitalWriteFast(pin, LOW);
    }
    delay(50);

    uint8_t okCount = 0;

    // 2) Reassign address for any always-on (TOF_XSHUT_NONE) slot first.
    //    Only one such sensor can exist on the bus at a time — multiple would
    //    collide at the default 0x29.
    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
        if (_xshutPins[i] != TOF_XSHUT_NONE) continue;
        _ok[i] = initSlot(i);
        if (_ok[i]) okCount++;
    }

    // 3) Wake each XSHUT-controlled sensor in sequence, reassign its address.
    //    The DFRobot lib's begin() internally starts at 0x29 then writes the
    //    new address to SLAVE_DEVICE_ADDRESS — so only one sensor can be
    //    awake at 0x29 at a time.
    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
        uint8_t pin = _xshutPins[i];
        if (pin == TOF_XSHUT_NOT_WIRED || pin == TOF_XSHUT_NONE) continue;

        digitalWriteFast(pin, HIGH);
        delay(50);   // VL53L0X tBOOT is ~1.2 ms; allow generous margin

        _ok[i] = initSlot(i);
        if (_ok[i]) okCount++;
    }

    uint8_t expected = 0;
    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
        if (_xshutPins[i] != TOF_XSHUT_NOT_WIRED) expected++;
    }
    Serial.print(F("[TOF] Init complete — "));
    Serial.print(okCount);
    Serial.print(F("/"));
    Serial.print(expected);
    Serial.println(F(" sensors ready"));
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
