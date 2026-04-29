#include "drivers/vl53l0x_array.h"
#include "config/pin_config.h"
#include "config/device_config.h"
#include "config/feature_config.h"
#include <Wire.h>

// VL53L0X MODEL_ID register (0xC0) reads back 0xEE on every healthy chip.
// Used as a post-begin sanity check to catch silent address-assignment failures.
// VL53L0X register addresses are 8-bit on the standard I2C interface, so we send
// only the low byte — matching the DFRobot library's own readByteData() format.
static constexpr uint8_t REG_MODEL_ID            = 0xC0;
static constexpr uint8_t REG_SLAVE_DEVICE_ADDR   = 0x8A;
static constexpr uint8_t EXPECTED_MODEL_ID       = 0xEE;
static constexpr uint8_t VL53L0X_DEFAULT_I2C     = 0x29;

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

// Write a 1-byte register value at `addr` on Wire2. Returns true on ACK.
static bool writeRegister(uint8_t addr, uint8_t reg, uint8_t value) {
    Wire2.beginTransmission(addr);
    Wire2.write(reg);
    Wire2.write(value);
    return Wire2.endTransmission() == 0;
}

// Recover an always-on VL53L0X whose I2C address is "stuck" from a prior
// Teensy boot. The sensor retains its programmed SLAVE_DEVICE_ADDRESS until
// power is removed (XSHUT pulled or VIN cycled). If the firmware reflashes
// without cycling sensor power, begin() will fail because the chip is not
// at 0x29 anymore.
//
// Strategy: scan 0x08..0x77 for any device whose MODEL_ID register reads
// 0xEE. If we find exactly one and it is NOT at 0x29, rewrite its
// SLAVE_DEVICE_ADDRESS back to 0x29 so the normal DFRobot::begin(target)
// flow can run. Returns true if a chip is now reachable at 0x29.
static bool recoverAlwaysOnTo29() {
    if (probeRegister(VL53L0X_DEFAULT_I2C, REG_MODEL_ID) == EXPECTED_MODEL_ID) {
        return true;
    }
    uint8_t found = 0xFF;
    for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
        if (addr == VL53L0X_DEFAULT_I2C) continue;
        if (probeRegister(addr, REG_MODEL_ID) == EXPECTED_MODEL_ID) {
            found = addr;
            break;
        }
    }
    if (found == 0xFF) return false;

    Serial.print(F("[TOF] Always-on chip found at 0x"));
    Serial.print(found, HEX);
    Serial.println(F(" — renaming to 0x29"));
    writeRegister(found, REG_SLAVE_DEVICE_ADDR, VL53L0X_DEFAULT_I2C);
    delay(5);
    return probeRegister(VL53L0X_DEFAULT_I2C, REG_MODEL_ID) == EXPECTED_MODEL_ID;
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
    //
    //    Always-on sensors keep their last-programmed address across Teensy
    //    reboots (no XSHUT to reset them). Try to recover any chip stuck at a
    //    non-default address back to 0x29 before calling DFRobot::begin().
    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
        if (_xshutPins[i] != TOF_XSHUT_NONE) continue;
        if (!recoverAlwaysOnTo29()) {
            Serial.print(F("[TOF] "));
            Serial.print(slotName(i));
            Serial.println(F(" — no chip reachable at 0x29 (HW?)"));
        }
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
// VL53L0X RESULT_RANGE_STATUS bits [6:3] are the raw device-error code, not
// ST's post-processed PalRangeStatus. Code 11 (RANGECOMPLETE) means ranging
// finished normally; ST's full API then validates by checking the signal
// rate against a threshold to derive RANGE_VALID. The DFRobot library skips
// that validation, so we approximate it here: RANGECOMPLETE plus signal
// count above a noise floor. Below the floor the result is dominated by
// cross-talk / ambient noise and should not be published as a distance.
static constexpr uint8_t  TOF_DEV_RANGECOMPLETE = 11;
// ST spec recommends signal limit ~32 (0.25 MCPS in 9.7 fixed-point) for
// confident measurements. The chips on this robot are delivering only sig:3–10
// on real targets — a 10–30x sensitivity loss from spec, likely due to lens
// contamination or hot-plug-induced chip damage. Threshold lowered so real
// readings are not filtered out; some noise will pass through and may need
// temporal smoothing downstream if it becomes a problem.
static constexpr uint16_t TOF_MIN_SIGNAL_COUNT  = 2;

uint16_t VL53L0XArray::readRange(Sensor idx) {
    if (idx >= SENSOR_COUNT || !_ok[idx]) return 0;
    float d = _tof[idx].getDistance();
    if (_tof[idx].getCachedStatus() != TOF_DEV_RANGECOMPLETE) return 0;
    if (_tof[idx].getCachedSignalCount() < TOF_MIN_SIGNAL_COUNT) return 0;
    if (d < 0.0f) return 0;
    if (d > 65535.0f) return 65535;
    return (uint16_t)d;
}

bool VL53L0XArray::readRangeValid(Sensor idx, uint16_t& out_mm) {
    if (idx >= SENSOR_COUNT || !_ok[idx]) return false;
    float d = _tof[idx].getDistance();
    if (_tof[idx].getCachedStatus() != TOF_DEV_RANGECOMPLETE) return false;
    if (_tof[idx].getCachedSignalCount() < TOF_MIN_SIGNAL_COUNT) return false;
    if (d < 0.0f) return false;
    if (d > 65535.0f) d = 65535.0f;
    out_mm = (uint16_t)d;
    return true;
}

uint16_t VL53L0XArray::cachedRawRange(Sensor idx) const {
    if (idx >= SENSOR_COUNT || !_ok[idx]) return 0;
    return _tof[idx].getCachedRawDistance();
}

uint8_t VL53L0XArray::cachedStatus(Sensor idx) const {
    if (idx >= SENSOR_COUNT || !_ok[idx]) return 0xFF;
    return _tof[idx].getCachedStatus();
}

uint16_t VL53L0XArray::cachedSignal(Sensor idx) const {
    if (idx >= SENSOR_COUNT || !_ok[idx]) return 0;
    return _tof[idx].getCachedSignalCount();
}

void VL53L0XArray::readAll(uint16_t distances_mm[SENSOR_COUNT]) {
    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
        distances_mm[i] = _ok[i] ? readRange((Sensor)i) : 0;
    }
}
