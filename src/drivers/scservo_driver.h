#pragma once
// =============================================================================
// scservo_driver.h — ST3215 servo driver wrapper (over SMS_STS library)
// =============================================================================
// Wraps the SCServo library's SMS_STS class with:
//   - Joint-name abstraction via ApplDeviceID
//   - Per-joint position clamping (safety)
//   - Bulk feedback reading
// =============================================================================

#include <Arduino.h>
#include <SMS_STS.h>
#include "comm/appl_protocol.h"

// Per-joint limit entry
struct JointLimits {
    uint8_t  hw_id;       // Servo hardware ID (EEPROM-programmed)
    uint8_t  appl_id;     // ApplDeviceID
    uint16_t pos_min;     // Min position (SMS_STS units, 0–4096)
    uint16_t pos_max;     // Max position (SMS_STS units, 0–4096)
};

struct ServoFeedback {
    int16_t  position;
    int16_t  speed;
    int16_t  load;        // 0-1000 → 0-100%
    uint8_t  voltage;
    uint8_t  temperature;
    int16_t  current;
    bool     moving;
};

class SCServoDriver {
public:
    explicit SCServoDriver(HardwareSerial* serial);

    void begin();

    // Move a single servo by ApplDeviceID
    // Position is clamped to the joint's limits.
    // Returns false if device_id is unknown.
    bool setPosition(uint8_t appl_device_id,
                     uint16_t position, uint16_t speed, uint8_t acc = 0);

    // Enable / disable torque
    bool setTorque(uint8_t appl_device_id, bool enable);

    // Read full feedback for one servo
    bool readFeedback(uint8_t appl_device_id, ServoFeedback& fb);

    // Resolve ApplDeviceID → hardware servo ID.  Returns 0 if unknown.
    uint8_t resolveHwId(uint8_t appl_device_id) const;

    // Number of known joints
    static constexpr uint8_t JOINT_COUNT = 7;

    // Direct access to underlying SMS_STS (for advanced use / sync writes)
    SMS_STS& sms() { return _sms; }

private:
    // Clamp position to joint limits.  Returns clamped value.
    uint16_t clampPosition(uint8_t joint_index, uint16_t pos) const;

    // Find joint index by ApplDeviceID.  Returns -1 if not found.
    int findJointIndex(uint8_t appl_device_id) const;

    SMS_STS         _sms;
    HardwareSerial* _serial;

    static const JointLimits _joints[JOINT_COUNT];
};
