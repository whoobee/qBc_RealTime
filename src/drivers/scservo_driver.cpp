#include "drivers/scservo_driver.h"
#include "config/device_config.h"
#include "config/safety_config.h"

// =============================================================================
// Joint table — maps ApplDeviceID → hardware ID + safety limits
// =============================================================================
const JointLimits SCServoDriver::_joints[JOINT_COUNT] = {
    { SERVO_ID_NECK,      DEV_SERVO_NECK,   SERVO_NECK_MIN,   SERVO_NECK_MAX   },
    { SERVO_ID_EAR_LEFT,  DEV_SERVO_EAR_L,  SERVO_EAR_L_MIN,  SERVO_EAR_L_MAX  },
    { SERVO_ID_EAR_RIGHT, DEV_SERVO_EAR_R,  SERVO_EAR_R_MIN,  SERVO_EAR_R_MAX  },
    { SERVO_ID_LEG_FL,    DEV_SERVO_LEG_FL, SERVO_LEG_FL_MIN, SERVO_LEG_FL_MAX },
    { SERVO_ID_LEG_FR,    DEV_SERVO_LEG_FR, SERVO_LEG_FR_MIN, SERVO_LEG_FR_MAX },
    { SERVO_ID_LEG_BL,    DEV_SERVO_LEG_BL, SERVO_LEG_BL_MIN, SERVO_LEG_BL_MAX },
    { SERVO_ID_LEG_BR,    DEV_SERVO_LEG_BR, SERVO_LEG_BR_MIN, SERVO_LEG_BR_MAX },
};

// =============================================================================
SCServoDriver::SCServoDriver(HardwareSerial* serial)
    : _serial(serial) {}

void SCServoDriver::begin() {
    _serial->begin(SERVO_BAUDRATE);
    _sms.pSerial = _serial;

    // Enable torque on every joint — ST3215 remembers the TORQUE_ENABLE register
    // across power cycles, so if a servo was last stored with torque off (or was
    // left in calibration mode), it will come up limp.  Explicitly enable torque
    // here so the robot always boots in a "holding position" state.
    delay(50);  // let the bus settle after Serial2 init
    for (uint8_t i = 0; i < JOINT_COUNT; i++) {
        _sms.EnableTorque(_joints[i].hw_id, 1);
    }
}

// =============================================================================
// Lookup helpers
// =============================================================================
int SCServoDriver::findJointIndex(uint8_t appl_device_id) const {
    for (uint8_t i = 0; i < JOINT_COUNT; i++) {
        if (_joints[i].appl_id == appl_device_id) return i;
    }
    return -1;
}

uint8_t SCServoDriver::resolveHwId(uint8_t appl_device_id) const {
    int idx = findJointIndex(appl_device_id);
    return (idx >= 0) ? _joints[idx].hw_id : 0;
}

uint16_t SCServoDriver::clampPosition(uint8_t joint_index, uint16_t pos) const {
    uint16_t lo = _joints[joint_index].pos_min;
    uint16_t hi = _joints[joint_index].pos_max;
    if (pos < lo) return lo;
    if (pos > hi) return hi;
    return pos;
}

// =============================================================================
// Public API
// =============================================================================
bool SCServoDriver::setPosition(uint8_t appl_device_id,
                                uint16_t position,
                                uint16_t speed, uint8_t acc) {
    int idx = findJointIndex(appl_device_id);
    if (idx < 0) return false;

    uint16_t clamped = clampPosition(idx, position);
    _sms.WritePosEx(_joints[idx].hw_id, clamped, speed, acc);
    return true;
}

bool SCServoDriver::setTorque(uint8_t appl_device_id, bool enable) {
    int idx = findJointIndex(appl_device_id);
    if (idx < 0) return false;

    _sms.EnableTorque(_joints[idx].hw_id, enable ? 1 : 0);
    return true;
}

bool SCServoDriver::readFeedback(uint8_t appl_device_id, ServoFeedback& fb) {
    int idx = findJointIndex(appl_device_id);
    if (idx < 0) return false;

    uint8_t hw = _joints[idx].hw_id;

    // FeedBack() does a bulk read of position/speed/load/voltage/temp/current/moving
    // into the Mem cache.  Pass ID=-1 to read from cache (no extra bus traffic).
    if (_sms.FeedBack(hw) == -1) return false;

    fb.position    = _sms.ReadPos(-1);
    fb.speed       = _sms.ReadSpeed(-1);
    fb.load        = _sms.ReadLoad(-1);
    fb.voltage     = _sms.ReadVoltage(-1);
    fb.temperature = _sms.ReadTemper(-1);
    fb.current     = _sms.ReadCurrent(-1);
    fb.moving      = (_sms.ReadMove(-1) != 0);
    return true;
}
