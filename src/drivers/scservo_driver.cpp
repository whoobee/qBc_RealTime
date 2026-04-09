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
    if (_sms.FeedBack(hw) == -1) return false;

    fb.position    = _sms.ReadPos(hw);
    fb.speed       = _sms.ReadSpeed(hw);
    fb.load        = _sms.ReadLoad(hw);
    fb.voltage     = _sms.ReadVoltage(hw);
    fb.temperature = _sms.ReadTemper(hw);
    fb.current     = _sms.ReadCurrent(hw);
    fb.moving      = (_sms.ReadMove(hw) != 0);
    return true;
}
