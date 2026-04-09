#include "drivers/bno055_imu.h"
#include "config/device_config.h"

BNO055IMU::BNO055IMU()
    : _bno(55, BNO055_I2C_ADDR, &Wire)
    , _ready(false) {}

bool BNO055IMU::begin() {
    if (!_bno.begin()) {
        _ready = false;
        return false;
    }
    _bno.setExtCrystalUse(true);
    _ready = true;
    return true;
}

bool BNO055IMU::read(IMUReading& out) {
    out.valid = false;
    if (!_ready) return false;

    // Euler angles
    sensors_event_t event;
    _bno.getEvent(&event, Adafruit_BNO055::VECTOR_EULER);
    out.yaw   = event.orientation.x;   // BNO055 Euler: x = heading/yaw
    out.roll  = event.orientation.y;   //               y = roll
    out.pitch = event.orientation.z;   //               z = pitch

    // Quaternion
    imu::Quaternion q = _bno.getQuat();
    out.qw = q.w();
    out.qx = q.x();
    out.qy = q.y();
    out.qz = q.z();

    // Linear acceleration (gravity-compensated)
    imu::Vector<3> accel = _bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    out.accel_x = accel.x();
    out.accel_y = accel.y();
    out.accel_z = accel.z();

    // Calibration status
    _bno.getCalibration(&out.cal_sys, &out.cal_gyro, &out.cal_accel, &out.cal_mag);

    out.valid = true;
    return true;
}
