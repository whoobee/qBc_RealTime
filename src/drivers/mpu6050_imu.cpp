#include "drivers/mpu6050_imu.h"
#include "config/device_config.h"
#include <math.h>

MPU6050IMU::MPU6050IMU() : _ready(false) {}

bool MPU6050IMU::begin() {
    // Wire.begin() is called from main.cpp (shared with any other Wire devices).
    // We only do the sensor-specific init here.
    _ready = _mpu.begin(MPU6050_I2C_ADDR, &Wire, 0);
    if (!_ready) return false;

    _mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
    _mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    _mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    return true;
}

bool MPU6050IMU::read(IMUReading& out) {
    if (!_ready) { out.valid = false; return false; }

    sensors_event_t a, g, t;
    if (!_mpu.getEvent(&a, &g, &t)) {
        out.valid = false;
        return false;
    }

    out.accel_x = a.acceleration.x;
    out.accel_y = a.acceleration.y;
    out.accel_z = a.acceleration.z;

    out.gyro_x  = g.gyro.x;
    out.gyro_y  = g.gyro.y;
    out.gyro_z  = g.gyro.z;

    // Gravity-vector tilt estimation (no magnetometer → no yaw).
    //   roll  around X: atan2(ay, az)
    //   pitch around Y: atan2(-ax, sqrt(ay^2 + az^2))
    const float ax = out.accel_x;
    const float ay = out.accel_y;
    const float az = out.accel_z;
    out.roll  = atan2f(ay, az) * (180.0f / (float)M_PI);
    out.pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * (180.0f / (float)M_PI);
    out.yaw   = 0.0f;

    // Identity quaternion (no fused orientation available from MPU6050).
    out.qw = 1.0f;
    out.qx = 0.0f;
    out.qy = 0.0f;
    out.qz = 0.0f;

    out.valid = true;
    return true;
}
