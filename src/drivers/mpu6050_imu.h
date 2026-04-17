#pragma once
// =============================================================================
// mpu6050_imu.h — MPU6050 IMU driver (6-axis: accel + gyro)
// =============================================================================
// Replaces the BNO055 (which required on-chip fusion). The MPU6050 is raw
// accel/gyro only — roll/pitch are computed from the gravity vector, yaw is
// left at zero (no magnetometer). Quaternion is identity.
//
// Bus: Wire (Teensy pins SDA=18, SCL=19). Address: 0x68.
// =============================================================================

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

struct IMUReading {
    // Euler angles (degrees) — roll/pitch from accel; yaw=0 (no magnetometer)
    float roll, pitch, yaw;
    // Quaternion (identity — MPU6050 does not provide fused orientation)
    float qw, qx, qy, qz;
    // Raw acceleration (m/s² — Adafruit_MPU6050 native units)
    float accel_x, accel_y, accel_z;
    // Gyro rates (rad/s)
    float gyro_x, gyro_y, gyro_z;
    // Validity
    bool valid;
};

class MPU6050IMU {
public:
    MPU6050IMU();

    // Initialise the sensor on the Wire bus. Returns true on success.
    bool begin();

    // Read accel + gyro in one shot and compute tilt.
    bool read(IMUReading& out);

    bool isReady() const { return _ready; }

private:
    Adafruit_MPU6050 _mpu;
    bool             _ready;
};
