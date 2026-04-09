#pragma once
// =============================================================================
// bno055_imu.h — BNO055 IMU driver (fused on-chip orientation)
// =============================================================================

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>

struct IMUReading {
    // Euler angles (degrees)
    float roll, pitch, yaw;
    // Quaternion
    float qw, qx, qy, qz;
    // Linear acceleration (m/s²) — gravity removed
    float accel_x, accel_y, accel_z;
    // Calibration status (0 = uncalibrated, 3 = fully calibrated)
    uint8_t cal_sys, cal_gyro, cal_accel, cal_mag;
    // Validity
    bool valid;
};

class BNO055IMU {
public:
    BNO055IMU();

    // Initialise the sensor on I2C.  Returns true on success.
    bool begin();

    // Read all fused data in one shot.
    bool read(IMUReading& out);

    // Check if sensor is responding
    bool isReady() const { return _ready; }

private:
    Adafruit_BNO055 _bno;
    bool            _ready;
};
