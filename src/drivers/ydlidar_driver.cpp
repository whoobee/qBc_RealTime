#include "drivers/ydlidar_driver.h"
#include "config/device_config.h"

YDLidarDriver::YDLidarDriver(HardwareSerial* serial)
    : _lidar(serial)  // library stores the pointer and calls begin() itself
    , _ready(false) {}

bool YDLidarDriver::begin() {
    GS_error status = _lidar.initialize(LIDAR_SENSOR_COUNT);
    if (status != GS_OK) {
        _ready = false;
        return false;
    }
    status = _lidar.startScanning();
    _ready = (status == GS_OK);
    return _ready;
}

bool YDLidarDriver::update(LidarSectors& sectors) {
    if (!_ready) return false;

    iter_Scan scan = _lidar.iter_scans(0x01);  // device address 1

    // Init to "no obstacle" (max uint16)
    sectors.front_mm = UINT16_MAX;
    sectors.right_mm = UINT16_MAX;
    sectors.back_mm  = UINT16_MAX;
    sectors.left_mm  = UINT16_MAX;

    // Sector definitions (degrees, measured CCW from front):
    //   Front:  315–360 and 0–45
    //   Right:  45–135
    //   Back:   135–225
    //   Left:   225–315
    for (int i = 0; i < MAX_SCAN; i++) {
        if (!scan.valid[i]) continue;
        if (scan.distance[i] == 0) continue;

        double  angle = scan.angle[i];
        uint16_t dist = scan.distance[i];

        // Normalise angle to [0, 360)
        while (angle < 0.0)   angle += 360.0;
        while (angle >= 360.0) angle -= 360.0;

        if (angle >= 315.0 || angle < 45.0) {
            if (dist < sectors.front_mm) sectors.front_mm = dist;
        } else if (angle >= 45.0 && angle < 135.0) {
            if (dist < sectors.right_mm) sectors.right_mm = dist;
        } else if (angle >= 135.0 && angle < 225.0) {
            if (dist < sectors.back_mm)  sectors.back_mm  = dist;
        } else {
            if (dist < sectors.left_mm)  sectors.left_mm  = dist;
        }
    }

    // Replace "no obstacle" marker with 0 (means sensor saw nothing in range)
    if (sectors.front_mm == UINT16_MAX) sectors.front_mm = 0;
    if (sectors.right_mm == UINT16_MAX) sectors.right_mm = 0;
    if (sectors.back_mm  == UINT16_MAX) sectors.back_mm  = 0;
    if (sectors.left_mm  == UINT16_MAX) sectors.left_mm  = 0;

    sectors.timestamp_ms = millis();
    return true;
}

void YDLidarDriver::stop() {
    if (_ready) {
        _lidar.stopScanning();
        _ready = false;
    }
}
