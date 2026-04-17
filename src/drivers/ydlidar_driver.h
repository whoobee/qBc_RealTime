#pragma once
// =============================================================================
// ydlidar_driver.h — YDLidar GS2 wrapper for qB Companion
// =============================================================================
// The underlying library (YDLiDar_GS2) manages its own serial port init.
// This wrapper provides:
//   - Sector-based min-distance map for obstacle detection
//   - Thread-safe single-call update
// =============================================================================

#include <Arduino.h>
#include <YDLiDar_gs2.h>

// Simplified sector map — four cardinal quadrants
struct LidarSectors {
    uint16_t front_mm;   // min distance in front sector
    uint16_t right_mm;   // min distance in right sector
    uint16_t back_mm;    // min distance in back sector
    uint16_t left_mm;    // min distance in left sector
    // Full 36-bin polar histogram (10 deg per bin, CCW from front).
    // 0 = no valid return in that sector.
    uint16_t bins[36];
    uint32_t timestamp_ms;
};

static constexpr uint8_t LIDAR_BIN_COUNT = 36;

class YDLidarDriver {
public:
    // serial: hardware UART wired to the GS2. This driver calls
    // serial->begin(921600) itself so it can drain pre-existing RX data
    // before letting the library ping.
    explicit YDLidarDriver(HardwareSerial* serial);

    // Initialise and start scanning.  Returns true on success.
    bool begin();

    // Fetch latest scan, compute sector map.  Call at ~10-20 Hz.
    bool update(LidarSectors& sectors);

    // Stop scanning (e.g. before power-down)
    void stop();

    bool isReady() const { return _ready; }

private:
    YDLiDar_GS2     _lidar;
    HardwareSerial* _serial;   // same pointer handed to the library
    bool            _ready;
};
