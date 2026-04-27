#include "tasks/task_perception.h"
#include <HeliOS_Arduino.h>
#include "system/shared_state.h"
#include "config/pin_config.h"
#include "config/feature_config.h"

#if FEATURE_TOF_ENABLED
#include "drivers/vl53l0x_array.h"
static VL53L0XArray s_tofs;
#endif

#if FEATURE_LIDAR_ENABLED
#include "drivers/ydlidar_driver.h"
static YDLidarDriver s_lidar(&SERIAL_LIDAR);
static bool          s_lidarOk = false;
#endif

// =============================================================================
// HeliOS timer callback — 20 Hz (50 000 µs)
// =============================================================================
#if FEATURE_TOF_ENABLED || FEATURE_LIDAR_ENABLED
static void perceptionCallback(TaskId_t id_) {
#if FEATURE_TOF_ENABLED
    uint16_t tofDist[VL53L0XArray::SENSOR_COUNT];
    s_tofs.readAll(tofDist);
    g_perceptionData.tof_left_mm  = tofDist[VL53L0XArray::LEFT];
    g_perceptionData.tof_right_mm = tofDist[VL53L0XArray::RIGHT];
    g_perceptionData.tof_front_mm = tofDist[VL53L0XArray::FRONT];
    g_perceptionData.tof_back_mm  = tofDist[VL53L0XArray::BACK];

#if DEBUG_TOF_ENABLED
    // Throttle to ~2 Hz so we don't flood Serial at the 20 Hz read rate.
    // Values come from the cache filled by readAll() above (one I2C read per
    // slot). raw = chip register, mm = published distance (0 when status !=
    // RANGE_VALID), sig = signal count, st = range status (0 = valid).
    static uint32_t s_lastTofLog = 0;
    uint32_t now = millis();
    if (now - s_lastTofLog >= 500) {
        s_lastTofLog = now;
        const char* names[VL53L0XArray::SENSOR_COUNT] = {"L", "R", "F", "B"};
        Serial.print(F("[TOF]"));
        for (uint8_t i = 0; i < VL53L0XArray::SENSOR_COUNT; i++) {
            VL53L0XArray::Sensor s = (VL53L0XArray::Sensor)i;
            Serial.print(F(" "));
            Serial.print(names[i]);
            Serial.print(F("=raw:"));
            Serial.print(s_tofs.cachedRawRange(s));
            Serial.print(F("/mm:"));
            Serial.print(tofDist[i]);
            Serial.print(F("/sig:"));
            Serial.print(s_tofs.cachedSignal(s));
            Serial.print(F("/st:"));
            Serial.print(s_tofs.cachedStatus(s));
        }
        Serial.println();
    }
#endif
#endif

#if FEATURE_LIDAR_ENABLED
    if (s_lidarOk) {
        LidarSectors sectors;
        if (s_lidar.update(sectors)) {
            g_perceptionData.lidar_min_front_mm = sectors.front_mm;
            g_perceptionData.lidar_min_left_mm  = sectors.left_mm;
            g_perceptionData.lidar_min_right_mm = sectors.right_mm;
            g_perceptionData.lidar_min_back_mm  = sectors.back_mm;
            for (uint8_t b = 0; b < LIDAR_BIN_COUNT; b++) {
                g_perceptionData.lidar_bins[b] = sectors.bins[b];
            }
        }
    }
#endif

    g_perceptionData.timestamp_ms = millis();
}
#endif // FEATURE_TOF_ENABLED || FEATURE_LIDAR_ENABLED

// =============================================================================
void task_perception_init() {
#if !FEATURE_TOF_ENABLED && !FEATURE_LIDAR_ENABLED
    return;
#endif

#if FEATURE_TOF_ENABLED
    // Wire2.begin() is called from main.cpp — DFRobot fork speaks on Wire2.
    s_tofs.begin();
    g_monitorData.sensor_ok_tof[0] = s_tofs.isReady(VL53L0XArray::LEFT);
    g_monitorData.sensor_ok_tof[1] = s_tofs.isReady(VL53L0XArray::RIGHT);
    g_monitorData.sensor_ok_tof[2] = s_tofs.isReady(VL53L0XArray::FRONT);
    g_monitorData.sensor_ok_tof[3] = s_tofs.isReady(VL53L0XArray::BACK);
#endif

#if FEATURE_LIDAR_ENABLED
    for (int attempt = 0; attempt < 3 && !s_lidarOk; attempt++) {
        s_lidarOk = s_lidar.begin();
        if (!s_lidarOk) delay(500);
    }
    g_monitorData.sensor_ok_lidar = s_lidarOk;
#endif

#if FEATURE_TOF_ENABLED || FEATURE_LIDAR_ENABLED
    TaskId_t id = xTaskAdd("Percept", &perceptionCallback);
    xTaskWait(id);
    xTaskSetTimer(id, 50000);   // 20 Hz
#endif
}
