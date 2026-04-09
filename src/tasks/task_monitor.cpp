#include "tasks/task_monitor.h"
#include <HeliOS_Arduino.h>
#include "system/shared_state.h"
#include "config/pin_config.h"
#include "config/safety_config.h"
#include "config/feature_config.h"

#if BATTERY_MONITOR_ENABLED
static constexpr float ADC_REF_V      = 3.3f;
static constexpr float ADC_RESOLUTION = 1023.0f;
static constexpr float VBAT_DIVIDER   = 4.0f;
static constexpr float CURRENT_SCALE  = 5.0f;
#endif

static bool    s_ledState = false;
static uint8_t s_blinkCnt = 0;

// =============================================================================
// HeliOS timer callback — 10 Hz (100 000 µs)
// =============================================================================
static void monitorCallback(TaskId_t id_) {
    float battV = 0.0f;
    float battI = 0.0f;
#if BATTERY_MONITOR_ENABLED
    float rawV = (analogRead(PIN_BATTERY_VOLTAGE) / ADC_RESOLUTION) * ADC_REF_V;
    float rawI = (analogRead(PIN_BATTERY_CURRENT) / ADC_RESOLUTION) * ADC_REF_V;
    battV = rawV * VBAT_DIVIDER;
    battI = rawI * CURRENT_SCALE;
#endif

    g_monitorData.battery_voltage  = battV;
    g_monitorData.battery_current  = battI;

#if FEATURE_MOTORS_ENABLED
    g_monitorData.motor_temp_left  = g_motorStatus[0].temperature_c;
    g_monitorData.motor_temp_right = g_motorStatus[1].temperature_c;
#endif

    // ---- Status LED: fast blink if safety event, slow blink if healthy ----
    if (g_safetyBits & SAFETY_ANY_CRITICAL) {
        s_ledState = !s_ledState;
    } else {
        if (++s_blinkCnt >= 5) {
            s_blinkCnt = 0;
            s_ledState = !s_ledState;
        }
    }
    digitalWrite(PIN_STATUS_LED, s_ledState ? HIGH : LOW);
}

// =============================================================================
void task_monitor_init() {
#if BATTERY_MONITOR_ENABLED
    analogReadResolution(10);
    pinMode(PIN_BATTERY_VOLTAGE, INPUT);
    pinMode(PIN_BATTERY_CURRENT, INPUT);
#endif
    pinMode(PIN_STATUS_LED, OUTPUT);

    TaskId_t id = xTaskAdd("Monitor", &monitorCallback);
    xTaskWait(id);
    xTaskSetTimer(id, 100000);   // 10 Hz
}
