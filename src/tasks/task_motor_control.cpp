#include "tasks/task_motor_control.h"
#include <HeliOS_Arduino.h>
#include "system/shared_state.h"
#include "config/pin_config.h"
#include "config/device_config.h"
#include "config/feature_config.h"
#include "debug/debug_comm.h"

#if FEATURE_MOTORS_ENABLED
#include "drivers/ddsm210.h"
static DDSM210 s_motorL(&SERIAL_MOTOR_LEFT);
static DDSM210 s_motorR(&SERIAL_MOTOR_RIGHT);
#endif

#if FEATURE_SERVOS_ENABLED
#include "drivers/scservo_driver.h"
static SCServoDriver s_servos(&SERIAL_SERVOS);
static uint8_t       s_servoRR = 0;
#endif

// =============================================================================
// HeliOS timer callback — 50 Hz (20 000 µs)
// =============================================================================
static void motorControlCallback(TaskId_t id_) {
#if FEATURE_MOTORS_ENABLED
    // ---- Process wheel commands (latest only) ----
    if (g_motorCmdPending) {
        bool okL = s_motorL.setSpeed(DDSM210_LEFT_ID,  g_motorCmd.left_rpm);
        bool okR = s_motorR.setSpeed(DDSM210_RIGHT_ID, -g_motorCmd.right_rpm);
        g_motorCmdPending = false;
#if DEBUG_COMM_ENABLED
#if DEBUG_MOTOR_COMMANDS_ENABLED
        SERIAL_DEBUG.print(F("[MOTOR] L="));
        SERIAL_DEBUG.print(g_motorCmd.left_rpm);
        SERIAL_DEBUG.print(okL ? F(" OK") : F(" FAIL"));
        SERIAL_DEBUG.print(F("  R="));
        SERIAL_DEBUG.print(g_motorCmd.right_rpm);
        SERIAL_DEBUG.println(okR ? F(" OK") : F(" FAIL"));
#endif
#endif
    }

    // ---- Publish motor feedback to shared state ----
    const DDSM210Feedback& fbL = s_motorL.lastFeedback();
    g_motorStatus[0].speed_rpm     = fbL.speed_rpm;
    g_motorStatus[0].current_a     = fbL.current_a;
    g_motorStatus[0].temperature_c = fbL.temperature_c;
    g_motorStatus[0].fault_code    = fbL.fault_code;

    const DDSM210Feedback& fbR = s_motorR.lastFeedback();
    g_motorStatus[1].speed_rpm     = fbR.speed_rpm;
    g_motorStatus[1].current_a     = fbR.current_a;
    g_motorStatus[1].temperature_c = fbR.temperature_c;
    g_motorStatus[1].fault_code    = fbR.fault_code;
#endif

#if FEATURE_SERVOS_ENABLED
    // ---- Process servo commands (all pending) ----
    ServoCommand scmd;
    while (g_servoCmdBuf.pop(scmd)) {
        bool ok = s_servos.setPosition(scmd.servo_appl_id,
                                       scmd.position, scmd.speed, scmd.acceleration);
#if DEBUG_COMM_ENABLED
        SERIAL_DEBUG.print(F("[SERVO] dev=0x"));
        SERIAL_DEBUG.print(scmd.servo_appl_id, HEX);
        SERIAL_DEBUG.print(F(" pos="));
        SERIAL_DEBUG.print(scmd.position);
        SERIAL_DEBUG.print(F(" spd="));
        SERIAL_DEBUG.print(scmd.speed);
        SERIAL_DEBUG.print(F(" acc="));
        SERIAL_DEBUG.print(scmd.acceleration);
        SERIAL_DEBUG.println(ok ? F(" OK") : F(" FAIL"));
#endif
    }

    // ---- Read servo feedback (round-robin, one servo per tick) ----
    ServoFeedback sfb;
    uint8_t appl_id = DEV_SERVO_NECK + s_servoRR;
    if (s_servos.readFeedback(appl_id, sfb)) {
        g_servoStatus[s_servoRR].position    = sfb.position;
        g_servoStatus[s_servoRR].speed       = sfb.speed;
        g_servoStatus[s_servoRR].load        = sfb.load;
        g_servoStatus[s_servoRR].voltage     = sfb.voltage;
        g_servoStatus[s_servoRR].temperature = sfb.temperature;
        g_servoStatus[s_servoRR].current     = sfb.current;
    }
    s_servoRR = (s_servoRR + 1) % SCServoDriver::JOINT_COUNT;
#endif
}

// =============================================================================
void task_motor_control_init() {
#if !FEATURE_MOTORS_ENABLED && !FEATURE_SERVOS_ENABLED
    return;
#endif

#if FEATURE_MOTORS_ENABLED
    s_motorL.begin();
    s_motorR.begin();
    bool modeL = s_motorL.setMode(DDSM210_LEFT_ID,  2);   // speed loop
    bool modeR = s_motorR.setMode(DDSM210_RIGHT_ID, 2);
#if DEBUG_COMM_ENABLED
#if DEBUG_MOTOR_COMMANDS_ENABLED
    SERIAL_DEBUG.print(F("[MOTOR] init setMode L="));
    SERIAL_DEBUG.print(modeL ? F("OK") : F("FAIL"));
    SERIAL_DEBUG.print(F("  R="));
    SERIAL_DEBUG.println(modeR ? F("OK") : F("FAIL"));
#endif /* DEBUG_MOTOR_COMMANDS_ENABLED */
#endif /* DEBUG_COMM_ENABLED */
#endif /* FEATURE_MOTORS_ENABLED */

#if FEATURE_SERVOS_ENABLED
    s_servos.begin();
#endif

    TaskId_t id = xTaskAdd("MotorCtrl", &motorControlCallback);
    xTaskWait(id);
    xTaskSetTimer(id, 20000);   // 50 Hz
}
