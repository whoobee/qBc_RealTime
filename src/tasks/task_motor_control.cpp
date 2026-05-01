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

// Per-wheel cached DDSM210 control mode (2=speed, 3=position).
// We only call setMode when the requested mode changes, since each
// setMode is a separate UART round-trip (~5 ms settling).
static uint8_t s_motorModeL = 2;
static uint8_t s_motorModeR = 2;

// Per-wheel accumulated absolute target (degrees, 0..360 with wrap).
// The wire protocol sends a signed delta; we keep a running absolute
// target so the host can issue successive deltas without tracking the
// motor's physical wrap-around. Initialised on first position cmd
// from the encoder's actual reading via getInfo().
static float s_targetAbsL = 0.0f;
static float s_targetAbsR = 0.0f;
static bool  s_targetSeededL = false;
static bool  s_targetSeededR = false;

// getInfo polling state — alternates wheels every N ticks so each side
// gets fresh mileage/position telemetry without doubling UART load on
// the motor task. Polls only when no driving cmd is pending.
static uint8_t s_infoTickCounter = 0;
static bool    s_pollLeftNext = true;
#define MOTOR_INFO_POLL_PERIOD_TICKS 10   // 50 Hz / 10 = 5 Hz alternating

static inline float wrap_deg_360(float d) {
    // Normalise to [0, 360). fmodf can return negative — adjust.
    float r = fmodf(d, 360.0f);
    if (r < 0.0f) r += 360.0f;
    return r;
}

// Send a position-delta command to one wheel. Switches mode to 3 if
// needed, accumulates target, calls setPosition with the absolute
// target. Returns the driver's success flag.
static bool driveWheelPosition(DDSM210& motor, uint8_t id,
                               float delta_deg,
                               uint8_t& cached_mode,
                               float& target_abs,
                               bool& target_seeded) {
    if (cached_mode != 3) {
        if (motor.setMode(id, 3)) cached_mode = 3;
    }
    if (!target_seeded) {
        DDSM210InfoFeedback info{};
        if (motor.getInfo(id, info)) {
            target_abs = info.position * (360.0f / 32767.0f);
            target_seeded = true;
        } else {
            target_abs = 0.0f;
            target_seeded = true;   // best-effort fallback
        }
    }
    target_abs = wrap_deg_360(target_abs + delta_deg);
    return motor.setPosition(id, target_abs);
}

static bool driveWheelVelocity(DDSM210& motor, uint8_t id, float rpm,
                               uint8_t& cached_mode,
                               bool& target_seeded) {
    if (cached_mode != 2) {
        if (motor.setMode(id, 2)) cached_mode = 2;
    }
    // Reset the position accumulator so the next position cmd
    // re-syncs against the encoder rather than against a stale target
    // that doesn't reflect any drift during velocity mode.
    target_seeded = false;
    return motor.setSpeed(id, rpm);
}
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
        bool okL = false, okR = false;

        if (g_motorCmd.mode == MOTOR_CMD_VELOCITY) {
            okL = driveWheelVelocity(s_motorL, DDSM210_LEFT_ID,
                                      g_motorCmd.left_value,
                                      s_motorModeL, s_targetSeededL);
            // Right motor is mirrored — sign convention preserved.
            okR = driveWheelVelocity(s_motorR, DDSM210_RIGHT_ID,
                                      -g_motorCmd.right_value,
                                      s_motorModeR, s_targetSeededR);
        } else {
            // POSITION: delta in degrees of wheel rotation.
            okL = driveWheelPosition(s_motorL, DDSM210_LEFT_ID,
                                      g_motorCmd.left_value,
                                      s_motorModeL,
                                      s_targetAbsL, s_targetSeededL);
            // Right motor's encoder runs opposite to forward travel
            // (mirror mounting) so invert the delta to keep the wire
            // contract "positive = both wheels forward".
            okR = driveWheelPosition(s_motorR, DDSM210_RIGHT_ID,
                                      -g_motorCmd.right_value,
                                      s_motorModeR,
                                      s_targetAbsR, s_targetSeededR);
        }

        g_motorCmdPending = false;
#if DEBUG_MOTOR_COMMANDS_ENABLED
        SERIAL_DEBUG.print(F("[MOTOR] mode="));
        SERIAL_DEBUG.print(g_motorCmd.mode == MOTOR_CMD_POSITION ? F("POS") : F("VEL"));
        SERIAL_DEBUG.print(F(" L="));
        SERIAL_DEBUG.print(g_motorCmd.left_value);
        SERIAL_DEBUG.print(okL ? F(" OK") : F(" FAIL"));
        SERIAL_DEBUG.print(F("  R="));
        SERIAL_DEBUG.print(g_motorCmd.right_value);
        SERIAL_DEBUG.println(okR ? F(" OK") : F(" FAIL"));
#else
        (void)okL; (void)okR;
#endif
    } else {
        // ---- Idle: poll getInfo to refresh mileage + encoder position ----
        // Alternate L/R every poll period; the cycle costs ~1 UART round-
        // trip per poll, well within the 50 Hz tick budget.
        if (++s_infoTickCounter >= MOTOR_INFO_POLL_PERIOD_TICKS) {
            s_infoTickCounter = 0;
            DDSM210InfoFeedback info{};
            if (s_pollLeftNext) {
                if (s_motorL.getInfo(DDSM210_LEFT_ID, info)) {
                    g_motorStatus[0].mileage  = info.mileage;
                    g_motorStatus[0].position = info.position;
                }
            } else {
                if (s_motorR.getInfo(DDSM210_RIGHT_ID, info)) {
                    g_motorStatus[1].mileage  = info.mileage;
                    g_motorStatus[1].position = info.position;
                }
            }
            s_pollLeftNext = !s_pollLeftNext;
        }
    }

    // ---- Publish motor feedback to shared state (from last setSpeed/setPosition) ----
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
#if DEBUG_SERVOS_ENABLED
        SERIAL_DEBUG.print(F("[SERVO] dev=0x"));
        SERIAL_DEBUG.print(scmd.servo_appl_id, HEX);
        SERIAL_DEBUG.print(F(" pos="));
        SERIAL_DEBUG.print(scmd.position);
        SERIAL_DEBUG.print(F(" spd="));
        SERIAL_DEBUG.print(scmd.speed);
        SERIAL_DEBUG.print(F(" acc="));
        SERIAL_DEBUG.print(scmd.acceleration);
        SERIAL_DEBUG.println(ok ? F(" OK") : F(" FAIL"));
#else
        (void)ok;
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
#if DEBUG_MOTOR_COMMANDS_ENABLED
    SERIAL_DEBUG.print(F("[MOTOR] init setMode L="));
    SERIAL_DEBUG.print(modeL ? F("OK") : F("FAIL"));
    SERIAL_DEBUG.print(F("  R="));
    SERIAL_DEBUG.println(modeR ? F("OK") : F("FAIL"));
#else
    (void)modeL; (void)modeR;
#endif /* DEBUG_MOTOR_COMMANDS_ENABLED */
#endif /* FEATURE_MOTORS_ENABLED */

#if FEATURE_SERVOS_ENABLED
    s_servos.begin();
#endif

    TaskId_t id = xTaskAdd("MotorCtrl", &motorControlCallback);
    xTaskWait(id);
    xTaskSetTimer(id, 20000);   // 50 Hz
}
