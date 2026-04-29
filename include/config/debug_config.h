#pragma once
// =============================================================================
// debug_config.h — Per-channel debug logging switches
// =============================================================================
// All debug output goes to SERIAL_DEBUG (USB Serial — defined in pin_config.h)
// at DEBUG_BAUDRATE. Each channel is independent: set to 1 to enable, 0 to
// silence. Disabled channels compile out entirely (no flash/RAM cost).
//
// Channel        Tag            Source files
// -----------    -----------    -------------------------------------------
// COMM           [COMM] [RAW]   tasks/task_comm.cpp, debug/debug_comm.cpp
// COBS           [COBS]         comm/serial_transport.cpp
// REQUESTS       [RX] [TX]      debug/debug_comm.cpp (per-packet decode)
// TOF            [TOF]          tasks/task_perception.cpp
// SERVOS         [SERVO]        tasks/task_motor_control.cpp
// MOTOR_COMMANDS [MOTOR]        tasks/task_motor_control.cpp
// SAFETY         [SAFETY]       safety/safety_manager.cpp
// =============================================================================

// --- Per-channel switches ---
#define DEBUG_COMM_ENABLED            0   // High-level comm activity + raw Serial1 byte stats
#define DEBUG_COBS_ENABLED            0   // Hex dump of every decoded COBS frame (very chatty)
#define DEBUG_REQUESTS_ENABLED        0   // Per-packet RX/TX decode (cmd/dev/param/value)
#define DEBUG_TOF_ENABLED             1   // Periodic VL53L0X readings (~2 Hz, throttled)
#define DEBUG_SERVOS_ENABLED          0   // ST3215 servo command echo
#define DEBUG_MOTOR_COMMANDS_ENABLED  0   // DDSM210 wheel motor command echo
#define DEBUG_SAFETY_ENABLED          0   // Safety state-change logs (entered/cleared)

// --- Debug serial settings ---
#define DEBUG_BAUDRATE  115200            // USB Serial baud rate

// --- Aggregate — true when ANY channel is on. Used to gate debug_comm_init(). ---
#define DEBUG_ANY_ENABLED ( \
    DEBUG_COMM_ENABLED            || \
    DEBUG_COBS_ENABLED            || \
    DEBUG_REQUESTS_ENABLED        || \
    DEBUG_TOF_ENABLED             || \
    DEBUG_SERVOS_ENABLED          || \
    DEBUG_MOTOR_COMMANDS_ENABLED  || \
    DEBUG_SAFETY_ENABLED)
