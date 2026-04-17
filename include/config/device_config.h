#pragma once
// =============================================================================
// device_config.h — Hardware device IDs, baud rates, I2C addresses
// =============================================================================

// --- DDSM210 Wheel Motors (UART, fixed-length 10-byte packets) ---
// Each motor is on a DEDICATED serial port (no shared bus).
// Motor IDs may both be 1 (factory default) since they don't share a bus.
#define DDSM210_BAUDRATE        115200
#define DDSM210_LEFT_ID         1       // Motor ID on its dedicated Serial5
#define DDSM210_RIGHT_ID        1       // Motor ID on its dedicated Serial3
#define DDSM210_CMD_DELAY_MS    4       // min ms between consecutive commands
#define DDSM210_RX_TIMEOUT_MS   4       // feedback read timeout

// --- ST3215 Servo Bus (UART via Waveshare control board) ---
#define SERVO_BAUDRATE          1000000  // 1 Mbaud (default for ST3215)

// Servo hardware IDs (programmed into each servo's EEPROM)
#define SERVO_ID_NECK           200
#define SERVO_ID_EAR_LEFT       201
#define SERVO_ID_EAR_RIGHT      202
#define SERVO_ID_LEG_FL         101
#define SERVO_ID_LEG_FR         102
#define SERVO_ID_LEG_BL         103
#define SERVO_ID_LEG_BR         104
#define SERVO_COUNT             7

// Servo zero (home) positions — from qB calibration data
#define SERVO_ZERO_NECK        1913
#define SERVO_ZERO_EAR_L       1824
#define SERVO_ZERO_EAR_R       2316
#define SERVO_ZERO_LEG_FL      2048
#define SERVO_ZERO_LEG_FR      2467
#define SERVO_ZERO_LEG_BL      2243
#define SERVO_ZERO_LEG_BR      2131

// Convenience array (populate in .cpp if needed)
static const uint8_t SERVO_IDS[SERVO_COUNT] = {
    SERVO_ID_NECK, SERVO_ID_EAR_LEFT, SERVO_ID_EAR_RIGHT,
    SERVO_ID_LEG_FL, SERVO_ID_LEG_FR, SERVO_ID_LEG_BL, SERVO_ID_LEG_BR
};

// --- VL53L0X TOF Sensor (I2C via Wire2) ---
// Single sensor at default I2C address 0x29. DFRobot library convention
// takes the 8-bit form (0x52 = 0x29 << 1). No address reassignment needed
// since only one sensor is on Wire2.
#define TOF_DEFAULT_ADDR_7BIT   0x29
#define TOF_DEFAULT_ADDR_8BIT   0x52
#define TOF_I2C_CLOCK_HZ        100000  // 100 kHz — matches qB_Test_TOF

// --- MPU6050 IMU (I2C via Wire) ---
// AD0 pin low/floating → 0x68, high → 0x69.
#define MPU6050_I2C_ADDR        0x68
#define MPU6050_I2C_CLOCK_HZ    100000  // 100 kHz — matches qB_Test_TOF

// --- YDLidar GS2 (UART @ 921600 baud, handled internally by library) ---
// The library calls Serial.begin() itself — do NOT pre-init the port.
#define LIDAR_SENSOR_COUNT      1       // single GS2 unit

// --- Pi Communication Link (UART, COBS-framed via PacketSerial) ---
#define PI_LINK_BAUDRATE        115200
