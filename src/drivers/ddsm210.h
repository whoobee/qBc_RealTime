#pragma once
// =============================================================================
// ddsm210.h — Waveshare DDSM210 brushless DC motor driver (UART)
// =============================================================================
// Protocol: 10-byte fixed-length packets, CRC-8/MAXIM, 115200 baud 8N1
// Modes:    speed (0.1 RPM units, ±210 RPM), position (0–32767 → 0–360°)
// =============================================================================

#include <Arduino.h>

struct DDSM210Feedback {
    float   speed_rpm;       // Actual speed in RPM
    float   current_a;       // Motor current in amps
    uint8_t acc_time;        // Acceleration time register
    uint8_t temperature_c;   // Motor temperature °C
    uint8_t fault_code;      // 0 = OK
};

struct DDSM210InfoFeedback {
    int32_t mileage;         // Cumulative encoder counts
    int16_t position;        // 0–32767 → 0–360°
    uint8_t fault_code;
};

class DDSM210 {
public:
    explicit DDSM210(HardwareSerial* serial);

    void begin();

    // Speed control — RPM, clamped to ±210
    // Returns true if feedback was received successfully.
    bool setSpeed(uint8_t id, float rpm, uint8_t acc_time = 0);

    // Position control — 0.0–360.0 degrees
    bool setPosition(uint8_t id, float degrees, uint8_t acc_time = 0);

    // Change motor control mode: 0=open-loop, 2=speed, 3=position
    bool setMode(uint8_t id, uint8_t mode);

    // Request extended info (mileage + position)
    bool getInfo(uint8_t id, DDSM210InfoFeedback& info);

    // Stop motor (send zero speed)
    bool stop(uint8_t id);

    // Last feedback from the most recent setSpeed/setPosition call
    const DDSM210Feedback& lastFeedback() const { return _fb; }

private:
    // Protocol constants
    static constexpr uint8_t CMD_CTRL        = 0x64;
    static constexpr uint8_t CMD_GET_INFO    = 0x74;
    static constexpr uint8_t CMD_CHANGE_MODE = 0xA0;
    static constexpr size_t  PACKET_LEN      = 10;

    static uint8_t crc8_maxim(const uint8_t* data, size_t len);

    void buildCtrlPacket(uint8_t* pkt, uint8_t id,
                         int16_t cmd_value, uint8_t acc);
    bool sendAndReceive(const uint8_t* tx, uint8_t* rx);
    bool parseCtrlFeedback(const uint8_t* rx);

    HardwareSerial* _serial;
    DDSM210Feedback _fb;
    uint32_t        _lastCmdTime;
};
