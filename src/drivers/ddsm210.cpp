#include "drivers/ddsm210.h"
#include "config/device_config.h"

// =============================================================================
// CRC-8/MAXIM (Dallas/1-Wire) — poly 0x8C reflected, init 0x00
// =============================================================================
uint8_t DDSM210::crc8_maxim(const uint8_t* data, size_t len) {
    uint8_t crc = 0;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) {
            crc = (crc & 0x01) ? (crc >> 1) ^ 0x8C : (crc >> 1);
        }
    }
    return crc;
}

// =============================================================================
DDSM210::DDSM210(HardwareSerial* serial)
    : _serial(serial), _fb{}, _lastCmdTime(0) {}

void DDSM210::begin() {
    _serial->begin(DDSM210_BAUDRATE);
    _lastCmdTime = millis();
}

// =============================================================================
// Build a 10-byte control packet
// =============================================================================
void DDSM210::buildCtrlPacket(uint8_t* pkt, uint8_t id,
                              int16_t cmd_value, uint8_t acc) {
    pkt[0] = id;
    pkt[1] = CMD_CTRL;
    pkt[2] = (cmd_value >> 8) & 0xFF;   // high byte
    pkt[3] = cmd_value & 0xFF;           // low byte
    pkt[4] = 0x00;
    pkt[5] = 0x00;
    pkt[6] = acc;
    pkt[7] = 0x00;
    pkt[8] = 0x00;
    pkt[9] = crc8_maxim(pkt, 9);
}

// =============================================================================
// Send 10 bytes, wait for 10-byte response
// =============================================================================
bool DDSM210::sendAndReceive(const uint8_t* tx, uint8_t* rx) {
    // Enforce minimum inter-command delay
    uint32_t now = millis();
    uint32_t elapsed = now - _lastCmdTime;
    if (elapsed < DDSM210_CMD_DELAY_MS) {
        delay(DDSM210_CMD_DELAY_MS - elapsed);
    }

    // Flush stale data
    while (_serial->available()) _serial->read();

    _serial->write(tx, PACKET_LEN);
    _serial->flush();  // wait for TX to complete
    _lastCmdTime = millis();

    // Read response with timeout
    uint32_t start = millis();
    size_t idx = 0;
    while (idx < PACKET_LEN) {
        if (_serial->available()) {
            rx[idx++] = _serial->read();
        }
        if (millis() - start > DDSM210_RX_TIMEOUT_MS + 10) {
            return false;  // timeout
        }
    }

    // Validate CRC
    uint8_t expected = crc8_maxim(rx, 9);
    return (rx[9] == expected);
}

// =============================================================================
// Parse DDSM210 control feedback (response to CMD_CTRL)
// =============================================================================
bool DDSM210::parseCtrlFeedback(const uint8_t* rx) {
    if (rx[1] != CMD_CTRL) return false;

    int16_t raw_speed = (int16_t)((rx[2] << 8) | rx[3]);
    _fb.speed_rpm     = raw_speed * 0.1f;  // 0.1 RPM units → RPM

    int16_t raw_current = (int16_t)((rx[4] << 8) | rx[5]);
    _fb.current_a     = raw_current * (8.0f / 32767.0f);  // ±8A range

    _fb.acc_time      = rx[6];
    _fb.temperature_c = rx[7];
    _fb.fault_code    = rx[8];
    return true;
}

// =============================================================================
// Public API
// =============================================================================
bool DDSM210::setSpeed(uint8_t id, float rpm, uint8_t acc_time) {
    // Clamp to DDSM210 speed range: ±210 RPM → ±2100 in 0.1 RPM units
    int16_t cmd = (int16_t)(rpm * 10.0f);
    if (cmd >  2100) cmd =  2100;
    if (cmd < -2100) cmd = -2100;

    uint8_t tx[PACKET_LEN], rx[PACKET_LEN];
    buildCtrlPacket(tx, id, cmd, acc_time);

    if (!sendAndReceive(tx, rx)) return false;
    return parseCtrlFeedback(rx);
}

bool DDSM210::setPosition(uint8_t id, float degrees, uint8_t acc_time) {
    // 0–360° → 0–32767
    float clamped = degrees;
    if (clamped < 0.0f)   clamped = 0.0f;
    if (clamped > 360.0f) clamped = 360.0f;
    int16_t cmd = (int16_t)(clamped * (32767.0f / 360.0f));

    uint8_t tx[PACKET_LEN], rx[PACKET_LEN];
    buildCtrlPacket(tx, id, cmd, acc_time);

    if (!sendAndReceive(tx, rx)) return false;
    return parseCtrlFeedback(rx);
}

bool DDSM210::setMode(uint8_t id, uint8_t mode) {
    uint8_t tx[PACKET_LEN] = {};
    tx[0] = id;
    tx[1] = CMD_CHANGE_MODE;
    tx[2] = mode;   // 0=open-loop, 2=speed, 3=position
    // bytes [3..8] = 0
    tx[9] = crc8_maxim(tx, 9);

    uint8_t rx[PACKET_LEN];
    bool ok = sendAndReceive(tx, rx);
    delay(5);  // mode change settling time
    return ok;
}

bool DDSM210::getInfo(uint8_t id, DDSM210InfoFeedback& info) {
    uint8_t tx[PACKET_LEN] = {};
    tx[0] = id;
    tx[1] = CMD_GET_INFO;
    // bytes [2..8] = 0
    tx[9] = crc8_maxim(tx, 9);

    uint8_t rx[PACKET_LEN];
    if (!sendAndReceive(tx, rx)) return false;
    if (rx[1] != CMD_GET_INFO) return false;

    // mileage: signed 32-bit big-endian in bytes [2..5]
    info.mileage = ((int32_t)rx[2] << 24) | ((int32_t)rx[3] << 16)
                 | ((int32_t)rx[4] << 8)  | rx[5];
    // position: signed 16-bit big-endian in bytes [6..7]
    info.position   = (int16_t)((rx[6] << 8) | rx[7]);
    info.fault_code = rx[8];
    return true;
}

bool DDSM210::stop(uint8_t id) {
    return setSpeed(id, 0.0f, 0);
}
