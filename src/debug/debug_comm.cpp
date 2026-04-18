#include <Arduino.h>
#include "debug/debug_comm.h"
#include "config/pin_config.h"
#include "system/shared_state.h"

#if DEBUG_COMM_ENABLED

// =============================================================================
// Name lookup tables — kept short for memory efficiency
// =============================================================================
static const char* cmdName(uint8_t cmd) {
    switch (cmd) {
        case CMD_DRIVE:     return "DRIVE";
        case CMD_CONFIGURE: return "CONFIG";
        case CMD_READ:      return "READ";
        default:            return "?CMD";
    }
}

static const char* devName(uint8_t dev) {
    switch (dev) {
        case DEV_SYSTEM:       return "SYS";
        case DEV_WHEEL_LEFT:   return "WHL_L";
        case DEV_WHEEL_RIGHT:  return "WHL_R";
        case DEV_SERVO_NECK:   return "NECK";
        case DEV_SERVO_EAR_L:  return "EAR_L";
        case DEV_SERVO_EAR_R:  return "EAR_R";
        case DEV_SERVO_LEG_FL: return "LEG_FL";
        case DEV_SERVO_LEG_FR: return "LEG_FR";
        case DEV_SERVO_LEG_BL: return "LEG_BL";
        case DEV_SERVO_LEG_BR: return "LEG_BR";
        case DEV_TOF_LEFT:     return "TOF_L";
        case DEV_TOF_RIGHT:    return "TOF_R";
        case DEV_TOF_BACK:     return "TOF_B";
        case DEV_IMU:          return "IMU";
        case DEV_LIDAR:        return "LIDAR";
        case DEV_BATTERY:      return "BATT";
        case GRP_ALL_WHEELS:   return "GRP_WHL";
        case GRP_ALL_SERVOS:   return "GRP_SRV";
        default:               return "?DEV";
    }
}

static const char* errName(uint8_t err) {
    switch (err) {
        case ERR_OK:             return "OK";
        case ERR_UNKNOWN_CMD:    return "UNK_CMD";
        case ERR_UNKNOWN_DEVICE: return "UNK_DEV";
        case ERR_UNKNOWN_PARAM:  return "UNK_PAR";
        case ERR_DEVICE_FAULT:   return "FAULT";
        case ERR_SAFETY_BLOCK:   return "SAFETY";
        case ERR_TIMEOUT:        return "TIMEOUT";
        case ERR_INVALID_VALUE:  return "INV_VAL";
        case ERR_NOT_READY:      return "NOT_RDY";
        default:                 return "?ERR";
    }
}

// =============================================================================
// Raw RX monitoring state
// =============================================================================
static uint32_t s_totalBytes    = 0;
static uint32_t s_lastReportMs  = 0;
static const uint32_t REPORT_INTERVAL_MS = 2000;  // print stats every 2 s
static bool     s_firstBytes    = true;            // hex-dump first burst

// =============================================================================
void debug_comm_init() {
    SERIAL_DEBUG.begin(DEBUG_BAUDRATE);
    SERIAL_DEBUG.println(F("[DBG] Comm debug enabled"));
    s_lastReportMs = millis();
}

void debug_print_request(const RequestPacket& req) {
    SERIAL_DEBUG.print(F("[RX] seq="));
    SERIAL_DEBUG.print(req.sequence);
    SERIAL_DEBUG.print(F(" cmd="));
    SERIAL_DEBUG.print(cmdName(req.command));
    SERIAL_DEBUG.print(F(" dev="));
    SERIAL_DEBUG.print(devName(req.device_id));
    SERIAL_DEBUG.print(req.rw_flag == RW_WRITE ? F(" W") : F(" R"));
    SERIAL_DEBUG.print(F(" par=0x"));
    SERIAL_DEBUG.print(req.parameter, HEX);

    // PARAM_POSITION on servos: value = [pos:u16][speed:u16], not float
    if (req.parameter == PARAM_POSITION &&
        req.device_id >= DEV_SERVO_NECK && req.device_id <= DEV_SERVO_LEG_BR) {
        uint16_t pos, spd;
        memcpy(&pos, req.value, 2);
        memcpy(&spd, req.value + 2, 2);
        SERIAL_DEBUG.print(F(" pos="));
        SERIAL_DEBUG.print(pos);
        SERIAL_DEBUG.print(F(" spd="));
        SERIAL_DEBUG.println(spd);
    } else {
        float val = appl_unpack_float(req.value);
        SERIAL_DEBUG.print(F(" val="));
        SERIAL_DEBUG.println(val, 2);
    }
}

void debug_comm_rx_tick(HardwareSerial& port) {
    int avail = port.available();
    if (avail > 0) {
        s_totalBytes += avail;

        // Log that first data arrived (without consuming any bytes —
        // PacketSerial needs every byte for correct COBS decoding).
        if (s_firstBytes) {
            s_firstBytes = false;
            SERIAL_DEBUG.print(F("[RAW] First data on Serial1: "));
            SERIAL_DEBUG.print(avail);
            SERIAL_DEBUG.println(F(" bytes available"));
        }
    }

    // Periodic stats
    uint32_t now = millis();
    if (now - s_lastReportMs >= REPORT_INTERVAL_MS) {
        s_lastReportMs = now;
        SERIAL_DEBUG.print(F("[COMM] Serial1 total_bytes="));
        SERIAL_DEBUG.print(s_totalBytes);
        SERIAL_DEBUG.print(F("  rx_buf_pending="));
        SERIAL_DEBUG.print(g_rxCommandBuf.count());
        SERIAL_DEBUG.print(F("  tx_buf_pending="));
        SERIAL_DEBUG.println(g_txResponseBuf.count());
    }
}

void debug_print_response(const ResponsePacket& rsp) {
    // Skip telemetry to avoid flooding the debug output
    if (rsp.pkt_type == PKT_TELEMETRY) return;

    float val = appl_unpack_float(rsp.value);

    SERIAL_DEBUG.print(F("[TX] seq="));
    SERIAL_DEBUG.print(rsp.sequence);
    SERIAL_DEBUG.print(F(" cmd="));
    SERIAL_DEBUG.print(cmdName(rsp.command));
    SERIAL_DEBUG.print(F(" dev="));
    SERIAL_DEBUG.print(devName(rsp.device_id));
    SERIAL_DEBUG.print(F(" par=0x"));
    SERIAL_DEBUG.print(rsp.parameter, HEX);
    SERIAL_DEBUG.print(F(" val="));
    SERIAL_DEBUG.print(val, 2);
    SERIAL_DEBUG.print(F(" err="));
    SERIAL_DEBUG.println(errName(rsp.error));
}

#endif
