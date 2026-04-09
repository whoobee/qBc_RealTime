#include "comm/serial_transport.h"
#include "system/shared_state.h"
#include "debug/debug_comm.h"

SerialTransport* SerialTransport::_instance = nullptr;

SerialTransport::SerialTransport() {
    _instance = this;
}

void SerialTransport::begin(HardwareSerial* serial, uint32_t baud) {
    serial->begin(baud);
    _pktSerial.setStream(serial);
    _pktSerial.setPacketHandler(&SerialTransport::onPacketReceived);
}

void SerialTransport::update() {
    _pktSerial.update();
}

void SerialTransport::send(const ResponsePacket& rsp) {
    _pktSerial.send(reinterpret_cast<const uint8_t*>(&rsp),
                    sizeof(ResponsePacket));
}

// Called synchronously from update() — runs in CommRX task context.
void SerialTransport::onPacketReceived(const uint8_t* buf, size_t len) {
    if (!_instance) return;

#if DEBUG_COMM_ENABLED
    // Log every decoded COBS frame regardless of validity
    SERIAL_DEBUG.print(F("[COBS] frame len="));
    SERIAL_DEBUG.print(len);
    SERIAL_DEBUG.print(F(" hex="));
    for (size_t i = 0; i < len && i < 16; i++) {
        if (buf[i] < 0x10) SERIAL_DEBUG.print('0');
        SERIAL_DEBUG.print(buf[i], HEX);
        SERIAL_DEBUG.print(' ');
    }
    SERIAL_DEBUG.println();
#endif

    if (len != sizeof(RequestPacket)) return;
    if (buf[0] != PKT_REQUEST) return;

    RequestPacket req;
    memcpy(&req, buf, sizeof(RequestPacket));

    g_rxCommandBuf.push(req);
    debug_print_request(req);
}
