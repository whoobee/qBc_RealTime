#pragma once
// =============================================================================
// serial_transport.h — Teensy-side COBS serial transport (PacketSerial)
// =============================================================================

#include <Arduino.h>
#include <PacketSerial.h>
#include "comm/appl_protocol.h"

class SerialTransport {
public:
    SerialTransport();

    void begin(HardwareSerial* serial, uint32_t baud);

    // Poll for incoming COBS frames.  Call frequently from CommRX task.
    // Decoded RequestPackets are pushed to g_rxCommandBuf.
    void update();

    // Encode and transmit a ResponsePacket as a COBS frame.
    void send(const ResponsePacket& rsp);

private:
    static void onPacketReceived(const uint8_t* buf, size_t len);
    static SerialTransport* _instance;

    PacketSerial _pktSerial;
};
