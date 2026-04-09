#pragma once
// =============================================================================
// debug_comm.h — Communication debug logging over USB Serial
// =============================================================================
// Prints human-readable summaries of RX requests and TX responses.
// Controlled by DEBUG_COMM_ENABLED in pin_config.h.
// =============================================================================

#include "config/pin_config.h"
#include "comm/appl_protocol.h"

#if DEBUG_COMM_ENABLED

void debug_comm_init();
void debug_print_request(const RequestPacket& req);
void debug_print_response(const ResponsePacket& rsp);

// Call from CommRX callback — monitors raw Serial1 activity
void debug_comm_rx_tick(HardwareSerial& port);

#else

inline void debug_comm_init() {}
inline void debug_print_request(const RequestPacket&) {}
inline void debug_print_response(const ResponsePacket&) {}
inline void debug_comm_rx_tick(HardwareSerial&) {}

#endif
