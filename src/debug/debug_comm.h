#pragma once
// =============================================================================
// debug_comm.h — Communication debug logging over USB Serial
// =============================================================================
// Per-function gates match the channel that controls each output. See
// config/debug_config.h for the channel switches.
// =============================================================================

#include "config/debug_config.h"
#include "comm/appl_protocol.h"
#include <Arduino.h>

#if DEBUG_ANY_ENABLED
void debug_comm_init();
#else
inline void debug_comm_init() {}
#endif

#if DEBUG_REQUESTS_ENABLED
void debug_print_request(const RequestPacket& req);
void debug_print_response(const ResponsePacket& rsp);
#else
inline void debug_print_request(const RequestPacket&) {}
inline void debug_print_response(const ResponsePacket&) {}
#endif

#if DEBUG_COMM_ENABLED
void debug_comm_rx_tick(HardwareSerial& port);
#else
inline void debug_comm_rx_tick(HardwareSerial&) {}
#endif
