#pragma once
#include <Arduino.h>

// ============================================================================
//  VESC Tool compatible BLE bridge.
//
//  VESC Tool (and the official mobile app) look for a Nordic UART Service:
//      6E400001-... service
//      6E400002-... RX, write         phone -> VESC
//      6E400003-... TX, notify        VESC  -> phone
//  The phone speaks the same framed protocol as the VESC's own UART, and
//  every payload reaches the VESC unchanged (tunnelled over CAN, or written
//  to the UART - see vesc_link.h), so firmware updates, motor detection and
//  configuration all work through the display exactly as they would with an
//  nRF module.
//
//  Modelled on A-Emile/VescBLEBridge, adapted to share the VESC link with the
//  dash's own traffic.
// ============================================================================

namespace blebridge {

void begin();
bool isConnected();
// Push bytes received from the VESC out to the connected phone.
void notify(const uint8_t *data, size_t len);
// Negotiated ATT MTU, useful for diagnostics.
uint16_t mtu();

} // namespace blebridge
