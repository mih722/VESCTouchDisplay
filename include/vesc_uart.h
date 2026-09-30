#pragma once
#include <Arduino.h>

// ============================================================================
//  Thin hardware transport: a plain UART to the VESC's COMM port.
//
//  The UART counterpart of vesc_can.h, used in its place when config.h's
//  VESC_LINK_UART is 1. Bytes only - framing, and working out whose reply is
//  whose, happen in vesc_link.cpp - so the host tests can stub these three
//  calls out the same way they stub vescan.
// ============================================================================

namespace vescuart {

void begin();

// Non-blocking. Copies up to `cap` bytes that have already arrived into
// `buf`. Returns how many; 0 means nothing was waiting.
size_t read(uint8_t *buf, size_t cap);

// Blocks until the driver has taken all `len` bytes.
size_t write(const uint8_t *data, size_t len);

} // namespace vescuart
