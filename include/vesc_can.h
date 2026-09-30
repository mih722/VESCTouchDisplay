#pragma once
#include <Arduino.h>

// ============================================================================
//  Thin hardware transport: the ESP32's built-in TWAI (CAN) controller.
//
//  This is the only file that touches driver/twai.h - vesc_link.cpp and
//  vesc_protocol.cpp only ever see CanFrame, so the host tests can stub this
//  API out exactly the way they used to stub HardwareSerial.
// ============================================================================

struct CanFrame {
    uint32_t id  = 0;        // 29-bit extended CAN id
    uint8_t  dlc = 0;        // 0..8
    uint8_t  data[8] = {0};
};

namespace vescan {

void begin();

// Non-blocking. Returns true if a frame was waiting.
bool poll(CanFrame &out);

// Non-blocking best-effort transmit. Returns true if it was queued.
bool send(const CanFrame &f);

} // namespace vescan
