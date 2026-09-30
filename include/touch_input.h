#pragma once
#include <Arduino.h>
#include "config.h"

// ============================================================================
//  One touch API, five controllers.
//
//  The 2.8" CYD ships in two flavours. The very common ESP32-2432S028**R**
//  carries an XPT2046 resistive panel on its own SPI bus; the capacitive
//  members of the same Sunton family carry a CST816/CST820 (or, on some
//  batches, a GT911) on I2C. Both are supported - pick the environment in
//  platformio.ini. The capacitive driver auto-detects which chip is present.
//
//  The other boards each have exactly one controller, picked by the board
//  define: the ESP32-C6-LCD-1.9's CST8xx uses that same capacitive driver,
//  the ESP32-C6-Touch-LCD-1.47 has an AXS5106L and the
//  ESP32-S3-Touch-AMOLED-1.64 an FT3168.
//
//  Output is always screen coordinates for the configured rotation.
// ============================================================================

namespace touch {

void        begin();
// True while a finger/stylus is down. x,y are screen pixels.
bool        read(int16_t &x, int16_t &y);
const char *driverName();

// Exposed so the UI can draw a "hold" progress indicator.
struct Gesture {
    bool     pressed      = false;
    bool     justPressed  = false;
    bool     justReleased = false;
    int16_t  x = 0, y = 0;          // current (or last) position
    int16_t  downX = 0, downY = 0;  // where the press started
    uint32_t heldMs = 0;
};

// Debounced, edge-detected wrapper. Call once per UI frame.
const Gesture &poll();

} // namespace touch
