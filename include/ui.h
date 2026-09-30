#pragma once
#include <Arduino.h>
#include "dash_model.h"
#include "config.h"

// ============================================================================
//  The dashboard's drawing interface, implemented once per display library:
//  ui.cpp for the CYD (TFT_eSPI, 320x240 landscape), ui_c6.cpp for every
//  other board (Arduino_GFX, portrait or landscape - its header comment has
//  those layouts). The CYD's:
//
//   +--------------------------------------------------------------+
//   | [####battery###] 62%      54.2V         *BLE*   o OK         |  status
//   +--------------------------------------+-----------------------+
//   |                                      |  TRIP      12.4       |
//   |      4 8 .5                          |  WH USED   146        |
//   |         km/h                         |  WH/KM     11.8       |
//   |   [ regen |=====drive=====  ] 1240 W |  MOTOR C   47         |
//   +--------------------------------------+-----------------------+
//   | ( SPORT )  HOLD                      ODO 431.2 km            |  footer
//   +--------------------------------------------------------------+
//
//  The profile badge in the bottom-left corner is the long-press target.
// ============================================================================

namespace ui {

// Layout constants are private to each target's implementation file
// (ui.cpp for the CYD, ui_c6.cpp for every other board) - nothing outside
// either file needs them, only the function surface below.

void begin();
void splash(const char *line1, const char *line2);

// holdProgress: 0..1, drives the hold-progress sliver along the bottom edge
// of the profile badge.
void render(const DashModel &m, float holdProgress);

void    setPage(uint8_t page);
uint8_t page();
void    nextPage();

void toast(const char *msg, uint32_t durationMs = 1400);
void setBrightness(uint8_t pct);
void forceFullRedraw();

// Re-samples the onboard LDR and adjusts the backlight, at most once every
// AUTO_BRIGHTNESS_UPDATE_MS. Safe to call every loop() iteration - it rate-
// limits itself, and no-ops entirely when AUTO_BRIGHTNESS is false. See the
// AUTO_BRIGHTNESS comment in config.h for the sensor's quirks.
void updateAutoBrightness();

// --- hit testing ----------------------------------------------------------
bool inProfileBadge(int16_t x, int16_t y);
bool inStatColumn(int16_t x, int16_t y);
bool inSpeedArea(int16_t x, int16_t y);

} // namespace ui
