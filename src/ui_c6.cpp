#include "ui.h"
#include <Arduino_GFX_Library.h>
#include <math.h>
#include "vesc_protocol.h"
#include "profiles.h"

// ============================================================================
//  Two layouts for the Waveshare ESP32-C6-LCD-1.9 / ESP32-C6-Touch-LCD-1.47
//  and the ESP32-S3-Touch-AMOLED-1.64 (everything but the CYD), selected at
//  compile time by UI_LAYOUT_PORTRAIT in config.h. All three panels are
//  physically portrait-native; this only picks which way the firmware
//  treats them, not different wiring. The two layouts share the same zones
//  - battery strip, a BLE/link status strip, a speed/telemetry zone (tap to
//  cycle, hold to reset trip), a profile band (hold to change), and a power
//  meter - so drawBleIcon/drawPrimary/drawFault/render are shared outright
//  and drawBattery differs only in its text. UI_LAYOUT_PORTRAIT picks the
//  proportions, the profile band's hit area, and a separate drawStatus()/
//  drawProfile()/drawPowerBar() for each layout.
//
//  Landscape (UI_LAYOUT_PORTRAIT false):
//
//   +------------------------------+----------+
//   |    [====>    ] 96%  55.0V    |*BLE* o OK|  battery (icon + % + volt,
//   +------------------------------+----------+   3/4 width) | status (1/4)
//   +-----------------------------------------+
//   |                  48.5                    |  speed by default;
//   |                  km/h                    |  tap to cycle,
//   +-----------------------------------------+  hold to reset trip
//   +---------------+-------------------------+
//   |     SPORT     |      [regen|drive]       |  profile (hold to
//   |               |         1240 W           |  change) | power meter
//   +---------------+-------------------------+
//
//  Portrait (UI_LAYOUT_PORTRAIT true):
//
//   +-------------------------+-------+
//   |     [===>  ] 55.0V      | *B* o |  battery (icon + voltage, no % -
//   +-------------------------+-------+   3/4 width, see drawBattery) |
//   +---------------------------------+   status (icon + dot, no text -
//   |              48.5                |   not enough width)
//   |              km/h                |  speed by default; tap to cycle,
//   +---------------------------------+  hold to reset trip
//   |          [regen|drive]           |  power meter
//   |             1240 W               |
//   +---------------------------------+
//   |              SPORT               |  profile (hold to change)
//   +---------------------------------+
//
//  Tapping the speed/telemetry zone cycles through the same 8 fields the CYD
//  shows across its two stat pages (README: page 1 = trip/Wh used/Wh per km/
//  motor temp, page 2 = range/max speed/avg speed/FET temp), then back to
//  speed. Long-press the same zone resets the trip - tap and long-press are
//  checked independently in main.cpp's handleTouch() (one on release, one
//  while held), so one zone does both without conflict.
//
//  The AMOLED's panel is about 1.5x the C6 boards' each way, so the layout
//  scales - see S()/TS() below - and it draws through a framebuffer rather
//  than straight to the panel - see TrackedCanvas.
//
//  Uses Arduino_GFX rather than TFT_eSPI: TFT_eSPI does not reliably support
//  ESP32-C6 (open upstream issues - compile failures / boot loops), and
//  Arduino_GFX is what Waveshare's own examples for the C6 boards use (and
//  the one of the two with a CO5300 driver, for the AMOLED). Pin
//  numbers, panel offsets and rotation are from those vendor examples and
//  have NOT been verified on real hardware - if the image comes up rotated,
//  mirrored, or offset, this is the place to fix it (config.h has the
//  rotation/offset values; layout geometry below is independent of that).
// ============================================================================

namespace ui {

// --- palette (matches ui.cpp's) --------------------------------------------
static const uint16_t C_BG     = 0x0000;
static const uint16_t C_PANEL  = 0x18E3;
static const uint16_t C_LINE   = 0x31A6;
static const uint16_t C_DIM    = 0x8410;
static const uint16_t C_TEXT   = 0xFFFF;
static const uint16_t C_GREEN  = 0x07E6;
static const uint16_t C_ORANGE = 0xFD20;
static const uint16_t C_RED    = 0xF800;
static const uint16_t C_BLUE   = 0x04FF;
static const uint16_t C_CYAN   = 0x07FF;

// --- scale -------------------------------------------------------------
// The layout below was drawn for the C6 boards' 170/172 x 320 panels. The S3
// AMOLED's 280 x 456 is about 1.5x that each way, so every size and offset
// goes through S() and every text size through TS() (rounded up, so the
// smallest text stays readable); on the C6 boards both change nothing. The
// portrait zone boundaries are fractions of SCREEN_H instead, since 1.5x of
// 320 would overshoot 456.
#if defined(BOARD_S3_AMOLED164)
static const int16_t UI_SCALE_NUM = 3, UI_SCALE_DEN = 2;
#else
static const int16_t UI_SCALE_NUM = 1, UI_SCALE_DEN = 1;
#endif
static constexpr int16_t S(int16_t v)  { return (int16_t)(v * UI_SCALE_NUM / UI_SCALE_DEN); }
static constexpr uint8_t TS(uint8_t n) { return (uint8_t)((n * UI_SCALE_NUM + UI_SCALE_DEN - 1) / UI_SCALE_DEN); }

// --- layout ------------------------------------------------------------
#if UI_LAYOUT_PORTRAIT
static const int16_t TOP_H       = S(24);  // battery strip height (shortened)
static const int16_t SPEED_Y0    = TOP_H;
static const int16_t SPEED_Y1    = SCREEN_H * 170 / 320;  // speed/telemetry zone: SPEED_Y0..SPEED_Y1
static const int16_t POWERBAR_Y2 = SCREEN_H * 250 / 320;  // power meter zone: SPEED_Y1..POWERBAR_Y2
                                            // profile zone:     POWERBAR_Y2..SCREEN_H
static const int16_t BATT_W      = (SCREEN_W * 3) / 4;  // battery: left 3/4 of the top strip,
                                                          // status (icon + dot) takes the rest
static const int16_t BATT_ICON_W = S(50);  // battery glyph width - as wide as the larger
                                            // (size 2) voltage text next to it allows
static const int16_t BATT_TEXT_GAP = S(4); // icon-to-text gap, trimmed to make room for that text
#else
static const int16_t TOP_H     = S(24);    // battery/status strip height (shortened)
static const int16_t BOTTOM_H  = S(50);    // profile/power-meter row height
static const int16_t SPEED_Y0  = TOP_H;
static const int16_t SPEED_Y1  = SCREEN_H - BOTTOM_H;  // speed/telemetry zone
static const int16_t BATT_W    = (SCREEN_W * 3) / 4;   // battery: left 3/4 of the top row
static const int16_t BATT_ICON_W = S(100); // battery glyph width - the extra strip width from
                                            // the 3/4 split goes here, percent+volt text still fits
static const int16_t BATT_TEXT_GAP = S(6);
#endif
static const int16_t BATT_ICON_H = S(16);  // battery glyph height, shared by both layouts
static const int16_t BL_PWM_FREQ = 5000;
static const int16_t BL_PWM_RES  = 8;

static Arduino_DataBus *s_bus = nullptr;
static Arduino_GFX     *gfx   = nullptr;

#if defined(BOARD_S3_AMOLED164)
// ---------------------------------------------------------------------------
//  The CO5300 only takes windows that start on an even pixel and are an even
//  number of pixels wide and tall (Waveshare's own driver rounds every update
//  out to that), and it can't rotate in hardware - so nothing here draws to
//  it directly. Everything goes into a full-screen framebuffer (280x456x2 =
//  255 KB, in PSRAM) that handles rotation itself, and present() pushes the
//  whole frame, which is always aligned. Only when something was drawn since
//  the last push: every Arduino_GFX drawing call ends in one of the four
//  primitives below, so they are where "something changed" is caught.
// ---------------------------------------------------------------------------
class TrackedCanvas : public Arduino_Canvas {
public:
    using Arduino_Canvas::Arduino_Canvas;
    bool dirty = true;

    void writePixelPreclipped(int16_t x, int16_t y, uint16_t c) override {
        dirty = true;
        Arduino_Canvas::writePixelPreclipped(x, y, c);
    }
    void writeFastVLine(int16_t x, int16_t y, int16_t h, uint16_t c) override {
        dirty = true;
        Arduino_Canvas::writeFastVLine(x, y, h, c);
    }
    void writeFastHLine(int16_t x, int16_t y, int16_t w, uint16_t c) override {
        dirty = true;
        Arduino_Canvas::writeFastHLine(x, y, w, c);
    }
    void writeFillRectPreclipped(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t c) override {
        dirty = true;
        Arduino_Canvas::writeFillRectPreclipped(x, y, w, h, c);
    }
};

static Arduino_CO5300 *s_panel  = nullptr;
static TrackedCanvas  *s_canvas = nullptr;   // null if the framebuffer couldn't be had
#endif

// Pushes whatever was drawn out to the panel. Only the AMOLED draws into a
// framebuffer - the LCD boards draw straight to their panels, so there it
// does nothing.
static void present() {
#if defined(BOARD_S3_AMOLED164)
    if (s_canvas && s_canvas->dirty) {
        s_canvas->flush();
        s_canvas->dirty = false;
    }
#endif
}

// 0 = speed (both layouts), 1..8 = the other 8 fields, tap-to-cycle.
static uint8_t  s_page = 0;
static const uint8_t PAGE_COUNT = 9;
static bool     s_fullRedraw = true;

static char     s_toast[28] = "";
static uint32_t s_toastUntil = 0;

// --- cached values so we only repaint what actually changed -----------------
static int8_t  s_cacheBatt      = -1;
static char    s_cacheBattText[20] = "";
static int8_t  s_cacheProfile   = -1;
static int16_t s_cacheHoldPx    = -1;
static uint8_t s_cacheFault     = 255;
static char    s_cachePrimary[40] = "";
static char    s_cachePowerTxt[14] = "";
static int16_t s_cachePowerBarPx   = -9999;
static int8_t  s_cacheBle  = -1;
static int8_t  s_cacheLink = -1;

// ---------------------------------------------------------------------------
void setPage(uint8_t p) { s_page = p % PAGE_COUNT; s_fullRedraw = true; }
uint8_t page() { return s_page; }
void nextPage() { setPage((uint8_t)(s_page + 1)); }
void forceFullRedraw() { s_fullRedraw = true; }

void toast(const char *msg, uint32_t durationMs) {
    strncpy(s_toast, msg, sizeof(s_toast) - 1);
    s_toast[sizeof(s_toast) - 1] = '\0';
    s_toastUntil = millis() + durationMs;
}

// ---------------------------------------------------------------------------
//  The speed/telemetry zone is the same in both layouts: full width, between
//  the battery strip and the profile/power row. Tap cycles telemetry, hold
//  resets the trip - independent checks in main.cpp, so one zone does both.
// ---------------------------------------------------------------------------
bool inStatColumn(int16_t x, int16_t y) {
    (void)x;
    return y >= SPEED_Y0 && y < SPEED_Y1;
}
bool inSpeedArea(int16_t x, int16_t y) {
    (void)x;
    return y >= SPEED_Y0 && y < SPEED_Y1;
}

#if UI_LAYOUT_PORTRAIT
bool inProfileBadge(int16_t x, int16_t y) {
    (void)x;
    return y >= POWERBAR_Y2;
}
#else
bool inProfileBadge(int16_t x, int16_t y) {
    return x < SCREEN_W / 2 && y >= SCREEN_H - BOTTOM_H;
}
#endif

// ---------------------------------------------------------------------------
void setBrightness(uint8_t pct) {
    pct = constrain(pct, (uint8_t)5, (uint8_t)100);
#if defined(BOARD_S3_AMOLED164)
    // No backlight on an AMOLED: the CO5300 takes a 0-255 brightness value
    // as a display command (0x51) instead.
    s_panel->setBrightness((uint8_t)map(pct, 0, 100, 0, 255));
#else
    // arduino-esp32 3.x LEDC API: attach/write take the GPIO directly, not a
    // channel number (this changed from the 2.x API the CYD target's
    // ui.cpp still uses). Assumes the backlight is active-high (higher duty
    // = brighter) - confirmed for the 1.47 board (its own example does
    // digitalWrite(GFX_BL, HIGH) for "on"); unverified for the 1.9 board -
    // if it comes up inverted there, swap `pct` for `100 - pct` here.
    ledcWrite(LCD_BL_PIN, map(pct, 0, 100, 0, 255));
#endif
}

#if defined(BOARD_C6_LCD147)
// ---------------------------------------------------------------------------
//  JD9853 needs this extra vendor register-tuning sequence beyond what
//  Arduino_ST7789's generic init does (gamma curves, power settings, etc.) -
//  copied verbatim from Waveshare's own example (see the BOARD_C6_LCD147
//  comment in config.h for exactly where). Not hand-derived like the rest
//  of this port - this is vendor-supplied, opaque tuning data, left as-is.
// ---------------------------------------------------------------------------
static void jd9853RegInit() {
    static const uint8_t ops[] = {
        BEGIN_WRITE,
        WRITE_COMMAND_8, 0x11,
        END_WRITE,
        DELAY, 120,

        BEGIN_WRITE,
        WRITE_C8_D16, 0xDF, 0x98, 0x53,
        WRITE_C8_D8, 0xB2, 0x23,

        WRITE_COMMAND_8, 0xB7,
        WRITE_BYTES, 4,
        0x00, 0x47, 0x00, 0x6F,

        WRITE_COMMAND_8, 0xBB,
        WRITE_BYTES, 6,
        0x1C, 0x1A, 0x55, 0x73, 0x63, 0xF0,

        WRITE_C8_D16, 0xC0, 0x44, 0xA4,
        WRITE_C8_D8, 0xC1, 0x16,

        WRITE_COMMAND_8, 0xC3,
        WRITE_BYTES, 8,
        0x7D, 0x07, 0x14, 0x06, 0xCF, 0x71, 0x72, 0x77,

        WRITE_COMMAND_8, 0xC4,
        WRITE_BYTES, 12,
        0x00, 0x00, 0xA0, 0x79, 0x0B, 0x0A, 0x16, 0x79, 0x0B, 0x0A, 0x16, 0x82,

        WRITE_COMMAND_8, 0xC8,
        WRITE_BYTES, 32,
        0x3F, 0x32, 0x29, 0x29, 0x27, 0x2B, 0x27, 0x28, 0x28, 0x26, 0x25, 0x17, 0x12, 0x0D, 0x04, 0x00,
        0x3F, 0x32, 0x29, 0x29, 0x27, 0x2B, 0x27, 0x28, 0x28, 0x26, 0x25, 0x17, 0x12, 0x0D, 0x04, 0x00,

        WRITE_COMMAND_8, 0xD0,
        WRITE_BYTES, 5,
        0x04, 0x06, 0x6B, 0x0F, 0x00,

        WRITE_C8_D16, 0xD7, 0x00, 0x30,
        WRITE_C8_D8, 0xE6, 0x14,
        WRITE_C8_D8, 0xDE, 0x01,

        WRITE_COMMAND_8, 0xB7,
        WRITE_BYTES, 5,
        0x03, 0x13, 0xEF, 0x35, 0x35,

        WRITE_COMMAND_8, 0xC1,
        WRITE_BYTES, 3,
        0x14, 0x15, 0xC0,

        WRITE_C8_D16, 0xC2, 0x06, 0x3A,
        WRITE_C8_D16, 0xC4, 0x72, 0x12,
        WRITE_C8_D8, 0xBE, 0x00,
        WRITE_C8_D8, 0xDE, 0x02,

        WRITE_COMMAND_8, 0xE5,
        WRITE_BYTES, 3,
        0x00, 0x02, 0x00,

        WRITE_COMMAND_8, 0xE5,
        WRITE_BYTES, 3,
        0x01, 0x02, 0x00,

        WRITE_C8_D8, 0xDE, 0x00,
        WRITE_C8_D8, 0x35, 0x00,
        WRITE_C8_D8, 0x3A, 0x05,

        WRITE_COMMAND_8, 0x2A,
        WRITE_BYTES, 4,
        0x00, 0x22, 0x00, 0xCD,

        WRITE_COMMAND_8, 0x2B,
        WRITE_BYTES, 4,
        0x00, 0x00, 0x01, 0x3F,

        WRITE_C8_D8, 0xDE, 0x02,

        WRITE_COMMAND_8, 0xE5,
        WRITE_BYTES, 3,
        0x00, 0x02, 0x00,

        WRITE_C8_D8, 0xDE, 0x00,
        WRITE_C8_D8, 0x36, 0x00,
        WRITE_COMMAND_8, 0x21,
        END_WRITE,

        DELAY, 10,

        BEGIN_WRITE,
        WRITE_COMMAND_8, 0x29,
        END_WRITE
    };
    s_bus->batchOperation(ops, sizeof(ops));
}
#endif

// ---------------------------------------------------------------------------
void begin() {
#if defined(BOARD_S3_AMOLED164)
    s_bus = new Arduino_ESP32QSPI(LCD_CS_PIN, LCD_SCK_PIN,
                                  LCD_D0_PIN, LCD_D1_PIN, LCD_D2_PIN, LCD_D3_PIN);
    // Both take the panel's NATIVE (portrait) size. The panel itself stays at
    // rotation 0 - it has no real rotation to give - and the canvas applies
    // SCREEN_ROTATION instead.
    s_panel  = new Arduino_CO5300(s_bus, LCD_RST_PIN, 0, TOUCH_PANEL_W, TOUCH_PANEL_H,
                                  LCD_COL_OFFSET, LCD_ROW_OFFSET,
                                  LCD_COL_OFFSET, LCD_ROW_OFFSET);
    s_canvas = new TrackedCanvas(TOUCH_PANEL_W, TOUCH_PANEL_H, s_panel, 0, 0, SCREEN_ROTATION);
    gfx = s_canvas;
    if (!s_canvas->begin()) {
        // No framebuffer (PSRAM missing or not enabled for this build). Draw
        // straight to the panel rather than crash on the first draw - updates
        // that break its alignment rule may smear, but the serial log and a
        // mostly readable screen beat a boot loop.
        Serial.println("[ui] framebuffer allocation failed - is PSRAM enabled? "
                       "drawing straight to the panel");
        delete s_canvas;
        s_canvas = nullptr;
        gfx = s_panel;
    }
#else
    s_bus = new Arduino_HWSPI(LCD_DC_PIN, LCD_CS_PIN, LCD_SCK_PIN, LCD_MOSI_PIN);
    // Constructor takes the panel's NATIVE (portrait) dimensions - the
    // library swaps effective width/height itself based on rotation.
    gfx = new Arduino_ST7789(s_bus, LCD_RST_PIN, SCREEN_ROTATION, true /* IPS */,
                              TOUCH_PANEL_W, TOUCH_PANEL_H,
                              LCD_COL_OFFSET, LCD_ROW_OFFSET,
                              LCD_COL_OFFSET, LCD_ROW_OFFSET);
    if (!gfx->begin()) {
        Serial.println("[ui] gfx->begin() failed");
    }
#if defined(BOARD_C6_LCD147)
    jd9853RegInit();
    gfx->setRotation(SCREEN_ROTATION);
#endif
    ledcAttach(LCD_BL_PIN, BL_PWM_FREQ, BL_PWM_RES);
#endif
    gfx->fillScreen(C_BG);
    setBrightness(BACKLIGHT_PCT);
    present();

    s_fullRedraw = true;
}

void splash(const char *line1, const char *line2) {
    gfx->fillScreen(C_BG);
    int16_t bx, by; uint16_t bw, bh;

    gfx->setTextColor(C_TEXT);
    gfx->setTextSize(TS(2));
    gfx->getTextBounds(line1, 0, 0, &bx, &by, &bw, &bh);
    gfx->setCursor((SCREEN_W - bw) / 2, S(60));
    gfx->println(line1);

    gfx->setTextColor(C_DIM);
    gfx->setTextSize(TS(1));
    gfx->getTextBounds(line2, 0, 0, &bx, &by, &bw, &bh);
    gfx->setCursor((SCREEN_W - bw) / 2, S(90));
    gfx->println(line2);

    present();
    s_fullRedraw = true;
}

// ---------------------------------------------------------------------------
static void drawCentered(const char *text, int16_t cx, int16_t cy, uint8_t textSize, uint16_t color) {
    int16_t bx, by; uint16_t bw, bh;
    gfx->setTextSize(textSize);
    gfx->getTextBounds(text, 0, 0, &bx, &by, &bw, &bh);
    gfx->setTextColor(color, C_BG);
    gfx->setCursor(cx - bw / 2, cy - bh / 2);
    gfx->print(text);
}

// For text that changes in place without its zone being cleared first (the
// wattage): drawCentered()'s background only covers the new string's own
// cells, so a longer old value - or one centred a few pixels over - would
// leave its edges behind. This also blanks the rest of the line between x0
// and x1, around the text rather than under it, so the value doesn't flicker.
static void drawCenteredPadded(const char *text, int16_t x0, int16_t x1, int16_t cy,
                               uint8_t textSize, uint16_t color) {
    int16_t bx, by; uint16_t bw, bh;
    gfx->setTextSize(textSize);
    gfx->getTextBounds(text, 0, 0, &bx, &by, &bw, &bh);
    const int16_t tx = (x0 + x1) / 2 - bw / 2;
    const int16_t ty = cy - bh / 2;
    if (tx > x0)      gfx->fillRect(x0, ty, tx - x0, bh, C_BG);
    if (x1 > tx + bw) gfx->fillRect(tx + bw, ty, x1 - (tx + bw), bh, C_BG);
    gfx->setTextColor(color, C_BG);
    gfx->setCursor(tx, ty);
    gfx->print(text);
}

// ---------------------------------------------------------------------------
//  Same battery glyph as the CYD's ui.cpp: rounded-rect outline + terminal
//  nub, a fill bar coloured by charge threshold. Portrait's strip is too
//  narrow for both a percentage and much else, so it shows pack voltage
//  beside the icon instead (the fill bar still carries the percentage
//  visually); landscape has room to spare and shows both - percentage then
//  voltage (dimmed, secondary) to its right.
// ---------------------------------------------------------------------------
// `m.battKnown` false means the VESC hasn't told us its battery config yet -
// there is no hardcoded guess to fill the bar/text with, so this shows "?"
// instead of a number that might be built on a wrong assumption.
static void drawBattery(const DashModel &m) {
    const bool   known = m.battKnown;
    const int8_t p     = known ? (int8_t)roundf(constrain(m.batteryPct, 0.0f, 100.0f)) : -1;

    char buf[8];
    char vBuf[8] = "";
    if (!known) {
        snprintf(buf, sizeof(buf), "?");
    }
#if UI_LAYOUT_PORTRAIT
    else {
        snprintf(buf, sizeof(buf), "%.1fV", m.voltage);
    }
#else
    else {
        snprintf(buf, sizeof(buf), "%d%%", p);
        snprintf(vBuf, sizeof(vBuf), "%.1fV", m.voltage);
    }
#endif

    char combined[20];
    snprintf(combined, sizeof(combined), "%s %s", buf, vBuf);
    if (p == s_cacheBatt && strcmp(combined, s_cacheBattText) == 0 && !s_fullRedraw) return;
    s_cacheBatt = p;
    strcpy(s_cacheBattText, combined);

    gfx->fillRect(0, 0, BATT_W, TOP_H, C_BG);

    const uint16_t col   = (p <= 15) ? C_RED : (p <= 30) ? C_ORANGE : C_GREEN;
    const int16_t iconX  = S(4);
    const int16_t iconY  = (TOP_H - BATT_ICON_H) / 2;
    const int16_t fillX  = iconX + S(2);
    const int16_t fillY  = iconY + S(2);
    const int16_t fillW  = BATT_ICON_W - 2 * S(2);
    const int16_t fillH  = BATT_ICON_H - 2 * S(2);
    const int16_t w      = known ? (fillW * p) / 100 : 0;

    gfx->drawRoundRect(iconX, iconY, BATT_ICON_W, BATT_ICON_H, S(3), C_DIM);
    gfx->fillRect(iconX + BATT_ICON_W, iconY + BATT_ICON_H / 2 - S(4), S(3), S(8), C_DIM);
    gfx->fillRect(fillX, fillY, fillW, fillH, C_PANEL);
    if (known && w > 0) gfx->fillRect(fillX, fillY, w, fillH, col);

    int16_t bx, by; uint16_t bw, bh;
    gfx->setTextSize(TS(2));
    gfx->getTextBounds(buf, 0, 0, &bx, &by, &bw, &bh);
    gfx->setTextColor(C_TEXT, C_BG);
    const int16_t textX0 = iconX + BATT_ICON_W + S(3) + BATT_TEXT_GAP;
    gfx->setCursor(textX0, TOP_H / 2 - bh / 2);
    gfx->print(buf);

#if !UI_LAYOUT_PORTRAIT
    int16_t vbx, vby; uint16_t vbw, vbh;
    gfx->getTextBounds(vBuf, 0, 0, &vbx, &vby, &vbw, &vbh);
    gfx->setTextColor(C_DIM, C_BG);
    gfx->setCursor(textX0 + bw + S(10), TOP_H / 2 - vbh / 2);
    gfx->print(vBuf);
#endif
}

// ---------------------------------------------------------------------------
//  Shared - a stylised bluetooth rune (same shape as the CYD's ui.cpp).
// ---------------------------------------------------------------------------
static void drawBleIcon(int16_t x, int16_t y, bool connected) {
    gfx->fillRect(x - S(2), y - S(2), S(18), S(20), C_BG);
    const uint16_t c = connected ? C_BLUE : C_LINE;
    gfx->drawLine(x + S(6),  y,          x + S(6),  y + S(16), c);
    gfx->drawLine(x + S(6),  y,          x + S(11), y + S(5),  c);
    gfx->drawLine(x + S(11), y + S(5),   x + S(1),  y + S(11), c);
    gfx->drawLine(x + S(6),  y + S(16),  x + S(11), y + S(11), c);
    gfx->drawLine(x + S(11), y + S(11),  x + S(1),  y + S(5),  c);
}

#if UI_LAYOUT_PORTRAIT
// ---------------------------------------------------------------------------
//  Portrait's top strip is narrow (battery takes the left 3/4), so this is
//  just the icon + a coloured VESC-link dot - no room for "OK"/"NO" text.
// ---------------------------------------------------------------------------
static void drawStatus(const DashModel &m) {
    const int8_t ble  = m.bleConnected ? 1 : 0;
    const int8_t link = m.linkOk ? 1 : 0;
    if (!s_fullRedraw && ble == s_cacheBle && link == s_cacheLink) return;
    s_cacheBle  = ble;
    s_cacheLink = link;

    const int16_t x0 = BATT_W;
    gfx->fillRect(x0, 0, SCREEN_W - x0, TOP_H, C_BG);

    drawBleIcon(x0 + S(4), TOP_H / 2 - S(8), m.bleConnected);
    gfx->fillCircle(x0 + S(36), TOP_H / 2, S(5), m.linkOk ? C_GREEN : C_RED);
}
#else
// ---------------------------------------------------------------------------
//  Landscape only - the battery strip's remaining quarter still leaves room
//  for the bluetooth rune plus a coloured dot + OK/NO for the VESC link,
//  same meaning as the CYD's green/red dot.
// ---------------------------------------------------------------------------
static void drawStatus(const DashModel &m) {
    const int8_t ble  = m.bleConnected ? 1 : 0;
    const int8_t link = m.linkOk ? 1 : 0;
    if (!s_fullRedraw && ble == s_cacheBle && link == s_cacheLink) return;
    s_cacheBle  = ble;
    s_cacheLink = link;

    const int16_t x0 = BATT_W;
    gfx->fillRect(x0, 0, SCREEN_W - x0, TOP_H, C_BG);

    drawBleIcon(x0 + S(6), TOP_H / 2 - S(8), m.bleConnected);

    const int16_t cx = x0 + S(36);
    gfx->fillCircle(cx, TOP_H / 2, S(5), link ? C_GREEN : C_RED);
    drawCentered(link ? "OK" : "NO", cx + S(18), TOP_H / 2, TS(1), link ? C_DIM : C_RED);
}
#endif

// ---------------------------------------------------------------------------
//  The tap-to-cycle field set, shared by both layouts. Page 0 is speed -
//  drawPrimary() below gives it its own big-number treatment instead of
//  this generic label/value one. Pages 1-8 are the same 8 fields the CYD
//  shows across its two stat pages (README: page 1 = trip/Wh used/Wh per
//  km/motor temp, page 2 = range/max speed/avg speed/FET temp).
// ---------------------------------------------------------------------------
static void formatField(const DashModel &m, uint8_t page, char *label, char *value) {
    switch (page) {
    case 1: strcpy(label, "TRIP");
        snprintf(value, 20, "%.1f %s", m.tripDist, DashStats::distUnit()); break;
    case 2: strcpy(label, "WH USED");
        snprintf(value, 20, "%.0f", m.whUsed); break;
    case 3: strcpy(label, "WH/KM");
        snprintf(value, 20, "%.1f", m.whPerDist); break;
    case 4: strcpy(label, "MOTOR C");
        snprintf(value, 20, "%.0f", m.tempMotor); break;
    case 5: strcpy(label, "RANGE");
        snprintf(value, 20, "%.0f %s", m.rangeLeft, DashStats::distUnit()); break;
    case 6: strcpy(label, "MAX SPD");
        snprintf(value, 20, "%.0f", m.maxSpeed); break;
    case 7: strcpy(label, "AVG SPD");
        snprintf(value, 20, "%.0f", m.avgSpeed); break;
    case 8: strcpy(label, "FET C");
        snprintf(value, 20, "%.0f", m.tempFet); break;
    default: strcpy(label, ""); value[0] = '\0'; break;
    }
}

// ---------------------------------------------------------------------------
//  Speed by default (page 0); tapping this zone cycles through the other 8
//  fields via formatField(), then back to speed. Shared by both layouts -
//  only SPEED_Y0/SPEED_Y1 differ between them.
// ---------------------------------------------------------------------------
static void drawPrimary(const DashModel &m) {
    if (s_page == 0) {
        // "--" until the VESC has reported its geometry - see ui.cpp's
        // drawSpeed() for why not "0.0".
        char buf[10];
        if (m.geomKnown) snprintf(buf, sizeof(buf), "%.1f", m.speed);
        else             snprintf(buf, sizeof(buf), "--");
        if (strcmp(buf, s_cachePrimary) == 0 && !s_fullRedraw) return;
        strcpy(s_cachePrimary, buf);

        gfx->fillRect(0, SPEED_Y0, SCREEN_W, SPEED_Y1 - SPEED_Y0, C_BG);
        drawCentered(buf, SCREEN_W / 2, SPEED_Y0 + (SPEED_Y1 - SPEED_Y0) / 2 - S(10), TS(5), m.linkOk ? C_TEXT : C_DIM);
        drawCentered(m.linkOk ? DashStats::speedUnit() : "NO LINK", SCREEN_W / 2, SPEED_Y1 - S(16), TS(2), C_DIM);
        return;
    }

    char label[12], value[20];
    formatField(m, s_page, label, value);
    char combined[40];
    snprintf(combined, sizeof(combined), "%s|%s", label, value);
    if (strcmp(combined, s_cachePrimary) == 0 && !s_fullRedraw) return;
    strcpy(s_cachePrimary, combined);

    gfx->fillRect(0, SPEED_Y0, SCREEN_W, SPEED_Y1 - SPEED_Y0, C_BG);
    drawCentered(label, SCREEN_W / 2, SPEED_Y0 + S(30), TS(2), C_DIM);
    drawCentered(value, SCREEN_W / 2, SPEED_Y0 + (SPEED_Y1 - SPEED_Y0) / 2 + S(10), TS(4), C_TEXT);
}

// ---------------------------------------------------------------------------
static void drawFault(const DashModel &m) {
    if (m.faultCode == s_cacheFault && !s_fullRedraw) return;
    gfx->fillRect(0, TOP_H, SCREEN_W, SCREEN_H - TOP_H, C_RED);
    drawCentered(vesc::faultToString(m.faultCode), SCREEN_W / 2, TOP_H + (SCREEN_H - TOP_H) / 2, TS(2), C_TEXT);
}

#if UI_LAYOUT_PORTRAIT
// ---------------------------------------------------------------------------
static void drawProfile(const DashModel &m, float holdProgress) {
    const RiderProfile &p = profiles::current();
    const int16_t y0 = POWERBAR_Y2, h = SCREEN_H - POWERBAR_Y2;

    if (s_fullRedraw || (int8_t)m.profileIndex != s_cacheProfile) {
        s_cacheProfile = (int8_t)m.profileIndex;
        gfx->fillRect(0, y0, SCREEN_W, h, p.color);
        drawCentered(p.name, SCREEN_W / 2, y0 + h / 2, TS(3), C_BG);
        s_cacheHoldPx = -1;
    }

    // hold-progress sliver along the bottom of the band
    const int16_t full = SCREEN_W - S(16);
    const int16_t px = (int16_t)(constrain(holdProgress, 0.0f, 1.0f) * full);
    if (px != s_cacheHoldPx) {
        s_cacheHoldPx = px;
        gfx->fillRect(S(8), y0 + h - S(6), full, S(3), p.color);
        if (px > 0) gfx->fillRect(S(8), y0 + h - S(6), px, S(3), C_BG);
    }
}

// ---------------------------------------------------------------------------
//  power meter - zero at centre, regen to the left, drive to the right -
//  same logic as the CYD's ui.cpp drawPower(), adapted to this screen.
// ---------------------------------------------------------------------------
static void drawPowerBar(const DashModel &m) {
    const int16_t x0 = S(8), x1 = SCREEN_W - S(8), y = SPEED_Y1 + S(10), h = S(24);
    const int16_t cx = (int16_t)((x0 + x1) / 2);

    if (s_fullRedraw) {
        gfx->fillRect(0, SPEED_Y1, SCREEN_W, POWERBAR_Y2 - SPEED_Y1, C_BG);
        gfx->drawRect(x0 - 1, y - 1, (x1 - x0) + 2, h + 2, C_LINE);
        gfx->fillRect(x0, y, x1 - x0, h, C_PANEL);
        gfx->drawFastVLine(cx, y - S(3), h + S(6), C_DIM);
        drawCentered("REGEN", x0 + S(26), y + h + S(10), TS(1), C_DIM);
        drawCentered("DRIVE", x1 - S(26), y + h + S(10), TS(1), C_DIM);
        s_cachePowerBarPx = -9999;
    }

    int16_t px;
    if (m.powerW >= 0) {
        px = (int16_t)((m.powerW / POWER_BAR_MAX_W) * (float)(x1 - cx));
        px = constrain(px, (int16_t)0, (int16_t)(x1 - cx));
    } else {
        px = (int16_t)((m.powerW / POWER_BAR_REGEN_W) * (float)(cx - x0));
        px = constrain(px, (int16_t)-(cx - x0), (int16_t)0);
    }

    if (px != s_cachePowerBarPx) {
        s_cachePowerBarPx = px;
        gfx->fillRect(x0, y, x1 - x0, h, C_PANEL);
        if (px > 0) {
            const uint16_t c = (m.powerW > POWER_BAR_MAX_W * 0.8f) ? C_ORANGE : C_CYAN;
            gfx->fillRect(cx, y, px, h, c);
        } else if (px < 0) {
            gfx->fillRect(cx + px, y, -px, h, C_GREEN);
        }
        gfx->drawFastVLine(cx, y, h, C_DIM);
    }

    char p[14];
    snprintf(p, sizeof(p), "%d W", (int)roundf(m.powerW));
    if (s_fullRedraw || strcmp(p, s_cachePowerTxt) != 0) {
        strcpy(s_cachePowerTxt, p);
        drawCenteredPadded(p, x0, x1, y + h + S(26), TS(2), m.powerW < -5 ? C_GREEN : C_TEXT);
    }
}

#else // !UI_LAYOUT_PORTRAIT

// ---------------------------------------------------------------------------
//  Bottom-left quadrant.
// ---------------------------------------------------------------------------
static void drawProfile(const DashModel &m, float holdProgress) {
    const RiderProfile &p = profiles::current();
    const int16_t x1 = SCREEN_W / 2, y0 = SCREEN_H - BOTTOM_H;

    if (s_fullRedraw || (int8_t)m.profileIndex != s_cacheProfile) {
        s_cacheProfile = (int8_t)m.profileIndex;
        gfx->fillRect(0, y0, x1, BOTTOM_H, p.color);
        drawCentered(p.name, x1 / 2, y0 + BOTTOM_H / 2, TS(2), C_BG);
        s_cacheHoldPx = -1;
    }

    // hold-progress sliver along the bottom of the band
    const int16_t full = x1 - S(8);
    const int16_t px = (int16_t)(constrain(holdProgress, 0.0f, 1.0f) * full);
    if (px != s_cacheHoldPx) {
        s_cacheHoldPx = px;
        gfx->fillRect(S(4), y0 + BOTTOM_H - S(5), full, S(3), p.color);
        if (px > 0) gfx->fillRect(S(4), y0 + BOTTOM_H - S(5), px, S(3), C_BG);
    }
}

// ---------------------------------------------------------------------------
//  Bottom-right quadrant. Same regen/drive-split logic as the CYD's ui.cpp
//  drawPower(), scaled down to fit - no REGEN/DRIVE labels here, there is
//  not enough height left for them once the bar and wattage both fit.
// ---------------------------------------------------------------------------
static void drawPowerBar(const DashModel &m) {
    const int16_t qx0 = SCREEN_W / 2, qy0 = SCREEN_H - BOTTOM_H;
    const int16_t x0 = qx0 + S(8), x1 = SCREEN_W - S(8), y = qy0 + S(8), h = S(16);
    const int16_t cx = (int16_t)((x0 + x1) / 2);

    if (s_fullRedraw) {
        gfx->fillRect(qx0, qy0, SCREEN_W - qx0, BOTTOM_H, C_BG);
        gfx->drawRect(x0 - 1, y - 1, (x1 - x0) + 2, h + 2, C_LINE);
        gfx->fillRect(x0, y, x1 - x0, h, C_PANEL);
        gfx->drawFastVLine(cx, y - S(3), h + S(6), C_DIM);
        s_cachePowerBarPx = -9999;
    }

    int16_t px;
    if (m.powerW >= 0) {
        px = (int16_t)((m.powerW / POWER_BAR_MAX_W) * (float)(x1 - cx));
        px = constrain(px, (int16_t)0, (int16_t)(x1 - cx));
    } else {
        px = (int16_t)((m.powerW / POWER_BAR_REGEN_W) * (float)(cx - x0));
        px = constrain(px, (int16_t)-(cx - x0), (int16_t)0);
    }

    if (px != s_cachePowerBarPx) {
        s_cachePowerBarPx = px;
        gfx->fillRect(x0, y, x1 - x0, h, C_PANEL);
        if (px > 0) {
            const uint16_t c = (m.powerW > POWER_BAR_MAX_W * 0.8f) ? C_ORANGE : C_CYAN;
            gfx->fillRect(cx, y, px, h, c);
        } else if (px < 0) {
            gfx->fillRect(cx + px, y, -px, h, C_GREEN);
        }
        gfx->drawFastVLine(cx, y, h, C_DIM);
    }

    char p[14];
    snprintf(p, sizeof(p), "%d W", (int)roundf(m.powerW));
    if (s_fullRedraw || strcmp(p, s_cachePowerTxt) != 0) {
        strcpy(s_cachePowerTxt, p);
        drawCenteredPadded(p, x0, x1, y + h + S(14), TS(2), m.powerW < -5 ? C_GREEN : C_TEXT);
    }
}
#endif // UI_LAYOUT_PORTRAIT

// ---------------------------------------------------------------------------
void render(const DashModel &m, float holdProgress) {
    const bool showToast = (millis() < s_toastUntil);

    // The link state picks the speed's colour and its "NO LINK" label, but
    // drawPrimary()'s cache only compares the speed text - a speed already at
    // 0.0 when the link drops would otherwise never change. Links come and go
    // rarely, so just repaint everything.
    if (s_cacheLink >= 0 && s_cacheLink != (m.linkOk ? 1 : 0)) s_fullRedraw = true;

    // A fault clearing must force every zone below the battery strip to
    // redraw over the red banner it just painted - plain cache comparisons
    // would otherwise skip a redraw whose text happens to match what was
    // cached before the fault started.
    if (m.faultCode == 0 && s_cacheFault != 0) {
        s_cachePrimary[0] = '\0';
        s_cachePowerTxt[0] = '\0';
        s_cacheProfile = -1;
    }

    drawBattery(m);
    drawStatus(m);

    if (m.faultCode != 0 && !showToast) {
        drawFault(m);
    } else if (showToast) {
        if (strcmp(s_cachePrimary, s_toast) != 0 || s_fullRedraw) {
            strcpy(s_cachePrimary, s_toast);
            gfx->fillRect(0, SPEED_Y0, SCREEN_W, SPEED_Y1 - SPEED_Y0, C_BG);
            drawCentered(s_toast, SCREEN_W / 2, SPEED_Y0 + (SPEED_Y1 - SPEED_Y0) / 2, TS(2), C_TEXT);
        }
        drawProfile(m, holdProgress);
        drawPowerBar(m);
    } else {
        drawPrimary(m);
        drawProfile(m, holdProgress);
        drawPowerBar(m);
    }

    s_cacheFault = m.faultCode;
    s_fullRedraw = false;
    present();
}

// No onboard LDR on these boards - see AUTO_BRIGHTNESS in config.h.
void updateAutoBrightness() {}

} // namespace ui
