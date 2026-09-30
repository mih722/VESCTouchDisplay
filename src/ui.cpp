#include "ui.h"
#include <TFT_eSPI.h>
#include <math.h>
#include <string.h>
#include "vesc_protocol.h"

namespace ui {

static TFT_eSPI  tft;
static TFT_eSprite speedSprite(&tft);   // big number only: everything else is
                                        // small enough to redraw in place

// --- layout ------------------------------------------------------------
static const int16_t HEADER_H     = 28;
static const int16_t FOOTER_Y     = 202;
static const int16_t COL_X        = 200;    // right stat column starts here

static const int16_t BADGE_X      = 6;
static const int16_t BADGE_Y      = 206;
static const int16_t BADGE_W      = 100;
static const int16_t BADGE_H      = 30;

// --- palette ---------------------------------------------------------------
static const uint16_t C_BG     = 0x0000;
static const uint16_t C_PANEL  = 0x18E3;
static const uint16_t C_LINE   = 0x31A6;
static const uint16_t C_DIM    = 0x8410;
static const uint16_t C_TEXT   = 0xFFFF;
static const uint16_t C_GREEN  = 0x07E6;
static const uint16_t C_YELLOW = 0xFFE0;
static const uint16_t C_ORANGE = 0xFD20;
static const uint16_t C_RED    = 0xF800;
static const uint16_t C_BLUE   = 0x04FF;
static const uint16_t C_CYAN   = 0x07FF;

// --- backlight -------------------------------------------------------------
static const uint8_t BL_CHANNEL = 0;

// --- cached values so we only repaint what actually changed ----------------
struct Cache {
    int16_t speedInt   = -1;
    int8_t  speedDec   = -1;
    int8_t  battPct    = -1;
    char    voltage[10]  = "";
    char    powerTxt[12] = "";
    int16_t powerBarPx   = -9999;
    char    tile[4][14]  = { "", "", "", "" };
    char    tileLabel[4][10] = { "", "", "", "" };
    char    odo[16]      = "";
    int8_t  profile      = -1;
    int8_t  bleState     = -1;
    int8_t  linkState    = -1;
    uint8_t fault        = 255;
    int16_t holdPx       = -1;
};
static Cache  cache;
static uint8_t s_page = 0;
static bool    s_fullRedraw = true;

static char     s_toast[28] = "";
static uint32_t s_toastUntil = 0;

// ---------------------------------------------------------------------------
void setBrightness(uint8_t pct) {
    pct = constrain(pct, (uint8_t)5, (uint8_t)100);
    ledcWrite(BL_CHANNEL, map(pct, 0, 100, 0, 255));
}

// ---------------------------------------------------------------------------
//  Auto brightness. See the AUTO_BRIGHTNESS comment in config.h: a lower raw
//  reading means brighter light, and the sensor's proximity to the backlight
//  makes it partly see its own output, not just ambient light - the heavy
//  exponential smoothing here (rather than reacting to each sample) is what
//  keeps that self-feedback from visibly hunting.
// ---------------------------------------------------------------------------
static float   s_ldrFilt        = -1.0f;   // -1 = not yet seeded
static int8_t  s_ldrLastPct     = -1;
static uint32_t s_ldrLastMs     = 0;

void updateAutoBrightness() {
    if (!AUTO_BRIGHTNESS) return;

    const uint32_t now = millis();
    if (now - s_ldrLastMs < AUTO_BRIGHTNESS_UPDATE_MS) return;
    s_ldrLastMs = now;

    const int raw = analogRead(LDR_PIN);
    if (s_ldrFilt < 0.0f) s_ldrFilt = (float)raw;      // seed on the first sample
    s_ldrFilt += ((float)raw - s_ldrFilt) * 0.2f;

    // Lower raw reading = brighter ambient light (config.h) - map inverted,
    // and clamp: the reading can sit outside [BRIGHT, DARK] on either side.
    float t = (s_ldrFilt - LDR_ADC_BRIGHT) / (float)(LDR_ADC_DARK - LDR_ADC_BRIGHT);
    t = constrain(t, 0.0f, 1.0f);                       // 0 = brightest, 1 = darkest

    const int8_t pct = (int8_t)(BACKLIGHT_MAX_PCT - t * (BACKLIGHT_MAX_PCT - BACKLIGHT_MIN_PCT));
    if (pct != s_ldrLastPct) {
        s_ldrLastPct = pct;
        setBrightness((uint8_t)pct);
    }
}

void setPage(uint8_t p) { s_page = p % 2; s_fullRedraw = true; }
uint8_t page() { return s_page; }
void nextPage() { setPage((uint8_t)(s_page + 1)); }
void forceFullRedraw() { s_fullRedraw = true; }

void toast(const char *msg, uint32_t durationMs) {
    strncpy(s_toast, msg, sizeof(s_toast) - 1);
    s_toast[sizeof(s_toast) - 1] = '\0';
    s_toastUntil = millis() + durationMs;
}

bool inProfileBadge(int16_t x, int16_t y) {
    // Generous target: it is pressed with a thumb, in gloves, on a bumpy road.
    return x >= (BADGE_X - 6) && x <= (BADGE_X + BADGE_W + 10) &&
           y >= (BADGE_Y - 8) && y <= (BADGE_Y + BADGE_H + 8);
}
bool inStatColumn(int16_t x, int16_t y) {
    return x >= COL_X && y > HEADER_H && y < FOOTER_Y;
}
bool inSpeedArea(int16_t x, int16_t y) {
    return x < COL_X && y > HEADER_H && y < FOOTER_Y;
}

// ---------------------------------------------------------------------------
void begin() {
    tft.init();
    tft.setRotation(SCREEN_ROTATION);
    tft.fillScreen(C_BG);

    // After tft.init(): TFT_eSPI drives TFT_BL with digitalWrite, which would
    // detach the LEDC channel if we set PWM up first.
    ledcSetup(BL_CHANNEL, 5000, 8);
    ledcAttachPin(TFT_BL, BL_CHANNEL);
    setBrightness(BACKLIGHT_PCT);

    tft.setTextWrap(false);

    // 150x78 at 16bpp = ~23 kB. Worth it: repainting a 75-pixel font in place
    // flickers badly at 20 fps.
    speedSprite.setColorDepth(16);
    if (!speedSprite.createSprite(150, 78)) {
        Serial.println("[ui] speed sprite alloc failed, falling back to direct draw");
    }
    s_fullRedraw = true;
}

void splash(const char *line1, const char *line2) {
    tft.fillScreen(C_BG);
    tft.setTextDatum(MC_DATUM);
    tft.setTextColor(C_CYAN, C_BG);
    tft.drawString(line1, SCREEN_W / 2, 96, 4);
    tft.setTextColor(C_DIM, C_BG);
    tft.drawString(line2, SCREEN_W / 2, 132, 2);
    s_fullRedraw = true;
}

// ---------------------------------------------------------------------------
//  static chrome
// ---------------------------------------------------------------------------
static void drawChrome() {
    tft.fillScreen(C_BG);
    tft.drawFastHLine(0, HEADER_H, SCREEN_W, C_LINE);
    tft.drawFastHLine(0, FOOTER_Y - 2, SCREEN_W, C_LINE);
    tft.drawFastVLine(COL_X - 4, HEADER_H + 1, FOOTER_Y - HEADER_H - 4, C_LINE);

    // battery outline + terminal nub
    tft.drawRoundRect(6, 5, 104, 18, 3, C_DIM);
    tft.fillRect(110, 10, 3, 8, C_DIM);

    // stat tile separators
    for (uint8_t i = 1; i < 4; i++) {
        const int16_t y = HEADER_H + 6 + i * 42;
        tft.drawFastHLine(COL_X + 2, y - 4, SCREEN_W - COL_X - 6, C_LINE);
    }

    // long-press hint next to the badge
    tft.setTextDatum(TL_DATUM);
    tft.setTextColor(C_DIM, C_BG);
    //tft.drawString("HOLD 2s", BADGE_X + BADGE_W + 8, BADGE_Y + 3, 1);
    //tft.drawString("PROFILE", BADGE_X + BADGE_W + 8, BADGE_Y + 15, 1);
    tft.drawString("HOLD", BADGE_X + BADGE_W + 8, BADGE_Y + 15, 1);
}

// ---------------------------------------------------------------------------
//  header
// ---------------------------------------------------------------------------
// `known` false means the VESC hasn't told us its battery config (cell count
// + a chemistry this dash has a curve for) yet - there is no hardcoded guess
// to fill the bar with, so it shows "?" instead of a number that might be
// built on a wrong assumption.
static void drawBattery(bool known, float pct) {
    // -1 doubles as "not yet drawn" (the cache's own init value) and "known
    // battery state is unknown" - both draw the same "?", so the collision
    // is harmless.
    const int8_t p = known ? (int8_t)roundf(constrain(pct, 0.0f, 100.0f)) : -1;
    if (p == cache.battPct && !s_fullRedraw) return;
    cache.battPct = p;

    tft.fillRect(8, 7, 100, 14, C_PANEL);

    char buf[8];
    if (known) {
        const uint16_t col = (p <= 15) ? C_RED : (p <= 30) ? C_ORANGE : C_GREEN;
        const int16_t  w   = (int16_t)((100 * p) / 100);
        if (w > 0) tft.fillRect(8, 7, w, 14, col);
        snprintf(buf, sizeof(buf), "%d%%", p);
    } else {
        snprintf(buf, sizeof(buf), "?");
    }

    tft.setTextDatum(TL_DATUM);
    tft.setTextColor(C_TEXT, C_BG);
    tft.setTextPadding(40);
    tft.drawString(buf, 120, 7, 2);
    tft.setTextPadding(0);
}

static void drawBleIcon(bool connected, bool changed) {
    if (!changed) return;
    const int16_t x = 258, y = 7;
    tft.fillRect(x - 2, y - 2, 18, 20, C_BG);
    const uint16_t c = connected ? C_BLUE : C_LINE;
    // stylised bluetooth rune
    tft.drawLine(x + 6, y,     x + 6, y + 16, c);
    tft.drawLine(x + 6, y,     x + 11, y + 5,  c);
    tft.drawLine(x + 11, y + 5, x + 1, y + 11, c);
    tft.drawLine(x + 6, y + 16, x + 11, y + 11, c);
    tft.drawLine(x + 11, y + 11, x + 1, y + 5, c);
}

static void drawHeader(const DashModel &m) {
    drawBattery(m.battKnown, m.batteryPct);

    char v[10];
    if (m.battKnown) {
        snprintf(v, sizeof(v), "%.1fV", m.voltage);
    } else {
        snprintf(v, sizeof(v), "?");
    }
    if (s_fullRedraw || strcmp(v, cache.voltage) != 0) {
        strcpy(cache.voltage, v);
        tft.setTextDatum(TR_DATUM);
        tft.setTextColor(m.linkOk ? C_TEXT : C_DIM, C_BG);
        tft.setTextPadding(70);
        tft.drawString(v, 248, 7, 2);
        tft.setTextPadding(0);
    }

    const int8_t ble = m.bleConnected ? 1 : 0;
    drawBleIcon(m.bleConnected, s_fullRedraw || ble != cache.bleState);
    cache.bleState = ble;

    const int8_t link = m.linkOk ? 1 : 0;
    if (s_fullRedraw || link != cache.linkState) {
        cache.linkState = link;
        tft.fillCircle(288, 14, 5, link ? C_GREEN : C_RED);
        tft.setTextDatum(TL_DATUM);
        tft.setTextColor(link ? C_DIM : C_RED, C_BG);
        tft.setTextPadding(0);
        tft.fillRect(296, 8, 24, 12, C_BG);
        tft.drawString(link ? "OK" : "NO", 298, 9, 1);
    }
}

// ---------------------------------------------------------------------------
//  speed
// ---------------------------------------------------------------------------
// `m.geomKnown` false means the VESC hasn't told us its pole count/gear
// ratio/wheel diameter yet, so there is no speed to show: "--" rather than a
// "0" that reads as standing still. -2 keeps that distinct from the cache's
// own -1 "not yet drawn".
static void drawSpeed(const DashModel &m) {
    const float  s   = max(0.0f, m.speed);
    const int16_t iv = m.geomKnown ? (int16_t)s : -2;
    const int8_t  dv = m.geomKnown ? (int8_t)((s - iv) * 10.0f) : -2;

    if (!s_fullRedraw && iv == cache.speedInt && dv == cache.speedDec) return;
    const bool intChanged = s_fullRedraw || iv != cache.speedInt;
    cache.speedInt = iv;
    cache.speedDec = dv;

    const uint16_t col = m.linkOk ? C_TEXT : C_DIM;

    if (intChanged) {
        // Font 8 is digits plus ":-." only - "--" fits it, "?" would not.
        if (speedSprite.created()) {
            speedSprite.fillSprite(C_BG);
            speedSprite.setTextDatum(TR_DATUM);
            speedSprite.setTextColor(col, C_BG);
            if (iv < 0) speedSprite.drawString("--", 148, 0, 8);
            else        speedSprite.drawNumber(iv, 148, 0, 8);
            speedSprite.pushSprite(0, 36);
        } else {
            tft.setTextDatum(TR_DATUM);
            tft.setTextColor(col, C_BG);
            tft.setTextPadding(148);
            if (iv < 0) tft.drawString("--", 148, 36, 8);
            else        tft.drawNumber(iv, 148, 36, 8);
            tft.setTextPadding(0);
        }
    }

    // tenths, in a smaller font, hanging off the big number (blank - just
    // the padding clearing the old digit - while the speed is unknown)
    char dec[6] = " ";
    if (m.geomKnown) snprintf(dec, sizeof(dec), ".%d", dv);
    tft.setTextDatum(TL_DATUM);
    tft.setTextColor(col, C_BG);
    tft.setTextPadding(34);
    tft.drawString(dec, 152, 84, 4);
    tft.setTextPadding(0);

    if (s_fullRedraw) {
        tft.setTextDatum(TL_DATUM);
        tft.setTextColor(C_DIM, C_BG);
        tft.drawString(DashStats::speedUnit(), 152, 62, 2);
    }
}

// ---------------------------------------------------------------------------
//  power bar - zero in the middle, regen to the left
// ---------------------------------------------------------------------------
static void drawPower(const DashModel &m) {
    const int16_t x0 = 10, x1 = 186, y = 150, h = 20;
    const int16_t cx = (int16_t)((x0 + x1) / 2);

    if (s_fullRedraw) {
        tft.drawRect(x0 - 1, y - 1, (x1 - x0) + 2, h + 2, C_LINE);
        tft.fillRect(x0, y, x1 - x0, h, C_PANEL);
        tft.drawFastVLine(cx, y - 3, h + 6, C_DIM);
        tft.setTextDatum(TL_DATUM);
        tft.setTextColor(C_DIM, C_BG);
        tft.drawString("REGEN", x0, y + h + 4, 1);
        tft.setTextDatum(TR_DATUM);
        tft.drawString("DRIVE", x1, y + h + 4, 1);
        cache.powerBarPx = -9999;
    }

    int16_t px;
    if (m.powerW >= 0) {
        px = (int16_t)((m.powerW / POWER_BAR_MAX_W) * (float)(x1 - cx));
        px = constrain(px, (int16_t)0, (int16_t)(x1 - cx));
    } else {
        px = (int16_t)((m.powerW / POWER_BAR_REGEN_W) * (float)(cx - x0));
        px = constrain(px, (int16_t)-(cx - x0), (int16_t)0);
    }

    if (px != cache.powerBarPx) {
        cache.powerBarPx = px;
        tft.fillRect(x0, y, x1 - x0, h, C_PANEL);
        if (px > 0) {
            const uint16_t c = (m.powerW > POWER_BAR_MAX_W * 0.8f) ? C_ORANGE : C_CYAN;
            tft.fillRect(cx, y, px, h, c);
        } else if (px < 0) {
            tft.fillRect(cx + px, y, -px, h, C_GREEN);
        }
        tft.drawFastVLine(cx, y, h, C_DIM);
    }

    char p[14];
    snprintf(p, sizeof(p), "%d W", (int)roundf(m.powerW));
    if (s_fullRedraw || strcmp(p, cache.powerTxt) != 0) {
        strcpy(cache.powerTxt, p);
        tft.setTextDatum(MC_DATUM);
        tft.setTextColor(m.powerW < -5 ? C_GREEN : C_TEXT, C_BG);
        tft.setTextPadding(90);
        tft.drawString(p, (x0 + x1) / 2, y + h + 14, 2);
        tft.setTextPadding(0);
    }
}

// ---------------------------------------------------------------------------
//  right-hand stat column (two pages, tap to switch)
// ---------------------------------------------------------------------------
static void drawTile(uint8_t i, const char *label, const char *value, uint16_t valueCol) {
    const int16_t y = HEADER_H + 6 + i * 42;

    if (s_fullRedraw || strcmp(label, cache.tileLabel[i]) != 0) {
        strncpy(cache.tileLabel[i], label, sizeof(cache.tileLabel[i]) - 1);
        tft.setTextDatum(TL_DATUM);
        tft.setTextColor(C_DIM, C_BG);
        tft.setTextPadding(SCREEN_W - COL_X - 8);
        tft.drawString(label, COL_X + 4, y, 1);
    }

    if (s_fullRedraw || strcmp(value, cache.tile[i]) != 0) {
        strncpy(cache.tile[i], value, sizeof(cache.tile[i]) - 1);
        tft.setTextDatum(TL_DATUM);
        tft.setTextColor(valueCol, C_BG);
        tft.setTextPadding(SCREEN_W - COL_X - 8);
        tft.drawString(value, COL_X + 4, y + 12, 4);
    }
    tft.setTextPadding(0);
}

static void drawStats(const DashModel &m) {
    char buf[16];

    if (s_page == 0) {
        snprintf(buf, sizeof(buf), "%.1f", m.tripDist);
        drawTile(0, "TRIP", buf, C_TEXT);

        snprintf(buf, sizeof(buf), "%.0f", m.whUsed);
        drawTile(1, "WH USED", buf, C_TEXT);

        if (m.whPerDist > 0.5f) snprintf(buf, sizeof(buf), "%.1f", m.whPerDist);
        else                    snprintf(buf, sizeof(buf), "--");
        drawTile(2, USE_IMPERIAL ? "WH/MI" : "WH/KM", buf, C_TEXT);

        snprintf(buf, sizeof(buf), "%.0f", m.tempMotor);
        drawTile(3, "MOTOR C", buf, m.tempMotor > TEMP_WARN_MOTOR_C ? C_RED : C_TEXT);
    } else {
        if (m.rangeLeft > 0.5f) snprintf(buf, sizeof(buf), "%.0f", m.rangeLeft);
        else                    snprintf(buf, sizeof(buf), "--");
        drawTile(0, USE_IMPERIAL ? "RANGE mi" : "RANGE km", buf, C_TEXT);

        snprintf(buf, sizeof(buf), "%.1f", m.maxSpeed);
        drawTile(1, "MAX", buf, C_TEXT);

        snprintf(buf, sizeof(buf), "%.1f", m.avgSpeed);
        drawTile(2, "AVG", buf, C_TEXT);

        snprintf(buf, sizeof(buf), "%.0f", m.tempFet);
        drawTile(3, "ESC C", buf, m.tempFet > TEMP_WARN_FET_C ? C_RED : C_TEXT);
    }
}

// ---------------------------------------------------------------------------
//  footer: profile badge + odometer + toast + fault banner
// ---------------------------------------------------------------------------
static void drawProfileBadge(const DashModel &m, float holdProgress) {
    const RiderProfile &p = RIDER_PROFILES[m.profileIndex % PROFILE_COUNT];

    if (s_fullRedraw || m.profileIndex != cache.profile) {
        cache.profile = (int8_t)m.profileIndex;
        tft.fillRoundRect(BADGE_X, BADGE_Y, BADGE_W, BADGE_H, 6, p.color);
        tft.drawRoundRect(BADGE_X, BADGE_Y, BADGE_W, BADGE_H, 6, C_TEXT);
        tft.setTextDatum(MC_DATUM);
        tft.setTextColor(C_BG, p.color);
        tft.drawString(p.name, BADGE_X + BADGE_W / 2, BADGE_Y + BADGE_H / 2, 4);
        cache.holdPx = -1;
    }

    // progress sliver along the bottom of the badge while it is held
    const int16_t px = (int16_t)(constrain(holdProgress, 0.0f, 1.0f) * (BADGE_W - 8));
    if (px != cache.holdPx) {
        cache.holdPx = px;
        tft.fillRect(BADGE_X + 4, BADGE_Y + BADGE_H - 6, BADGE_W - 8, 3, p.color);
        if (px > 0) tft.fillRect(BADGE_X + 4, BADGE_Y + BADGE_H - 6, px, 3, C_BG);
    }
}

static void drawFooter(const DashModel &m, float holdProgress) {
    drawProfileBadge(m, holdProgress);

    const bool showToast = (millis() < s_toastUntil);
    static bool wasToast = false;

    if (showToast) {
        tft.setTextDatum(MR_DATUM);
        tft.setTextColor(C_YELLOW, C_BG);
        tft.setTextPadding(170);
        tft.drawString(s_toast, SCREEN_W - 6, BADGE_Y + BADGE_H / 2, 2);
        tft.setTextPadding(0);
        cache.odo[0] = '\0';
        wasToast = true;
        return;
    }

    char o[16];
    snprintf(o, sizeof(o), "ODO %.1f %s", m.odoDist, DashStats::distUnit());
    if (s_fullRedraw || wasToast || strcmp(o, cache.odo) != 0) {
        strcpy(cache.odo, o);
        wasToast = false;
        tft.setTextDatum(MR_DATUM);
        tft.setTextColor(C_DIM, C_BG);
        tft.setTextPadding(170);
        tft.drawString(o, SCREEN_W - 6, BADGE_Y + BADGE_H / 2, 2);
        tft.setTextPadding(0);
    }
}

static bool s_faultDrawn = false;

static void drawFault(const DashModel &m) {
    if (!s_fullRedraw && m.faultCode == cache.fault) return;
    cache.fault = m.faultCode;

    if (m.faultCode == 0) {
        if (s_faultDrawn) {               // the banner was covering the dash
            s_faultDrawn = false;
            tft.fillRect(0, 118, COL_X - 6, 26, C_BG);
            s_fullRedraw = true;          // repaint whatever it hid
        }
        return;
    }
    s_faultDrawn = true;
    tft.fillRect(4, 118, COL_X - 12, 26, C_RED);
    tft.setTextDatum(MC_DATUM);
    tft.setTextColor(C_TEXT, C_RED);
    tft.drawString(vesc::faultToString(m.faultCode), (COL_X - 8) / 2, 131, 2);
}

// ---------------------------------------------------------------------------
void render(const DashModel &m, float holdProgress) {
    // The link state picks the colour of the speed and voltage text, but
    // their caches only compare the text itself - a speed already at 0 when
    // the link drops would otherwise never grey out. Links come and go
    // rarely, so just repaint everything.
    if (cache.linkState >= 0 && cache.linkState != (m.linkOk ? 1 : 0)) s_fullRedraw = true;

    if (s_fullRedraw) {
        drawChrome();
        cache = Cache();               // invalidate every cached string
    }

    drawHeader(m);
    drawSpeed(m);
    drawPower(m);
    drawStats(m);
    drawFooter(m, holdProgress);

    // Cleared only now that every zone above has seen it - their
    // `if (s_fullRedraw)` blocks draw the parts no cache covers (the speed
    // unit, the power bar's frame and labels). Before drawFault(), which may
    // ask for a full redraw of the *next* frame.
    s_fullRedraw = false;
    drawFault(m);
}

} // namespace ui
