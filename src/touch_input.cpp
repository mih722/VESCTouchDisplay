#include "touch_input.h"

#if defined(TOUCH_RESISTIVE)
  #include <SPI.h>
#else
  #include <Wire.h>
#endif

namespace touch {

// Native panel geometry, before rotation is applied. Board-specific - see
// TOUCH_PANEL_W/TOUCH_PANEL_H in config.h.
static const int16_t PANEL_W = TOUCH_PANEL_W;
static const int16_t PANEL_H = TOUCH_PANEL_H;

static const char *s_driver = "none";
const char *driverName() { return s_driver; }

// ---------------------------------------------------------------------------
//  Native panel coordinates -> screen coordinates
// ---------------------------------------------------------------------------
static void mapToScreen(int16_t nx, int16_t ny, int16_t &sx, int16_t &sy) {
    nx = constrain(nx, (int16_t)0, (int16_t)(PANEL_W - 1));
    ny = constrain(ny, (int16_t)0, (int16_t)(PANEL_H - 1));

    if (TOUCH_SWAP_XY) { sx = ny; sy = nx; }
    else               { sx = nx; sy = ny; }

    if (TOUCH_INVERT_X) sx = (int16_t)(SCREEN_W - 1 - sx);
    if (TOUCH_INVERT_Y) sy = (int16_t)(SCREEN_H - 1 - sy);

    sx = constrain(sx, (int16_t)0, (int16_t)(SCREEN_W - 1));
    sy = constrain(sy, (int16_t)0, (int16_t)(SCREEN_H - 1));
}

// ===========================================================================
//  Resistive: XPT2046
// ===========================================================================
#if defined(TOUCH_RESISTIVE)

static SPIClass s_tspi(TOUCH_SPI_HOST);
static SPISettings s_tset(2000000, MSBFIRST, SPI_MODE0);

static uint16_t xptRead(uint8_t cmd) {
    s_tspi.transfer(cmd);
    delayMicroseconds(10);
    return (uint16_t)(s_tspi.transfer16(0x00) >> 3);   // 12-bit result
}

void begin() {
    pinMode(TOUCH_CS_PIN, OUTPUT);
    digitalWrite(TOUCH_CS_PIN, HIGH);
#if TOUCH_IRQ_PIN >= 0
    pinMode(TOUCH_IRQ_PIN, INPUT);
#endif
    s_tspi.begin(TOUCH_CLK_PIN, TOUCH_MISO_PIN, TOUCH_MOSI_PIN, TOUCH_CS_PIN);
    s_driver = "XPT2046";
    Serial.println("[touch] XPT2046 resistive");
}

bool read(int16_t &x, int16_t &y) {
    s_tspi.beginTransaction(s_tset);
    digitalWrite(TOUCH_CS_PIN, LOW);

    // Pressure first: Z1 high and Z2 low means a real contact.
    const uint16_t z1 = xptRead(0xB1);
    const uint16_t z2 = xptRead(0xC1);
    const int32_t  z  = (int32_t)z1 + 4095 - (int32_t)z2;

    bool pressed = (z > TOUCH_PRESSURE_MIN);
    uint32_t sx = 0, sy = 0;

    if (pressed) {
        xptRead(0xD1);                       // discard first, it settles slowly
        for (uint8_t i = 0; i < 3; i++) {
            sx += xptRead(0xD1);
            sy += xptRead(0x91);
        }
        sx /= 3;
        sy /= 3;
    }
    xptRead(0xD0);                           // return to power-down
    digitalWrite(TOUCH_CS_PIN, HIGH);
    s_tspi.endTransaction();

    if (!pressed) return false;
    if (sx < 100 || sy < 100) return false;  // rail readings = no contact

    const int16_t nx = (int16_t)map((long)sx, TOUCH_RAW_X_MIN, TOUCH_RAW_X_MAX, 0, PANEL_W - 1);
    const int16_t ny = (int16_t)map((long)sy, TOUCH_RAW_Y_MIN, TOUCH_RAW_Y_MAX, 0, PANEL_H - 1);

#if TOUCH_DEBUG
    Serial.printf("[touch] raw=%lu,%lu z=%ld -> n=%d,%d\n", sx, sy, z, nx, ny);
#endif

    mapToScreen(nx, ny, x, y);
    return true;
}

// ===========================================================================
//  Capacitive: AXS5106L (ESP32-C6-Touch-LCD-1.47)
//
//  A different chip/protocol from the CST8xx family below: fixed I2C
//  address 0x63, a single 14-byte read from register 0x01 carrying up to 5
//  touch points (this project only ever tracks one, same as the CST8xx
//  path). Ported from Waveshare's own esp_lcd_touch_axs5106l.cpp (files.
//  waveshare.com/wiki/ESP32-C6-Touch-LCD-1.47/ESP32-C6-Touch-LCD-1.47-
//  Demo.zip) minus its interrupt-driven read gate and its own built-in
//  rotation remapping - like every other driver in this file, this one
//  polls and hands raw native coordinates to mapToScreen(), so the usual
//  TOUCH_SWAP_XY/TOUCH_INVERT_X/TOUCH_INVERT_Y knobs are what fix orientation.
// ===========================================================================
#elif defined(BOARD_C6_LCD147)

static const uint8_t AXS5106L_ADDR = 0x63;

static bool axsReadRegs(uint8_t reg, uint8_t *buf, uint8_t len) {
    Wire.beginTransmission(AXS5106L_ADDR);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom((int)AXS5106L_ADDR, (int)len) != len) return false;
    for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read();
    return true;
}

void begin() {
#if TOUCH_RST_PIN >= 0
    pinMode(TOUCH_RST_PIN, OUTPUT);
    digitalWrite(TOUCH_RST_PIN, LOW);
    delay(200);
    digitalWrite(TOUCH_RST_PIN, HIGH);
    delay(300);
#endif
    Wire.begin(TOUCH_SDA_PIN, TOUCH_SCL_PIN, TOUCH_I2C_HZ);
    s_driver = "AXS5106L";
    Serial.printf("[touch] %s at 0x%02X (SDA=%d SCL=%d)\n",
                  s_driver, AXS5106L_ADDR, TOUCH_SDA_PIN, TOUCH_SCL_PIN);
}

bool read(int16_t &x, int16_t &y) {
    uint8_t b[14];
    if (!axsReadRegs(0x01, b, sizeof(b))) return false;
    const uint8_t fingers = b[1];
    if (fingers == 0 || fingers > 5) return false;

    const int16_t nx = (int16_t)((((uint16_t)b[2] & 0x0F) << 8) | b[3]);
    const int16_t ny = (int16_t)((((uint16_t)b[4] & 0x0F) << 8) | b[5]);

#if TOUCH_DEBUG
    Serial.printf("[touch] native=%d,%d\n", nx, ny);
#endif

    mapToScreen(nx, ny, x, y);
    return true;
}

// ===========================================================================
//  Capacitive: FT3168 (ESP32-S3-Touch-AMOLED-1.64)
//
//  FocalTech register layout at fixed address 0x38: TD_STATUS (touch count)
//  at 0x02, then the first point's X/Y high/low bytes at 0x03-0x06, the top
//  bits of each high byte carrying flags rather than coordinate. That is the
//  same shape the CST8xx branch below reads, but its init writes two
//  CST-only registers, so this chip gets its own branch. Ported from
//  Waveshare's FT3168.cpp (files.waveshare.com/wiki/ESP32-S3-Touch-AMOLED-
//  1.64/ESP32-S3-Touch-AMOLED-1.64-Demo.zip, Arduino/examples/06_LVGL_Test):
//  one write to put it in normal mode, then poll.
// ===========================================================================
#elif defined(BOARD_S3_AMOLED164)

static const uint8_t FT3168_ADDR = 0x38;

static bool ftReadRegs(uint8_t reg, uint8_t *buf, uint8_t len) {
    Wire.beginTransmission(FT3168_ADDR);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom((int)FT3168_ADDR, (int)len) != len) return false;
    for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read();
    return true;
}

void begin() {
    Wire.begin(TOUCH_SDA_PIN, TOUCH_SCL_PIN, TOUCH_I2C_HZ);
    Wire.beginTransmission(FT3168_ADDR);
    Wire.write(0x00);                        // DEVICE_MODE
    Wire.write(0x00);                        // normal operating mode
    const bool found = Wire.endTransmission() == 0;
    s_driver = "FT3168";
    Serial.printf("[touch] %s at 0x%02X (SDA=%d SCL=%d)%s\n",
                  s_driver, FT3168_ADDR, TOUCH_SDA_PIN, TOUCH_SCL_PIN,
                  found ? "" : " - not answering");
}

bool read(int16_t &x, int16_t &y) {
    uint8_t b[5];
    if (!ftReadRegs(0x02, b, sizeof(b))) return false;
    const uint8_t fingers = b[0] & 0x0F;
    if (fingers == 0 || fingers > 2) return false;

    const int16_t nx = (int16_t)((((uint16_t)b[1] & 0x0F) << 8) | b[2]);
    const int16_t ny = (int16_t)((((uint16_t)b[3] & 0x0F) << 8) | b[4]);

#if TOUCH_DEBUG
    Serial.printf("[touch] native=%d,%d\n", nx, ny);
#endif

    mapToScreen(nx, ny, x, y);
    return true;
}

// ===========================================================================
//  Capacitive: CST816/CST820 (0x15) or GT911 (0x5D / 0x14)
// ===========================================================================
#else

static uint8_t s_addr = 0;
static bool    s_isGt911 = false;

static bool i2cPresent(uint8_t addr) {
    Wire.beginTransmission(addr);
    return Wire.endTransmission() == 0;
}

// --- CST8xx: 8-bit register addressing ---
static bool cstWrite(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(s_addr);
    Wire.write(reg);
    Wire.write(val);
    return Wire.endTransmission() == 0;
}

static bool cstReadRegs(uint8_t reg, uint8_t *buf, uint8_t len) {
    Wire.beginTransmission(s_addr);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom((int)s_addr, (int)len) != len) return false;
    for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read();
    return true;
}

// --- GT911: 16-bit register addressing ---
static bool gtReadRegs(uint16_t reg, uint8_t *buf, uint8_t len) {
    Wire.beginTransmission(s_addr);
    Wire.write((uint8_t)(reg >> 8));
    Wire.write((uint8_t)(reg & 0xFF));
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom((int)s_addr, (int)len) != len) return false;
    for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read();
    return true;
}

static bool gtWriteReg(uint16_t reg, uint8_t val) {
    Wire.beginTransmission(s_addr);
    Wire.write((uint8_t)(reg >> 8));
    Wire.write((uint8_t)(reg & 0xFF));
    Wire.write(val);
    return Wire.endTransmission() == 0;
}

void begin() {
#if TOUCH_RST_PIN >= 0
    pinMode(TOUCH_RST_PIN, OUTPUT);
    digitalWrite(TOUCH_RST_PIN, LOW);
    delay(20);
    digitalWrite(TOUCH_RST_PIN, HIGH);
    delay(300);                              // GT911 needs a long settle
#endif
#if TOUCH_INT_PIN >= 0
    pinMode(TOUCH_INT_PIN, INPUT);
#endif

    Wire.begin(TOUCH_SDA_PIN, TOUCH_SCL_PIN, TOUCH_I2C_HZ);

    if (i2cPresent(0x5D))       { s_addr = 0x5D; s_isGt911 = true;  s_driver = "GT911"; }
    else if (i2cPresent(0x14))  { s_addr = 0x14; s_isGt911 = true;  s_driver = "GT911"; }
    else if (i2cPresent(0x15))  { s_addr = 0x15; s_isGt911 = false; s_driver = "CST816/820"; }
    else {
        // CST816 family does not always answer a bare address probe.
        s_addr = 0x15; s_isGt911 = false; s_driver = "CST820?";
        Serial.println("[touch] no controller answered - assuming CST820 @0x15");
    }

    if (!s_isGt911) {
        cstWrite(0xFA, 0x60);                // INT: pulse on touch
        cstWrite(0xFE, 0xFF);                // disable auto sleep
    }
    Serial.printf("[touch] %s at 0x%02X (SDA=%d SCL=%d)\n",
                  s_driver, s_addr, TOUCH_SDA_PIN, TOUCH_SCL_PIN);
}

bool read(int16_t &x, int16_t &y) {
    int16_t nx = 0, ny = 0;

    if (s_isGt911) {
        uint8_t status = 0;
        if (!gtReadRegs(0x814E, &status, 1)) return false;
        if (!(status & 0x80)) return false;

        const uint8_t points = status & 0x0F;
        uint8_t p[8];
        bool ok = points > 0 && gtReadRegs(0x8150, p, 8);
        gtWriteReg(0x814E, 0x00);            // ack the buffer
        if (!ok) return false;

        nx = (int16_t)((uint16_t)p[1] | ((uint16_t)p[2] << 8));
        ny = (int16_t)((uint16_t)p[3] | ((uint16_t)p[4] << 8));
    } else {
        uint8_t b[6];
        if (!cstReadRegs(0x01, b, 6)) return false;
        const uint8_t fingers = b[1];
        if (fingers == 0 || fingers > 2) return false;

        nx = (int16_t)((((uint16_t)b[2] & 0x0F) << 8) | b[3]);
        ny = (int16_t)((((uint16_t)b[4] & 0x0F) << 8) | b[5]);
    }

#if TOUCH_DEBUG
    Serial.printf("[touch] native=%d,%d\n", nx, ny);
#endif

    mapToScreen(nx, ny, x, y);
    return true;
}

#endif  // TOUCH_RESISTIVE

// ===========================================================================
//  Gesture layer
// ===========================================================================
static Gesture s_g;
static uint32_t s_downMs = 0;
static uint8_t  s_missCount = 0;

const Gesture &poll() {
    int16_t x, y;
    const bool raw = read(x, y);

    s_g.justPressed = false;
    s_g.justReleased = false;

    if (raw) {
        s_missCount = 0;
        s_g.x = x;
        s_g.y = y;
        if (!s_g.pressed) {
            s_g.pressed = true;
            s_g.justPressed = true;
            s_g.downX = x;
            s_g.downY = y;
            s_downMs = millis();
        }
        s_g.heldMs = millis() - s_downMs;
    } else if (s_g.pressed) {
        // Capacitive panels drop the occasional frame; do not treat a single
        // missed read as a release or long presses become impossible.
        if (++s_missCount >= 3) {
            s_missCount = 0;
            s_g.pressed = false;
            s_g.justReleased = true;
            s_g.heldMs = millis() - s_downMs;
        }
    } else {
        s_g.heldMs = 0;
    }

    return s_g;
}

} // namespace touch
