// ===========================================================================
//  Headless integration test.
//
//  Links the real vesc_link / dash_model / profiles code against stub
//  peripherals so the byte-level behaviour can be checked on a PC:
//    - what actually goes out on the wire for a telemetry poll
//    - what actually goes out on the wire for a profile change
//    - that a VESC reply is decoded into the right telemetry
//    - that trip / odometer / state-of-charge maths behaves over time
//  Build it twice: as-is for the CAN link, and with -DVESC_LINK_UART=1 for
//  the UART link. Everything that isn't transport-specific runs on both.
// ===========================================================================
// STL first: the Arduino core's min/max macros break libstdc++ headers.
#include <deque>
#include <vector>
#include <cstdarg>

#include <Arduino.h>
#include <Preferences.h>
#include <SPI.h>
#include <Wire.h>

#include "vesc_can.h"
#include "vesc_uart.h"
#include "vesc_link.h"
#include "dash_model.h"
#include "profiles.h"

// ---------------------------------------------------------------------------
//  Stub peripherals
// ---------------------------------------------------------------------------
static uint32_t g_millis = 0;
uint32_t millis() { return g_millis; }
void delay(uint32_t ms) { g_millis += ms; }
void delayMicroseconds(uint32_t) {}
void pinMode(int, int) {}
void digitalWrite(int, int) {}
int  digitalRead(int) { return 0; }
long map(long x, long a, long b, long c, long d) { return (x - a) * (d - c) / (b - a) + c; }
void ledcSetup(uint8_t, double, uint8_t) {}
void ledcAttachPin(uint8_t, uint8_t) {}
void ledcWrite(uint8_t, uint32_t) {}

void vTaskDelay(TickType_t) {}
BaseType_t xTaskCreatePinnedToCore(void (*)(void*), const char*, uint32_t, void*, unsigned, TaskHandle_t*, int) { return 1; }
static int g_mutexToken = 1;
SemaphoreHandle_t xSemaphoreCreateMutex() { return &g_mutexToken; }
BaseType_t xSemaphoreTake(SemaphoreHandle_t, TickType_t) { return pdTRUE; }
BaseType_t xSemaphoreGive(SemaphoreHandle_t) { return pdTRUE; }

bool Preferences::begin(const char*, bool) { return true; }
float   Preferences::getFloat(const char*, float def) { return def; }
size_t  Preferences::putFloat(const char*, float) { return 4; }
uint8_t Preferences::getUChar(const char*, uint8_t def) { return def; }
size_t  Preferences::putUChar(const char*, uint8_t) { return 1; }

void SPIClass::begin(int, int, int, int) {}
void SPIClass::beginTransaction(SPISettings) {}
void SPIClass::endTransaction() {}
uint8_t  SPIClass::transfer(uint8_t) { return 0; }
uint16_t SPIClass::transfer16(uint16_t) { return 0; }
bool TwoWire::begin(int, int, uint32_t) { return true; }
void TwoWire::beginTransmission(uint8_t) {}
size_t TwoWire::write(uint8_t) { return 1; }
uint8_t TwoWire::endTransmission(bool) { return 1; }
uint8_t TwoWire::requestFrom(int, int) { return 0; }
int TwoWire::read() { return 0; }
TwoWire Wire;

#if VESC_LINK_UART
// --- scriptable UART -----------------------------------------------------
static std::deque<uint8_t>  g_uartRx;   // bytes the "VESC" will send us
static std::vector<uint8_t> g_uartTx;   // bytes we sent to the "VESC"

namespace vescuart {
void begin() {}
size_t read(uint8_t *buf, size_t cap) {
    size_t n = 0;
    while (n < cap && !g_uartRx.empty()) {
        buf[n++] = g_uartRx.front();
        g_uartRx.pop_front();
    }
    return n;
}
size_t write(const uint8_t *data, size_t len) {
    g_uartTx.insert(g_uartTx.end(), data, data + len);
    return len;
}
} // namespace vescuart

static void clearTx() { g_uartTx.clear(); }
static void clearRx() { g_uartRx.clear(); }
#else
// --- scriptable CAN bus ------------------------------------------------
static std::deque<CanFrame>  g_rx;     // frames the "VESC" will send us
static std::vector<CanFrame> g_tx;     // frames we sent to the "VESC"

namespace vescan {
void begin() {}
bool poll(CanFrame &out) {
    if (g_rx.empty()) return false;
    out = g_rx.front();
    g_rx.pop_front();
    return true;
}
bool send(const CanFrame &f) {
    g_tx.push_back(f);
    return true;
}
} // namespace vescan

static void clearTx() { g_tx.clear(); }
static void clearRx() { g_rx.clear(); }
#endif

// Captures whatever the link hands to the BLE byte sink.
static std::vector<uint8_t> g_bleOut;
static void captureSink(const uint8_t *data, size_t len) {
    g_bleOut.insert(g_bleOut.end(), data, data + len);
}

void HardwareSerial::begin(unsigned long, uint32_t, int8_t, int8_t) {}
void HardwareSerial::setRxBufferSize(size_t) {}
int  HardwareSerial::available() { return 0; }
size_t HardwareSerial::readBytes(uint8_t *, size_t) { return 0; }
size_t HardwareSerial::write(const uint8_t *, size_t len) { return len; }
void HardwareSerial::println(const char *s) { printf("%s\n", s); }
void HardwareSerial::print(const char *s) { printf("%s", s); }
int  HardwareSerial::printf(const char *fmt, ...) {
    va_list ap; va_start(ap, fmt);
    int r = vprintf(fmt, ap);
    va_end(ap);
    return r;
}
HardwareSerial Serial;

// ---------------------------------------------------------------------------
static int failures = 0;
#define CHECK(cond, msg) do { if(!(cond)){ printf("FAIL: %s\n", msg); failures++; } } while(0)
#define NEAR(a, b, eps) (fabsf((float)(a) - (float)(b)) < (eps))

using namespace vesc;

#if VESC_LINK_UART
// Pull the next complete COMM payload out of the bytes we wrote towards the
// (simulated) VESC, the way the VESC's own UART packet decoder would.
static bool takeSentPayload(std::vector<uint8_t> &payload) {
    FrameSniffer sn;
    for (size_t used = 0; used < g_uartTx.size(); ) {
        if (sn.feed(g_uartTx[used++])) {
            payload.assign(sn.payload(), sn.payload() + sn.length());
            g_uartTx.erase(g_uartTx.begin(), g_uartTx.begin() + used);
            return true;
        }
    }
    g_uartTx.clear();
    return false;
}

// Queue a COMM payload as the VESC would send it back: one framed packet.
static void injectReply(const uint8_t *p, uint16_t len) {
    std::vector<uint8_t> frame((size_t)len + 8);
    const uint16_t n = buildFrame(p, len, frame.data(), (uint16_t)frame.size());
    g_uartRx.insert(g_uartRx.end(), frame.begin(), frame.begin() + n);
}
#else
// Reassemble the tunnelled COMM payload out of the CAN frames we sent
// towards the (simulated) VESC, the way a real VESC's rx buffer would.
static bool takeSentPayload(std::vector<uint8_t> &payload) {
    CanTunnel rx;
    while (!g_tx.empty()) {
        CanFrame f = g_tx.front();
        g_tx.erase(g_tx.begin());
        if (rx.feed(f, VESC_CAN_TARGET_ID)) {
            payload.assign(rx.payload(), rx.payload() + rx.length());
            return true;
        }
    }
    return false;
}

// Queue a COMM payload as the VESC would send it back: tunnelled to the
// dash's own poll id.
static void injectReply(const uint8_t *p, uint16_t len) {
    std::vector<CanFrame> frames((size_t)len / 6 + 4);
    const int n = buildTunnelFrames(VESC_CAN_LOCAL_ID, VESC_CAN_TARGET_ID, p, len, false,
                                     frames.data(), (int)frames.size());
    for (int k = 0; k < n; k++) g_rx.push_back(frames[k]);
}
#endif

// Drain every payload we sent, in order. The periodic fault poll
// shares the wire with everything else, so a test looking for one specific
// command must not assume it is the only thing queued.
static std::vector<std::vector<uint8_t>> takeAllSentPayloads() {
    std::vector<std::vector<uint8_t>> all;
    std::vector<uint8_t> p;
    while (takeSentPayload(p)) all.push_back(p);
    return all;
}

static const std::vector<uint8_t> *findPayload(
        const std::vector<std::vector<uint8_t>> &all, uint8_t commandId) {
    for (const auto &p : all) if (!p.empty() && p[0] == commandId) return &p;
    return nullptr;
}

// Read back a float in the firmware's "auto" encoding (IEEE-754 compatible).
static float readFloat32Auto(const uint8_t *b, int32_t *i) {
    uint32_t v = ((uint32_t)b[*i] << 24) | ((uint32_t)b[*i + 1] << 16) |
                 ((uint32_t)b[*i + 2] << 8) | (uint32_t)b[*i + 3];
    *i += 4;
    float f;
    memcpy(&f, &v, 4);
    return f;
}

#if !VESC_LINK_UART
// Feed synthetic CAN status broadcasts into the receive queue - the way the
// real VESC's periodic telemetry arrives, no request needed.
static void injectStatusBroadcast(float volts, float erpm, float currentIn,
                                  int32_t tacho, float wh) {
    CanFrame f;
    int32_t i;

    i = 0;
    f = CanFrame{};
    f.id = ((uint32_t)CAN_PACKET_STATUS << 8) | VESC_CAN_TARGET_ID;
    bufAppendInt32(f.data, (int32_t)erpm, &i);
    bufAppendInt16(f.data, (int16_t)(20.0f * 10), &i);    // motor current
    bufAppendInt16(f.data, (int16_t)(0.5f * 1000), &i);   // duty
    f.dlc = 8;
    g_rx.push_back(f);

    i = 0;
    f = CanFrame{};
    f.id = ((uint32_t)CAN_PACKET_STATUS_3 << 8) | VESC_CAN_TARGET_ID;
    bufAppendInt32(f.data, (int32_t)(wh * 10000.0f), &i);
    bufAppendInt32(f.data, 0, &i);
    f.dlc = 8;
    g_rx.push_back(f);

    i = 0;
    f = CanFrame{};
    f.id = ((uint32_t)CAN_PACKET_STATUS_4 << 8) | VESC_CAN_TARGET_ID;
    bufAppendInt16(f.data, (int16_t)(35.0f * 10), &i);    // temp fet
    bufAppendInt16(f.data, (int16_t)(40.0f * 10), &i);    // temp motor
    bufAppendInt16(f.data, (int16_t)(currentIn * 10), &i);
    bufAppendInt16(f.data, 0, &i);                        // pid pos (unused)
    f.dlc = 8;
    g_rx.push_back(f);

    i = 0;
    f = CanFrame{};
    f.id = ((uint32_t)CAN_PACKET_STATUS_5 << 8) | VESC_CAN_TARGET_ID;
    bufAppendInt32(f.data, tacho, &i);
    bufAppendInt16(f.data, (int16_t)(volts * 10), &i);
    f.dlc = 6;
    g_rx.push_back(f);
}

// Feed a synthetic tunnelled fault-code reply into the receive queue - the
// same COMM_GET_VALUES_SELECTIVE the periodic fault poll asks for.
static void injectFaultReply(uint8_t fault) {
    uint8_t p[6];
    int32_t i = 0;
    p[i++] = COMM_GET_VALUES_SELECTIVE;
    bufAppendUint32(p, VAL_FAULT, &i);
    p[i++] = fault;

    injectReply(p, (uint16_t)i);
}
#endif

// A COMM_GET_VALUES_SELECTIVE reply carrying exactly the fields `mask` asks
// for, in the firmware's field order - built from the mask rather than
// hardcoded so it keeps matching parseValues() if a poll mask changes.
static std::vector<uint8_t> valuesReply(uint32_t mask, float volts, float erpm,
                                        float currentIn, int32_t tachoAbs, float wh,
                                        uint8_t fault) {
    uint8_t p[96];
    int32_t i = 0;
    p[i++] = COMM_GET_VALUES_SELECTIVE;
    bufAppendUint32(p, mask, &i);
    if (mask & VAL_TEMP_FET)           bufAppendInt16(p, (int16_t)(35.0f * 10), &i);
    if (mask & VAL_TEMP_MOTOR)         bufAppendInt16(p, (int16_t)(40.0f * 10), &i);
    if (mask & VAL_CURRENT_MOTOR)      bufAppendInt32(p, (int32_t)(20.0f * 100), &i);
    if (mask & VAL_CURRENT_IN)         bufAppendInt32(p, (int32_t)(currentIn * 100), &i);
    if (mask & VAL_ID)                 bufAppendInt32(p, 0, &i);
    if (mask & VAL_IQ)                 bufAppendInt32(p, 0, &i);
    if (mask & VAL_DUTY)               bufAppendInt16(p, (int16_t)(0.5f * 1000), &i);
    if (mask & VAL_RPM)                bufAppendInt32(p, (int32_t)erpm, &i);
    if (mask & VAL_V_IN)               bufAppendInt16(p, (int16_t)(volts * 10), &i);
    if (mask & VAL_AMP_HOURS)          bufAppendInt32(p, 0, &i);
    if (mask & VAL_AMP_HOURS_CHARGED)  bufAppendInt32(p, 0, &i);
    if (mask & VAL_WATT_HOURS)         bufAppendInt32(p, (int32_t)(wh * 10000.0f), &i);
    if (mask & VAL_WATT_HOURS_CHARGED) bufAppendInt32(p, 0, &i);
    if (mask & VAL_TACHOMETER)         bufAppendInt32(p, tachoAbs, &i);
    if (mask & VAL_TACHOMETER_ABS)     bufAppendInt32(p, tachoAbs, &i);
    if (mask & VAL_FAULT)              p[i++] = fault;
    return std::vector<uint8_t>(p, p + i);
}

// Feed a synthetic COMM_GET_APPCONF reply: a recognisable pattern
// everywhere except a sentinel at the app_mode offset, so a test can tell a
// byte-exact round trip from a corrupted one. Writes the pattern used (with
// no sentinel) into `fieldsOut[APPCONF_LEN]` for the caller to compare against.
static void injectAppConfReply(uint8_t seed, uint8_t *fieldsOut) {
    for (uint16_t i = 0; i < APPCONF_LEN; i++) fieldsOut[i] = (uint8_t)(i * 3 + seed);

    uint8_t reply[1 + APPCONF_LEN];
    reply[0] = COMM_GET_APPCONF;
    memcpy(reply + 1, fieldsOut, APPCONF_LEN);
    reply[1 + APPCONF_OFF_APP_TO_USE] = 77;              // sentinel - must not survive

    injectReply(reply, sizeof(reply));
}

// A full COMM_GET_VALUES reply with deliberately wild values - what a Real
// Time Data reply to the phone might carry, and nothing like the dash's own
// telemetry, so a test can tell if it leaked into the display.
static std::vector<uint8_t> wildValuesReply() {
    uint8_t wild[64];
    int32_t wi = 0;
    wild[wi++] = COMM_GET_VALUES;
    bufAppendInt16(wild, (int16_t)(11.0f * 10), &wi);     // temp fet
    bufAppendInt16(wild, (int16_t)(22.0f * 10), &wi);     // temp motor
    bufAppendInt32(wild, (int32_t)(33.0f * 100), &wi);    // motor current
    bufAppendInt32(wild, (int32_t)(44.0f * 100), &wi);    // input current
    bufAppendInt32(wild, 0, &wi);                         // id
    bufAppendInt32(wild, 0, &wi);                         // iq
    bufAppendInt16(wild, (int16_t)(0.9f * 1000), &wi);    // duty
    bufAppendInt32(wild, 99999, &wi);                     // erpm
    bufAppendInt16(wild, (int16_t)(99.9f * 10), &wi);     // v_in
    bufAppendInt32(wild, 0, &wi);                         // Ah
    bufAppendInt32(wild, 0, &wi);                         // Ah charged
    bufAppendInt32(wild, 0, &wi);                         // Wh
    bufAppendInt32(wild, 0, &wi);                         // Wh charged
    bufAppendInt32(wild, 777777, &wi);                    // tacho
    bufAppendInt32(wild, 777777, &wi);                    // tacho abs
    wild[wi++] = 9;                                       // fault
    return std::vector<uint8_t>(wild, wild + wi);
}

#if VESC_LINK_UART
// True if what the link handed the BLE sink contains a frame for `cmd`.
static bool bleGot(uint8_t cmd) {
    FrameSniffer sn;
    for (uint8_t b : g_bleOut) {
        if (sn.feed(b) && sn.length() > 0 && sn.payload()[0] == cmd) return true;
    }
    return false;
}
#endif

// Feed a synthetic tunnelled COMM_GET_MCCONF reply: like injectAppConfReply
// above, a recognisable per-seed pattern so a test can tell a byte-exact
// round trip from a corrupted one - but with sane geometry and battery fields
// patched in, since parseMcconfGeometry()/parseMcconfBattery() range-check
// those and the link keeps re-polling until both pass. Writes the exact blob
// sent into `fieldsOut[MCCONF_LEN]`.
static void injectMcconfReply(uint8_t seed, uint8_t *fieldsOut, uint16_t len = MCCONF_LEN) {
    for (uint16_t i = 0; i < MCCONF_LEN; i++) fieldsOut[i] = (uint8_t)(i * 7 + seed);

    int32_t gi = (int32_t)MCCONF_OFF_SI_GEAR_RATIO;
    fieldsOut[MCCONF_OFF_SI_MOTOR_POLES] = 30;
    bufAppendFloat32Auto(fieldsOut, 1.0f, &gi);          // gear ratio
    bufAppendFloat32Auto(fieldsOut, 0.584f, &gi);        // wheel diameter, right behind it

    int32_t bi = (int32_t)MCCONF_OFF_SI_BATTERY_AH;
    fieldsOut[MCCONF_OFF_SI_BATTERY_TYPE]  = MCCONF_BATTERY_TYPE_LIFEPO4;
    fieldsOut[MCCONF_OFF_SI_BATTERY_CELLS] = 15;
    bufAppendFloat32Auto(fieldsOut, 6.0f, &bi);

    std::vector<uint8_t> reply(1 + (size_t)len);
    reply[0] = COMM_GET_MCCONF;
    memcpy(reply.data() + 1, fieldsOut, len);

    injectReply(reply.data(), (uint16_t)reply.size());
}

// ---------------------------------------------------------------------------
int main() {
    printf("--- headless integration test ---\n\n");

    vescLink.begin();
    dashStats.begin();
    profiles::begin();
    clearTx();

    // =====================================================================
    // 0. app_mode (COMM_SET_APPCONF_NO_STORE) fetch-patch-resend round trip.
    //    Regression coverage for two things that are easy to get wrong:
    //    (a) this command carries the VESC's *entire* app config, not a
    //    delta, so everything except the patched byte must survive intact;
    //    (b) the fetch is deliberately NOT cached - every switch re-fetches
    //    fresh, so a setting changed elsewhere (e.g. VESC Tool) between
    //    switches is preserved, not reverted to a stale snapshot.
    // =====================================================================
#if VESC_LINK_UART
    // =====================================================================
    // 0 (UART). app_mode over UART: the same fetch-patch-resend as CAN, but
    //    the byte written is always an app that keeps the COMM UART running -
    //    throttle on (ADC) as ADC+UART, throttle off (NONE) as UART. Anything
    //    else would cut the very link the dash is talking over.
    // =====================================================================
    {
        RiderProfile rp{};
        rp.name          = "TEST";
        rp.current_scale = 1.0f;
        rp.brake_scale   = 1.0f;
        rp.speed_kph     = 0.0f;
        rp.watt_max      = 0.0f;
        rp.app_mode      = APP_MODE_ADC;

        // Round trip one switch and return what was written back, or an
        // empty vector if no write went out.
        auto switchTo = [&](AppMode mode, uint8_t seed, uint8_t *fields) {
            rp.app_mode = mode;
            clearTx();
            vescLink.applyProfile(rp);
            const auto req = takeAllSentPayloads();
            CHECK(findPayload(req, COMM_SET_MCCONF_TEMP_SETUP) != nullptr,
                  "UART: a profile still sends its limits");
            CHECK(findPayload(req, COMM_GET_APPCONF) != nullptr,
                  "UART: a profile fetches the app config for its app_mode");
            injectAppConfReply(seed, fields);
            clearTx();
            vescLink.loop();
            const auto out = takeAllSentPayloads();
            const auto *w = findPayload(out, COMM_SET_APPCONF_NO_STORE);
            return w ? *w : std::vector<uint8_t>();
        };
        auto restUnchanged = [](const std::vector<uint8_t> &w, const uint8_t *fields) {
            if (w.size() != 1 + APPCONF_LEN) return false;
            for (uint16_t i = 0; i < APPCONF_LEN; i++) {
                if (i != APPCONF_OFF_APP_TO_USE && w[1 + i] != fields[i]) return false;
            }
            return true;
        };

        uint8_t fields[APPCONF_LEN];
        std::vector<uint8_t> w = switchTo(APP_MODE_ADC, 1, fields);
        CHECK(w.size() == 1 + APPCONF_LEN && w[1 + APPCONF_OFF_APP_TO_USE] == APP_USE_ADC_UART,
              "UART: throttle on is written as ADC + UART, keeping the link");
        CHECK(restUnchanged(w, fields), "UART: every other appconf byte survives the round trip");

        w = switchTo(APP_MODE_NONE, 2, fields);
        CHECK(w.size() == 1 + APPCONF_LEN && w[1 + APPCONF_OFF_APP_TO_USE] == APP_USE_UART,
              "UART: throttle off is written as UART only, keeping the link");
        CHECK(restUnchanged(w, fields), "UART: ...and the rest of that blob survives too");

        // Switched while VESC Tool is connected: the limits go out at once,
        // but the app config waits until the phone has gone - a config reply
        // could be the phone's, relayed from another VESC.
        vescLink.setBleConnected(true);
        rp.app_mode = APP_MODE_ADC;
        clearTx();
        vescLink.applyProfile(rp);
        auto req = takeAllSentPayloads();
        CHECK(findPayload(req, COMM_SET_MCCONF_TEMP_SETUP) != nullptr,
              "UART: limits still apply while VESC Tool is connected");
        CHECK(findPayload(req, COMM_GET_APPCONF) == nullptr,
              "UART: no app config fetch while VESC Tool is connected");
        vescLink.loop();
        CHECK(findPayload(takeAllSentPayloads(), COMM_GET_APPCONF) == nullptr,
              "UART: ...not even from loop()");

        vescLink.setBleConnected(false);
        clearTx();
        vescLink.loop();
        CHECK(findPayload(takeAllSentPayloads(), COMM_GET_APPCONF) != nullptr,
              "UART: the deferred switch fetches once VESC Tool disconnects");
        injectAppConfReply(3, fields);
        clearTx();
        vescLink.loop();
        const auto lateOut = takeAllSentPayloads();
        const auto *late = findPayload(lateOut, COMM_SET_APPCONF_NO_STORE);
        CHECK(late && late->size() == 1 + APPCONF_LEN &&
              (*late)[1 + APPCONF_OFF_APP_TO_USE] == APP_USE_ADC_UART,
              "UART: ...and writes the mode it was asked for");

        // VESC Tool connects while a fetch is already in flight: the answer
        // isn't trusted, and is fetched again once the phone has gone.
        rp.app_mode = APP_MODE_NONE;
        clearTx();
        vescLink.applyProfile(rp);
        CHECK(findPayload(takeAllSentPayloads(), COMM_GET_APPCONF) != nullptr,
              "UART: mid-fetch test: the fetch goes out before VESC Tool connects");
        vescLink.setBleConnected(true);
        injectAppConfReply(4, fields);
        clearTx();
        vescLink.loop();
        CHECK(findPayload(takeAllSentPayloads(), COMM_SET_APPCONF_NO_STORE) == nullptr,
              "UART: a fetch answered after VESC Tool connected is not written back");

        vescLink.setBleConnected(false);
        clearTx();
        vescLink.loop();
        CHECK(findPayload(takeAllSentPayloads(), COMM_GET_APPCONF) != nullptr,
              "UART: ...it is fetched again once VESC Tool disconnects");
        injectAppConfReply(5, fields);
        clearTx();
        vescLink.loop();
        const auto againOut = takeAllSentPayloads();
        const auto *again = findPayload(againOut, COMM_SET_APPCONF_NO_STORE);
        CHECK(again && again->size() == 1 + APPCONF_LEN &&
              (*again)[1 + APPCONF_OFF_APP_TO_USE] == APP_USE_UART,
              "UART: ...and then written with the requested mode");
    }
#else
    {
        RiderProfile rp{};
        rp.name          = "TEST";
        rp.current_scale = 1.0f;
        rp.brake_scale   = 1.0f;
        rp.speed_kph     = 0.0f;
        rp.watt_max      = 0.0f;
        rp.app_mode      = APP_MODE_ADC_PAS;

        // Every switch requests a fresh app config and defers, not crash or
        // silently drop the change.
        clearTx();
        vescLink.applyProfile(rp);

        std::vector<uint8_t> mc, ac;
        CHECK(takeSentPayload(mc), "profile apply still sends the mcconf limits");
        CHECK(takeSentPayload(ac), "profile apply also requests the app config");
        CHECK(!ac.empty() && ac[0] == COMM_GET_APPCONF, "app_mode request uses COMM_GET_APPCONF");

        uint8_t fields1[APPCONF_LEN];
        injectAppConfReply(1, fields1);
        clearTx();
        vescLink.loop();

        std::vector<uint8_t> sent;
        CHECK(takeSentPayload(sent), "the fetched app config flushes the pending app_mode change");
        CHECK(sent.size() == 1 + APPCONF_LEN, "SET_APPCONF_NO_STORE carries the full blob");
        CHECK(!sent.empty() && sent[0] == COMM_SET_APPCONF_NO_STORE, "uses COMM_SET_APPCONF_NO_STORE");
        CHECK(sent.size() == 1 + APPCONF_LEN &&
              sent[1 + APPCONF_OFF_APP_TO_USE] == (uint8_t)APP_MODE_ADC_PAS,
              "app_mode byte is patched to the requested value");

        bool restUnchanged = (sent.size() == 1 + APPCONF_LEN);
        for (uint16_t i = 0; restUnchanged && i < APPCONF_LEN; i++) {
            if (i == APPCONF_OFF_APP_TO_USE) continue;
            if (sent[1 + i] != fields1[i]) restUnchanged = false;
        }
        CHECK(restUnchanged, "every other appconf byte survives the round trip untouched");

        // A second, different app_mode must re-fetch, not reuse a cache -
        // and the freshly fetched blob (a different pattern this time,
        // standing in for "something changed via VESC Tool meanwhile") is
        // what comes back, not the first fetch's stale copy.
        rp.app_mode = APP_MODE_PAS;
        clearTx();
        vescLink.applyProfile(rp);

        std::vector<uint8_t> mc2, ac2;
        CHECK(takeSentPayload(mc2), "second apply still sends the mcconf limits");
        CHECK(takeSentPayload(ac2), "second app_mode change re-fetches the app config");
        CHECK(!ac2.empty() && ac2[0] == COMM_GET_APPCONF, "second fetch also uses COMM_GET_APPCONF");

        uint8_t fields2[APPCONF_LEN];
        injectAppConfReply(2, fields2);
        clearTx();
        vescLink.loop();

        std::vector<uint8_t> sent2;
        CHECK(takeSentPayload(sent2), "second fetch flushes the second app_mode change");
        CHECK(sent2.size() == 1 + APPCONF_LEN &&
              sent2[1 + APPCONF_OFF_APP_TO_USE] == (uint8_t)APP_MODE_PAS,
              "second app_mode change patches the new value");

        bool restUnchanged2 = (sent2.size() == 1 + APPCONF_LEN);
        for (uint16_t i = 0; restUnchanged2 && i < APPCONF_LEN; i++) {
            if (i == APPCONF_OFF_APP_TO_USE) continue;
            if (sent2[1 + i] != fields2[i]) restUnchanged2 = false;
        }
        CHECK(restUnchanged2, "second round trip reflects the freshly fetched blob, not the first fetch");

        // Two switches in quick succession, before the first fetch answers:
        // must coalesce into one request, and the *later* mode must win.
        rp.app_mode = APP_MODE_ADC;
        clearTx();
        vescLink.applyProfile(rp);
        std::vector<uint8_t> mc3, ac3;
        CHECK(takeSentPayload(mc3), "coalescing test: first switch sends mcconf limits");
        CHECK(takeSentPayload(ac3), "coalescing test: first switch requests the app config");

        rp.app_mode = APP_MODE_NONE;
        clearTx();
        vescLink.applyProfile(rp);
        std::vector<uint8_t> mc4;
        CHECK(takeSentPayload(mc4), "coalescing test: second switch still sends mcconf limits");
        std::vector<uint8_t> ac4;
        CHECK(!takeSentPayload(ac4),
              "coalescing test: second switch does not re-fetch while one is already in flight");

        uint8_t fields3[APPCONF_LEN];
        injectAppConfReply(3, fields3);
        clearTx();
        vescLink.loop();

        std::vector<uint8_t> sent3;
        CHECK(takeSentPayload(sent3), "coalescing test: the single fetch flushes once it answers");
        CHECK(sent3.size() == 1 + APPCONF_LEN &&
              sent3[1 + APPCONF_OFF_APP_TO_USE] == (uint8_t)APP_MODE_NONE,
              "coalescing test: the later of the two coalesced modes wins");
    }

#endif
    clearTx();

    // =====================================================================
    // 1. What a telemetry poll actually puts on the wire
    // =====================================================================
    vescLink.requestValues();
#if !VESC_LINK_UART
    CHECK(g_tx.size() == 1, "CAN poll is a single frame");
#endif
    std::vector<uint8_t> pl;
    CHECK(takeSentPayload(pl), "poll produced a valid tunnelled payload");
    CHECK(pl.size() == 5, "poll payload is id + 32-bit mask");
    CHECK(pl[0] == COMM_GET_VALUES_SELECTIVE, "poll uses GET_VALUES_SELECTIVE");
    {
        int32_t i = 1;
        uint32_t mask = (uint32_t)bufGetInt32(pl.data(), &i);
#if VESC_LINK_UART
        // No broadcasts over UART: the poll is all of the telemetry.
        CHECK(mask == DASH_VALUE_MASK, "poll carries the dashboard mask");
        CHECK((mask & VAL_RPM) && (mask & VAL_V_IN) && (mask & VAL_FAULT),
              "mask includes rpm, voltage and fault");
        CHECK(!(mask & VAL_PID_POS), "mask excludes fields the dash never shows");
#else
        // The broadcasts carry everything else - see section 3 for why the
        // poll must not ask for any of it too.
        CHECK(mask == VAL_FAULT, "CAN poll asks for the fault code only");
#endif
    }

    // =====================================================================
    // 2. What a profile change actually puts on the wire
    // =====================================================================
    clearTx();
    profiles::set(2);                                    // TOUR: has speed and power limits
    CHECK(takeSentPayload(pl), "profile change produced a valid tunnelled payload");
    CHECK(pl[0] == COMM_SET_MCCONF_TEMP_SETUP, "uses COMM_SET_MCCONF_TEMP_SETUP");
    CHECK(pl.size() == 5 + 8 * 4, "payload is 4 flags + 8 floats");
    {
        int32_t i = 1;
        const uint8_t store = pl[i++];
        const uint8_t fwdCan = pl[i++];
        const uint8_t ack = pl[i++];
        const uint8_t divide = pl[i++];
        CHECK(store == 0, "store flag is 0: the VESC's saved config is untouched");
        CHECK(fwdCan == (PROFILE_FORWARD_CAN ? 1 : 0), "forward_can flag matches config");
        CHECK(ack == 0 && divide == 0, "ack / divide_by_controllers are 0");

        const float brake   = readFloat32Auto(pl.data(), &i);
        const float current = readFloat32Auto(pl.data(), &i);
        const float speedMin = readFloat32Auto(pl.data(), &i);
        const float speedMax = readFloat32Auto(pl.data(), &i);
        const float dutyMin  = readFloat32Auto(pl.data(), &i);
        const float dutyMax  = readFloat32Auto(pl.data(), &i);
        const float wattMin  = readFloat32Auto(pl.data(), &i);
        const float wattMax  = readFloat32Auto(pl.data(), &i);

        const RiderProfile &tour = RIDER_PROFILES[2];
        CHECK(NEAR(brake, tour.brake_scale, 1e-5), "brake scale encodes correctly");
        CHECK(NEAR(current, tour.current_scale, 1e-5), "current scale encodes correctly");
        CHECK(NEAR(speedMax, tour.speed_kph / 3.6f, 1e-3), "speed limit converted to m/s");
        CHECK(NEAR(speedMin, -tour.speed_kph / 3.6f, 1e-3), "reverse limit mirrors forward");
        CHECK(dutyMin > 0.0f && dutyMax > 0.9f, "duty limits left permissive");
        CHECK(NEAR(wattMax, tour.watt_max, 1.0f), "power limit encodes correctly");
        CHECK(NEAR(wattMin, -tour.watt_max, 1.0f), "regen power limit mirrors drive");
        printf("  info: TOUR -> cur=%.2f brk=%.2f  %.2f m/s  %.0f W\n",
               current, brake, speedMax, wattMax);
    }

    // an unlimited profile must not send a literal zero limit
    clearTx();
    profiles::set(3);                                    // SPORT: 0 = no limit
    CHECK(takeSentPayload(pl), "unlimited profile framed");
    {
        int32_t i = 5 + 3 * 4;                          // skip brake, current, speed_min
        const float speedMax = readFloat32Auto(pl.data(), &i);
        i += 2 * 4;                                     // skip duty min/max
        const float wattMin = readFloat32Auto(pl.data(), &i);
        const float wattMax = readFloat32Auto(pl.data(), &i);
        CHECK(speedMax > 100.0f, "unlimited speed sends a large value, not 0");
        CHECK(wattMax > 1e5f && wattMin < -1e5f, "unlimited power sends a large value, not 0");
    }
    profiles::set(1);

    // =====================================================================
    // 2b. Persisting a profile (COMM_SET_MCCONF) - the one path that writes
    //     the VESC's flash. What matters here is that it is a byte-exact
    //     round trip of the VESC's own live config: the dash must never
    //     invent or patch motor-config bytes, because every field it doesn't
    //     understand is one it could permanently corrupt.
    // =====================================================================
    {
        clearTx();
        clearRx();

        vescLink.storeConfig();
        const auto fetched = takeAllSentPayloads();
        const auto *req = findPayload(fetched, COMM_GET_MCCONF);
        CHECK(req != nullptr, "a write request fetches the live config first (COMM_GET_MCCONF)");
        CHECK(req && req->size() == 1, "the fetch is just the command id");

        uint8_t live[MCCONF_LEN];
        injectMcconfReply(9, live);
        clearTx();
        vescLink.loop();

        const auto all = takeAllSentPayloads();
        const auto *sent = findPayload(all, COMM_SET_MCCONF);
        CHECK(sent != nullptr, "the fetched config is written straight back (COMM_SET_MCCONF)");
        CHECK(sent && sent->size() == 1 + MCCONF_LEN, "the write carries the full mcconf blob");

        bool identical = (sent && sent->size() == 1 + MCCONF_LEN);
        for (uint16_t i = 0; identical && i < MCCONF_LEN; i++) {
            if ((*sent)[1 + i] != live[i]) identical = false;
        }
        CHECK(identical, "every mcconf byte round trips untouched - nothing is patched");

        // The reply also still feeds the battery gauge and the speed/distance
        // maths - one fetch, every use.
        CHECK(vescLink.batteryConfig().valid, "the same reply still populates the battery config");
        const GeometryConfig g = vescLink.geometryConfig();
        CHECK(g.valid, "the same reply populates the geometry");
        CHECK(g.motorPoles == 30 && NEAR(g.gearRatio, 1.0f, 0.001f) &&
              NEAR(g.wheelDiameterM, 0.584f, 0.0001f),
              "geometry decoded from si_motor_poles/_gear_ratio/_wheel_diameter");

        // One request writes flash exactly once: a second reply arriving
        // behind the first (a retry that raced) must not write again.
        clearTx();
        uint8_t live2[MCCONF_LEN];
        injectMcconfReply(11, live2);
        vescLink.loop();
        CHECK(findPayload(takeAllSentPayloads(), COMM_SET_MCCONF) == nullptr,
              "a second config reply does not write flash again");
    }

    // A reply whose length does not match this firmware's mcconf must be
    // skipped, not written - a mis-sized blob is exactly the case where
    // guessing would brick the motor config.
    {
        clearTx();
        clearRx();
        vescLink.storeConfig();
        CHECK(findPayload(takeAllSentPayloads(), COMM_GET_MCCONF) != nullptr,
              "second write request also fetches first");

        uint8_t shortLive[MCCONF_LEN];
        injectMcconfReply(13, shortLive, MCCONF_LEN - 4);   // wrong-size firmware
        clearTx();
        vescLink.loop();
        CHECK(findPayload(takeAllSentPayloads(), COMM_SET_MCCONF) == nullptr,
              "a wrong-size mcconf reply is never written back");

        // ...and the request is dropped, not left armed to fire on the next
        // unrelated fetch minutes later.
        clearTx();
        uint8_t goodLive[MCCONF_LEN];
        injectMcconfReply(14, goodLive);
        vescLink.loop();
        CHECK(findPayload(takeAllSentPayloads(), COMM_SET_MCCONF) == nullptr,
              "a rejected write does not stay armed for a later config reply");
    }

    // No reply at all: retried a few times, then dropped. A write must not
    // fire minutes late, possibly while moving again.
    {
        clearTx();
        clearRx();
        vescLink.storeConfig();
        CHECK(findPayload(takeAllSentPayloads(), COMM_GET_MCCONF) != nullptr,
              "third write request fetches first");

        int retries = 0;
        for (int k = 0; k < 6; k++) {
            g_millis += 2100;
            clearTx();
            vescLink.loop();
            if (findPayload(takeAllSentPayloads(), COMM_GET_MCCONF)) retries++;
        }
        CHECK(retries > 0 && retries < 6, "an unanswered write retries, but gives up");
        printf("  info: unanswered write retried %d time(s) before giving up\n", retries);

        // Whatever arrives after it gave up is not turned into a write.
        clearTx();
        uint8_t lateLive[MCCONF_LEN];
        injectMcconfReply(15, lateLive);
        vescLink.loop();
        CHECK(findPayload(takeAllSentPayloads(), COMM_SET_MCCONF) == nullptr,
              "a config reply after giving up does not write flash");
    }

    // The VESC's acknowledgement is what tells the rider it actually landed,
    // and it is one-shot - a toast must not repeat on every later frame.
    {
        clearTx();
        clearRx();
        CHECK(!vescLink.takeStoreAck(), "no ack reported before the VESC answers");

        vescLink.storeConfig();
        uint8_t live[MCCONF_LEN];
        injectMcconfReply(16, live);
        vescLink.loop();
        CHECK(findPayload(takeAllSentPayloads(), COMM_SET_MCCONF) != nullptr,
              "ack test: the write went out");
        CHECK(!vescLink.takeStoreAck(), "sending the write is not the VESC confirming it");

        const uint8_t ackPayload[1] = { COMM_SET_MCCONF };
        injectReply(ackPayload, 1);
        vescLink.loop();

        CHECK(vescLink.takeStoreAck(), "the VESC's COMM_SET_MCCONF echo is reported once");
        CHECK(!vescLink.takeStoreAck(), "...and only once");
    }

    clearTx();
    clearRx();

#if !VESC_LINK_UART
    // =====================================================================
    // 3. A VESC reply is decoded through the real link path
    // =====================================================================
    clearRx();
    injectStatusBroadcast(52.0f, 4200.0f, 25.0f, 0, 0.0f);
    g_millis = 1000;
    vescLink.loop();
    {
        Telemetry t = vescLink.snapshot();
        CHECK(NEAR(t.voltage, 52.0f, 0.05f), "voltage decoded through the link");
        CHECK(NEAR(t.erpm, 4200.0f, 0.5f), "erpm decoded through the link");
        CHECK(NEAR(t.currentIn, 25.0f, 0.02f), "input current decoded");
        CHECK(vescLink.isAlive(), "link reports alive after a fresh status broadcast");
        g_millis += TELEMETRY_STALE_MS + 100;
        CHECK(!vescLink.isAlive(), "link goes stale when packets stop");
    }

    // the fault code has no broadcast frame - it arrives via the tunnelled
    // fallback poll, decoded through the same handleFrame() path
    clearRx();
    injectFaultReply(5);
    vescLink.loop();
    CHECK(vescLink.snapshot().faultCode == 5, "tunnelled fault reply decoded through the link");

    // status broadcasts trickling in one CAN frame per loop() still reassemble
    clearRx();
    injectStatusBroadcast(48.0f, 1000.0f, 5.0f, 100, 1.0f);
    {
        std::deque<CanFrame> all = g_rx;
        clearRx();
        g_millis += 100;
        while (!all.empty()) {                            // one frame per loop()
            g_rx.push_back(all.front());
            all.pop_front();
            vescLink.loop();
        }
        Telemetry t = vescLink.snapshot();
        CHECK(NEAR(t.voltage, 48.0f, 0.05f), "status frames trickling in one at a time reassemble");
    }

    // a status broadcast from an unrelated CAN node must not corrupt telemetry
    {
        CanFrame foreign{};
        foreign.id = ((uint32_t)CAN_PACKET_STATUS_5 << 8) | (uint8_t)(VESC_CAN_TARGET_ID + 1);
        int32_t i = 0;
        bufAppendInt32(foreign.data, 999999, &i);
        bufAppendInt16(foreign.data, (int16_t)(9.9f * 10), &i);
        foreign.dlc = 6;
        g_rx.push_back(foreign);
        vescLink.loop();
        CHECK(NEAR(vescLink.snapshot().voltage, 48.0f, 0.05f), "status from a different CAN id ignored");
    }

    // The poll's reply must never write a field the broadcasts own.
    // Regression test: the poll used to ask for every field, so STATUS_5's
    // signed tachometer and the poll's real tachometer_abs took turns in
    // tachometerAbs, and each swap added their difference to the odometer -
    // even parked. Answer the dash's actual request the way the VESC would,
    // with values unlike the broadcasts', and nothing they set may move.
    {
        clearRx();
        injectStatusBroadcast(48.0f, 1000.0f, 5.0f, 100, 1.0f);
        vescLink.loop();
        clearTx();                                        // whatever loop() sent
        vescLink.requestValues();
        std::vector<uint8_t> req;
        CHECK(takeSentPayload(req) && req.size() == 5, "poll request captured");
        int32_t i = 1;
        const uint32_t asked = (uint32_t)bufGetInt32(req.data(), &i);
        const auto r = valuesReply(asked, 11.0f, 9999.0f, 99.0f, 5000, 99.0f, 4);
        injectReply(r.data(), (uint16_t)r.size());
        vescLink.loop();
        const Telemetry t = vescLink.snapshot();
        CHECK(t.faultCode == 4, "poll reply's fault code decoded");
        CHECK(t.tachometerAbs == 100, "poll reply leaves the broadcast tachometer alone");
        CHECK(NEAR(t.voltage, 48.0f, 0.05f) && NEAR(t.erpm, 1000.0f, 0.5f),
              "poll reply leaves the other broadcast fields alone");
    }

    // =====================================================================
    // 3c. BLE-forwarded traffic must be fully isolated from the dash's own
    //     telemetry. Regression test for a real bug: the dash's own poll and
    //     BLE-forwarded phone requests used to share one CAN id, so VESC
    //     Tool's Real Time Data replies were bleeding into the display.
    // =====================================================================
    vescLink.setByteSink(captureSink);
    vescLink.setBleConnected(true);

    // The phone asks for a full COMM_GET_VALUES over the BLE bridge.
    {
        const uint8_t req[1] = { COMM_GET_VALUES };
        uint8_t reqFrame[16];
        const uint16_t reqN = buildFrame(req, 1, reqFrame, sizeof(reqFrame));
        clearTx();
        vescLink.writeRaw(reqFrame, reqN);

        CanTunnel rx;
        bool got = false;
        while (!g_tx.empty()) {
            CanFrame f = g_tx.front();
            g_tx.erase(g_tx.begin());
            if (rx.feed(f, VESC_CAN_TARGET_ID)) { got = true; break; }
        }
        CHECK(got, "BLE-forwarded request produced a valid tunnelled payload");
        CHECK(rx.sourceId() == VESC_CAN_BLE_ID,
              "BLE-forwarded request claims VESC_CAN_BLE_ID, not the dash's own poll id");
        CHECK(rx.payload()[0] == COMM_GET_VALUES, "forwarded payload is the phone's own request");
    }

    // The VESC replies with a full GET_VALUES carrying deliberately wild
    // values, tunnelled back to VESC_CAN_BLE_ID - as a real Real Time Data
    // reply would be.
    {
        const Telemetry before = vescLink.snapshot();

        const std::vector<uint8_t> wild = wildValuesReply();
        CanFrame frames[16];
        const int n = buildTunnelFrames(VESC_CAN_BLE_ID, VESC_CAN_TARGET_ID,
                                         wild.data(), (uint16_t)wild.size(), false, frames, 16);
        for (int k = 0; k < n; k++) g_rx.push_back(frames[k]);

        g_bleOut.clear();
        vescLink.loop();

        const Telemetry after = vescLink.snapshot();
        CHECK(NEAR(after.voltage, before.voltage, 0.05f),
              "a BLE reply must not change the dash's own voltage");
        CHECK(after.tachometerAbs == before.tachometerAbs,
              "a BLE reply must not change the dash's own tachometer");
        CHECK(after.faultCode == before.faultCode,
              "a BLE reply must not change the dash's own fault code");

        FrameSniffer sn;
        bool gotFramed = false;
        for (uint8_t b : g_bleOut) { if (sn.feed(b)) { gotFramed = true; break; } }
        CHECK(gotFramed, "the BLE reply was still forwarded out to the phone");
        CHECK(gotFramed && sn.payload()[0] == COMM_GET_VALUES,
              "the forwarded reply carries the GET_VALUES id");
    }

    // Conversely, the dash's own poll reply must not leak out to the phone.
    {
        g_bleOut.clear();
        injectFaultReply(3);
        vescLink.loop();
        CHECK(vescLink.snapshot().faultCode == 3, "the dash's own poll reply still updates telemetry");
        CHECK(g_bleOut.empty(), "the dash's own poll reply must not leak out to the phone");
    }

    vescLink.setBleConnected(false);
#else
    // =====================================================================
    // 3. Over UART the poll IS the telemetry: there are no broadcasts, so a
    //    reply to the dash's own request is the only thing that moves the
    //    display.
    // =====================================================================
    // Past the link's 3 s reply timeout, so nothing asked before this point
    // can claim what comes next.
    g_millis += 3100;
    clearTx();
    clearRx();
    {
        vescLink.requestValues();
        const auto r = valuesReply(DASH_VALUE_MASK, 52.0f, 4200.0f, 25.0f, 0, 0.0f, 0);
        injectReply(r.data(), (uint16_t)r.size());
        vescLink.loop();
        Telemetry t = vescLink.snapshot();
        CHECK(NEAR(t.voltage, 52.0f, 0.05f), "UART: voltage decoded from the poll reply");
        CHECK(NEAR(t.erpm, 4200.0f, 0.5f), "UART: erpm decoded from the poll reply");
        CHECK(NEAR(t.currentIn, 25.0f, 0.02f), "UART: input current decoded");
        CHECK(vescLink.isAlive(), "UART: link reports alive after a fresh reply");
        g_millis += TELEMETRY_STALE_MS + 100;
        CHECK(!vescLink.isAlive(), "UART: link goes stale when replies stop");
    }

    // the fault code rides in the same reply
    {
        vescLink.requestValues();
        const auto r = valuesReply(DASH_VALUE_MASK, 52.0f, 4200.0f, 25.0f, 0, 0.0f, 5);
        injectReply(r.data(), (uint16_t)r.size());
        vescLink.loop();
        CHECK(vescLink.snapshot().faultCode == 5, "UART: fault code decoded from the poll reply");
    }

    // a reply trickling in one byte per loop() still reassembles
    {
        vescLink.requestValues();
        const auto r = valuesReply(DASH_VALUE_MASK, 48.0f, 1000.0f, 5.0f, 100, 1.0f, 0);
        injectReply(r.data(), (uint16_t)r.size());
        std::deque<uint8_t> all = g_uartRx;
        clearRx();
        while (!all.empty()) {                            // one byte per loop()
            g_uartRx.push_back(all.front());
            all.pop_front();
            vescLink.loop();
        }
        CHECK(NEAR(vescLink.snapshot().voltage, 48.0f, 0.05f),
              "UART: a reply arriving one byte at a time reassembles");
    }

    // A reply to nothing - no request outstanding, or only ones past their
    // timeout - is dropped, not decoded.
    {
        g_millis += 3100;
        const auto r = valuesReply(DASH_VALUE_MASK, 11.0f, 99.0f, 1.0f, 0, 0.0f, 0);
        injectReply(r.data(), (uint16_t)r.size());
        vescLink.loop();
        CHECK(NEAR(vescLink.snapshot().voltage, 48.0f, 0.05f),
              "UART: a reply nobody is waiting for is dropped");
    }

    // =====================================================================
    // 3c. The BLE bridge over UART. There are no ids to keep the phone's
    //     traffic apart from the dash's, so the link does it by remembering
    //     what the dash itself asked for.
    // =====================================================================
    vescLink.setByteSink(captureSink);
    vescLink.setBleConnected(true);
    g_millis += 3100;                               // start with nothing outstanding

    // A phone request goes out byte-for-byte as VESC Tool framed it.
    {
        const uint8_t req[1] = { COMM_GET_VALUES };
        uint8_t reqFrame[16];
        const uint16_t reqN = buildFrame(req, 1, reqFrame, sizeof(reqFrame));
        clearTx();
        vescLink.writeRaw(reqFrame, reqN);
        CHECK(g_uartTx == std::vector<uint8_t>(reqFrame, reqFrame + reqN),
              "UART: a phone request goes out byte-for-byte");
    }

    // A phone frame split across two BLE writes, with a dash poll going out
    // in between, still reaches the VESC in one piece.
    {
        uint8_t big[120];
        big[0] = 20;                                    // COMM_TERMINAL_CMD - any id will do
        for (int k = 1; k < (int)sizeof(big); k++) big[k] = (uint8_t)k;
        uint8_t bigFrame[128];
        const uint16_t bigN = buildFrame(big, sizeof(big), bigFrame, sizeof(bigFrame));

        clearTx();
        vescLink.writeRaw(bigFrame, bigN / 2);
        vescLink.requestValues();
        vescLink.writeRaw(bigFrame + bigN / 2, bigN - bigN / 2);

        const auto all = takeAllSentPayloads();
        CHECK(all.size() == 2, "UART: a split phone frame and a dash poll both arrive intact");
        CHECK(all.size() == 2 && all[0][0] == COMM_GET_VALUES_SELECTIVE &&
              all[1] == std::vector<uint8_t>(big, big + sizeof(big)),
              "UART: the dash's poll never lands inside the phone's frame");
    }

    // The VESC's answer to the phone goes to the phone, never the display.
    {
        const Telemetry before = vescLink.snapshot();
        const std::vector<uint8_t> wild = wildValuesReply();
        injectReply(wild.data(), (uint16_t)wild.size());
        g_bleOut.clear();
        vescLink.loop();

        const Telemetry after = vescLink.snapshot();
        CHECK(NEAR(after.voltage, before.voltage, 0.05f) &&
              after.tachometerAbs == before.tachometerAbs &&
              after.faultCode == before.faultCode,
              "UART: a reply to the phone must not change the dash's telemetry");
        CHECK(bleGot(COMM_GET_VALUES), "UART: the phone's reply is forwarded to it");
    }

    // The dash's own poll reply feeds the dash and never leaks to the phone.
    {
        vescLink.requestValues();
        const auto r = valuesReply(DASH_VALUE_MASK, 48.0f, 1000.0f, 5.0f, 100, 1.0f, 3);
        injectReply(r.data(), (uint16_t)r.size());
        g_bleOut.clear();
        vescLink.loop();
        CHECK(vescLink.snapshot().faultCode == 3, "UART: the dash's own poll reply updates telemetry");
        CHECK(g_bleOut.empty(), "UART: the dash's own poll reply must not leak to the phone");
    }

    // A selective reply with someone else's mask is the phone's, even while
    // the dash is waiting on a selective reply of its own.
    {
        vescLink.requestValues();
        const Telemetry before = vescLink.snapshot();
        const auto theirs = valuesReply(VAL_V_IN | VAL_FAULT, 11.0f, 0.0f, 0.0f, 0, 0.0f, 7);
        injectReply(theirs.data(), (uint16_t)theirs.size());
        g_bleOut.clear();
        vescLink.loop();
        CHECK(vescLink.snapshot().faultCode == before.faultCode &&
              NEAR(vescLink.snapshot().voltage, before.voltage, 0.05f),
              "UART: a selective reply with another mask is not the dash's");
        CHECK(bleGot(COMM_GET_VALUES_SELECTIVE), "UART: ...it goes to the phone instead");
    }

    // No config traffic of the dash's own while the phone is connected: a
    // reply relayed from another VESC would look just like its own.
    {
        clearTx();
        vescLink.storeConfig();
        CHECK(findPayload(takeAllSentPayloads(), COMM_GET_MCCONF) == nullptr,
              "UART: a save is refused while VESC Tool is connected");

        // ...so any config reply that shows up is the phone's.
        uint8_t cfg[MCCONF_LEN];
        injectMcconfReply(21, cfg);
        g_bleOut.clear();
        vescLink.loop();
        CHECK(bleGot(COMM_GET_MCCONF), "UART: a config reply while connected goes to the phone");
        CHECK(findPayload(takeAllSentPayloads(), COMM_SET_MCCONF) == nullptr,
              "UART: ...and is never written back");
    }

    vescLink.setBleConnected(false);
#endif

    // =====================================================================
    // 4. Ride statistics over a simulated ride
    // =====================================================================
    dashStats.resetTrip();

    // No config.h fallback exists any more: without a VESC-reported battery
    // config, the dash must show the gauge as unknown rather than guess.
    {
        Telemetry unconfigured{};
        unconfigured.voltage = 58.0f; unconfigured.lastUpdateMs = g_millis;
        dashStats.update(unconfigured, true, false, 1);   // no BatteryConfig passed
        CHECK(!dashStats.model().battKnown,
              "no battery config from the VESC -> gauge is unknown, not guessed");
    }

    // The rest of this section exercises the SoC/range math itself, so it
    // needs a stand-in for what COMM_GET_MCCONF would have reported.
    vesc::BatteryConfig testBatt;
    testBatt.cells     = 15;
    testBatt.ah        = 6.0f;
    testBatt.chemistry = 0;      // LiFePO4
    testBatt.valid     = true;

    // No geometry from the VESC: no speed to show and no distance to count -
    // a spinning motor must register as neither.
    {
        const float odoBefore = dashStats.model().odoDist;
        Telemetry spinning{};
        spinning.voltage = 58.0f; spinning.erpm = 4200;
        for (int step = 1; step <= 5; step++) {
            spinning.tachometerAbs = 10000 * step;
            spinning.lastUpdateMs  = g_millis;
            dashStats.update(spinning, true, false, 1, testBatt);   // no GeometryConfig
            g_millis += 100;
        }
        CHECK(!dashStats.model().geomKnown, "no geometry from the VESC -> speed is unknown");
        CHECK(dashStats.model().speed == 0.0f, "...not computed from a guess");
        CHECK(dashStats.model().tripDist == 0.0f &&
              NEAR(dashStats.model().odoDist, odoBefore, 0.0001f),
              "...and no distance is counted");
    }

    // ...and the stand-in for the geometry COMM_GET_MCCONF would have
    // reported: 30 poles, direct drive, 0.584 m wheel - independent of
    // whatever bike config.h happens to be set up for.
    vesc::GeometryConfig testGeom;
    testGeom.motorPoles     = 30;
    testGeom.gearRatio      = 1.0f;
    testGeom.wheelDiameterM = 0.584f;
    testGeom.valid          = true;

    // Tachometer counts for 1 km, using the same scale the firmware uses.
    const float metersPerCount = (testGeom.wheelDiameterM * (float)M_PI) /
                                 (3.0f * (float)testGeom.motorPoles * testGeom.gearRatio);
    const int32_t countsPerKm = (int32_t)(1000.0f / metersPerCount);

    Telemetry t{};
    t.voltage = 58.0f; t.erpm = 0; t.currentIn = 0; t.tachometerAbs = 0;
    t.wattHours = 0;   t.lastUpdateMs = g_millis;

    for (int step = 0; step < 40; step++) {              // settle the SoC filter
        dashStats.update(t, true, false, 1, testBatt, testGeom);
        g_millis += 100;
    }
    CHECK(dashStats.model().battKnown, "battery config from the VESC marks the gauge known");
    CHECK(dashStats.model().batteryPct > 93.0f,
          "full pack (4.14 V/cell) reads near 100%");
    printf("  info: 58.0 V on %dS -> %.0f%%\n", testBatt.cells, dashStats.model().batteryPct);

    // ride 2 km, drawing 20 Wh
    t.erpm = 4200; t.currentIn = 20.0f;
    for (int step = 1; step <= 20; step++) {
        t.tachometerAbs = (int32_t)(countsPerKm * 2.0f * step / 20.0f);
        t.wattHours = 20.0f * step / 20.0f;
        t.lastUpdateMs = g_millis;
        dashStats.update(t, true, false, 1, testBatt, testGeom);
        g_millis += 100;
    }
    const DashModel &m = dashStats.model();
    printf("  info: trip=%.2f km  odo=%.2f km  wh=%.1f  wh/km=%.1f  speed=%.1f\n",
           m.tripDist, m.odoDist, m.whUsed, m.whPerDist, m.speed);
    CHECK(NEAR(m.tripDist, 2.0f, 0.02f), "trip distance matches the tachometer");
    CHECK(NEAR(m.odoDist, 2.0f, 0.02f), "odometer accumulated the same distance");
    CHECK(NEAR(m.whUsed, 20.0f, 0.1f), "energy used tracked");
    CHECK(NEAR(m.whPerDist, 10.0f, 0.2f), "efficiency = 20 Wh / 2 km");
    CHECK(m.speed > 25.0f && m.speed < 35.0f, "speed settled at the right value");
    CHECK(m.maxSpeed >= m.speed - 0.01f, "max speed recorded");

    // A VESC reboot resets its tachometer: the trip must re-baseline, and the
    // odometer must not gain a phantom two kilometres.
    const float odoBefore = dashStats.model().odoDist;
    t.tachometerAbs = 0;
    t.lastUpdateMs = g_millis;
    dashStats.update(t, true, false, 1, testBatt, testGeom);
    g_millis += 100;
    CHECK(NEAR(dashStats.model().odoDist, odoBefore, 0.01f),
          "odometer survives a VESC tachometer reset");
    CHECK(dashStats.model().tripDist < 0.05f, "trip re-baselines after the reset");

    // sag compensation: same pack, heavy load, gauge must not collapse
    {
        dashStats.resetTrip();
        Telemetry loaded{};
        loaded.voltage = 50.0f; loaded.currentIn = 40.0f; loaded.lastUpdateMs = g_millis;
        for (int i = 0; i < 60; i++) { dashStats.update(loaded, true, false, 1, testBatt, testGeom); g_millis += 100; }
        const float sagged = dashStats.model().batteryPct;

        Telemetry idle{};
        idle.voltage = 50.0f; idle.currentIn = 0.0f; idle.lastUpdateMs = g_millis;
        for (int i = 0; i < 60; i++) { dashStats.update(idle, true, false, 1, testBatt, testGeom); g_millis += 100; }
        const float resting = dashStats.model().batteryPct;

        printf("  info: 50.0 V under 40 A -> %.0f%%, at rest -> %.0f%%\n", sagged, resting);
        CHECK(sagged > resting, "sag compensation reads higher under load");
        CHECK(sagged - resting < 30.0f, "sag compensation stays proportionate");
    }

    // stale link blanks the live figures but keeps the trip
    {
        const float tripBefore = dashStats.model().tripDist;
        Telemetry dead{};
        dashStats.update(dead, false, false, 1);
        CHECK(dashStats.model().speed == 0.0f, "no link -> speed reads zero");
        CHECK(NEAR(dashStats.model().tripDist, tripBefore, 0.001f),
              "no link -> trip is preserved, not zeroed");
    }

    printf(failures ? "\n%d FAILURE(S)\n" : "\nall integration tests passed\n", failures);
    return failures ? 1 : 0;
}
