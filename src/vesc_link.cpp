#include "vesc_link.h"

using namespace vesc;

VescLink vescLink;

// ---------------------------------------------------------------------------
void VescLink::begin() {
    _dataMutex = xSemaphoreCreateMutex();
    _txMutex   = xSemaphoreCreateMutex();

#if VESC_LINK_UART
    _awaitMutex = xSemaphoreCreateMutex();
    vescuart::begin();
#else
    vescan::begin();
#endif
}

// ---------------------------------------------------------------------------
void VescLink::loop() {
#if VESC_LINK_UART
    // ---- UART -> (dash | BLE) ---------------------------------------------
    uint8_t chunk[256];
    size_t got;
    while ((got = vescuart::read(chunk, sizeof(chunk))) > 0) {
        for (size_t i = 0; i < got; i++) {
            if (_uartSniffer.feed(chunk[i])) {
                routeReply(_uartSniffer.payload(), _uartSniffer.length());
            }
        }
    }
#else
    // ---- CAN -> (telemetry | tunnel | BLE) --------------------------------
    CanFrame f;
    while (vescan::poll(f)) {
        const uint8_t packetId = canEidPacketId(f.id);
        const uint8_t addr     = canEidAddress(f.id);

        // Passive telemetry: status broadcasts from the VESC we care about.
        // Each status id carries a fixed, non-overlapping set of fields, so
        // there is nothing to merge - just write straight into _data.
        if (addr == VESC_CAN_TARGET_ID) {
            bool handled = false;
            if (xSemaphoreTake(_dataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                handled = parseCanStatus(packetId, f.data, f.dlc, _data);
                xSemaphoreGive(_dataMutex);
            }
            if (handled) continue;
        }

        // A reply to our own poll: feeds the dash's telemetry, never the phone.
        if (_rxTunnel.feed(f, VESC_CAN_LOCAL_ID)) {
            handleFrame(_rxTunnel.payload(), _rxTunnel.length());
        }

        // A reply to something the phone asked for: feeds the phone, never
        // the dash's own telemetry - see the class-level note on why these
        // two stay on separate ids/tunnels.
        if (_bleTunnel.feed(f, VESC_CAN_BLE_ID) && _sink && _bleConnected) {
            const uint16_t n = buildFrame(_bleTunnel.payload(), _bleTunnel.length(),
                                           _rxFrameScratch, sizeof(_rxFrameScratch));
            if (n) _sink(_rxFrameScratch, n);
        }
    }
#endif

    // ---- local polling -----------------------------------------------------
    // Over CAN every other field the dash shows already arrived passively
    // above, and this only exists because fault code has no status broadcast
    // frame. Over UART there are no broadcasts: this is all of the telemetry.
    const uint32_t now = millis();
    const uint32_t interval = _bleConnected
                                ? (POLL_WHILE_BLE_CONNECTED ? POLL_INTERVAL_BLE_MS : 0xFFFFFFFF)
                                : POLL_INTERVAL_MS;

    if (interval != 0xFFFFFFFF && (now - _lastPollMs) >= interval) {
        _lastPollMs = now;
        requestValues();
    }

    // A fetch that never got a reply (bus hiccup, VESC busy): try again.
    if (_appconfRequestPending && !configTrafficBlocked() && (now - _appconfReqMs) > 2000) {
        requestAppConf();
    }
    // UART: a profile switch made while VESC Tool was connected gets its
    // app_mode now that it has gone.
    if (_appconfDeferred && !configTrafficBlocked()) {
        _appconfDeferred = false;
        if (!_appconfRequestPending) requestAppConf();
    }

    // Setup info (battery config + geometry, both from one COMM_GET_MCCONF
    // reply): fetch once, retry until both land, then leave it alone - see
    // the notes by the declarations.
    bool setupValid = false;
    if (xSemaphoreTake(_dataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        setupValid = _battCfg.valid && _geomCfg.valid;
        xSemaphoreGive(_dataMutex);
    }
    if (!setupValid && !configTrafficBlocked() && (now - _mcconfReqMs) > 2000) {
        requestMcconf();
    }

    // A pending "persist this profile" whose mcconf fetch went unanswered.
    // Unlike the setup info above this does NOT retry forever: a write is
    // a discrete thing the rider asked for once, and silently firing it
    // minutes later - possibly while moving again - is worse than not firing
    // it at all. Three tries, then drop it.
    if (_mcconfStorePending && (now - _mcconfStoreReqMs) > 2000) {
        if (configTrafficBlocked()) {
            _mcconfStorePending = false;
            Serial.println("[link] store: VESC Tool connected over UART - giving up");
        } else if (++_mcconfStoreTries >= 3) {
            _mcconfStorePending = false;
            Serial.println("[link] store: no mcconf reply after 3 tries - giving up");
        } else {
            _mcconfStoreReqMs = now;
            requestMcconf();
        }
    }
}

// ---------------------------------------------------------------------------
void VescLink::handleFrame(const uint8_t *payload, uint16_t len) {
    if (len < 1) return;

    switch (payload[0]) {
    case COMM_GET_VALUES:
    case COMM_GET_VALUES_SETUP:
    case COMM_GET_VALUES_SELECTIVE:
    case COMM_GET_VALUES_SETUP_SELECTIVE: {
        Telemetry t;
        if (xSemaphoreTake(_dataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            t = _data;                       // keep fields the mask omitted
            xSemaphoreGive(_dataMutex);
        }
        if (parseValues(payload, len, t)) {
            if (xSemaphoreTake(_dataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                _data = t;
                xSemaphoreGive(_dataMutex);
            }
        }
        break;
    }

    case COMM_GET_MCCONF: {
        BatteryConfig cfg;
        if (parseMcconfBattery(payload, len, cfg)) {
            if (xSemaphoreTake(_dataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                _battCfg = cfg;
                xSemaphoreGive(_dataMutex);
            }
            Serial.printf("[link] battery config from VESC: %uS %.1fAh\n",
                          (unsigned)cfg.cells, cfg.ah);
        }
        GeometryConfig geom;
        if (parseMcconfGeometry(payload, len, geom)) {
            if (xSemaphoreTake(_dataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                _geomCfg = geom;
                xSemaphoreGive(_dataMutex);
            }
            Serial.printf("[link] geometry from VESC: %u poles, gear %.2f, wheel %.3f m\n",
                          (unsigned)geom.motorPoles, geom.gearRatio, geom.wheelDiameterM);
        }
        // A reply that fails either sanity/length check just leaves that
        // half's .valid false - loop() keeps retrying at the same 2s cadence,
        // and callers keep showing it as unknown in the meantime.

        // ...and if a write is pending, this reply *is* the thing to write:
        // it is the VESC's live config, profile limits already applied. Send
        // it straight back under COMM_SET_MCCONF, byte for byte. Cleared
        // first either way, so a reply the length check rejects drops the
        // request rather than leaving it armed for the next fetch - and so a
        // second reply arriving behind this one cannot write flash twice.
        if (_mcconfStorePending) {
            _mcconfStorePending = false;
            if (configTrafficBlocked()) {
                // The phone connected while the fetch was in flight: over
                // UART this reply may not even be from this VESC.
                Serial.println("[link] store: VESC Tool connected over UART - not writing");
            } else if (len == 1 + MCCONF_LEN) {
                sendStoreMcconf(payload + 1);
            } else {
                Serial.printf("[link] store: mcconf reply wrong size (%u, expected %u) - "
                              "firmware version mismatch? not writing\n",
                              len, (unsigned)(1 + MCCONF_LEN));
            }
        }
        break;
    }

    case COMM_GET_APPCONF:
        _appconfRequestPending = false;
        if (configTrafficBlocked()) {
            // The phone connected while the fetch was in flight: over UART
            // this reply may not even be from this VESC. Fetch again once
            // it has gone.
            if (_pendingAppModeValid) _appconfDeferred = true;
        } else if (len == 1 + APPCONF_LEN) {
            memcpy(_appconf, payload + 1, APPCONF_LEN);
            if (_pendingAppModeValid) {
                sendAppMode(_pendingAppMode);
                _pendingAppModeValid = false;
            }
        } else {
            Serial.printf("[link] app config reply wrong size (%u, expected %u) - "
                          "firmware version mismatch? app_mode profile switching disabled\n",
                          len, (unsigned)(1 + APPCONF_LEN));
        }
        break;

    case COMM_SET_MCCONF:
        // The VESC echoes the command id back once the config is written. The
        // dash only uses it to tell the rider the save actually landed - see
        // takeStoreAck().
        _mcconfStoreAcked = true;
        Serial.println("[link] store: VESC acknowledged the config write");
        break;

    default:
        break;                               // ours, nothing to act on - e.g. the
                                             // COMM_SET_APPCONF_NO_STORE ack
    }
}

// ---------------------------------------------------------------------------
Telemetry VescLink::snapshot() {
    Telemetry copy;
    if (xSemaphoreTake(_dataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        copy = _data;
        xSemaphoreGive(_dataMutex);
    }
    return copy;
}

bool VescLink::isAlive() {
    const uint32_t last = snapshot().lastUpdateMs;
    return last != 0 && (millis() - last) < TELEMETRY_STALE_MS;
}

// ---------------------------------------------------------------------------
BatteryConfig VescLink::batteryConfig() {
    BatteryConfig copy;
    if (xSemaphoreTake(_dataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        copy = _battCfg;
        xSemaphoreGive(_dataMutex);
    }
    return copy;
}

GeometryConfig VescLink::geometryConfig() {
    GeometryConfig copy;
    if (xSemaphoreTake(_dataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        copy = _geomCfg;
        xSemaphoreGive(_dataMutex);
    }
    return copy;
}

// ---------------------------------------------------------------------------
//  Bytes the phone sent, already framed by VESC Tool (0x02/len/payload/crc/
//  0x03). Recover complete frames first, then send each payload on: over CAN
//  it has to be re-tunnelled anyway, and over UART it keeps the dash's own
//  requests from ever being written into the middle of a phone frame split
//  across several BLE writes.
// ---------------------------------------------------------------------------
void VescLink::writeRaw(const uint8_t *data, size_t len) {
    for (size_t i = 0; i < len; i++) {
        if (_bleSniffer.feed(data[i])) {
            sendPayload(_bleSniffer.payload(), _bleSniffer.length(),
                        FROM_PHONE, /*expectReply=*/true);
        }
    }
}

// ---------------------------------------------------------------------------
void VescLink::sendPayload(const uint8_t *payload, uint16_t len, Origin from, bool expectReply) {
    if (xSemaphoreTake(_txMutex, pdMS_TO_TICKS(50)) != pdTRUE) return;

#if VESC_LINK_UART
    const uint16_t n = buildFrame(payload, len, _txScratch, sizeof(_txScratch));
    if (n) {
        // Before the bytes go out, so even an instant reply finds it.
        if (from == FROM_DASH && expectReply) awaitReply(payload[0]);
        vescuart::write(_txScratch, n);
    }
#else
    const uint8_t localId = (from == FROM_PHONE) ? VESC_CAN_BLE_ID : VESC_CAN_LOCAL_ID;
    const int n = buildTunnelFrames(VESC_CAN_TARGET_ID, localId,
                                     payload, len, expectReply,
                                     _txScratch, (int)(sizeof(_txScratch) / sizeof(_txScratch[0])));
    for (int i = 0; i < n; i++) vescan::send(_txScratch[i]);
#endif

    xSemaphoreGive(_txMutex);
}

#if VESC_LINK_UART
// ---------------------------------------------------------------------------
//  UART reply routing - see the class-level note in vesc_link.h.
//
//  How long a request keeps its claim on a reply. Generous on purpose: the
//  slowest answer the dash waits for is COMM_SET_MCCONF's, which only comes
//  once the VESC has finished writing its flash. A reply later than this is
//  dropped rather than handed to whoever asked last - the dash just asks
//  again on its next cycle.
// ---------------------------------------------------------------------------
static const uint32_t UART_REPLY_TIMEOUT_MS = 3000;

static bool expired(uint32_t deadlineMs, uint32_t now) {
    return (int32_t)(now - deadlineMs) > 0;
}

void VescLink::awaitReply(uint8_t cmd) {
    if (xSemaphoreTake(_awaitMutex, pdMS_TO_TICKS(50)) != pdTRUE) return;

    const uint32_t now = millis();
    AwaitedReply *slot = nullptr;
    for (AwaitedReply &a : _awaited) {
        if (a.count && a.cmd == cmd) { slot = &a; break; }
        if (!slot && (a.count == 0 || expired(a.deadlineMs, now))) slot = &a;
    }
    if (slot) {
        // Reusing an expired slot, or one for another command: start over.
        if (slot->cmd != cmd || expired(slot->deadlineMs, now)) slot->count = 0;
        slot->cmd        = cmd;
        slot->count      = (uint8_t)min(slot->count + 1, 255);
        slot->deadlineMs = now + UART_REPLY_TIMEOUT_MS;
    }
    // No free slot can only mean more command ids than _awaited was sized
    // for; the reply then goes unclaimed and the dash asks again later.

    xSemaphoreGive(_awaitMutex);
}

bool VescLink::claimReply(const uint8_t *payload, uint16_t len) {
    if (len < 1) return false;
    const uint8_t cmd = payload[0];

    // The dash's poll always carries DASH_VALUE_MASK, so a selective reply
    // with any other mask was asked for by someone else, whatever the dash
    // happens to be waiting on.
    if (cmd == COMM_GET_VALUES_SELECTIVE) {
        if (len < 5) return false;
        int32_t i = 1;
        if ((uint32_t)bufGetInt32(payload, &i) != DASH_VALUE_MASK) return false;
    }

    if (xSemaphoreTake(_awaitMutex, pdMS_TO_TICKS(50)) != pdTRUE) return false;
    const uint32_t now = millis();
    bool claimed = false;
    for (AwaitedReply &a : _awaited) {
        if (a.count && a.cmd == cmd && !expired(a.deadlineMs, now)) {
            a.count--;
            claimed = true;
            break;
        }
    }
    xSemaphoreGive(_awaitMutex);
    return claimed;
}

// A complete frame from the VESC goes to exactly one place: the dash, if it
// answers something the dash asked, otherwise the phone. Neither (nobody is
// waiting any more - a timed-out request, or the phone's after it left):
// dropped.
void VescLink::routeReply(const uint8_t *payload, uint16_t len) {
    if (claimReply(payload, len)) {
        handleFrame(payload, len);
    } else if (_sink && _bleConnected) {
        const uint16_t n = buildFrame(payload, len, _rxFrameScratch, sizeof(_rxFrameScratch));
        if (n) _sink(_rxFrameScratch, n);
    }
}
#endif // VESC_LINK_UART

// ---------------------------------------------------------------------------
void VescLink::requestValues() {
    uint8_t payload[5];
    int32_t ind = 0;
    payload[ind++] = COMM_GET_VALUES_SELECTIVE;
    bufAppendUint32(payload, VESC_LINK_UART ? DASH_VALUE_MASK : CAN_POLL_VALUE_MASK, &ind);
    sendPayload(payload, (uint16_t)ind, FROM_DASH, /*expectReply=*/true);
}

// ---------------------------------------------------------------------------
void VescLink::requestAppConf() {
    _appconfRequestPending = true;
    _appconfReqMs = millis();
    const uint8_t payload[1] = { COMM_GET_APPCONF };
    sendPayload(payload, 1, FROM_DASH, /*expectReply=*/true);
}

// ---------------------------------------------------------------------------
void VescLink::requestMcconf() {
    _mcconfReqMs = millis();
    const uint8_t payload[1] = { COMM_GET_MCCONF };
    sendPayload(payload, 1, FROM_DASH, /*expectReply=*/true);
}

// ---------------------------------------------------------------------------
//  COMM_SET_APPCONF_NO_STORE - applies live (app_set_configuration()) without
//  writing flash, the app-config equivalent of COMM_SET_MCCONF_TEMP_SETUP's
//  store=false. Unlike that command, it takes the VESC's entire serialized
//  app_configuration, so this patches just the app_mode byte into the copy
//  requestAppConf() just fetched and sends the whole thing back unchanged
//  otherwise - see the APPCONF_LEN / APPCONF_OFF_APP_TO_USE comment in
//  vesc_protocol.h for exactly where that byte lives and why. Only ever
//  called right after a fresh fetch (see handleFrame()'s COMM_GET_APPCONF
//  case), so `_appconf` is always current - never a stale, reused copy.
//
//  Over UART the profile's AppMode is not written as-is: every one of them
//  stops the COMM-port UART this link runs on. It is swapped for the app that
//  gives the same rider input with the UART still running - throttle on
//  (ADC) becomes ADC+UART, throttle off (NONE) becomes UART only. PAS has no
//  such app, and config.h refuses to build a UART firmware whose profiles
//  use it, so those two are the only modes that get here.
// ---------------------------------------------------------------------------
void VescLink::sendAppMode(AppMode mode) {
#if VESC_LINK_UART
    const uint8_t appUse = (mode == APP_MODE_ADC) ? APP_USE_ADC_UART : APP_USE_UART;
#else
    const uint8_t appUse = (uint8_t)mode;
#endif
    _appconf[APPCONF_OFF_APP_TO_USE] = appUse;

    uint8_t payload[1 + APPCONF_LEN];
    payload[0] = COMM_SET_APPCONF_NO_STORE;
    memcpy(payload + 1, _appconf, APPCONF_LEN);
    sendPayload(payload, sizeof(payload), FROM_DASH, /*expectReply=*/true);

    Serial.printf("[link] app_mode -> %d\n", (int)appUse);
}

// ---------------------------------------------------------------------------
//  COMM_SET_MCCONF_TEMP_SETUP - the same command VESC Tool's profile buttons
//  use. Payload, in order:
//      u8  store                (0 = RAM only, saved config untouched)
//      u8  forward_can
//      u8  ack
//      u8  divide_by_controllers
//      f32 l_current_min_scale  (braking)
//      f32 l_current_max_scale  (drive)
//      f32 speed min  [m/s]
//      f32 speed max  [m/s]
//      f32 duty min
//      f32 duty max
//      f32 watt min   (regen, negative)
//      f32 watt max
//  All floats use the firmware's "auto" encoding.
// ---------------------------------------------------------------------------
void VescLink::applyProfile(const RiderProfile &p) {
    if (!PROFILE_APPLY_TO_VESC) return;

    const float speedMax = (p.speed_kph > 0.0f) ? (p.speed_kph / 3.6f) : 300.0f;
    const float wattMax  = (p.watt_max > 0.0f)  ? p.watt_max            : 1.0e6f;

    uint8_t payload[64];
    int32_t ind = 0;
    payload[ind++] = COMM_SET_MCCONF_TEMP_SETUP;
    payload[ind++] = 0;                                  // store
    payload[ind++] = PROFILE_FORWARD_CAN ? 1 : 0;        // forward_can
    payload[ind++] = 0;                                  // ack
    payload[ind++] = 0;                                  // divide_by_controllers

    bufAppendFloat32Auto(payload, p.brake_scale,   &ind);
    bufAppendFloat32Auto(payload, p.current_scale, &ind);
    bufAppendFloat32Auto(payload, -speedMax,       &ind);
    bufAppendFloat32Auto(payload,  speedMax,       &ind);
    bufAppendFloat32Auto(payload, 0.005f,          &ind);   // duty min
    bufAppendFloat32Auto(payload, 0.95f,           &ind);   // duty max
    bufAppendFloat32Auto(payload, -wattMax,        &ind);   // regen watts
    bufAppendFloat32Auto(payload,  wattMax,        &ind);

    sendPayload(payload, (uint16_t)ind, FROM_DASH, /*expectReply=*/false);
    Serial.printf("[link] profile -> %s  cur=%.2f brk=%.2f  %.0f km/h  %.0f W\n",
                  p.name, p.current_scale, p.brake_scale, p.speed_kph, p.watt_max);

    // Always fetch a fresh app config before patching app_mode in - see the
    // class-level comment on why this isn't cached. sendAppMode() fires as
    // soon as the fetch answers, in handleFrame()'s COMM_GET_APPCONF case.
    // If a fetch is already in flight (a rapid repeated switch), just update
    // which mode wins when it lands - no redundant fetch. Over UART with
    // VESC Tool connected, the fetch waits until it disconnects (loop()).
    _pendingAppMode = p.app_mode;
    _pendingAppModeValid = true;
    if (configTrafficBlocked())       _appconfDeferred = true;
    else if (!_appconfRequestPending) requestAppConf();
}

// ---------------------------------------------------------------------------
//  COMM_SET_MCCONF - the one place this dashboard writes the VESC's flash.
//
//  Everything applyProfile() sends is RAM-only by design (store=0), so the
//  VESC forgets it on a power cycle and reapply() pushes it again. This is
//  the rider explicitly asking for the opposite: make what is running right
//  now permanent. See the PROFILE_LONG_PRESS_WRITE_MS comment in config.h for
//  the gesture, and for why this is a rare, deliberate act rather than
//  something to do on every profile change.
//
//  Two steps, because COMM_SET_MCCONF carries the entire motor config and we
//  have no business inventing one: fetch the live config (COMM_GET_MCCONF -
//  which already reflects the applied profile), then send those exact bytes
//  back. handleFrame()'s COMM_GET_MCCONF case does the second half.
// ---------------------------------------------------------------------------
void VescLink::storeConfig() {
    if (!PROFILE_APPLY_TO_VESC || !PROFILE_WRITE_ENABLED) return;
    if (configTrafficBlocked()) {
        Serial.println("[link] store: refused while VESC Tool is connected over UART - "
                       "disconnect it, or save from VESC Tool (see config.h 1b)");
        return;
    }

    _mcconfStorePending = true;
    _mcconfStoreTries   = 0;
    _mcconfStoreReqMs   = millis();
    requestMcconf();
    Serial.println("[link] store: fetching live mcconf to write back");
}

// ---------------------------------------------------------------------------
void VescLink::sendStoreMcconf(const uint8_t *mcconf) {
    _mcconfTx[0] = COMM_SET_MCCONF;
    memcpy(_mcconfTx + 1, mcconf, MCCONF_LEN);

    // Deliberately NOT forwarded over CAN, unlike the temp profile's
    // PROFILE_FORWARD_CAN flag. Every controller on the bus has its own motor
    // configuration - motor type, sensor mode, current limits sized to that
    // motor - and this blob is *this* VESC's. Forwarding it would overwrite a
    // slave's entire setup with the master's.
    sendPayload(_mcconfTx, sizeof(_mcconfTx), FROM_DASH, /*expectReply=*/true);
    Serial.println("[link] store: COMM_SET_MCCONF sent - waiting for the VESC's ack");
}

// ---------------------------------------------------------------------------
bool VescLink::takeStoreAck() {
    if (!_mcconfStoreAcked) return false;
    _mcconfStoreAcked = false;
    return true;
}
