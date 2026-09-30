#pragma once
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include "vesc_protocol.h"
#include "vesc_can.h"
#include "vesc_uart.h"
#include "config.h"

// ============================================================================
//  Owns the link to the VESC - CAN bus or UART, per config.h's VESC_LINK_UART
//  - and arbitrates the two consumers of it: the local dashboard poller and
//  the BLE bridge.
//
//  CAN design note - two tunnels, not one:
//  Regular telemetry (erpm/current/duty/temps/Ah/Wh/tacho/voltage) needs no
//  request at all: it rides in from the VESC's own periodic CAN status
//  broadcasts. Only the fault code (no broadcast frame for it) is actively
//  polled, over the buffer tunnel, addressed as VESC_CAN_LOCAL_ID.
//
//  The BLE bridge is a byte-for-byte passthrough of whatever VESC Tool sends
//  - firmware upload / config read all need to survive untouched - so its
//  traffic also rides the buffer tunnel, but addressed as the *separate*
//  VESC_CAN_BLE_ID. This is deliberate: an earlier version shared one id for
//  both, and VESC Tool's Real Time Data view (which polls fast) ended up
//  colliding with the dash's own poll on the VESC's rx buffer, and its
//  replies were being sniffed into the dash's own telemetry the way the old
//  UART sniffer used to. Two ids means two independent CanTunnel instances,
//  and BLE replies only ever go to the phone - they never touch `_data`.
//
//  UART design note - no ids to separate by:
//  A UART is one byte stream with no addressing, so the same isolation is
//  kept by bookkeeping instead. Every request the dash sends that expects an
//  answer is remembered by command id (awaitReply), and a frame from the VESC
//  is the dash's only if it answers one of those (claimReply); everything
//  else goes to the phone, untouched. Phone frames are reassembled before
//  they go out, exactly as for CAN, so the dash's own requests can never land
//  in the middle of one. Telemetry has no broadcasts to ride on here - the
//  COMM_GET_VALUES_SELECTIVE poll is all of it.
// ============================================================================

typedef void (*VescByteSink)(const uint8_t *data, size_t len);

class VescLink {
public:
    void begin();

    // Pump the link. Call this as often as you can from a dedicated task.
    void loop();

    // ---- telemetry -------------------------------------------------------
    vesc::Telemetry snapshot();
    bool  isAlive();

    // ---- battery config ----------------------------------------------------
    // Fetched once (COMM_GET_MCCONF) and retried until it succeeds; a static
    // config, not worth re-polling for the life of the connection. .valid is
    // false until a sane reply has arrived - there is no config.h fallback,
    // so callers should show the battery gauge as unknown until then.
    vesc::BatteryConfig batteryConfig();

    // ---- motor/wheel geometry ----------------------------------------------
    // Read from the same COMM_GET_MCCONF reply as the battery config, on the
    // same fetch-once, retry-until-sane schedule. .valid is false until then -
    // there is no config.h fallback, so callers should show speed as unknown
    // and not count distance until it lands.
    vesc::GeometryConfig geometryConfig();

    // ---- BLE bridge hooks ------------------------------------------------
    // Everything the VESC says is handed to this sink verbatim, framed the
    // way VESC Tool expects (0x02/len/payload/crc/0x03).
    void setByteSink(VescByteSink sink) { _sink = sink; }
    // Bytes the phone sent, already framed by VESC Tool: reassembled into
    // whole frames, then tunnelled over CAN or written to the UART.
    void writeRaw(const uint8_t *data, size_t len);
    void setBleConnected(bool connected) { _bleConnected = connected; }
    bool bleConnected() const { return _bleConnected; }

    // ---- commands --------------------------------------------------------
    void requestValues();
    // store=false: limits live in the VESC's RAM only, its saved config is
    // untouched and a power cycle restores it.
    void applyProfile(const RiderProfile &p);
    // ...and the deliberate exception: persist whatever is live right now to
    // the VESC's flash, so the applied profile survives a power cycle. Async
    // (fetch-then-resend, like the app_mode path below) and best-effort - it
    // gives up quietly after a few tries rather than retrying forever.
    void storeConfig();
    // One-shot, consumed by the UI: true once for each config write the VESC
    // acknowledges, so the rider is told the save actually landed rather than
    // just that the dash asked for it.
    bool takeStoreAck();

private:
    // Who a request is from, which decides who its reply goes to.
    enum Origin : uint8_t { FROM_DASH, FROM_PHONE };

    // Sends a raw COMM payload to the VESC. Over CAN it is tunnelled to
    // VESC_CAN_TARGET_ID, claiming VESC_CAN_LOCAL_ID (FROM_DASH) or
    // VESC_CAN_BLE_ID (FROM_PHONE) as the reply address. Over UART it is
    // framed and written, and a FROM_DASH request expecting a reply is
    // remembered so that reply can be claimed - see the class-level note.
    void sendPayload(const uint8_t *payload, uint16_t len, Origin from, bool expectReply);
    void handleFrame(const uint8_t *payload, uint16_t len);

    // Over UART, the dash's own config reads and writes wait while the phone
    // is connected - see config.h section 1b. Never over CAN, where separate
    // ids keep the two apart.
    bool configTrafficBlocked() const { return VESC_LINK_UART && _bleConnected; }

    // ---- app_mode (COMM_SET_APPCONF_NO_STORE) -----------------------------
    // COMM_SET_APPCONF_NO_STORE takes the VESC's entire app config, not a
    // small delta like COMM_SET_MCCONF_TEMP_SETUP does. Rather than cache a
    // copy and risk it going stale against anything changed via VESC Tool
    // in the meantime, every profile switch fetches a fresh copy
    // (COMM_GET_APPCONF) immediately beforehand, patches just the app-mode
    // byte, and sends the whole thing straight back - whatever the VESC's
    // live config is at fetch time is exactly what goes back out, unchanged
    // but for that one byte. See the APPCONF_LEN / APPCONF_OFF_APP_TO_USE
    // comment in vesc_protocol.h.
    void requestAppConf();
    void sendAppMode(AppMode mode);

    // ---- setup info: battery config + geometry (COMM_GET_MCCONF) -----------
    void requestMcconf();

    // ---- persisting a profile (COMM_SET_MCCONF) ---------------------------
    // Same fetch-patch-resend shape as the app_mode path above, minus the
    // patch: COMM_GET_MCCONF reports the VESC's *live* configuration, which
    // already includes whatever applyProfile() pushed into it, so persisting
    // it is just sending that exact blob back under COMM_SET_MCCONF. Nothing
    // here needs to know where any individual limit lives in the blob - only
    // the setup-info *readers* do (see MCCONF_OFF_* in vesc_protocol.h) - so a
    // firmware whose mcconf layout moved cannot be silently mis-written by
    // this path; it can only fail the length check and be skipped.
    void sendStoreMcconf(const uint8_t *mcconf);

    // Written from the UI task (storeConfig) and the link task (handleFrame /
    // loop), like the app_mode fields above - volatile for the same reason
    // _bleConnected is.
    volatile bool _mcconfStorePending = false;
    volatile bool _mcconfStoreAcked   = false;
    uint32_t _mcconfStoreReqMs   = 0;
    uint8_t  _mcconfStoreTries   = 0;
    // Outgoing COMM_SET_MCCONF payload (id byte + blob). A member, not a
    // stack local: the link task's stack is 6 KB and this is ~0.5 KB.
    uint8_t  _mcconfTx[1 + vesc::MCCONF_LEN];

    vesc::BatteryConfig  _battCfg;            // guarded by _dataMutex, like _data
    vesc::GeometryConfig _geomCfg;            // likewise
    uint32_t             _mcconfReqMs = 0;

    uint8_t  _appconf[vesc::APPCONF_LEN];   // scratch: the blob currently in flight / just fetched
    bool     _appconfRequestPending = false;
    uint32_t _appconfReqMs          = 0;
    // The most recently requested app_mode, applied the moment the in-flight
    // fetch above completes. A repeated profile switch before that happens
    // just overwrites this - no redundant fetch, only the latest mode wins.
    bool     _pendingAppModeValid = false;
    AppMode  _pendingAppMode      = APP_MODE_ADC_PAS;
    // UART only: a switch made while VESC Tool was connected (config traffic
    // blocked - see configTrafficBlocked()), still owed a fetch once it
    // disconnects. Never set over CAN.
    volatile bool _appconfDeferred = false;

#if VESC_LINK_UART
    // ---- UART: whose reply is whose (see the class-level note) ------------
    // One entry per command id the dash is waiting on: how many requests are
    // still unanswered, and when the newest of them stops counting. The
    // deadline keeps a request the VESC never answered from claiming a much
    // later reply the phone asked for.
    struct AwaitedReply {
        uint8_t  cmd        = 0;
        uint8_t  count      = 0;
        uint32_t deadlineMs = 0;
    };
    // The dash awaits 5 command ids: the telemetry poll, the mcconf fetch
    // and write, and the appconf fetch and write.
    AwaitedReply      _awaited[6];
    SemaphoreHandle_t _awaitMutex = nullptr;
    void awaitReply(uint8_t cmd);
    bool claimReply(const uint8_t *payload, uint16_t len);
    void routeReply(const uint8_t *payload, uint16_t len);

    vesc::FrameSniffer _uartSniffer;  // recovers complete frames from the VESC
#else
    vesc::CanTunnel    _rxTunnel;     // reassembles replies to our own polls (VESC_CAN_LOCAL_ID) - feeds _data
    vesc::CanTunnel    _bleTunnel;    // reassembles replies to the phone's requests (VESC_CAN_BLE_ID) - feeds BLE only
#endif
    vesc::FrameSniffer _bleSniffer;   // recovers complete frames from the phone
    vesc::Telemetry    _data;
    SemaphoreHandle_t  _dataMutex = nullptr;
    SemaphoreHandle_t  _txMutex   = nullptr;

    VescByteSink _sink = nullptr;
    volatile bool _bleConnected = false;

    uint32_t _lastPollMs = 0;

#if VESC_LINK_UART
    // Scratch space for one outgoing frame - a member, not a stack local,
    // since a phone request (config write) can be ~1 KB. Guarded by _txMutex.
    uint8_t  _txScratch[vesc::FrameSniffer::kMaxPayload + 8];
#else
    // Scratch space for outgoing tunnel frames - a member, not a stack local,
    // since a large phone request (config write) can span many CAN frames.
    CanFrame _txScratch[200];
#endif
    // Scratch space for re-framing a reply before it goes out over BLE.
    uint8_t  _rxFrameScratch[vesc::FrameSniffer::kMaxPayload + 8];
};

extern VescLink vescLink;
