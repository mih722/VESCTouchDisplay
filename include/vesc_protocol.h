#pragma once
#include <Arduino.h>
#include "vesc_can.h"

// ============================================================================
//  VESC COMM protocol primitives.
//
//  Frame layout (identical on UART, USB and over the BLE NUS characteristics):
//      [0x02][len8]              payload  [crc16 hi][crc16 lo][0x03]
//      [0x03][len16 hi][len16 lo] payload [crc16 hi][crc16 lo][0x03]
//  CRC is CRC16/XMODEM (poly 0x1021, init 0x0000) over the payload only.
//
//  The BLE bridge always speaks this framing to the phone (VESC Tool expects
//  it). On the wire to the VESC it is sent as-is in a UART build, or
//  tunnelled over CAN in a CAN build - see the CAN_PACKET_ID / CanTunnel
//  section below.
//
//  Reference: vedder.se "Communicating with the VESC using UART",
//  SolidGeek/VescUart, TecnicoFuelCell/ComEVesc, 3mrotaha/pyVESC-uart.
// ============================================================================

namespace vesc {

// ---- COMM packet ids (bldc datatypes.h, release_7_00) ----------------------
enum CommPacketId : uint8_t {
    COMM_FW_VERSION               = 0,
    COMM_GET_VALUES               = 4,
    COMM_SET_DUTY                 = 5,
    COMM_SET_CURRENT              = 6,
    COMM_SET_CURRENT_BRAKE        = 7,
    COMM_SET_RPM                  = 8,
    COMM_SET_HANDBRAKE            = 10,
    COMM_SET_MCCONF               = 13,
    COMM_GET_MCCONF               = 14,
    COMM_GET_APPCONF              = 17,
    COMM_REBOOT                   = 29,
    COMM_ALIVE                    = 30,
    COMM_FORWARD_CAN              = 34,
    COMM_GET_VALUES_SETUP         = 47,
    COMM_SET_MCCONF_TEMP          = 48,
    COMM_SET_MCCONF_TEMP_SETUP    = 49,
    COMM_GET_VALUES_SELECTIVE     = 50,
    COMM_GET_VALUES_SETUP_SELECTIVE = 51,
    COMM_SET_APPCONF_NO_STORE     = 149,
};

// ---- COMM_{GET,SET_..._NO_STORE}_APPCONF payload layout --------------------
// confgenerator_serialize_appconf()'s output: a 4-byte build-specific
// signature followed by every app_configuration field in declaration order.
// Hand-derived by walking bldc firmware release_7_00 (FW 7.0.0)'s
// confgenerator.c - re-derive both constants from that file if you change
// firmware version, since fields can be added/reordered between releases.
// The dashboard never needs to know the signature value itself: it always
// round-trips whatever COMM_GET_APPCONF returned, so a version-mismatched
// signature only ever fails safely (the VESC rejects it, no reply), it does
// not risk being silently wrong.
static const uint16_t APPCONF_LEN            = 284;  // signature + all fields
static const uint16_t APPCONF_OFF_APP_TO_USE = 33;    // 1 byte - AppMode in config.h

// The two app_use values (bldc datatypes.h, release_7_00) a UART build
// writes at that offset instead of the profile's AppMode as-is: the only
// rider inputs that also keep the VESC's COMM-port UART running. See
// config.h section 1b.
static const uint8_t APP_USE_UART     = 3;   // app_use::APP_UART     - throttle off
static const uint8_t APP_USE_ADC_UART = 5;   // app_use::APP_ADC_UART - throttle on

// ---- COMM_GET_MCCONF payload layout ----------------------------------------
// confgenerator_serialize_mcconf()'s output, same signature+fields shape as
// appconf above. Hand-derived (byte-counted from confgenerator.c) the same
// way as the appconf offset - re-derive if you change firmware version.
// Only the si_* "setup info" block is needed here - si_motor_poles/
// _gear_ratio/_wheel_diameter for speed and distance, si_battery_type/
// _cells/_ah for the battery gauge. The dash has no hardcoded values of its
// own to fall back to, so a length mismatch (a firmware version too
// different for these offsets to trust) just means speed and the battery
// gauge stay "unknown" - see GeometryConfig / BatteryConfig below.
static const uint16_t MCCONF_LEN                   = 488;  // signature + all fields
static const uint16_t MCCONF_OFF_SI_MOTOR_POLES    = 452;  // 1 byte - magnet count, not pole pairs
static const uint16_t MCCONF_OFF_SI_GEAR_RATIO     = 453;  // f32_auto - motor revs per wheel rev
static const uint16_t MCCONF_OFF_SI_WHEEL_DIAMETER = 457;  // f32_auto - metres
static const uint16_t MCCONF_OFF_SI_BATTERY_TYPE   = 461;  // 1 byte - BATTERY_TYPE enum
static const uint16_t MCCONF_OFF_SI_BATTERY_CELLS  = 462;  // 1 byte - series cell count
static const uint16_t MCCONF_OFF_SI_BATTERY_AH     = 463;  // f32_auto - pack capacity in Ah

// bldc's BATTERY_TYPE enum (datatypes.h). Only the first two have an
// equivalent SoC curve in dash_model.cpp; BATTERY_TYPE_LEAD_ACID (and any
// other value - e.g. a mismatched firmware) is left for the caller to treat
// as "unknown" - see BatteryConfig::chemistry below.
enum McconfBatteryType : uint8_t {
    MCCONF_BATTERY_TYPE_LIION     = 0,   // BATTERY_TYPE_LIION_3_0__4_2
    MCCONF_BATTERY_TYPE_LIFEPO4   = 1,   // BATTERY_TYPE_LIIRON_2_6__3_6
    MCCONF_BATTERY_TYPE_LEAD_ACID = 2,   // no matching curve - unmapped
};

// ---- Field bit positions for COMM_GET_VALUES_SELECTIVE ---------------------
// The mask bits follow the order the fields appear in COMM_GET_VALUES.
enum ValueMaskBit : uint32_t {
    VAL_TEMP_FET            = 1UL << 0,
    VAL_TEMP_MOTOR          = 1UL << 1,
    VAL_CURRENT_MOTOR       = 1UL << 2,
    VAL_CURRENT_IN          = 1UL << 3,
    VAL_ID                  = 1UL << 4,
    VAL_IQ                  = 1UL << 5,
    VAL_DUTY                = 1UL << 6,
    VAL_RPM                 = 1UL << 7,
    VAL_V_IN                = 1UL << 8,
    VAL_AMP_HOURS           = 1UL << 9,
    VAL_AMP_HOURS_CHARGED   = 1UL << 10,
    VAL_WATT_HOURS          = 1UL << 11,
    VAL_WATT_HOURS_CHARGED  = 1UL << 12,
    VAL_TACHOMETER          = 1UL << 13,
    VAL_TACHOMETER_ABS      = 1UL << 14,
    VAL_FAULT               = 1UL << 15,
    VAL_PID_POS             = 1UL << 16,
    VAL_CONTROLLER_ID       = 1UL << 17,
    VAL_TEMPS_MOS           = 1UL << 18,
    VAL_VD_VQ               = 1UL << 19,
    VAL_STATUS              = 1UL << 20,
};

// Everything the dashboard needs, and nothing it does not - the UART poll's
// mask, since over UART the poll is all of the telemetry.
static const uint32_t DASH_VALUE_MASK =
    VAL_TEMP_FET | VAL_TEMP_MOTOR | VAL_CURRENT_MOTOR | VAL_CURRENT_IN |
    VAL_DUTY | VAL_RPM | VAL_V_IN | VAL_AMP_HOURS | VAL_AMP_HOURS_CHARGED |
    VAL_WATT_HOURS | VAL_WATT_HOURS_CHARGED | VAL_TACHOMETER |
    VAL_TACHOMETER_ABS | VAL_FAULT;

// The CAN poll's mask: only what the status broadcasts don't carry. Asking
// for more would give those fields two writers - STATUS_5's signed
// tachometer and the poll's real tachometer_abs used to take turns in
// Telemetry::tachometerAbs, and every swap added their difference to the
// odometer, even parked. As a bonus the request and its reply (id + mask +
// one byte) each fit a single PROCESS_SHORT_BUFFER frame.
static const uint32_t CAN_POLL_VALUE_MASK = VAL_FAULT;

enum FaultCode : uint8_t {
    FAULT_NONE = 0, FAULT_OVER_VOLTAGE, FAULT_UNDER_VOLTAGE, FAULT_DRV,
    FAULT_ABS_OVER_CURRENT, FAULT_OVER_TEMP_FET, FAULT_OVER_TEMP_MOTOR,
    FAULT_GATE_DRIVER_OVER_VOLTAGE, FAULT_GATE_DRIVER_UNDER_VOLTAGE,
    FAULT_MCU_UNDER_VOLTAGE, FAULT_BOOTING_FROM_WATCHDOG_RESET,
    FAULT_ENCODER_SPI, FAULT_ENCODER_SINCOS_BELOW_MIN_AMPLITUDE,
    FAULT_ENCODER_SINCOS_ABOVE_MAX_AMPLITUDE, FAULT_FLASH_CORRUPTION,
    FAULT_HIGH_OFFSET_CURRENT_SENSOR_1, FAULT_HIGH_OFFSET_CURRENT_SENSOR_2,
    FAULT_HIGH_OFFSET_CURRENT_SENSOR_3, FAULT_UNBALANCED_CURRENTS,
};
const char *faultToString(uint8_t code);

// ---- Live values -----------------------------------------------------------
struct Telemetry {
    float    tempFet          = 0;
    float    tempMotor        = 0;
    float    currentMotor     = 0;   // A
    float    currentIn        = 0;   // A (battery side, negative = regen)
    float    duty             = 0;   // -1..1
    float    erpm             = 0;
    float    voltage          = 0;   // V
    float    ampHours         = 0;   // Ah consumed
    float    ampHoursCharged  = 0;   // Ah recovered
    float    wattHours        = 0;
    float    wattHoursCharged = 0;
    int32_t  tachometer       = 0;
    // Over UART this is the VESC's real absolute count. Over CAN it is the
    // signed tachometer again: CAN_PACKET_STATUS_5 carries no absolute count,
    // and the CAN poll doesn't ask for one (CAN_POLL_VALUE_MASK). So over
    // CAN, rolling backwards winds the trip distance back.
    int32_t  tachometerAbs    = 0;
    uint8_t  faultCode        = 0;
    uint32_t lastUpdateMs     = 0;
};

// ---- Big-endian buffer helpers (VESC byte order) ---------------------------
int16_t  bufGetInt16(const uint8_t *b, int32_t *i);
int32_t  bufGetInt32(const uint8_t *b, int32_t *i);
float    bufGetFloat16(const uint8_t *b, float scale, int32_t *i);
float    bufGetFloat32(const uint8_t *b, float scale, int32_t *i);
void     bufAppendInt16(uint8_t *b, int16_t v, int32_t *i);
void     bufAppendInt32(uint8_t *b, int32_t v, int32_t *i);
void     bufAppendUint32(uint8_t *b, uint32_t v, int32_t *i);
// The "auto" float encoding used by COMM_SET_MCCONF_TEMP* (frexp based).
void     bufAppendFloat32Auto(uint8_t *b, float v, int32_t *i);
// Decoder for the above - used to read the si_* floats (gear ratio, wheel
// diameter, battery Ah) out of COMM_GET_MCCONF.
float    bufGetFloat32Auto(const uint8_t *b, int32_t *i);

uint16_t crc16(const uint8_t *buf, uint16_t len);

// Wrap a payload in a full frame. Returns the frame length, 0 if it will not
// fit. `out` must have room for len + 6 bytes.
uint16_t buildFrame(const uint8_t *payload, uint16_t len, uint8_t *out, uint16_t outCap);

// Parse a GET_VALUES / GET_VALUES_SELECTIVE payload into `t`.
// `payload[0]` is the command id. Returns false on a short or unknown packet.
bool parseValues(const uint8_t *payload, uint16_t len, Telemetry &t);

// ---- Battery config, read from the VESC - there is no hardcoded fallback ---
// The dash has no config.h guess to fall back on: a bad guess baked into
// firmware is worse than an honest "unknown" gauge, so callers must check
// `valid` (and, to pick a SoC curve, `chemistry >= 0`) before trusting this.
struct BatteryConfig {
    uint8_t cells      = 0;      // si_battery_cells - series cell count
    float   ah         = 0;      // si_battery_ah - pack capacity
    // -1 = unknown/unmapped (BATTERY_TYPE_LEAD_ACID, or a value this dash
    // has no SoC curve for). 0 = LiFePO4, 1 = Li-ion/LiPo otherwise.
    int8_t  chemistry  = -1;
    bool    valid      = false;  // cells/ah - chemistry validity is separate, see above
};

// Parse a COMM_GET_MCCONF reply, pulling out si_battery_type/_cells/_ah.
// Returns false if the packet isn't a COMM_GET_MCCONF reply, is the wrong
// length for the offsets above (different firmware version), or cells/ah
// are outside a physically sane range - any of which means the caller
// should treat the battery as unknown. Chemistry has no bearing on the
// return value: it comes back as -1 (see BatteryConfig) whenever the VESC's
// si_battery_type doesn't map to a curve this dash has.
bool parseMcconfBattery(const uint8_t *payload, uint16_t len, BatteryConfig &out);

// ---- Motor/wheel geometry, read from the VESC - no hardcoded fallback ------
// The three numbers that turn ERPM and tachometer counts into speed and
// distance. These are the VESC's own values, the same ones its
// mc_interface_get_speed()/_get_distance() use, so the dash agrees with VESC
// Tool by construction. Callers must check `valid` before trusting them.
struct GeometryConfig {
    uint8_t motorPoles     = 0;      // si_motor_poles - magnet count, NOT pole pairs
    float   gearRatio      = 0;      // si_gear_ratio - motor revs per wheel rev
    float   wheelDiameterM = 0;      // si_wheel_diameter - metres
    bool    valid          = false;
};

// Parse a COMM_GET_MCCONF reply, pulling out si_motor_poles/_gear_ratio/
// _wheel_diameter. Returns false if the packet isn't a COMM_GET_MCCONF
// reply, is the wrong length for the offsets above, or any value is outside
// a physically sane range - including an odd pole count, since magnets come
// in pairs and an odd count can only be a misaligned read. Any of which
// means the caller should treat speed and distance as unknown.
bool parseMcconfGeometry(const uint8_t *payload, uint16_t len, GeometryConfig &out);

// ---------------------------------------------------------------------------
//  Streaming frame extractor.
//
//  Feed it one direction's bytes, one at a time, and it hands back each
//  CRC-valid payload. Noise between frames, frames with a bad CRC or
//  terminator, and frames longer than kMaxPayload are skipped. VescLink runs
//  one per direction that needs it - the phone's bytes from BLE, and in a
//  UART build the VESC's - and forwards only what it hands back, so nothing
//  that fails to parse ever reaches the other side.
// ---------------------------------------------------------------------------
class FrameSniffer {
public:
    static const uint16_t kMaxPayload = 1024;

    // Returns true when a CRC-valid payload is ready in payload()/length().
    bool feed(uint8_t byte);
    void reset() { _state = WAIT_START; _idx = 0; }

    const uint8_t *payload() const { return _buf; }
    uint16_t       length()  const { return _len; }

private:
    enum State : uint8_t { WAIT_START, LEN_HI, LEN_LO, PAYLOAD, CRC_HI, CRC_LO, END };
    State    _state = WAIT_START;
    uint8_t  _buf[kMaxPayload];
    uint16_t _len = 0;
    uint16_t _idx = 0;
    uint16_t _crc = 0;
    bool     _oversize = false;   // frame too long for us: skip it cleanly
};

// ---------------------------------------------------------------------------
//  VESC CAN protocol.
//
//  Extended (29-bit) CAN id = (CAN_PACKET_ID << 8) | controller_id. For the
//  periodic STATUS broadcasts, controller_id is the SENDING VESC's own id.
//  For the buffer tunnel (FILL/PROCESS), controller_id is the RECIPIENT of
//  that frame - the sender's own id instead travels inside the payload, so
//  the recipient knows who to address a reply to.
//
//  The tunnel is how arbitrary COMM_* packets (the same ones used over UART)
//  ride over CAN: short payloads (<=6 bytes) go in one PROCESS_SHORT_BUFFER
//  frame; longer ones are split into 7-byte (FILL_RX_BUFFER) or, past a
//  255-byte offset, 6-byte (FILL_RX_BUFFER_LONG) chunks, followed by a
//  PROCESS_RX_BUFFER trailer carrying the total length and a CRC16/XMODEM
//  over the reassembled payload. This is the same mechanism VESC's own
//  CAN/BLE bridge firmware (vesc_express) and COMM_FORWARD_CAN use.
//
//  Reference: vedderb/bldc comm/comm_can.c.
// ---------------------------------------------------------------------------
enum CanPacketId : uint8_t {
    CAN_PACKET_FILL_RX_BUFFER       = 5,
    CAN_PACKET_FILL_RX_BUFFER_LONG  = 6,
    CAN_PACKET_PROCESS_RX_BUFFER    = 7,
    CAN_PACKET_PROCESS_SHORT_BUFFER = 8,
    CAN_PACKET_STATUS               = 9,
    CAN_PACKET_STATUS_2             = 14,
    CAN_PACKET_STATUS_3             = 15,
    CAN_PACKET_STATUS_4             = 16,
    CAN_PACKET_STATUS_5             = 27,
};

inline uint8_t  canEidPacketId(uint32_t eid) { return (uint8_t)(eid >> 8); }
inline uint8_t  canEidAddress(uint32_t eid)  { return (uint8_t)(eid & 0xFF); }

// Decode a CAN_PACKET_STATUS.._5 broadcast into `t`. Only touches the fields
// that packet carries; everything else in `t` is left as the caller passed
// it in, mirroring parseValues()'s selective-update behaviour. Returns false
// for a packet id this dash does not consume, or a frame shorter than that
// packet's payload.
bool parseCanStatus(uint8_t canPacketId, const uint8_t *data, uint8_t len, Telemetry &t);

// ---------------------------------------------------------------------------
//  Send-side: wrap a raw COMM payload (the same bytes buildFrame() would
//  wrap for UART, minus the 0x02/len/crc/0x03 envelope) as one or more CAN
//  tunnel frames addressed to `targetId`, claiming `localId` as the reply
//  address.
//
//  `expectReply` sets the tunnel's "send" byte, per comm_can.c's
//  comm_can_send_buffer(): 0 runs commands_process_packet() on the target
//  and tunnels the reply back to us, 2 runs it with no reply at all. (1 is
//  a different, unrelated mode - it tells the target to treat our payload
//  as an already-built reply to re-forward, not a command to process - so
//  it must never be used here.) The value only matters on the way out; on
//  the way back CanTunnel::feed() reassembles the payload regardless of
//  what "send" byte the reply itself carries.
//
//  Returns the number of frames written to `out` (capacity `outCap`), 0 if
//  it does not fit.
// ---------------------------------------------------------------------------
int buildTunnelFrames(uint8_t targetId, uint8_t localId, const uint8_t *payload,
                      uint16_t len, bool expectReply, CanFrame *out, int outCap);

// ---------------------------------------------------------------------------
//  Receive-side: reassembles FILL_RX_BUFFER(_LONG) + PROCESS_RX_BUFFER /
//  PROCESS_SHORT_BUFFER sequences addressed to `localId` into a raw COMM
//  payload. A frame addressed to anyone else is ignored outright. A stray
//  FILL with no matching PROCESS just sits in the scratch buffer until a
//  PROCESS frame's length and CRC validate a complete payload; one that
//  doesn't validate is dropped.
// ---------------------------------------------------------------------------
class CanTunnel {
public:
    static const uint16_t kMaxPayload = 1024;

    // Returns true when `f` completed a valid payload, now in payload()/length().
    bool feed(const CanFrame &f, uint8_t localId);

    const uint8_t *payload()  const { return _buf; }
    uint16_t       length()   const { return _len; }
    uint8_t        sourceId() const { return _sourceId; }

private:
    uint8_t  _buf[kMaxPayload];
    uint16_t _len      = 0;
    uint8_t  _sourceId = 0;
};

} // namespace vesc
