#include "vesc_protocol.h"
#include <math.h>

namespace vesc {

// ---------------------------------------------------------------------------
//  Buffer helpers
// ---------------------------------------------------------------------------
int16_t bufGetInt16(const uint8_t *b, int32_t *i) {
    int16_t v = (int16_t)(((uint16_t)b[*i] << 8) | (uint16_t)b[*i + 1]);
    *i += 2;
    return v;
}

int32_t bufGetInt32(const uint8_t *b, int32_t *i) {
    int32_t v = (int32_t)(((uint32_t)b[*i] << 24) | ((uint32_t)b[*i + 1] << 16) |
                          ((uint32_t)b[*i + 2] << 8) | (uint32_t)b[*i + 3]);
    *i += 4;
    return v;
}

float bufGetFloat16(const uint8_t *b, float scale, int32_t *i) {
    return (float)bufGetInt16(b, i) / scale;
}

float bufGetFloat32(const uint8_t *b, float scale, int32_t *i) {
    return (float)bufGetInt32(b, i) / scale;
}

void bufAppendInt16(uint8_t *b, int16_t v, int32_t *i) {
    b[(*i)++] = (uint8_t)(v >> 8);
    b[(*i)++] = (uint8_t)(v);
}

void bufAppendInt32(uint8_t *b, int32_t v, int32_t *i) {
    b[(*i)++] = (uint8_t)(v >> 24);
    b[(*i)++] = (uint8_t)(v >> 16);
    b[(*i)++] = (uint8_t)(v >> 8);
    b[(*i)++] = (uint8_t)(v);
}

void bufAppendUint32(uint8_t *b, uint32_t v, int32_t *i) {
    bufAppendInt32(b, (int32_t)v, i);
}

// Mirror of buffer_append_float32_auto() in the bldc firmware, which is what
// COMM_SET_MCCONF_TEMP* expects. Built portably from frexp() - the exponent
// biased by 126, the significand as the fraction above 0.5 - rather than by
// copying the float's bits, but for normal values the result is bit-for-bit
// an IEEE-754 float. Anything smaller than ~1.5e-38 is sent as 0.
void bufAppendFloat32Auto(uint8_t *b, float number, int32_t *i) {
    if (fabsf(number) < 1.5e-38f) number = 0.0f;

    int   e   = 0;
    float sig = frexpf(number, &e);
    float sigAbs = fabsf(sig);
    uint32_t sigI = 0;

    if (sigAbs >= 0.5f) {
        sigI = (uint32_t)((sigAbs - 0.5f) * 2.0f * 8388608.0f);
        e += 126;
    }

    uint32_t res = (((uint32_t)e & 0xFF) << 23) | (sigI & 0x7FFFFF);
    if (sig < 0) res |= 1UL << 31;

    bufAppendUint32(b, res, i);
}

// Mirror of buffer_get_float32_auto() in the bldc firmware - the exact
// inverse of bufAppendFloat32Auto() above.
float bufGetFloat32Auto(const uint8_t *b, int32_t *i) {
    const uint32_t res = (uint32_t)bufGetInt32(b, i);

    int      e     = (int)((res >> 23) & 0xFF);
    uint32_t sigI  = res & 0x7FFFFF;
    const bool neg = res & (1UL << 31);

    float sig = 0.0f;
    if (e != 0 || sigI != 0) {
        sig = (float)sigI / (8388608.0f * 2.0f) + 0.5f;
        e -= 126;
    }
    if (neg) sig = -sig;

    return ldexpf(sig, e);
}

// ---------------------------------------------------------------------------
//  CRC16/XMODEM
// ---------------------------------------------------------------------------
uint16_t crc16(const uint8_t *buf, uint16_t len) {
    uint16_t crc = 0;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= (uint16_t)buf[i] << 8;
        for (uint8_t b = 0; b < 8; b++) {
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

uint16_t buildFrame(const uint8_t *payload, uint16_t len, uint8_t *out, uint16_t outCap) {
    if (len == 0) return 0;
    const uint16_t need = (uint16_t)(len + (len <= 255 ? 5 : 6));
    if (need > outCap) return 0;

    int32_t idx = 0;
    if (len <= 255) {
        out[idx++] = 2;
        out[idx++] = (uint8_t)len;
    } else {
        out[idx++] = 3;
        out[idx++] = (uint8_t)(len >> 8);
        out[idx++] = (uint8_t)(len & 0xFF);
    }
    memcpy(&out[idx], payload, len);
    idx += len;

    const uint16_t crc = crc16(payload, len);
    out[idx++] = (uint8_t)(crc >> 8);
    out[idx++] = (uint8_t)(crc & 0xFF);
    out[idx++] = 3;
    return (uint16_t)idx;
}

// ---------------------------------------------------------------------------
//  Frame sniffer
// ---------------------------------------------------------------------------
bool FrameSniffer::feed(uint8_t byte) {
    switch (_state) {
    case WAIT_START:
        if (byte == 2) {            // short frame, 1 length byte
            _state = LEN_LO;
            _len = 0;
        } else if (byte == 3) {     // long frame, 2 length bytes
            _state = LEN_HI;
            _len = 0;
        }
        // anything else is noise between frames; stay put
        break;

    case LEN_HI:
        _len = (uint16_t)byte << 8;
        _state = LEN_LO;
        break;

    case LEN_LO:
        _len |= byte;
        if (_len == 0) { reset(); break; }
        _idx = 0;
        _oversize = (_len > kMaxPayload);
        _state = PAYLOAD;
        break;

    case PAYLOAD:
        if (!_oversize) _buf[_idx] = byte;
        if (++_idx >= _len) _state = CRC_HI;
        break;

    case CRC_HI:
        _crc = (uint16_t)byte << 8;
        _state = CRC_LO;
        break;

    case CRC_LO:
        _crc |= byte;
        _state = END;
        break;

    case END: {
        const bool ok = (byte == 3) && !_oversize && (crc16(_buf, _len) == _crc);
        reset();
        return ok;
    }
    }
    return false;
}

// ---------------------------------------------------------------------------
//  GET_VALUES parsing
//
//  Works for the full packet and for the selective variant: the selective
//  reply carries the same fields in the same order, preceded by the mask.
// ---------------------------------------------------------------------------
bool parseValues(const uint8_t *payload, uint16_t len, Telemetry &t) {
    if (len < 2) return false;

    const uint8_t id = payload[0];
    int32_t  ind  = 1;
    uint32_t mask = 0xFFFFFFFF;

    switch (id) {
    case COMM_GET_VALUES:
    case COMM_GET_VALUES_SETUP:
        break;
    case COMM_GET_VALUES_SELECTIVE:
    case COMM_GET_VALUES_SETUP_SELECTIVE:
        if (len < 5) return false;
        mask = (uint32_t)bufGetInt32(payload, &ind);
        break;
    default:
        return false;
    }

    // Bounds-checked field reader: bail out the moment the packet runs short
    // rather than walking off the end of a truncated reply.
    #define NEED(n) do { if (ind + (n) > (int32_t)len) return false; } while (0)

    if (mask & VAL_TEMP_FET)           { NEED(2); t.tempFet      = bufGetFloat16(payload, 10.0f, &ind); }
    if (mask & VAL_TEMP_MOTOR)         { NEED(2); t.tempMotor    = bufGetFloat16(payload, 10.0f, &ind); }
    if (mask & VAL_CURRENT_MOTOR)      { NEED(4); t.currentMotor = bufGetFloat32(payload, 100.0f, &ind); }
    if (mask & VAL_CURRENT_IN)         { NEED(4); t.currentIn    = bufGetFloat32(payload, 100.0f, &ind); }
    if (mask & VAL_ID)                 { NEED(4); ind += 4; }
    if (mask & VAL_IQ)                 { NEED(4); ind += 4; }
    if (mask & VAL_DUTY)               { NEED(2); t.duty         = bufGetFloat16(payload, 1000.0f, &ind); }
    if (mask & VAL_RPM)                { NEED(4); t.erpm         = bufGetFloat32(payload, 1.0f, &ind); }
    if (mask & VAL_V_IN)               { NEED(2); t.voltage      = bufGetFloat16(payload, 10.0f, &ind); }
    if (mask & VAL_AMP_HOURS)          { NEED(4); t.ampHours     = bufGetFloat32(payload, 10000.0f, &ind); }
    if (mask & VAL_AMP_HOURS_CHARGED)  { NEED(4); t.ampHoursCharged = bufGetFloat32(payload, 10000.0f, &ind); }
    if (mask & VAL_WATT_HOURS)         { NEED(4); t.wattHours    = bufGetFloat32(payload, 10000.0f, &ind); }
    if (mask & VAL_WATT_HOURS_CHARGED) { NEED(4); t.wattHoursCharged = bufGetFloat32(payload, 10000.0f, &ind); }
    if (mask & VAL_TACHOMETER)         { NEED(4); t.tachometer    = bufGetInt32(payload, &ind); }
    if (mask & VAL_TACHOMETER_ABS)     { NEED(4); t.tachometerAbs = bufGetInt32(payload, &ind); }
    if (mask & VAL_FAULT)              { NEED(1); t.faultCode     = payload[ind++]; }

    #undef NEED

    t.lastUpdateMs = millis();
    return true;
}

// ---------------------------------------------------------------------------
//  COMM_GET_MCCONF battery config
// ---------------------------------------------------------------------------
bool parseMcconfBattery(const uint8_t *payload, uint16_t len, BatteryConfig &out) {
    if (len != 1 + MCCONF_LEN || payload[0] != COMM_GET_MCCONF) return false;

    const uint8_t battType = payload[1 + MCCONF_OFF_SI_BATTERY_TYPE];
    const uint8_t cells    = payload[1 + MCCONF_OFF_SI_BATTERY_CELLS];
    int32_t ind = 1 + (int32_t)MCCONF_OFF_SI_BATTERY_AH;
    const float ah = bufGetFloat32Auto(payload, &ind);

    // Sanity range: catches a firmware build whose fields happen to still
    // land on a MCCONF_LEN-sized reply but shifted, without decoding garbage
    // into the SoC/range math.
    if (cells < 1 || cells > 200 || !(ah > 0.05f && ah < 500.0f)) return false;

    out.cells = cells;
    out.ah    = ah;
    switch (battType) {
    case MCCONF_BATTERY_TYPE_LIION:   out.chemistry = 1; break;
    case MCCONF_BATTERY_TYPE_LIFEPO4: out.chemistry = 0; break;
    default:                          out.chemistry = -1; break;
    }
    out.valid = true;
    return true;
}

// ---------------------------------------------------------------------------
bool parseMcconfGeometry(const uint8_t *payload, uint16_t len, GeometryConfig &out) {
    if (len != 1 + MCCONF_LEN || payload[0] != COMM_GET_MCCONF) return false;

    const uint8_t poles = payload[1 + MCCONF_OFF_SI_MOTOR_POLES];
    int32_t ind = 1 + (int32_t)MCCONF_OFF_SI_GEAR_RATIO;
    const float gear = bufGetFloat32Auto(payload, &ind);
    ind = 1 + (int32_t)MCCONF_OFF_SI_WHEEL_DIAMETER;
    const float wheelM = bufGetFloat32Auto(payload, &ind);

    // Same sanity range idea as the battery fields. Generous on purpose:
    // gear ratios below 1 exist, and the firmware's own default wheel is an
    // 83 mm skateboard wheel. Poles must be even - magnets come in pairs.
    if (poles < 2 || (poles & 1) ||
        !(gear > 0.05f && gear < 100.0f) ||
        !(wheelM > 0.01f && wheelM < 3.0f)) return false;

    out.motorPoles     = poles;
    out.gearRatio      = gear;
    out.wheelDiameterM = wheelM;
    out.valid          = true;
    return true;
}

// ---------------------------------------------------------------------------
//  CAN status broadcasts
//
//  Field widths/scales per vedderb/bldc comm/comm_can.c's send_status_*()
//  functions. STATUS carries erpm/current/duty; STATUS_2..5 spread the rest
//  of GET_VALUES across a handful of 8-byte frames.
// ---------------------------------------------------------------------------
bool parseCanStatus(uint8_t canPacketId, const uint8_t *data, uint8_t len, Telemetry &t) {
    int32_t ind = 0;

    switch (canPacketId) {
    case CAN_PACKET_STATUS:
        if (len < 8) return false;
        t.erpm         = (float)bufGetInt32(data, &ind);
        t.currentMotor = bufGetFloat16(data, 10.0f, &ind);
        t.duty         = bufGetFloat16(data, 1000.0f, &ind);
        break;

    case CAN_PACKET_STATUS_2:
        if (len < 8) return false;
        t.ampHours        = bufGetFloat32(data, 10000.0f, &ind);
        t.ampHoursCharged = bufGetFloat32(data, 10000.0f, &ind);
        break;

    case CAN_PACKET_STATUS_3:
        if (len < 8) return false;
        t.wattHours        = bufGetFloat32(data, 10000.0f, &ind);
        t.wattHoursCharged = bufGetFloat32(data, 10000.0f, &ind);
        break;

    case CAN_PACKET_STATUS_4:
        if (len < 6) return false;
        t.tempFet   = bufGetFloat16(data, 10.0f, &ind);
        t.tempMotor = bufGetFloat16(data, 10.0f, &ind);
        t.currentIn = bufGetFloat16(data, 10.0f, &ind);
        // pid_pos_now (int16/50) follows; the dash does not use it.
        break;

    case CAN_PACKET_STATUS_5:
        if (len < 6) return false;
        t.tachometer    = bufGetInt32(data, &ind);
        t.tachometerAbs = t.tachometer;   // see the note on Telemetry::tachometerAbs
        t.voltage       = bufGetFloat16(data, 10.0f, &ind);
        break;

    default:
        return false;
    }

    t.lastUpdateMs = millis();
    return true;
}

// ---------------------------------------------------------------------------
//  CAN buffer tunnel - send side
// ---------------------------------------------------------------------------
int buildTunnelFrames(uint8_t targetId, uint8_t localId, const uint8_t *payload,
                      uint16_t len, bool expectReply, CanFrame *out, int outCap) {
    if (len == 0 || len > CanTunnel::kMaxPayload || outCap < 1) return 0;

    if (len <= 6) {
        out[0].id  = ((uint32_t)CAN_PACKET_PROCESS_SHORT_BUFFER << 8) | targetId;
        out[0].dlc = (uint8_t)(2 + len);
        out[0].data[0] = localId;
        out[0].data[1] = expectReply ? 0 : 2;
        memcpy(out[0].data + 2, payload, len);
        return 1;
    }

    int n = 0;
    uint16_t offset = 0;
    while (offset < len) {
        if (n >= outCap) return 0;             // did not fit: caller must not send a partial tunnel
        CanFrame &f = out[n++];
        if (offset <= 255) {
            const uint8_t chunk = (uint8_t)min((uint16_t)7, (uint16_t)(len - offset));
            f.id  = ((uint32_t)CAN_PACKET_FILL_RX_BUFFER << 8) | targetId;
            f.dlc = (uint8_t)(1 + chunk);
            f.data[0] = (uint8_t)offset;
            memcpy(f.data + 1, payload + offset, chunk);
            offset = (uint16_t)(offset + chunk);
        } else {
            const uint8_t chunk = (uint8_t)min((uint16_t)6, (uint16_t)(len - offset));
            f.id  = ((uint32_t)CAN_PACKET_FILL_RX_BUFFER_LONG << 8) | targetId;
            f.dlc = (uint8_t)(2 + chunk);
            f.data[0] = (uint8_t)(offset >> 8);
            f.data[1] = (uint8_t)(offset & 0xFF);
            memcpy(f.data + 2, payload + offset, chunk);
            offset = (uint16_t)(offset + chunk);
        }
    }

    if (n >= outCap) return 0;
    CanFrame &trailer = out[n++];
    const uint16_t crc = crc16(payload, len);
    trailer.id  = ((uint32_t)CAN_PACKET_PROCESS_RX_BUFFER << 8) | targetId;
    trailer.dlc = 6;
    trailer.data[0] = localId;
    trailer.data[1] = expectReply ? 0 : 2;
    trailer.data[2] = (uint8_t)(len >> 8);
    trailer.data[3] = (uint8_t)(len & 0xFF);
    trailer.data[4] = (uint8_t)(crc >> 8);
    trailer.data[5] = (uint8_t)(crc & 0xFF);
    return n;
}

// ---------------------------------------------------------------------------
//  CAN buffer tunnel - receive side
// ---------------------------------------------------------------------------
bool CanTunnel::feed(const CanFrame &f, uint8_t localId) {
    if (f.dlc < 1 || canEidAddress(f.id) != localId) return false;

    switch (canEidPacketId(f.id)) {
    case CAN_PACKET_FILL_RX_BUFFER: {
        const uint16_t offset = f.data[0];
        const uint16_t n = (uint16_t)(f.dlc - 1);
        if ((uint32_t)offset + n <= kMaxPayload) memcpy(_buf + offset, f.data + 1, n);
        return false;
    }
    case CAN_PACKET_FILL_RX_BUFFER_LONG: {
        if (f.dlc < 2) return false;
        const uint16_t offset = ((uint16_t)f.data[0] << 8) | f.data[1];
        const uint16_t n = (uint16_t)(f.dlc - 2);
        if ((uint32_t)offset + n <= kMaxPayload) memcpy(_buf + offset, f.data + 2, n);
        return false;
    }
    case CAN_PACKET_PROCESS_SHORT_BUFFER: {
        if (f.dlc < 2) return false;
        _sourceId = f.data[0];
        _len = (uint16_t)(f.dlc - 2);
        memcpy(_buf, f.data + 2, _len);
        return true;
    }
    case CAN_PACKET_PROCESS_RX_BUFFER: {
        if (f.dlc < 6) return false;
        _sourceId = f.data[0];
        const uint16_t len = ((uint16_t)f.data[2] << 8) | f.data[3];
        const uint16_t crc = ((uint16_t)f.data[4] << 8) | f.data[5];
        if (len > kMaxPayload || crc16(_buf, len) != crc) return false;
        _len = len;
        return true;
    }
    default:
        return false;
    }
}

const char *faultToString(uint8_t code) {
    switch (code) {
    case FAULT_NONE:                    return "";
    case FAULT_OVER_VOLTAGE:            return "OVER VOLTAGE";
    case FAULT_UNDER_VOLTAGE:           return "UNDER VOLTAGE";
    case FAULT_DRV:                     return "DRV FAULT";
    case FAULT_ABS_OVER_CURRENT:        return "OVER CURRENT";
    case FAULT_OVER_TEMP_FET:           return "CONTROLLER HOT";
    case FAULT_OVER_TEMP_MOTOR:         return "MOTOR HOT";
    case FAULT_GATE_DRIVER_OVER_VOLTAGE:  return "GATE DRV OV";
    case FAULT_GATE_DRIVER_UNDER_VOLTAGE: return "GATE DRV UV";
    case FAULT_MCU_UNDER_VOLTAGE:       return "MCU UV";
    case FAULT_BOOTING_FROM_WATCHDOG_RESET: return "WATCHDOG RESET";
    case FAULT_ENCODER_SPI:             return "ENCODER SPI";
    case FAULT_FLASH_CORRUPTION:        return "FLASH CORRUPT";
    case FAULT_UNBALANCED_CURRENTS:     return "UNBALANCED I";
    default:                            return "FAULT";
    }
}

} // namespace vesc
