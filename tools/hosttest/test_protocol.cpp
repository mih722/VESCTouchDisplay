// Host-side sanity test for the VESC protocol layer.
// Not part of the firmware build; run from tools/hosttest/ with:
//   g++ -std=gnu++17 -Wall -Istub -I../../src -I../../include \
//       test_protocol.cpp ../../src/vesc_protocol.cpp -o t && ./t
#include "vesc_protocol.h"
#include <cstdio>
#include <cmath>

// The only stub symbol this test needs (parseValues timestamps the telemetry).
uint32_t millis() { return 12345; }

using namespace vesc;

static int failures = 0;
#define CHECK(cond, msg) do { if(!(cond)){ printf("FAIL: %s\n", msg); failures++; } } while(0)

// Independent CRC16/XMODEM reference to cross-check the firmware version.
static uint16_t crcRef(const uint8_t *buf, uint16_t len) {
    uint16_t cksum = 0;
    for (uint16_t i = 0; i < len; i++) {
        cksum ^= (uint16_t)buf[i] << 8;
        for (int b = 0; b < 8; b++)
            cksum = (cksum & 0x8000) ? (uint16_t)((cksum << 1) ^ 0x1021)
                                     : (uint16_t)(cksum << 1);
    }
    return cksum;
}

int main() {
    // ---- CRC16/XMODEM known answer -------------------------------------
    const uint8_t check[] = {'1','2','3','4','5','6','7','8','9'};
    CHECK(crc16(check, 9) == 0x31C3, "CRC16/XMODEM check value 0x31C3");
    CHECK(crc16(check, 9) == crcRef(check, 9), "CRC matches reference");

    // ---- framing --------------------------------------------------------
    const uint8_t getValues[1] = { COMM_GET_VALUES };
    uint8_t frame[16];
    uint16_t n = buildFrame(getValues, 1, frame, sizeof(frame));
    CHECK(n == 6, "short frame length");
    CHECK(frame[0] == 2 && frame[1] == 1 && frame[2] == COMM_GET_VALUES, "short frame header");
    CHECK(frame[n - 1] == 3, "frame terminator");

    // long frame uses the 0x03 start byte and a 16-bit length
    uint8_t big[300];
    memset(big, 0xAB, sizeof(big));
    uint8_t bigFrame[400];
    n = buildFrame(big, 300, bigFrame, sizeof(bigFrame));
    CHECK(n == 306, "long frame length");
    CHECK(bigFrame[0] == 3 && bigFrame[1] == 1 && bigFrame[2] == 44, "long frame header");

    // ---- sniffer round-trip --------------------------------------------
    {
        FrameSniffer sn;
        uint8_t f[16];
        uint16_t fn = buildFrame(getValues, 1, f, sizeof(f));
        bool got = false;
        // prepend junk to prove resynchronisation works
        uint8_t junk[] = {0x55, 0x00, 0xFF};
        for (uint8_t b : junk) sn.feed(b);
        for (uint16_t i = 0; i < fn; i++) got |= sn.feed(f[i]);
        CHECK(got, "sniffer recovered the frame after junk");
        CHECK(sn.length() == 1 && sn.payload()[0] == COMM_GET_VALUES,
              "payload stays readable after delivery");
    }
    {
        FrameSniffer sn;
        uint8_t f[16];
        uint16_t fn = buildFrame(getValues, 1, f, sizeof(f));
        f[fn - 2] ^= 0xFF;                 // corrupt the CRC
        bool got = false;
        for (uint16_t i = 0; i < fn; i++) got |= sn.feed(f[i]);
        CHECK(!got, "sniffer rejects a bad CRC");
    }

    // ---- float32 "auto" encoding ---------------------------------------
    // 1.0 -> exponent 127 (126+1), significand 0  => 0x3F800000
    {
        uint8_t b[4]; int32_t i = 0;
        bufAppendFloat32Auto(b, 1.0f, &i);
        uint32_t v = ((uint32_t)b[0] << 24) | ((uint32_t)b[1] << 16) | ((uint32_t)b[2] << 8) | b[3];
        CHECK(v == 0x3F800000u, "float32_auto(1.0) == 0x3F800000");
    }
    {
        uint8_t b[4]; int32_t i = 0;
        bufAppendFloat32Auto(b, -1.0f, &i);
        uint32_t v = ((uint32_t)b[0] << 24) | ((uint32_t)b[1] << 16) | ((uint32_t)b[2] << 8) | b[3];
        CHECK(v == 0xBF800000u, "float32_auto(-1.0) == 0xBF800000");
    }
    {
        uint8_t b[4]; int32_t i = 0;
        bufAppendFloat32Auto(b, 0.0f, &i);
        CHECK(b[0] == 0 && b[1] == 0 && b[2] == 0 && b[3] == 0, "float32_auto(0) == 0");
    }
    {   // 0.85 should round-trip through the IEEE-754 interpretation
        uint8_t b[4]; int32_t i = 0;
        bufAppendFloat32Auto(b, 0.85f, &i);
        uint32_t v = ((uint32_t)b[0] << 24) | ((uint32_t)b[1] << 16) | ((uint32_t)b[2] << 8) | b[3];
        float back; memcpy(&back, &v, 4);
        CHECK(fabsf(back - 0.85f) < 1e-6f, "float32_auto(0.85) round-trips");
    }

    // ---- GET_VALUES parsing --------------------------------------------
    {
        // Build a synthetic full GET_VALUES reply and read it back.
        uint8_t p[128];
        int32_t i = 0;
        p[i++] = COMM_GET_VALUES;
        bufAppendInt16(p, (int16_t)(38.5f * 10), &i);        // temp fet
        bufAppendInt16(p, (int16_t)(25.0f * 10), &i);        // temp motor
        bufAppendInt32(p, (int32_t)(12.34f * 100), &i);      // motor current
        bufAppendInt32(p, (int32_t)(6.50f * 100), &i);       // input current
        bufAppendInt32(p, 0, &i);                            // id
        bufAppendInt32(p, 0, &i);                            // iq
        bufAppendInt16(p, (int16_t)(0.234f * 1000), &i);     // duty
        bufAppendInt32(p, 4200, &i);                         // erpm
        bufAppendInt16(p, (int16_t)(52.4f * 10), &i);        // v_in
        bufAppendInt32(p, (int32_t)(1.2345f * 10000), &i);   // Ah
        bufAppendInt32(p, (int32_t)(0.1000f * 10000), &i);   // Ah charged
        bufAppendInt32(p, (int32_t)(6.0000f * 10000), &i);   // Wh
        bufAppendInt32(p, (int32_t)(0.5000f * 10000), &i);   // Wh charged
        bufAppendInt32(p, 123456, &i);                       // tacho
        bufAppendInt32(p, 123456, &i);                       // tacho abs
        p[i++] = 0;                                          // fault

        Telemetry t;
        CHECK(parseValues(p, (uint16_t)i, t), "full GET_VALUES parses");
        CHECK(fabsf(t.tempFet - 38.5f) < 0.05f, "temp fet");
        CHECK(fabsf(t.currentMotor - 12.34f) < 0.01f, "motor current");
        CHECK(fabsf(t.duty - 0.234f) < 0.001f, "duty");
        CHECK(fabsf(t.erpm - 4200.0f) < 0.5f, "erpm");
        CHECK(fabsf(t.voltage - 52.4f) < 0.05f, "voltage");
        CHECK(fabsf(t.wattHours - 6.0f) < 0.001f, "watt hours");
        CHECK(t.tachometerAbs == 123456, "tachometer abs");
        CHECK(t.faultCode == 0, "fault code");
    }

    {
        // Selective reply: only voltage + rpm + fault, in field order.
        uint8_t p[32];
        int32_t i = 0;
        p[i++] = COMM_GET_VALUES_SELECTIVE;
        bufAppendUint32(p, VAL_RPM | VAL_V_IN | VAL_FAULT, &i);
        bufAppendInt32(p, -3000, &i);                        // erpm (regen)
        bufAppendInt16(p, (int16_t)(48.0f * 10), &i);        // v_in
        p[i++] = 5;                                          // over temp fet

        Telemetry t;
        CHECK(parseValues(p, (uint16_t)i, t), "selective GET_VALUES parses");
        CHECK(fabsf(t.erpm + 3000.0f) < 0.5f, "selective erpm");
        CHECK(fabsf(t.voltage - 48.0f) < 0.05f, "selective voltage");
        CHECK(t.faultCode == 5, "selective fault");
        CHECK(t.currentMotor == 0.0f, "masked-out field untouched");
    }

    {   // truncated packet must be rejected, not read past the end
        uint8_t p[6] = { COMM_GET_VALUES, 0x01, 0x81, 0x00, 0xFA, 0x00 };
        Telemetry t;
        CHECK(!parseValues(p, 6, t), "truncated GET_VALUES rejected");
    }

    // ---- speed / distance maths ----------------------------------------
    {
        // 30 poles, 1:1, 0.584 m wheel, 4200 erpm
        const float poles = 30, gear = 1.0f, d = 0.584f;
        const float mechRpm  = 4200.0f / (poles / 2.0f);
        const float ms = (mechRpm / gear) * (d * (float)M_PI) / 60.0f;
        const float kph = ms * 3.6f;
        printf("  info: 4200 erpm -> %.1f km/h\n", kph);
        CHECK(kph > 25.0f && kph < 35.0f, "plausible speed for 4200 erpm");

        // distance: matches mc_interface_get_distance()
        const float scale = (d * (float)M_PI) / (3.0f * poles * gear);
        const float meters = 123456.0f * scale;
        printf("  info: 123456 tacho counts -> %.1f m\n", meters);
        CHECK(meters > 2000.0f && meters < 3000.0f, "plausible distance");
    }

    // ---- geometry from COMM_GET_MCCONF ------------------------------------
    {
        uint8_t p[1 + MCCONF_LEN] = {};
        p[0] = COMM_GET_MCCONF;
        p[1 + MCCONF_OFF_SI_MOTOR_POLES] = 16;
        int32_t i = 1 + MCCONF_OFF_SI_GEAR_RATIO;
        bufAppendFloat32Auto(p, 12.6f, &i);
        bufAppendFloat32Auto(p, 0.425f, &i);
        CHECK(i == 1 + MCCONF_OFF_SI_BATTERY_TYPE,
              "si_wheel_diameter ends exactly where si_battery_type starts");

        GeometryConfig g;
        CHECK(parseMcconfGeometry(p, sizeof(p), g), "sane geometry parses");
        CHECK(g.valid && g.motorPoles == 16, "geometry motor poles");
        CHECK(fabsf(g.gearRatio - 12.6f) < 0.001f, "geometry gear ratio");
        CHECK(fabsf(g.wheelDiameterM - 0.425f) < 0.0001f, "geometry wheel diameter");

        GeometryConfig bad;
        CHECK(!parseMcconfGeometry(p, sizeof(p) - 1, bad), "wrong-length mcconf rejected");
        p[1 + MCCONF_OFF_SI_MOTOR_POLES] = 15;
        CHECK(!parseMcconfGeometry(p, sizeof(p), bad), "odd pole count rejected (misaligned read)");
        p[1 + MCCONF_OFF_SI_MOTOR_POLES] = 16;
        i = 1 + MCCONF_OFF_SI_WHEEL_DIAMETER;
        bufAppendFloat32Auto(p, 0.0f, &i);
        CHECK(!parseMcconfGeometry(p, sizeof(p), bad), "zero wheel diameter rejected");
        CHECK(!bad.valid, "a rejected reply leaves the output invalid");
    }

    // ---- CAN status broadcasts -------------------------------------------
    {
        uint8_t d[8]; int32_t i = 0;
        bufAppendInt32(d, 4200, &i);                     // erpm
        bufAppendInt16(d, (int16_t)(12.34f * 10), &i);    // current
        bufAppendInt16(d, (int16_t)(0.234f * 1000), &i);  // duty

        Telemetry t;
        CHECK(parseCanStatus(CAN_PACKET_STATUS, d, 8, t), "CAN_PACKET_STATUS parses");
        CHECK(fabsf(t.erpm - 4200.0f) < 0.5f, "CAN status erpm");
        CHECK(fabsf(t.currentMotor - 12.34f) < 0.05f, "CAN status current");
        CHECK(fabsf(t.duty - 0.234f) < 0.001f, "CAN status duty");
    }
    {
        uint8_t d[8]; int32_t i = 0;
        bufAppendInt32(d, (int32_t)(1.2345f * 10000), &i);
        bufAppendInt32(d, (int32_t)(0.1000f * 10000), &i);
        Telemetry t;
        CHECK(parseCanStatus(CAN_PACKET_STATUS_2, d, 8, t), "CAN_PACKET_STATUS_2 parses");
        CHECK(fabsf(t.ampHours - 1.2345f) < 0.001f, "CAN status2 amp hours");
        CHECK(fabsf(t.ampHoursCharged - 0.1f) < 0.001f, "CAN status2 amp hours charged");
    }
    {
        uint8_t d[8]; int32_t i = 0;
        bufAppendInt16(d, (int16_t)(38.5f * 10), &i);     // temp fet
        bufAppendInt16(d, (int16_t)(25.0f * 10), &i);     // temp motor
        bufAppendInt16(d, (int16_t)(6.50f * 10), &i);     // current in
        bufAppendInt16(d, 0, &i);                         // pid pos (unused)
        Telemetry t;
        CHECK(parseCanStatus(CAN_PACKET_STATUS_4, d, 8, t), "CAN_PACKET_STATUS_4 parses");
        CHECK(fabsf(t.tempFet - 38.5f) < 0.05f, "CAN status4 temp fet");
        CHECK(fabsf(t.tempMotor - 25.0f) < 0.05f, "CAN status4 temp motor");
        CHECK(fabsf(t.currentIn - 6.5f) < 0.05f, "CAN status4 current in");
    }
    {
        uint8_t d[6]; int32_t i = 0;
        bufAppendInt32(d, 123456, &i);                    // tacho
        bufAppendInt16(d, (int16_t)(52.4f * 10), &i);      // v_in
        Telemetry t;
        CHECK(parseCanStatus(CAN_PACKET_STATUS_5, d, 6, t), "CAN_PACKET_STATUS_5 parses");
        CHECK(t.tachometerAbs == 123456, "CAN status5 tachometer");
        CHECK(fabsf(t.voltage - 52.4f) < 0.05f, "CAN status5 voltage");
    }
    {   // truncated status frame must be rejected, not read past the end
        uint8_t d[4] = {0, 0, 0, 0};
        Telemetry t;
        CHECK(!parseCanStatus(CAN_PACKET_STATUS, d, 4, t), "truncated CAN status rejected");
        CHECK(!parseCanStatus(200, d, 4, t), "unknown CAN packet id rejected");
    }

    // ---- CAN buffer tunnel -------------------------------------------------
    {   // short payload (<=6 bytes) fits in one PROCESS_SHORT_BUFFER frame
        const uint8_t payload[3] = { COMM_FW_VERSION, 0xAA, 0xBB };
        CanFrame frames[8];
        int n = buildTunnelFrames(/*target*/ 5, /*local*/ 100, payload, 3, true, frames, 8);
        CHECK(n == 1, "short payload -> one tunnel frame");
        CHECK(canEidPacketId(frames[0].id) == CAN_PACKET_PROCESS_SHORT_BUFFER,
              "short payload uses PROCESS_SHORT_BUFFER");
        CHECK(canEidAddress(frames[0].id) == 5, "frame addressed to the target id");
        CHECK(frames[0].data[0] == 100, "frame carries our local id as source");
        CHECK(frames[0].data[1] == 0, "expectReply sets the send flag to process+reply (0)");

        // The frame is addressed to the recipient (target id 5) - that is
        // the id its own rx buffer logic would check against, not ours.
        CanTunnel rx;
        bool got = false;
        for (int i = 0; i < n; i++) got |= rx.feed(frames[i], 5);
        CHECK(got, "short tunnel round-trips");
        CHECK(rx.length() == 3 && memcmp(rx.payload(), payload, 3) == 0,
              "short tunnel payload matches");
        CHECK(rx.sourceId() == 100, "short tunnel source id preserved");
    }
    {   // long payload spans multiple FILL_RX_BUFFER frames + a trailer
        uint8_t payload[40];
        for (int i = 0; i < 40; i++) payload[i] = (uint8_t)(i * 7 + 1);

        CanFrame frames[16];
        int n = buildTunnelFrames(5, 100, payload, sizeof(payload), false, frames, 16);
        CHECK(n > 1, "long payload spans multiple frames");
        CHECK(canEidPacketId(frames[n - 1].id) == CAN_PACKET_PROCESS_RX_BUFFER,
              "long payload ends with a PROCESS_RX_BUFFER trailer");
        for (int i = 0; i < n - 1; i++) {
            CHECK(canEidPacketId(frames[i].id) == CAN_PACKET_FILL_RX_BUFFER,
                  "40-byte payload only needs the short FILL_RX_BUFFER form");
        }

        CanTunnel rx;
        bool got = false;
        for (int i = 0; i < n; i++) got |= rx.feed(frames[i], 5);
        CHECK(got, "long tunnel round-trips");
        CHECK(rx.length() == sizeof(payload) &&
              memcmp(rx.payload(), payload, sizeof(payload)) == 0,
              "long tunnel payload matches");
    }
    {   // a corrupted trailer CRC must not produce a false-positive payload
        uint8_t payload[20];
        for (int i = 0; i < 20; i++) payload[i] = (uint8_t)i;
        CanFrame frames[8];
        int n = buildTunnelFrames(5, 100, payload, sizeof(payload), true, frames, 8);
        frames[n - 1].data[4] ^= 0xFF;                    // corrupt crc hi byte

        CanTunnel rx;
        bool got = false;
        for (int i = 0; i < n; i++) got |= rx.feed(frames[i], 5);
        CHECK(!got, "tunnel rejects a bad trailer CRC");
    }
    {   // a frame addressed to someone else must be ignored
        const uint8_t payload[3] = { 1, 2, 3 };
        CanFrame frames[4];
        int n = buildTunnelFrames(5, 100, payload, 3, true, frames, 4);
        CanTunnel rx;
        CHECK(!rx.feed(frames[0], 200), "tunnel frame for a different local id ignored");
    }

    printf(failures ? "\n%d FAILURE(S)\n" : "\nall protocol tests passed\n", failures);
    return failures ? 1 : 0;
}
