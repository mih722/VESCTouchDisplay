#include "vesc_can.h"
#include "config.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "esp_attr.h"

// ============================================================================
//  TWAI transport for the pioarduino / IDF-5.5 toolchain (the ESP32-C6 boards
//  and the ESP32-S3 AMOLED - the same driver API on both chips).
//
//  IDF 5.5 deprecates the legacy driver/twai.h API vesc_can.cpp (the CYD
//  target) uses in favour of a new, event-driven, handle-based API
//  (esp_driver_twai: twai_new_node_onchip(), callback-registered RX). There
//  is no polling-style receive in the new API - twai_node_receive_from_isr()
//  is only callable from inside the on_rx_done callback - so this file
//  reconstructs the old poll() interface on top of it with a small FreeRTOS
//  queue: the ISR callback copies each frame out and pushes it to the queue,
//  poll() just drains the queue non-blocking. vesc_link.cpp is unaware of
//  any of this - it only ever calls vescan::begin/poll/send.
//
//  The esp_driver_twai API is new in IDF 5.5, so a platform update is the
//  most likely thing to break this file. If it stops compiling, check the
//  field names used below against <esp_twai_onchip.h> / <esp_twai_types.h>
//  in the installed tree (~/.platformio/packages/
//  framework-arduinoespressif32-libs/<chip>/include/esp_driver_twai/
//  include/).
// ============================================================================

static twai_node_handle_t s_node = nullptr;
static QueueHandle_t      s_rxQueue = nullptr;

// Set from the ISR-context on_state_change callback, read/acted on from
// send() (task context). A bare bool read/write is atomic on both chips this
// file builds for - including the dual-core S3, where the ISR and the link
// task can be on different cores - but it must still be volatile so the
// compiler doesn't cache the task-side read across loop iterations.
static volatile bool s_busOff = false;
// Latched from the ISR-context on_error callback so send() can report which
// specific CAN-level error is actually happening (no-ACK vs a bit/stuff/form
// error) instead of just "bus-off" - see the .val union member's bit layout
// in twai_error_flags_t (esp_twai_types.h). Sticky rather than cleared per
// frame, so it always reflects the most recent failure reason.
static volatile uint32_t s_lastErrFlags = 0;
// Task-context only (both touched solely from send() below) - just tracks
// whether the "bus-off"/"recovered" transition has already been logged.
static bool     s_busOffLogged = false;
static uint32_t s_lastRecoverAttempt = 0;

// ---------------------------------------------------------------------------
//  Runs in ISR context. Copy the frame out immediately - the driver's own
//  buffer is only valid for the duration of this callback. IRAM_ATTR because
//  esp_twai_types.h documents that callbacks must live in IRAM when
//  TWAI_ISR_CACHE_SAFE is enabled - a non-IRAM callback invoked from an ISR
//  while flash cache happens to be disabled (e.g. during a flash write
//  elsewhere) crashes/reboots, which only shows up once real CAN traffic
//  actually starts arriving (nothing to receive/error on = never invoked).
// ---------------------------------------------------------------------------
static bool IRAM_ATTR onRxDone(twai_node_handle_t handle, const twai_rx_done_event_data_t *edata, void *ctx) {
    uint8_t buf[8];
    twai_frame_t frame = {};
    frame.buffer = buf;
    frame.buffer_len = sizeof(buf);

    if (twai_node_receive_from_isr(handle, &frame) != ESP_OK) return false;
    if (!frame.header.ide) return false;          // VESC only ever uses extended ids

    CanFrame f;
    f.id  = frame.header.id;
    f.dlc = (uint8_t)frame.buffer_len;
    memcpy(f.data, buf, min((size_t)f.dlc, sizeof(f.data)));

    BaseType_t woken = pdFALSE;
    xQueueSendFromISR(s_rxQueue, &f, &woken);
    return woken == pdTRUE;                        // request a context switch if needed
}

// ---------------------------------------------------------------------------
//  Also ISR context. The driver enters bus-off after the TX error counter
//  hits 256 - almost always a wiring problem (missing/wrong termination, the
//  transceiver not powered, CANH/CANL swapped or not reaching the VESC) or a
//  bitrate mismatch, not a firmware bug, but without this callback the node
//  would stay wedged in bus-off forever once it happens (the driver does not
//  auto-recover - see twai_node_recover()'s doc comment in esp_twai.h). Just
//  latch the flag here; the actual twai_node_recover() call happens from
//  send() below since recovery is not documented as ISR-safe.
// ---------------------------------------------------------------------------
static bool IRAM_ATTR onStateChange(twai_node_handle_t handle, const twai_state_change_event_data_t *edata, void *ctx) {
    s_busOff = (edata->new_sta == TWAI_ERROR_BUS_OFF);
    return false;
}

// ---------------------------------------------------------------------------
//  Also ISR context. Fires on every failed transmission with the specific
//  reason - this is what actually distinguishes "nothing is ACKing us"
//  (ack_err) from a signal-integrity/timing problem (bit_err/stuff_err/
//  form_err), which "bus off" alone doesn't tell us.
// ---------------------------------------------------------------------------
static bool IRAM_ATTR onError(twai_node_handle_t handle, const twai_error_event_data_t *edata, void *ctx) {
    s_lastErrFlags = edata->err_flags.val;
    return false;
}

namespace vescan {

// ---------------------------------------------------------------------------
void begin() {
    s_rxQueue = xQueueCreate(32, sizeof(CanFrame));

    twai_onchip_node_config_t cfg = {};
    cfg.io_cfg.tx = (gpio_num_t)VESC_LINK_TX_PIN;
    cfg.io_cfg.rx = (gpio_num_t)VESC_LINK_RX_PIN;
    // Zero-init leaves these at GPIO_NUM_0 (a real pin - BAT_ADC on the C6
    // boards, a strapping pin on the S3), not "unused" - esp_twai_onchip.h
    // documents GPIO_NUM_NC (-1) as the explicit opt-out, so set it.
    cfg.io_cfg.quanta_clk_out = GPIO_NUM_NC;
    cfg.io_cfg.bus_off_indicator = GPIO_NUM_NC;
    cfg.bit_timing.bitrate = VESC_CAN_BITRATE;
    cfg.tx_queue_depth = 16;
    cfg.intr_priority = 0;

    if (twai_new_node_onchip(&cfg, &s_node) != ESP_OK) {
        Serial.println("[can] twai_new_node_onchip failed");
        return;
    }

    twai_event_callbacks_t cbs = {};
    cbs.on_rx_done = onRxDone;
    cbs.on_state_change = onStateChange;
    cbs.on_error = onError;
    if (twai_node_register_event_callbacks(s_node, &cbs, nullptr) != ESP_OK) {
        Serial.println("[can] register_event_callbacks failed");
        return;
    }

    if (twai_node_enable(s_node) != ESP_OK) {
        Serial.println("[can] twai_node_enable failed");
        return;
    }

    Serial.printf("[can] TWAI (idf5) up @%d  tx=%d rx=%d\n",
                  VESC_CAN_BITRATE, VESC_LINK_TX_PIN, VESC_LINK_RX_PIN);
}

// ---------------------------------------------------------------------------
bool poll(CanFrame &out) {
    return xQueueReceive(s_rxQueue, &out, 0) == pdTRUE;
}

// ---------------------------------------------------------------------------
//  While bus-off, every twai_node_transmit() call fails and the driver logs
//  its own "node is bus off" error each time - that log line is coming from
//  the IDF driver itself, not from this file, so the only way to stop it
//  repeating is to stop calling transmit(). Attempt recovery on a 1s cadence
//  instead; a real recovery (see twai_node_recover()'s doc comment) needs
//  128 consecutive idle bus periods, which will never happen if the wiring
//  fault causing the bus-off is still present, so this will keep retrying
//  harmlessly until it is fixed.
// ---------------------------------------------------------------------------
bool send(const CanFrame &f) {
    if (!s_node) return false;

    if (s_busOff) {
        if (!s_busOffLogged) {
            s_busOffLogged = true;
            twai_error_flags_t flags;
            flags.val = s_lastErrFlags;
            Serial.printf("[can] bus-off - last TX error: %s%s%s%s%s(0x%02lX) - "
                          "check wiring (termination, transceiver power, CANH/CANL) and that "
                          "the VESC's CAN bitrate matches; retrying recovery\n",
                          flags.ack_err   ? "ACK "   : "",   // no node acknowledged the frame
                          flags.bit_err   ? "BIT "   : "",   // driven level != sensed level
                          flags.stuff_err ? "STUFF " : "",   // 6+ same-polarity bits in a row
                          flags.form_err  ? "FORM "  : "",   // fixed-form bit violated
                          flags.arb_lost  ? "ARB "   : "",   // lost arbitration (usually benign)
                          (unsigned long)flags.val);
        }
        const uint32_t now = millis();
        if (now - s_lastRecoverAttempt >= 1000) {
            s_lastRecoverAttempt = now;
            twai_node_recover(s_node);
        }
        return false;
    }
    if (s_busOffLogged) {
        s_busOffLogged = false;
        Serial.println("[can] bus recovered");
    }

    // static, not stack-local: twai_node_transmit() does not always transmit
    // synchronously - if the hardware is mid-frame (real bus traffic, which
    // is exactly what starts once actually connected to the VESC), the
    // driver queues a *pointer* to this frame/buffer for its TX-done ISR to
    // pick up later. A stack-local frame here would already be gone by then
    // (send() long since returned, that stack space reused for something
    // else) - the ISR would dereference freed memory, including buffer_len,
    // which is exactly what feeds twaifd_len2dlc() and asserts when the
    // garbage it reads happens to exceed 64
    // (components/hal/twai_hal_v2.c:131 in IDF 5.5.2).
    static uint8_t data[8];
    memcpy(data, f.data, min((size_t)f.dlc, sizeof(data)));

    static twai_frame_t frame;
    frame = {};
    frame.header.id  = f.id;
    frame.header.ide = true;
    frame.buffer     = data;
    frame.buffer_len = f.dlc;

    return twai_node_transmit(s_node, &frame, pdMS_TO_TICKS(20)) == ESP_OK;
}

} // namespace vescan
