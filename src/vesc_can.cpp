#include "vesc_can.h"
#include "config.h"
#include "driver/twai.h"

namespace vescan {

// ---------------------------------------------------------------------------
void begin() {
    twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(
        (gpio_num_t)VESC_LINK_TX_PIN, (gpio_num_t)VESC_LINK_RX_PIN, TWAI_MODE_NORMAL);
    g.rx_queue_len = 32;
    g.tx_queue_len = 16;

    twai_timing_config_t t;
#if VESC_CAN_BITRATE == 1000000
    t = TWAI_TIMING_CONFIG_1MBITS();
#elif VESC_CAN_BITRATE == 500000
    t = TWAI_TIMING_CONFIG_500KBITS();
#elif VESC_CAN_BITRATE == 250000
    t = TWAI_TIMING_CONFIG_250KBITS();
#elif VESC_CAN_BITRATE == 125000
    t = TWAI_TIMING_CONFIG_125KBITS();
#else
#error "Unsupported VESC_CAN_BITRATE - add a TWAI_TIMING_CONFIG_*BITS() case"
#endif

    twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g, &t, &f) != ESP_OK) {
        Serial.println("[can] driver install failed");
        return;
    }
    if (twai_start() != ESP_OK) {
        Serial.println("[can] start failed");
        return;
    }
    Serial.printf("[can] TWAI up @%d  tx=%d rx=%d\n",
                  VESC_CAN_BITRATE, VESC_LINK_TX_PIN, VESC_LINK_RX_PIN);
}

// ---------------------------------------------------------------------------
bool poll(CanFrame &out) {
    twai_message_t msg;
    if (twai_receive(&msg, 0) != ESP_OK) return false;
    if (!msg.extd) return false;               // VESC only ever uses extended ids

    out.id  = msg.identifier;
    out.dlc = msg.data_length_code;
    memcpy(out.data, msg.data, min((int)out.dlc, 8));
    return true;
}

// ---------------------------------------------------------------------------
bool send(const CanFrame &f) {
    twai_message_t msg = {};
    msg.identifier = f.id;
    msg.extd = 1;
    msg.data_length_code = f.dlc;
    memcpy(msg.data, f.data, min((int)f.dlc, 8));
    return twai_transmit(&msg, pdMS_TO_TICKS(20)) == ESP_OK;
}

} // namespace vescan
