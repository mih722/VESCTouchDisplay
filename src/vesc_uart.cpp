#include "vesc_uart.h"
#include "config.h"

// Compiled on every board, but only does anything in a UART build - the CAN
// build never calls it, and this keeps UART1 untouched there.
#if VESC_LINK_UART

static HardwareSerial s_uart(VESC_UART_NUM);

namespace vescuart {

// ---------------------------------------------------------------------------
void begin() {
    // Room for a whole COMM_GET_MCCONF reply (~500 bytes, the largest thing
    // the dash asks for) twice over, so a link task busy elsewhere for a few
    // milliseconds never loses bytes mid-frame.
    s_uart.setRxBufferSize(1024);
    s_uart.begin(VESC_UART_BAUD, SERIAL_8N1, VESC_LINK_RX_PIN, VESC_LINK_TX_PIN);
    Serial.printf("[uart] UART%d up @%d  tx=%d rx=%d\n",
                  VESC_UART_NUM, VESC_UART_BAUD, VESC_LINK_TX_PIN, VESC_LINK_RX_PIN);
}

// ---------------------------------------------------------------------------
size_t read(uint8_t *buf, size_t cap) {
    const int avail = s_uart.available();
    if (avail <= 0) return 0;
    // Never ask for more than has arrived: readBytes() would otherwise sit
    // out its timeout waiting for the rest.
    return s_uart.readBytes(buf, min((size_t)avail, cap));
}

// ---------------------------------------------------------------------------
size_t write(const uint8_t *data, size_t len) {
    return s_uart.write(data, len);
}

} // namespace vescuart

#endif // VESC_LINK_UART
