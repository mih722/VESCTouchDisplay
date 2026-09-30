#include "ble_bridge.h"
#include "config.h"
#include "vesc_link.h"
#include <NimBLEDevice.h>

namespace blebridge {

static NimBLEServer         *s_server = nullptr;
static NimBLECharacteristic *s_txChar = nullptr;
static volatile bool         s_connected = false;
static volatile uint16_t     s_mtu = 23;

// ---------------------------------------------------------------------------
//  NimBLE-Arduino v2 (every pioarduino board - the C6s need it for IDF5
//  support, and the S3 AMOLED shares their toolchain) changed these
//  callback signatures from v1 (the CYD targets, still on v1.4.2):
//  connection info moved from a raw ble_gap_conn_desc* to a
//  NimBLEConnInfo&, and onDisconnect gained a reason code. Gated on
//  BLE_NIMBLE_V2 rather than a library-version macro since the board
//  defines already track exactly which NimBLE major version each
//  environment pulls in.
// ---------------------------------------------------------------------------
#if defined(BOARD_PIOARDUINO)
#define BLE_NIMBLE_V2 1
#endif

class ServerCallbacks : public NimBLEServerCallbacks {
#if defined(BLE_NIMBLE_V2)
    void onConnect(NimBLEServer *server, NimBLEConnInfo &connInfo) override {
        s_connected = true;
        vescLink.setBleConnected(true);
        // Fast, reliable connection: 15-30 ms interval, 4 s supervision.
        server->updateConnParams(connInfo.getConnHandle(), 12, 24, 0, 400);
        Serial.println("[ble] client connected");
    }

    void onDisconnect(NimBLEServer *server, NimBLEConnInfo &connInfo, int reason) override {
        (void)connInfo; (void)reason;
        s_connected = false;
        s_mtu = 23;
        vescLink.setBleConnected(false);
        Serial.println("[ble] client disconnected, advertising again");
        NimBLEDevice::startAdvertising();
    }

    void onMTUChange(uint16_t mtuValue, NimBLEConnInfo &connInfo) override {
        (void)connInfo;
        s_mtu = mtuValue;
        Serial.printf("[ble] MTU = %u\n", mtuValue);
    }
#else
    void onConnect(NimBLEServer *server, ble_gap_conn_desc *desc) override {
        s_connected = true;
        vescLink.setBleConnected(true);
        // Fast, reliable connection: 15-30 ms interval, 4 s supervision.
        server->updateConnParams(desc->conn_handle, 12, 24, 0, 400);
        Serial.println("[ble] client connected");
    }

    void onDisconnect(NimBLEServer *server) override {
        s_connected = false;
        s_mtu = 23;
        vescLink.setBleConnected(false);
        Serial.println("[ble] client disconnected, advertising again");
        NimBLEDevice::startAdvertising();
    }

    void onMTUChange(uint16_t mtuValue, ble_gap_conn_desc *desc) override {
        s_mtu = mtuValue;
        Serial.printf("[ble] MTU = %u\n", mtuValue);
    }
#endif
};

// ---------------------------------------------------------------------------
class RxCallbacks : public NimBLECharacteristicCallbacks {
#if defined(BLE_NIMBLE_V2)
    void onWrite(NimBLECharacteristic *chr, NimBLEConnInfo &connInfo) override {
        (void)connInfo;
        NimBLEAttValue val = chr->getValue();
        if (val.length() > 0) {
            // Already framed by VESC Tool. The link reassembles whole frames
            // (dropping any that fail their CRC) and passes each payload on
            // unchanged - it never looks inside them, which is exactly why
            // config reads and firmware uploads survive the trip.
            vescLink.writeRaw(val.data(), val.length());
        }
    }
#else
    void onWrite(NimBLECharacteristic *chr) override {
        NimBLEAttValue val = chr->getValue();
        if (val.length() > 0) {
            vescLink.writeRaw(val.data(), val.length());
        }
    }
#endif
};

static ServerCallbacks s_serverCb;
static RxCallbacks     s_rxCb;

// ---------------------------------------------------------------------------
void begin() {
    if (!BLE_ENABLED) return;

    NimBLEDevice::init(BLE_DEVICE_NAME);
    NimBLEDevice::setPower(BLE_TX_POWER);
    NimBLEDevice::setMTU(BLE_MTU);

#ifdef ENABLE_BLE_SECURITY
    NimBLEDevice::setSecurityAuth(true, true, true);      // bond, MITM, SC
    NimBLEDevice::setSecurityPasskey(BLE_SECURITY_PASSKEY);
    NimBLEDevice::setSecurityIOCap(BLE_HS_IO_DISPLAY_ONLY);
#else
    NimBLEDevice::setSecurityAuth(true, false, false); // Enable bonding only
#endif

    s_server = NimBLEDevice::createServer();
    s_server->setCallbacks(&s_serverCb);

    NimBLEService *service = s_server->createService(VESC_SERVICE_UUID);

    // Characteristic property flags. When security is enabled, reads/writes
    // additionally require an authenticated (passkey-paired) connection.
    uint32_t txProperties = NIMBLE_PROPERTY::NOTIFY | NIMBLE_PROPERTY::READ;
    uint32_t rxProperties = NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR;
#ifdef ENABLE_BLE_SECURITY
    txProperties |= NIMBLE_PROPERTY::READ_ENC | NIMBLE_PROPERTY::READ_AUTHEN;
    rxProperties |= NIMBLE_PROPERTY::WRITE_ENC | NIMBLE_PROPERTY::WRITE_AUTHEN;
#endif

    NimBLECharacteristic *rx = service->createCharacteristic(
        VESC_CHARACTERISTIC_UUID_RX,
        rxProperties);

    rx->setCallbacks(&s_rxCb);

    s_txChar = service->createCharacteristic(
        VESC_CHARACTERISTIC_UUID_TX,
        txProperties);

#if !defined(BLE_NIMBLE_V2)
    service->start();     // NimBLE v2 (pioarduino targets): deprecated no-op - services
                           // start when the server does, so this is skipped there.
#endif

    NimBLEAdvertising *adv = NimBLEDevice::getAdvertising();
    adv->addServiceUUID(VESC_SERVICE_UUID);
#if defined(BLE_NIMBLE_V2)
    adv->enableScanResponse(true);
    adv->setPreferredParams(0x06, 0x12);
#else
    adv->setScanResponse(true);
    adv->setMinPreferred(0x06);
    adv->setMaxPreferred(0x12);
#endif
    NimBLEDevice::startAdvertising();

    Serial.printf("[ble] advertising as \"%s\"\n", BLE_DEVICE_NAME);
}

bool     isConnected() { return s_connected; }
uint16_t mtu()         { return s_mtu; }

// ---------------------------------------------------------------------------
void notify(const uint8_t *data, size_t len) {
    if (!s_connected || !s_txChar || len == 0) return;

    // One notification carries at most MTU-3 bytes; longer replies (mcconf,
    // firmware blobs) are split and reassembled by the client's own framer.
    const size_t chunk = (s_mtu > 23) ? (size_t)(s_mtu - 3) : 20;

    size_t offset = 0;
    while (offset < len) {
        const size_t n = min(chunk, len - offset);
        s_txChar->setValue(data + offset, n);
        s_txChar->notify();
        offset += n;
        if (offset < len) delay(2);       // let the stack drain its buffers
    }
}

} // namespace blebridge
