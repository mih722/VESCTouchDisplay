#pragma once
#include <Arduino.h>
#include <string>

typedef int esp_power_level_t;
#define ESP_PWR_LVL_P9 7
#define BLE_HS_IO_DISPLAY_ONLY 0

struct ble_gap_conn_desc { uint16_t conn_handle; };

namespace NIMBLE_PROPERTY {
    static const uint32_t READ     = 1;
    static const uint32_t WRITE    = 2;
    static const uint32_t WRITE_NR = 4;
    static const uint32_t NOTIFY   = 8;
}

class NimBLEAttValue {
public:
    const uint8_t* data() const;
    uint16_t length() const;
};

class NimBLECharacteristic;
class NimBLECharacteristicCallbacks {
public:
    virtual ~NimBLECharacteristicCallbacks() {}
    virtual void onWrite(NimBLECharacteristic*) {}
};

class NimBLECharacteristic {
public:
    void setCallbacks(NimBLECharacteristicCallbacks*);
    void setValue(const uint8_t*, size_t);
    void notify(bool is_notification = true);
    NimBLEAttValue getValue();
};

class NimBLEService {
public:
    NimBLECharacteristic* createCharacteristic(const char*, uint32_t props);
    bool start();
};

class NimBLEServer;
class NimBLEServerCallbacks {
public:
    virtual ~NimBLEServerCallbacks() {}
    virtual void onConnect(NimBLEServer*, ble_gap_conn_desc*) {}
    virtual void onDisconnect(NimBLEServer*) {}
    virtual void onMTUChange(uint16_t, ble_gap_conn_desc*) {}
};

class NimBLEServer {
public:
    void setCallbacks(NimBLEServerCallbacks*);
    NimBLEService* createService(const char*);
    bool updateConnParams(uint16_t, uint16_t, uint16_t, uint16_t, uint16_t);
};

class NimBLEAdvertising {
public:
    bool addServiceUUID(const char*);
    void setScanResponse(bool);
    void setMinPreferred(uint16_t);
    void setMaxPreferred(uint16_t);
};

class NimBLEDevice {
public:
    static bool init(const std::string&);
    static bool setPower(esp_power_level_t, int type = 0);
    static bool setMTU(uint16_t);
    static NimBLEServer* createServer();
    static NimBLEAdvertising* getAdvertising();
    static bool startAdvertising();
    static void setSecurityAuth(bool, bool, bool);
    static void setSecurityPasskey(uint32_t);
    static void setSecurityIOCap(uint8_t);
};
