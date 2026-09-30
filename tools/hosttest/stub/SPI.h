#pragma once
#include <Arduino.h>
class SPISettings {
public:
    SPISettings(uint32_t, uint8_t, uint8_t) {}
};
class SPIClass {
public:
    SPIClass(int bus = 0) {}
    void begin(int sck = -1, int miso = -1, int mosi = -1, int ss = -1);
    void beginTransaction(SPISettings);
    void endTransaction();
    uint8_t  transfer(uint8_t);
    uint16_t transfer16(uint16_t);
};
