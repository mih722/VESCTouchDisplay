#pragma once
#include <Arduino.h>
class Preferences {
public:
    bool begin(const char*, bool ro = false);
    float   getFloat(const char*, float def = 0);
    size_t  putFloat(const char*, float);
    uint8_t getUChar(const char*, uint8_t def = 0);
    size_t  putUChar(const char*, uint8_t);
};
