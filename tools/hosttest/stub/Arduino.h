#pragma once
#include <cstdint>
#include <cstring>
#include <cstdio>
#include <cmath>
#include <cstdlib>
#include <algorithm>
#include <string>   // pull in before the min/max macros, as the real core does

#define HIGH 1
#define LOW 0
#define OUTPUT 1
#define INPUT 0
#define SERIAL_8N1 0x800001c
#define MSBFIRST 1
#define SPI_MODE0 0
#define HSPI 2
#define VSPI 3

typedef bool boolean;
typedef uint8_t byte;

uint32_t millis();
void delay(uint32_t);
void delayMicroseconds(uint32_t);
void pinMode(int, int);
void digitalWrite(int, int);
int  digitalRead(int);
long map(long, long, long, long, long);
void ledcSetup(uint8_t, double, uint8_t);
void ledcAttachPin(uint8_t, uint8_t);
void ledcWrite(uint8_t, uint32_t);

template <class T, class U> auto _amin(T a, U b) -> decltype(a < b ? a : b) { return a < b ? a : b; }
template <class T, class U> auto _amax(T a, U b) -> decltype(a > b ? a : b) { return a > b ? a : b; }
#define min(a,b) _amin(a,b)
#define max(a,b) _amax(a,b)
#define constrain(x,l,h) ((x)<(l)?(l):((x)>(h)?(h):(x)))

class HardwareSerial {
public:
    HardwareSerial(int n = 0) {}
    void begin(unsigned long baud, uint32_t cfg = SERIAL_8N1, int8_t rx = -1, int8_t tx = -1);
    void setRxBufferSize(size_t);
    int  available();
    size_t readBytes(uint8_t*, size_t);
    size_t write(const uint8_t*, size_t);
    void println(const char*);
    void print(const char*);
    int  printf(const char*, ...);
};
extern HardwareSerial Serial;
