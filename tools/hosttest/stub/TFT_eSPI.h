#pragma once
#include <Arduino.h>

#define TL_DATUM 0
#define TC_DATUM 1
#define TR_DATUM 2
#define ML_DATUM 3
#define MC_DATUM 4
#define MR_DATUM 5

class TFT_eSPI {
public:
    TFT_eSPI(int16_t w = 0, int16_t h = 0) {}
    void init();
    void setRotation(uint8_t);
    void fillScreen(uint32_t);
    void setTextWrap(bool);
    void setTextDatum(uint8_t);
    void setTextColor(uint16_t fg, uint16_t bg);
    void setTextPadding(uint16_t);
    int16_t drawString(const char*, int32_t, int32_t, uint8_t font);
    int16_t drawNumber(long, int32_t, int32_t, uint8_t font);
    int16_t drawFloat(float, uint8_t, int32_t, int32_t, uint8_t font);
    void fillRect(int32_t, int32_t, int32_t, int32_t, uint32_t);
    void drawRect(int32_t, int32_t, int32_t, int32_t, uint32_t);
    void drawRoundRect(int32_t, int32_t, int32_t, int32_t, int32_t, uint32_t);
    void fillRoundRect(int32_t, int32_t, int32_t, int32_t, int32_t, uint32_t);
    void drawFastHLine(int32_t, int32_t, int32_t, uint32_t);
    void drawFastVLine(int32_t, int32_t, int32_t, uint32_t);
    void drawLine(int32_t, int32_t, int32_t, int32_t, uint32_t);
    void fillCircle(int32_t, int32_t, int32_t, uint32_t);
};

class TFT_eSprite : public TFT_eSPI {
public:
    TFT_eSprite(TFT_eSPI*) {}
    void  setColorDepth(int8_t);
    void* createSprite(int16_t, int16_t, uint8_t frames = 1);
    bool  created();
    void  fillSprite(uint32_t);
    void  pushSprite(int32_t, int32_t);
};
