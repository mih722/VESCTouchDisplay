#pragma once
#include <cstdint>
typedef void* SemaphoreHandle_t;
typedef void* TaskHandle_t;
typedef uint32_t TickType_t;
typedef int BaseType_t;
#define pdTRUE 1
#define pdFALSE 0
#define pdMS_TO_TICKS(x) ((TickType_t)(x))
void vTaskDelay(TickType_t);
BaseType_t xTaskCreatePinnedToCore(void (*)(void*), const char*, uint32_t, void*, unsigned, TaskHandle_t*, int);
