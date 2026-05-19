/*
 * LED1642GW Library
 * Original author: Pim Swinkels
 * https://github.com/PimSwinkelsCreative
 *
 * Copyright (c) 2026 Pim Swinkels
 * Licensed under the MIT License
 */

#pragma once

#include <Arduino.h>
#include <16bitPixelTypes.h>

#include <esp_heap_caps.h>

// ========================================
// ESP-IDF / Arduino compatibility layer
// ========================================

// Newer ESP-IDF / Arduino 3.x
#if __has_include(<esp_lcd_panel_io.h>)
    #include <esp_lcd_panel_io.h>
    #if __has_include(<esp_lcd_io_i80.h>)
        #include <esp_lcd_io_i80.h>
    #endif

// Older ESP-IDF / Arduino 2.x
#elif __has_include(<esp_lcd_panel_io_interface.h>)
    #include <esp_lcd_panel_io_interface.h>
#endif

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

// ========================================
// Fallback typedefs for older frameworks
// ========================================

#ifndef ESP_LCD_IO_I80_TRANS_QUEUE_DEPTH
    #define ESP_LCD_IO_I80_TRANS_QUEUE_DEPTH 10
#endif

#if !__has_include(<esp_lcd_panel_io.h>) && \
    !__has_include(<esp_lcd_panel_io_interface.h>)
    #error "ESP LCD driver not available in this framework version"
#endif

#define DEFAULT_DUMMY_PIN 1

#define DEFAULT_DMA_CLK_FREQUENCY 20000000 // 20MHz clk

#define LEDDOTSPERDRIVER 16
#define BYTESPERDRIVER (2 * LEDDOTSPERDRIVER)

#define MAXBRIGHTNESS 0x3F

// DMA settings:
//  Large buffering to avoid CPU stalls
// currently optimized for max 2000 LEDs. Increase/decrease if number of LEDs differs greatly.
// setting these block size and/or queue depth too low will not cause errors.
// It might only slow things down since the CPU will wait for a block to become free
#define DMA_BLOCK_SIZE 4096
#define DMA_QUEUE_DEPTH 8

enum LatchMode : uint8_t {
    NO_LATCH,
    LATCH_2,
    LATCH_4,
    LATCH_6,
    LATCH_7
};

class LED1642GW {
private:
    uint16_t* leds;
    uint16_t nLedDots;
    uint8_t clkPin;
    uint8_t dataPin;
    uint8_t latchPin;
    int8_t pwmClockPin;
    uint8_t dummyPin;
    uint32_t clkFrequency;
    uint8_t nLedDrivers;

    uint64_t lastSettingsUpdate;
    uint32_t settingUpdateInterval;

    uint8_t brightness;

    // DMA related variables:
    uint8_t* dmaBuffers[DMA_QUEUE_DEPTH];
    uint8_t* currentBuffer = nullptr;
    size_t currentIndex = 0;
    int writeBlockIndex = 0;
    SemaphoreHandle_t freeBlocks;
    SemaphoreHandle_t queuedBlocks;
    esp_lcd_i80_bus_handle_t i80_bus = nullptr;
    esp_lcd_panel_io_handle_t io_handle = nullptr;

    // lookup tables:
    LatchMode latchMasks[16];
    uint32_t expanded8_noLatch_A[256];
    uint32_t expanded8_2Latch_A[256];
    uint32_t expanded8_3Latch_A[256];
    uint32_t expanded8_noLatch_B[256];
    uint32_t expanded8_2Latch_B[256];
    uint32_t expanded8_4Latch_B[256];

    void init();

    void setConfigRegister();
    void enableOutputs(bool enable = true);
    void startPWMClock();

    // dma functions:
    bool setupDMA(uint32_t clockHz);
    void fillLookupTables();
    void acquireBlock();
    void submitCurrentBlock(size_t lengthBytes);
    static bool dmaDoneISR(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t* edata, void* user_ctx);
    void startMessage();
    void endMessage();

    inline __attribute__((always_inline)) void nextDMABlock(uint8_t*& out);
    inline __attribute__((always_inline)) void shiftOut16(uint16_t value, LatchMode latchMode, uint8_t*& out, uint8_t*& outEnd);

public:
    // constructors:
    LED1642GW(RGBColor16* _rgbLedData, uint16_t _nRGBLeds, uint8_t _clkPin,
        uint8_t _dataPin, uint8_t _latchPin, uint8_t _dummyPin = DEFAULT_DUMMY_PIN, int8_t _pwmClockPin = -1);
    LED1642GW(RGBWColor16* _rgbwLedData, uint16_t _nRGBWLeds, uint8_t _clkPin,
        uint8_t _dataPin, uint8_t _latchPin, uint8_t _dummyPin = DEFAULT_DUMMY_PIN, int8_t _pwmClockPin = -1);
    LED1642GW(uint16_t* _ledData, uint16_t _nLedDots, uint8_t _clkPin,
        uint8_t _dataPin, uint8_t _latchPin, uint8_t _dummyPin = DEFAULT_DUMMY_PIN, int8_t _pwmClockPin = -1);

    // setting led:
    void setLedTo(uint16_t ledIndex, struct RGBWColor16 color);
    void setLedTo(uint16_t ledIndex, struct RGBColor16 color);
    void setLedTo(uint16_t ledIndex, uint16_t brightness);

    // setting all leds:
    void setAllLedsTo(struct RGBWColor16 color);
    void setAllLedsTo(struct RGBColor16 color);
    void setAllLedsTo(uint16_t brightness);

    // clear all leds:
    void clearLeds();

    // write the ledData to the led drivers:
    void update();

    // function that sets the interval at which the driver config is being sent
    void setConfigUpdateInterval(uint32_t milliseconds);

    void setBrightness(uint8_t _brightness);

    void flush();
};