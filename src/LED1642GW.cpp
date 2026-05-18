#include "LED1642GW.h"
#include "driver/gpio.h"     //required for direct output register manipulation
#include "driver/i2s.h"      //required for 10MHz PWM clock
#include "soc/gpio_reg.h"    //required for direct output register manipulation
#include "soc/gpio_struct.h" //required for direct output register manipulation
#include "esp_err.h"

LED1642GW::LED1642GW(uint16_t *_ledData, uint16_t _nLedDots, uint8_t _clkPin,
                     uint8_t _dataPin, uint8_t _latchPin, uint8_t _dummyPin, int8_t _pwmClockPin)
    : leds(_ledData), nLedDots(_nLedDots), clkPin(_clkPin), dataPin(_dataPin), latchPin(_latchPin), dummyPin(_dummyPin), pwmClockPin(_pwmClockPin)
{
    init();
}

LED1642GW::LED1642GW(RGBColor16 *_rgbLedData, uint16_t _nRGBLeds, uint8_t _clkPin,
                     uint8_t _dataPin, uint8_t _latchPin, uint8_t _dummyPin, int8_t _pwmClockPin)
    : leds((uint16_t *)_rgbLedData), nLedDots(_nRGBLeds * (sizeof(_rgbLedData[0]) / sizeof(_rgbLedData[0].r))), clkPin(_clkPin), dataPin(_dataPin), latchPin(_latchPin), dummyPin(_dummyPin), pwmClockPin(_pwmClockPin)
{
    init();
}

LED1642GW::LED1642GW(RGBWColor16 *_rgbwData, uint16_t _nRGBWLeds, uint8_t _clkPin,
                     uint8_t _dataPin, uint8_t _latchPin, uint8_t _dummyPin, int8_t _pwmClockPin)
    : leds((uint16_t *)_rgbwData), nLedDots(_nRGBWLeds * (sizeof(_rgbwData[0]) / sizeof(_rgbwData[0].r))), clkPin(_clkPin), dataPin(_dataPin), latchPin(_latchPin), dummyPin(_dummyPin), pwmClockPin(_pwmClockPin)
{
    init();
}

void LED1642GW::init()
{
    nLedDrivers = nLedDots / LEDDOTSPERDRIVER;
    if (nLedDots % LEDDOTSPERDRIVER > 0)
    {
        nLedDrivers++;
    }

    fillLookupTables();

    if (!setupDMA(DEFAULT_DMA_CLK_FREQUENCY)) // TODO make clk frequency updateable
    {
        Serial.println("DMA init failed!");
        while (true)
            ;
    }
    Serial.println("DMA streamer ready");

    lastSettingsUpdate = 0;
    settingUpdateInterval = 1000;

    brightness = MAXBRIGHTNESS;

    setConfigRegister();
    enableOutputs();

    if (pwmClockPin >= 0)
    {
        startPWMClock();
    }
}

void LED1642GW::fillLookupTables()
{
    // fill the output lookup tables:
    for (int value = 0; value < 256; value++)
    {

        // create temporary byte arrays, to be packed into the 32bit pairs later
        // the latch4 variant is not calculated, since its A and B parts are already present in other variants
        uint8_t bytes_noLatch[8];
        uint8_t bytes_latch2[8];
        uint8_t bytes_latch6[8];
        uint8_t bytes_latch7[8];

        for (int bit = 0; bit < 8; bit++)
        {

            // DATA bit
            uint8_t out = (value & (0x80 >> bit)) ? 0x01 : 0x00;

            // No latch
            bytes_noLatch[bit] = out;

            // Latch2 -  last 2 bits active
            bytes_latch2[bit] = out | (bit >= 6 ? 0x02 : 0x00);

            // Latch6 - last 6 bits active
            bytes_latch6[bit] = out | (bit >= 2 ? 0x02 : 0x00);

            // Latch7 - last 7 bits active
            bytes_latch7[bit] = out | (bit >= 1 ? 0x02 : 0x00);
        }

        // Pack into aligned uint32_t pairs

        // MSB LUTs:
        memcpy(&expanded8_noLatch_A[value], &bytes_noLatch[0], 4); // pick the MSB from the noLatch
        memcpy(&expanded8_2Latch_A[value], &bytes_latch6[0], 4);   // pick the MSB from the latch6
        memcpy(&expanded8_3Latch_A[value], &bytes_latch7[0], 4);   // pick the MSB from the latch7

        // LSB LUTs:
        memcpy(&expanded8_noLatch_B[value], &bytes_noLatch[4], 4); // pick the LSB from the noLatch
        memcpy(&expanded8_2Latch_B[value], &bytes_latch2[4], 4);   // pick the LSB from the latch2
        memcpy(&expanded8_4Latch_B[value], &bytes_latch6[4], 4);   // pick the LSB from the latch6
    }

    // Latch Bitmask preparations
    for (int i = 0; i < 16; i++)
    {
        if (i > 0)
        {
            latchMasks[i] = LATCH_4;
        }
        else
        {
            latchMasks[i] = LATCH_6;
        }
    }
}

bool LED1642GW::setupDMA(uint32_t clockHz)
{
    // Allocate DMA buffers
    for (int i = 0; i < DMA_QUEUE_DEPTH; i++)
    {

        dmaBuffers[i] = (uint8_t *)heap_caps_malloc(
            DMA_BLOCK_SIZE,
            MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);

        if (!dmaBuffers[i])
        {
            Serial.println("DMA allocation failed");
            return false;
        }

        assert(((uintptr_t)dmaBuffers[i] & 0x3) == 0);
        memset(dmaBuffers[i], 0, DMA_BLOCK_SIZE);
    }

    // Create semaphores
    freeBlocks = xSemaphoreCreateCounting(
        DMA_QUEUE_DEPTH,
        DMA_QUEUE_DEPTH);

    queuedBlocks = xSemaphoreCreateCounting(
        DMA_QUEUE_DEPTH,
        0);

    // I80 BUS CONFIG
    esp_lcd_i80_bus_config_t bus_config = {};

    bus_config.dc_gpio_num = dummyPin;
    bus_config.wr_gpio_num = clkPin;
    bus_config.bus_width = 8;
    bus_config.max_transfer_bytes = DMA_BLOCK_SIZE;

#if defined(LCD_CLK_SRC_PLL160M)
    // newer IDF
    bus_config.clk_src = LCD_CLK_SRC_PLL160M;
#elif defined(LCD_CLOCK_SOURCE_PLL160M)
    // some intermediate versions
    bus_config.clk_src = LCD_CLOCK_SOURCE_PLL160M;
#endif

    bus_config.data_gpio_nums[0] = dataPin;
    bus_config.data_gpio_nums[1] = latchPin;

    // set all unused pins to the dummy pin:
    for (int i = 2; i < 8; i++)
    {
        bus_config.data_gpio_nums[i] = dummyPin;
    }

    esp_err_t err;

    err = esp_lcd_new_i80_bus(
        &bus_config,
        &i80_bus);

    if (err != ESP_OK)
    {
        Serial.printf(
            "esp_lcd_new_i80_bus failed: %d\n",
            err);
        return false;
    }

    // PANEL IO CONFIG
    esp_lcd_panel_io_i80_config_t io_config = {};

    io_config.cs_gpio_num = -1;
    io_config.pclk_hz = clockHz;
    io_config.trans_queue_depth = DMA_QUEUE_DEPTH;

    io_config.on_color_trans_done = dmaDoneISR;
    io_config.user_ctx = this;

    io_config.lcd_cmd_bits = 0;
    io_config.lcd_param_bits = 0;

    err = esp_lcd_new_panel_io_i80(
        i80_bus,
        &io_config,
        &io_handle);

    if (err != ESP_OK)
    {
        Serial.printf(
            "esp_lcd_new_panel_io_i80 failed: %d\n",
            err);
        return false;
    }
    return true;
}

void LED1642GW::setConfigRegister()
{
    // build the config register:
    uint16_t cfg = 0;
    cfg |= brightness; // CFG0..5 = max gain
    cfg |= (1 << 6);   // CFG6 = high current range
    cfg |= (1 << 7);   // CFG7 = normal mode
    cfg |= (1 << 13);  // CFG13 = SDO delay enable

    // start DMA message
    startMessage();

    // Direct DMA access
    uint8_t *out = currentBuffer;
    uint8_t *outEnd = currentBuffer + DMA_BLOCK_SIZE;

    for (int driver = nLedDrivers - 1; driver >= 0; driver--)
    {
        LatchMode latch = (driver == 0) ? LATCH_7 : NO_LATCH;
        shiftOut16(cfg, latch, out, outEnd);
    }
    // Submit remaining partial block
    currentIndex = out - currentBuffer;
    endMessage();
}

void LED1642GW::enableOutputs(bool enable)
{
    // start DMA message
    startMessage();

    // Direct DMA access
    uint8_t *out = currentBuffer;
    uint8_t *outEnd = currentBuffer + DMA_BLOCK_SIZE;

    uint16_t value = 0xFFFF; // all bits high for all drivers
    if (!enable)
    {
        value = 0x0000; // all bits low for all drivers
    }

    for (int driver = nLedDrivers - 1; driver >= 0; driver--)
    {

        LatchMode latch = (driver == 0) ? LATCH_2 : NO_LATCH; // last two bits high on last driver update
        shiftOut16(value, latch, out, outEnd);
    }
    // Submit remaining partial block
    currentIndex = out - currentBuffer;
    endMessage();
}

void LED1642GW::startPWMClock()
{
    // use the I2S clock as the 10MHz PWMclock:
    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
        .sample_rate = 625000, // clock is 16 times this value, so 10MHz
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = 0,
        .dma_buf_count = 2,
        .dma_buf_len = 8,
        .use_apll = true,
        .tx_desc_auto_clear = true,
        .fixed_mclk = 0};

    i2s_pin_config_t pin_config = {
        .bck_io_num = pwmClockPin,
        .ws_io_num = I2S_PIN_NO_CHANGE,
        .data_out_num = I2S_PIN_NO_CHANGE,
        .data_in_num = I2S_PIN_NO_CHANGE};

    i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
    i2s_set_pin(I2S_NUM_0, &pin_config);
}

void LED1642GW::update()
{
    // Start DMA message
    startMessage();

    // Direct DMA access
    uint8_t *out = currentBuffer;
    uint8_t *outEnd = currentBuffer + DMA_BLOCK_SIZE;

    // LED data generation
    for (int channel = 15; channel >= 0; channel--)
    {
        for (int driver = nLedDrivers - 1; driver >= 0; driver--)
        {
            uint16_t nodeIndex = driver * LEDDOTSPERDRIVER + channel;
            uint16_t value = leds[nodeIndex];
            LatchMode latch = (driver == 0) ? latchMasks[channel] : NO_LATCH;

            // shift out the data for 1 driver:
            shiftOut16(value, latch, out, outEnd);
        }
    }

    // Submit remaining partial block
    currentIndex = out - currentBuffer;
    endMessage();

    // Periodic config refresh
    if (millis() - lastSettingsUpdate > settingUpdateInterval)
    {
        lastSettingsUpdate = millis();
        setConfigRegister();
        enableOutputs();
    }
}

void LED1642GW::setLedTo(uint16_t ledIndex, struct RGBWColor16 color)
{
    ledIndex = ledIndex * sizeof(color) / sizeof(color.r);
    if (ledIndex >= nLedDots)
        return;
    memcpy(&leds[ledIndex], &color, sizeof(color));
}

void LED1642GW::setLedTo(uint16_t ledIndex, struct RGBColor16 color)
{
    if (ledIndex >= nLedDots * sizeof(color) / sizeof(color.r))
        return;
    memcpy(&leds[ledIndex], &color, sizeof(color));
}

void LED1642GW::setLedTo(uint16_t ledIndex, uint16_t brightness)
{
    if (ledIndex >= nLedDots)
        return; // catch out of bounds index
    leds[ledIndex] = brightness;
}

void LED1642GW::setAllLedsTo(struct RGBWColor16 color)
{
    for (int i = 0; i < nLedDots; i++)
    {
        switch (i % 4)
        {
        case 0:
            leds[i] = color.r;
            break;
        case 1:
            leds[i] = color.g;
            break;
        case 2:
            leds[i] = color.b;
            break;
        case 3:
            leds[i] = color.w;
            break;
        default:
            leds[i] = 0; // should not occur
            break;
        }
    }
}
void LED1642GW::setAllLedsTo(struct RGBColor16 color)
{
    for (int i = 0; i < nLedDots; i++)
    {
        switch (i % 3)
        {
        case 0:
            leds[i] = color.r;
            break;
        case 1:
            leds[i] = color.g;
            break;
        case 2:
            leds[i] = color.b;
            break;
        default:
            leds[i] = 0; // should not occur
            break;
        }
    }
}
void LED1642GW::setAllLedsTo(uint16_t brightness)
{
    for (int i = 0; i < nLedDots; i++)
    {
        leds[i] = brightness;
    }
}

void LED1642GW::clearLeds()
{
    for (int i = 0; i < nLedDots; i++)
    {
        leds[i] = 0;
    }
}

void LED1642GW::setBrightness(uint8_t _brightness)
{
    brightness = constrain(_brightness, 0, MAXBRIGHTNESS);
    setConfigRegister();
}

void LED1642GW::setConfigUpdateInterval(uint32_t milliseconds)
{
    settingUpdateInterval = milliseconds;
}

void LED1642GW::startMessage()
{
    currentIndex = 0;
    acquireBlock();
}

void LED1642GW::endMessage()
{
    // Send partially filled block
    if (currentIndex > 0)
    {

        submitCurrentBlock(currentIndex);
    }
    currentBuffer = nullptr;
    currentIndex = 0;
}

void LED1642GW::flush()
{
    // wait for the queued messages to be sent
    while (true)
    {
        if (uxSemaphoreGetCount(queuedBlocks) == 0 && uxSemaphoreGetCount(freeBlocks) == DMA_QUEUE_DEPTH)
        {
            break;
        }
        delayMicroseconds(10);
    }
}

void LED1642GW::acquireBlock()
{
    xSemaphoreTake(freeBlocks, portMAX_DELAY);
    currentBuffer = dmaBuffers[writeBlockIndex];
    writeBlockIndex++;
    writeBlockIndex %= DMA_QUEUE_DEPTH;
}

void LED1642GW::submitCurrentBlock(size_t lengthBytes)
{
    ESP_ERROR_CHECK(esp_lcd_panel_io_tx_color(io_handle, 0, currentBuffer, lengthBytes));
    xSemaphoreGive(queuedBlocks);
}

// DMA COMPLETE ISR
bool LED1642GW::dmaDoneISR(
    esp_lcd_panel_io_handle_t panel_io,
    esp_lcd_panel_io_event_data_t *edata,
    void *user_ctx)
{

    LED1642GW *self = (LED1642GW *)user_ctx;

    BaseType_t highTaskWoken = pdFALSE;

    xSemaphoreGiveFromISR(
        self->freeBlocks,
        &highTaskWoken);

    xSemaphoreTakeFromISR(
        self->queuedBlocks,
        &highTaskWoken);

    return highTaskWoken == pdTRUE;
}

inline __attribute__((always_inline)) void LED1642GW::nextDMABlock(uint8_t *&out)
{
    // submit current full block
    submitCurrentBlock(DMA_BLOCK_SIZE);

    // acquire next block
    acquireBlock();

    currentIndex = 0;

    // update local pointer
    out = currentBuffer;
}

inline __attribute__((always_inline)) void LED1642GW::shiftOut16(uint16_t value, LatchMode latchMode, uint8_t *&out, uint8_t *&outEnd)
{

    // ensure enough room for entire word:
    if ((outEnd - out) < 16)
    {
        nextDMABlock(out);
        outEnd = currentBuffer + DMA_BLOCK_SIZE;
    }

    // separate the 16bit value into two bytes:
    uint8_t highByte = value >> 8;
    uint8_t lowByte = value & 0xFF;

    // Upper byte is ALWAYS no-latch
    *(uint32_t *)(out + 0) = expanded8_noLatch_A[highByte];
    *(uint32_t *)(out + 4) = expanded8_noLatch_B[highByte];

    // Lower byte depends on latch mode
    switch (latchMode)
    {
    case NO_LATCH: // 16 bit value shifted out, once per led per frame
    default:
        *(uint32_t *)(out + 8) = expanded8_noLatch_A[lowByte];  // all latch low
        *(uint32_t *)(out + 12) = expanded8_noLatch_B[lowByte]; // all latch low
        break;

    case LATCH_4:                                              // 16 bit led values shifted through, 16 times per frame
        *(uint32_t *)(out + 8) = expanded8_noLatch_A[lowByte]; // all latch low
        *(uint32_t *)(out + 12) = expanded8_4Latch_B[lowByte]; // all latch high
        break;

    case LATCH_2:                                              // enable leds complete, once per config update (default is 1Hz)
        *(uint32_t *)(out + 8) = expanded8_noLatch_A[lowByte]; // all latch low
        *(uint32_t *)(out + 12) = expanded8_2Latch_B[lowByte]; // last 2 latch high
        break;

    case LATCH_6:                                              // frame of 16 bit led values complete, once per frame
        *(uint32_t *)(out + 8) = expanded8_2Latch_A[lowByte];  // last 2 latch high
        *(uint32_t *)(out + 12) = expanded8_4Latch_B[lowByte]; // all latch high
        break;

    case LATCH_7:                                              // config setting complete, once per config update  (default is 1Hz)
        *(uint32_t *)(out + 8) = expanded8_3Latch_A[lowByte];  // last 3 latch high
        *(uint32_t *)(out + 12) = expanded8_4Latch_B[lowByte]; // all latch high
        break;
    }

    // Advance output pointer
    out += 16;
}
