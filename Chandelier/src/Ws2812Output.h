#pragma once

#include <FastLED.h>
#include "driver/gpio.h"
#include "driver/rmt_tx.h"
#include "esp_err.h"

// The C6 has only two RMT TX channels. Reuse one channel for all candle pins.
// Each transfer completes, including its reset pulse, before changing pins.
template <size_t LedCount>
class Ws2812Output {
public:
    static_assert(LedCount > 0, "Each candle must have at least one LED");

    void begin(uint8_t firstPin) {
        activePin = (gpio_num_t)firstPin;
        rmt_tx_channel_config_t config = {};
        config.gpio_num = activePin;
        config.clk_src = RMT_CLK_SRC_DEFAULT;
        config.resolution_hz = 40000000; // 25 ns per tick.
        config.mem_block_symbols = 96;  // Both C6 TX memory blocks, one channel.
        config.trans_queue_depth = 1;
        ESP_ERROR_CHECK(rmt_new_tx_channel(&config, &channel));

        rmt_copy_encoder_config_t encoderConfig = {};
        ESP_ERROR_CHECK(rmt_new_copy_encoder(&encoderConfig, &encoder));
        ESP_ERROR_CHECK(gpio_set_pull_mode(activePin, GPIO_PULLDOWN_ONLY));
    }

    void showSolid(uint8_t pin, const CRGB& color) {
        if (activePin != (gpio_num_t)pin) {
            const gpio_num_t previousPin = activePin;
            ESP_ERROR_CHECK(gpio_set_pull_mode((gpio_num_t)pin, GPIO_PULLDOWN_ONLY));
            ESP_ERROR_CHECK(rmt_tx_switch_gpio(channel, (gpio_num_t)pin, false));
            // GPIO output mode disconnects the old RMT route. Pull-downs keep DIN
            // low during the brief disabled interval; gpio_reset_pin would enable a pull-up.
            ESP_ERROR_CHECK(gpio_set_level(previousPin, 0));
            ESP_ERROR_CHECK(gpio_set_direction(previousPin, GPIO_MODE_OUTPUT));
            activePin = (gpio_num_t)pin;
        }

        size_t index = 0;
        const uint8_t grb[] = {color.g, color.r, color.b};
        for (size_t led = 0; led < LedCount; ++led) {
            for (uint8_t value : grb) {
                for (uint8_t mask = 0x80; mask != 0; mask >>= 1) {
                    rmt_symbol_word_t& symbol = symbols[index++];
                    symbol.level0 = 1;
                    symbol.level1 = 0;
                    // Match the working FastLED 3.10.3 WS2812 timing (1.25 us/bit).
                    symbol.duration0 = (value & mask) ? 35 : 10;
                    symbol.duration1 = (value & mask) ? 15 : 40;
                }
            }
        }
        // WS2812 reset/latch: 300 us continuously low.
        symbols[index].level0 = 0;
        symbols[index].duration0 = 6000;
        symbols[index].level1 = 0;
        symbols[index].duration1 = 6000;

        rmt_transmit_config_t transmitConfig = {};
        ESP_ERROR_CHECK(rmt_enable(channel));
        ESP_ERROR_CHECK(rmt_transmit(channel, encoder, symbols, sizeof(symbols), &transmitConfig));
        ESP_ERROR_CHECK(rmt_tx_wait_all_done(channel, 100));
        ESP_ERROR_CHECK(rmt_disable(channel));
    }

private:
    rmt_channel_handle_t channel = nullptr;
    rmt_encoder_handle_t encoder = nullptr;
    gpio_num_t activePin = GPIO_NUM_NC;
    rmt_symbol_word_t symbols[LedCount * 24 + 1] = {};
};
