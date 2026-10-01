#include "../src/CandleEffect.h"
#include "../src/ChandelierControl.h"
#include "../src/Ws2812Output.h"
#include <array>
#include <cassert>
#include <cstring>
#include <iostream>
#include <set>
#include <vector>

uint32_t randomState = 12345;
long random(long minimum, long maximum) {
    assert(maximum > minimum);
    randomState = randomState * 1664525u + 1013904223u;
    return minimum + randomState % uint32_t(maximum - minimum);
}

uint16_t inoise16(uint32_t x, uint32_t y) {
    return uint16_t(32767.5 + 25000.0 * std::sin(double(x) * 0.00012 + double(y)));
}

namespace {
constexpr std::array<int, 5> pins = {1, 2, 21, 22, 23};
struct Pin { bool rmt = false; bool output = true; uint32_t level = 0; bool pullDown = false; } pinState[24];
bool enabled = false, transmitting = false;
int activePin = -1, channelCount = 0, encoderCount = 0, transfers = 0;
CRGB expectedColor;
std::vector<rmt_symbol_word_t> inFlight;
const void* inFlightPointer = nullptr;

void testCandles() {
    std::array<CandleEffect, 5> candles;
    for (uint8_t i = 0; i < candles.size(); ++i) candles[i].begin(1000, i, uint8_t(candles.size()));
    std::set<uint32_t> colors[5];
    size_t differences[5][5] = {};
    for (uint32_t elapsed = 0; elapsed < 120000; elapsed += 20) {
        CRGB frame[5];
        for (size_t i = 0; i < candles.size(); ++i) {
            frame[i] = candles[i].colorAt(1000 + elapsed);
            const int peak = std::max({frame[i].r, frame[i].g, frame[i].b});
            assert(peak >= 8 && peak <= 85);
            colors[i].insert((uint32_t(frame[i].r) << 16) | (uint32_t(frame[i].g) << 8) | frame[i].b);
        }
        // Rendering other candles must not mutate an earlier candle's state.
        for (size_t i = 0; i < candles.size(); ++i) {
            assert(candles[i].colorAt(1000 + elapsed) == frame[i]);
            for (size_t j = i + 1; j < candles.size(); ++j) {
                if (!(frame[i] == frame[j])) ++differences[i][j];
            }
        }
    }
    for (size_t i = 0; i < candles.size(); ++i) {
        assert(colors[i].size() > 100);
        for (size_t j = i + 1; j < candles.size(); ++j) assert(differences[i][j] > 3000);
    }

    // The same animation timeline must survive a millis() rollover unchanged.
    CandleEffect normal, wrapping;
    randomState = 9876;
    normal.begin(1000, 2, 5);
    randomState = 9876;
    const uint32_t nearWrap = UINT32_MAX - 1000;
    wrapping.begin(nearWrap, 2, 5);
    for (uint32_t elapsed = 0; elapsed < 20000; elapsed += 20) {
        const uint32_t before = randomState;
        const CRGB a = normal.colorAt(1000 + elapsed);
        const uint32_t after = randomState;
        randomState = before;
        const CRGB b = wrapping.colorAt(nearWrap + elapsed);
        assert(a == b && randomState == after);
    }
    std::cout << "PASS: five varied candle timelines, isolated state, brightness bounds, millis rollover\n";
}

void testCommands() {
    constexpr const char* deviceId = "CHANDELIER32";
    const char* targets[] = {deviceId, "CHANDELIER", "ALL"};
    const char* commands[] = {"CHANDELIER_ON", "CHANDELIER_OFF", "CHANDELIER_COLOR_RED",
                            "CHANDELIER_COLOR_NORMAL", "CHANDELIER_COLOR_GREEN", "SEND_UPDATE"};
    for (const char* target : targets) {
        ChandelierControl control;
        assert(control.isOn() && control.colorMode() == CandleColor::Normal);
        assert(control.handleCommand(target, deviceId, "CHANDELIER_OFF"));
        assert(!control.isOn());
        assert(control.handleCommand(target, deviceId, "CHANDELIER_COLOR_RED"));
        assert(!control.isOn() && control.colorMode() == CandleColor::Red);
        assert(control.handleCommand(target, deviceId, "SEND_UPDATE"));
        assert(!control.isOn() && control.colorMode() == CandleColor::Red);

        // Wrong destinations and unknown payloads must not change either setting.
        for (const char* command : commands) {
            assert(!control.handleCommand("OTHER_DEVICE", deviceId, command));
            assert(!control.isOn() && control.colorMode() == CandleColor::Red);
        }
        assert(!control.handleCommand(target, deviceId, "CHANDELIER_COLOR_BLUE"));
        assert(!control.handleCommand(target, deviceId, "CHANDELIER_ON_EXTRA"));
        assert(!control.isOn() && control.colorMode() == CandleColor::Red);

        assert(control.handleCommand(target, deviceId, "CHANDELIER_ON"));
        assert(control.isOn() && control.colorMode() == CandleColor::Red);
        assert(control.handleCommand(target, deviceId, "CHANDELIER_ON"));
        assert(control.isOn() && control.colorMode() == CandleColor::Red);
        assert(control.handleCommand(target, deviceId, "CHANDELIER_COLOR_GREEN"));
        assert(control.isOn() && control.colorMode() == CandleColor::Green);
        assert(control.handleCommand(target, deviceId, "CHANDELIER_COLOR_NORMAL"));
        assert(control.isOn() && control.colorMode() == CandleColor::Normal);
    }
    std::cout << "PASS: all commands, device/group/broadcast addressing, off-state colour selection, ignored messages\n";
}

void testColorModes() {
    std::array<CandleEffect, 5> candles;
    ChandelierControl control;
    for (uint8_t i = 0; i < candles.size(); ++i) candles[i].begin(1000, i, uint8_t(candles.size()));
    std::set<uint8_t> levels[5];
    size_t differences[5][5] = {};
    for (uint32_t now = 1000; now < 31000; now += 20) {
        CRGB normal[5], red[5];
        assert(control.handleCommand("CHANDELIER", "CHANDELIER32", "CHANDELIER_COLOR_NORMAL"));
        for (size_t i = 0; i < candles.size(); ++i) normal[i] = control.colorAt(candles[i], now);
        assert(control.handleCommand("CHANDELIER", "CHANDELIER32", "CHANDELIER_COLOR_RED"));
        for (size_t i = 0; i < candles.size(); ++i) {
            red[i] = control.colorAt(candles[i], now);
            assert(red[i].g == 0 && red[i].b == 0);
            assert(red[i].r == std::max({normal[i].r, normal[i].g, normal[i].b}));
            assert(red[i].r >= 8 && red[i].r <= 85);
            levels[i].insert(red[i].r);
        }
        assert(control.handleCommand("CHANDELIER", "CHANDELIER32", "CHANDELIER_OFF"));
        for (auto& candle : candles) assert(control.colorAt(candle, now) == CRGB(0, 0, 0));
        assert(control.handleCommand("CHANDELIER", "CHANDELIER32", "CHANDELIER_COLOR_GREEN"));
        for (auto& candle : candles) assert(control.colorAt(candle, now) == CRGB(0, 0, 0));
        assert(control.handleCommand("CHANDELIER", "CHANDELIER32", "CHANDELIER_ON"));
        for (size_t i = 0; i < candles.size(); ++i) {
            const CRGB green = control.colorAt(candles[i], now);
            assert(green == CRGB(0, red[i].r, 0));
            for (size_t j = i + 1; j < candles.size(); ++j) {
                if (red[i].r != red[j].r) ++differences[i][j];
            }
        }
        assert(control.handleCommand("CHANDELIER", "CHANDELIER32", "CHANDELIER_COLOR_NORMAL"));
        for (size_t i = 0; i < candles.size(); ++i) assert(control.colorAt(candles[i], now) == normal[i]);
    }
    for (size_t i = 0; i < candles.size(); ++i) {
        assert(levels[i].size() > 10);
        for (size_t j = i + 1; j < candles.size(); ++j) assert(differences[i][j] > 500);
    }
    std::cout << "PASS: red/green flicker independently with unchanged brightness; blackout and normal-mode continuity\n";
}

void testOutputs() {
    Ws2812Output<8> output;
    output.begin(uint8_t(pins[0]));
    for (int frame = 0; frame < 100; ++frame) {
        for (size_t i = 0; i < pins.size(); ++i) {
            expectedColor = CRGB(uint8_t(frame * 3 + i), uint8_t(frame * 5 + i * 7), uint8_t(frame * 11 + i * 13));
            output.showSolid(uint8_t(pins[i]), expectedColor);
            assert(activePin == pins[i] && !enabled && !transmitting);
        }
    }
    // Exercise command-driven blackout and reactivation through the actual encoder.
    ChandelierControl control;
    CandleEffect candle;
    candle.begin(1000, 0, 5);
    const char* commands[] = {"CHANDELIER_OFF", "CHANDELIER_COLOR_GREEN", "CHANDELIER_ON"};
    for (const char* command : commands) {
        assert(control.handleCommand("CHANDELIER", "CHANDELIER32", command));
        expectedColor = control.colorAt(candle, 1000);
        if (!control.isOn()) assert(expectedColor == CRGB(0, 0, 0));
        else assert(expectedColor.r == 0 && expectedColor.g > 0 && expectedColor.b == 0);
        for (int pin : pins) output.showSolid(uint8_t(pin), expectedColor);
    }
    assert(channelCount == 1 && encoderCount == 1 && transfers == 515);
    std::cout << "PASS: five outputs share one channel; GRB frames, reset timing, pin isolation, synchronous reuse\n";
}
} // namespace

esp_err_t rmt_new_tx_channel(const rmt_tx_channel_config_t* config, rmt_channel_handle_t* channel) {
    assert(++channelCount == 1);
    assert(config->resolution_hz == 40000000 && config->mem_block_symbols == 96 && config->trans_queue_depth == 1);
    activePin = config->gpio_num;
    pinState[activePin].rmt = true;
    *channel = &channelCount;
    return ESP_OK;
}
esp_err_t rmt_new_copy_encoder(const rmt_copy_encoder_config_t*, rmt_encoder_handle_t* encoder) {
    ++encoderCount;
    *encoder = &encoderCount;
    return ESP_OK;
}
esp_err_t rmt_tx_switch_gpio(rmt_channel_handle_t, gpio_num_t pin, bool inverted) {
    assert(!enabled && !transmitting && !inverted);
    assert(pinState[activePin].pullDown && pinState[pin].pullDown);
    // Like ESP-IDF: disable the previous pin, but leave its matrix route intact.
    pinState[activePin].output = false;
    activePin = pin;
    pinState[pin].rmt = true;
    pinState[pin].output = true;
    return ESP_OK;
}
esp_err_t gpio_set_pull_mode(gpio_num_t pin, int mode) {
    assert(mode == GPIO_PULLDOWN_ONLY);
    pinState[pin].pullDown = true;
    return ESP_OK;
}
esp_err_t gpio_set_level(gpio_num_t pin, uint32_t level) { pinState[pin].level = level; return ESP_OK; }
esp_err_t gpio_set_direction(gpio_num_t pin, int mode) {
    assert(mode == GPIO_MODE_OUTPUT && pin != activePin);
    // ESP-IDF's gpio_output_enable() restores the ordinary GPIO matrix route.
    pinState[pin].rmt = false;
    pinState[pin].output = true;
    return ESP_OK;
}
esp_err_t rmt_enable(rmt_channel_handle_t) { assert(!enabled && !transmitting); enabled = true; return ESP_OK; }
esp_err_t rmt_transmit(rmt_channel_handle_t, rmt_encoder_handle_t, const void* payload, size_t bytes, const rmt_transmit_config_t*) {
    assert(enabled && !transmitting);
    assert(activePin == pins[transfers % pins.size()]);
    for (int pin : pins) {
        if (pin != activePin) assert(!pinState[pin].rmt && pinState[pin].output && pinState[pin].level == 0);
    }
    const auto* symbols = static_cast<const rmt_symbol_word_t*>(payload);
    assert(bytes == (8 * 24 + 1) * sizeof(*symbols));
    const uint8_t expected[] = {expectedColor.g, expectedColor.r, expectedColor.b};
    for (size_t byte = 0; byte < 8 * 3; ++byte) {
        uint8_t value = 0;
        for (size_t bit = 0; bit < 8; ++bit) {
            const auto& symbol = symbols[byte * 8 + bit];
            assert(symbol.level0 == 1 && symbol.level1 == 0);
            assert(symbol.duration0 == 10 || symbol.duration0 == 35);
            assert(symbol.duration0 + symbol.duration1 == 50);
            value = uint8_t((value << 1) | (symbol.duration0 == 35));
        }
        assert(value == expected[byte % 3]);
    }
    const auto& reset = symbols[8 * 24];
    assert(reset.level0 == 0 && reset.level1 == 0 && reset.duration0 + reset.duration1 == 12000);
    inFlight.assign(symbols, symbols + bytes / sizeof(*symbols));
    inFlightPointer = payload;
    transmitting = true;
    ++transfers;
    return ESP_OK;
}
esp_err_t rmt_tx_wait_all_done(rmt_channel_handle_t, int timeout) {
    assert(enabled && transmitting && timeout > 0);
    assert(std::memcmp(inFlightPointer, inFlight.data(), inFlight.size() * sizeof(rmt_symbol_word_t)) == 0);
    transmitting = false;
    return ESP_OK;
}
esp_err_t rmt_disable(rmt_channel_handle_t) { assert(enabled && !transmitting); enabled = false; return ESP_OK; }

int main() {
    testCandles();
    testCommands();
    testColorModes();
    testOutputs();
}
