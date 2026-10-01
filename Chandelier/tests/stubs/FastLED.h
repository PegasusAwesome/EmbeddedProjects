#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>

// Host substitutes for library/Arduino dependencies, not the candle implementation.
constexpr float PI = 3.14159265358979323846f;
long random(long minimum, long maximum);
uint16_t inoise16(uint32_t x, uint32_t y);
template <typename T> T constrain(T x, T lo, T hi) { return std::clamp(x, lo, hi); }

struct CRGB {
    uint8_t r = 0, g = 0, b = 0;
    CRGB() = default;
    CRGB(uint8_t red, uint8_t green, uint8_t blue) : r(red), g(green), b(blue) {}
    void nscale8(uint8_t scale) {
        r = (uint16_t(r) * (uint16_t(scale) + 1)) >> 8;
        g = (uint16_t(g) * (uint16_t(scale) + 1)) >> 8;
        b = (uint16_t(b) * (uint16_t(scale) + 1)) >> 8;
    }
    bool operator==(const CRGB& other) const { return r == other.r && g == other.g && b == other.b; }
};
