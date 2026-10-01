#include "CandleEffect.h"

namespace {
constexpr uint16_t WARM_WHITE_K = 1600;
constexpr uint16_t COLD_WHITE_K = 7000;
constexpr float MIN_BRIGHTNESS = 0.10f;
constexpr float MAX_BRIGHTNESS = 1.00f;

uint8_t clampByte(float value) {
    if (value < 0.0f) return 0;
    if (value > 255.0f) return 255;
    return (uint8_t)(value + 0.5f);
}

CRGB colorTemperature(uint16_t kelvin) {
    const float temp = kelvin / 100.0f;
    const float red = temp <= 62.0f
        ? 255.0f : 200.0f * powf(temp - 60.0f, -0.1332047592f);
    const float green = temp <= 62.0f
        ? 99.4708025861f * logf(temp) - 161.1195681661f
        : 288.1221695283f * powf(temp - 60.0f, -0.0755148492f);
    const float blue = temp >= 62.0f ? 255.0f
        : (temp <= 19.0f ? 0.0f : 138.5177312231f * logf(temp - 10.0f) - 305.0447927307f);
    return CRGB(clampByte(red), clampByte(green), clampByte(blue));
}

float noiseSigned(uint32_t x, uint32_t y) {
    return (inoise16(x, y) / 32767.5f) - 1.0f;
}
} // namespace

void CandleEffect::begin(uint32_t now, uint8_t candleIndex, uint8_t candleCount) {
    startedAt = now;
    whiteShiftMs = random(24000, 36001);
    // Spread starting phases across the cycle, then vary the speed independently.
    const uint32_t phaseWidth = whiteShiftMs * 2 / candleCount;
    whitePhaseMs = phaseWidth * candleIndex + random(0, phaseWidth);
    seedSlow = (uint16_t)random(0, 65536);
    seedBody = (uint16_t)random(0, 65536);
    seedFast = (uint16_t)random(0, 65536);
    flickerSpeedPercent = (uint8_t)random(85, 116);
    baseBrightness = random(34, 43) * 0.01f;

    nextDipAt = now + random(700, 6500);
    nextFlareAt = now + random(500, 4200);
    dipDuration = 0;
    flareDuration = 0;
}

CRGB CandleEffect::colorAt(uint32_t now, CandleColor mode) {
    CRGB color;
    if (mode == CandleColor::Red) {
        color = CRGB(255, 0, 0);
    } else if (mode == CandleColor::Green) {
        color = CRGB(0, 255, 0);
    } else {
        const uint32_t cycleMs = whiteShiftMs * 2;
        const uint32_t cyclePosition = ((now - startedAt) % cycleMs + whitePhaseMs) % cycleMs;
        const uint32_t rampPosition = cyclePosition <= whiteShiftMs
            ? cyclePosition : cycleMs - cyclePosition;
        const float amount = rampPosition / (float)whiteShiftMs;
        const uint16_t kelvin = WARM_WHITE_K + (uint16_t)((COLD_WHITE_K - WARM_WHITE_K) * amount);
        color = colorTemperature(kelvin);
    }
    // Bake brightness into this candle's RGB values, retaining the old 1/3 cap.
    color.nscale8((uint8_t)(brightnessAt(now) * 255 / 3));
    return color;
}

float CandleEffect::brightnessAt(uint32_t now) {
    const uint32_t elapsed = now - startedAt;
    const uint32_t t = (uint32_t)((uint64_t)elapsed * flickerSpeedPercent / 100);

    if ((int32_t)(now - nextDipAt) >= 0) {
        dipStart = now;
        dipDuration = (uint16_t)random(80, 360);
        dipDepth = random(5, 22) * 0.01f;
        nextDipAt = now + random(1200, 8000);
    }

    if ((int32_t)(now - nextFlareAt) >= 0) {
        flareStart = now;
        flareDuration = (uint16_t)random(130, 520);
        flareAttack = (uint16_t)random(12, 55);
        flareHeight = random(7, 24) * 0.01f;
        nextFlareAt = now + random(900, 6500);
    }

    const float slow = noiseSigned(t * 2, seedSlow) * 0.10f;
    const float body = noiseSigned(t * 10, seedBody) * 0.055f;
    const float fast = noiseSigned(t * 60, seedFast) * 0.035f;

    float dip = 0.0f;
    const uint32_t dipAge = now - dipStart;
    if (dipAge < dipDuration) {
        const float u = dipAge / (float)dipDuration;
        dip = dipDepth * sinf(u * PI);
    }

    float flare = 0.0f;
    const uint32_t flareAge = now - flareStart;
    if (flareAge < flareDuration) {
        const float attack = flareAttack / (float)flareDuration;
        const float u = flareAge / (float)flareDuration;
        if (u < attack) {
            const float v = u / attack;
            flare = flareHeight * v * v * (3.0f - 2.0f * v);
        } else {
            const float v = (u - attack) / (1.0f - attack);
            flare = flareHeight * (1.0f - v) * (1.0f - v);
        }
    }

    return constrain(baseBrightness + slow + body + fast + flare - dip,
                     MIN_BRIGHTNESS, MAX_BRIGHTNESS);
}
