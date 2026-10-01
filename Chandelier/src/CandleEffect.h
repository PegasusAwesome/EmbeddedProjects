#pragma once

#include <FastLED.h>

enum class CandleColor : uint8_t { Normal, Red, Green };

// One instance per physical candle: no shared animation timers or noise seeds.
class CandleEffect {
public:
    void begin(uint32_t now, uint8_t candleIndex, uint8_t candleCount);
    CRGB colorAt(uint32_t now, CandleColor mode = CandleColor::Normal);

private:
    float brightnessAt(uint32_t now);

    uint32_t startedAt = 0;
    uint32_t whiteShiftMs = 30000;
    uint32_t whitePhaseMs = 0;
    uint16_t seedSlow = 0;
    uint16_t seedBody = 0;
    uint16_t seedFast = 0;
    uint8_t flickerSpeedPercent = 100;
    float baseBrightness = 0.38f;

    uint32_t nextDipAt = 0;
    uint32_t dipStart = 0;
    uint16_t dipDuration = 0;
    float dipDepth = 0.0f;

    uint32_t nextFlareAt = 0;
    uint32_t flareStart = 0;
    uint16_t flareDuration = 0;
    uint16_t flareAttack = 0;
    float flareHeight = 0.0f;
};
