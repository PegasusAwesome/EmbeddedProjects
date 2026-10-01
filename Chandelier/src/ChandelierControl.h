#pragma once

#include <cstring>
#include "CandleEffect.h"

// Shared controls for all candles; each candle retains its own animation state.
class ChandelierControl {
public:
    // True requests a status reply for a recognised, correctly addressed command.
    bool handleCommand(const char* targetId, const char* deviceId, const char* command) {
        if (std::strcmp(targetId, deviceId) != 0 &&
            std::strcmp(targetId, "CHANDELIER") != 0 &&
            std::strcmp(targetId, "ALL") != 0) {
            return false;
        }

        if (std::strcmp(command, "CHANDELIER_ON") == 0) {
            enabled = true;
        } else if (std::strcmp(command, "CHANDELIER_OFF") == 0) {
            enabled = false;
        } else if (std::strcmp(command, "CHANDELIER_COLOR_RED") == 0) {
            mode = CandleColor::Red;
        } else if (std::strcmp(command, "CHANDELIER_COLOR_NORMAL") == 0) {
            mode = CandleColor::Normal;
        } else if (std::strcmp(command, "CHANDELIER_COLOR_GREEN") == 0) {
            mode = CandleColor::Green;
        } else if (std::strcmp(command, "SEND_UPDATE") != 0) {
            return false;
        }
        return true;
    }

    CRGB colorAt(CandleEffect& candle, uint32_t now) const {
        // Keep timers moving while dark, so switching on never restarts the flicker.
        const CRGB color = candle.colorAt(now, mode);
        return enabled ? color : CRGB(0, 0, 0);
    }

    bool isOn() const { return enabled; }
    CandleColor colorMode() const { return mode; }

private:
    bool enabled = true;
    CandleColor mode = CandleColor::Normal;
};
