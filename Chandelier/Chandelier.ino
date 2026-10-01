#include <Arduino.h>
#include <FastLED.h>
#include "esp_system.h"
#include "src/Arcanet.h"

// Your device's unique ID
const String MY_ID = "HEART32";

// Battery sensing needs its own ADC pin; it is skipped if shared with LED data.
const uint8_t PIN_BATTERY = 1;

// White-temperature cycle.
const uint16_t WARM_WHITE_K      = 1600;
const uint16_t COLD_WHITE_K      = 7000;
const uint32_t WHITE_SHIFT_MS    = 30000;
const float RED_FADE_START_TEMP  = 62.0f;
const float RED_FADE_COEFFICIENT = 200.0f;

#define DATA_PIN       1
#define LED_TYPE       WS2812B
#define COLOR_ORDER    GRB
// Count individual LEDs across the entire daisy chain, not just the units.
#define NUM_LED_UNITS  2
#define LEDS_PER_UNIT  4
#define NUM_LEDS       (NUM_LED_UNITS * LEDS_PER_UNIT)

CRGB leds[NUM_LEDS];

// Candle brightness tuning (normalized to 0.0 .. 1.0).
constexpr float BASE_BRIGHTNESS = 0.38f;
constexpr float MIN_BRIGHTNESS = 0.10f;
constexpr float MAX_BRIGHTNESS = 1.00f;
constexpr uint32_t STATUS_UPDATE_PERIOD_MS = 10000;

uint32_t updateScheduledAt = 0;
uint32_t effectStartedAt = 0;
bool pendingUpdate = false;

uint8_t clampByte(float value) {
    if (value < 0.0f) {
        return 0;
    }
    if (value > 255.0f) {
        return 255;
    }
    return (uint8_t)(value + 0.5f);
}

CRGB colorTemperature(uint16_t kelvin) {
    float temp = kelvin / 100.0f;
    float red;
    float green;
    float blue;

    if (temp <= RED_FADE_START_TEMP) {
        red = 255.0f;
        green = 99.4708025861f * logf(temp) - 161.1195681661f;
    } else {
        red = RED_FADE_COEFFICIENT * powf(temp - 60.0f, -0.1332047592f);
        green = 288.1221695283f * powf(temp - 60.0f, -0.0755148492f);
    }

    if (temp >= RED_FADE_START_TEMP) {
        blue = 255.0f;
    } else if (temp <= 19.0f) {
        blue = 0.0f;
    } else {
        blue = 138.5177312231f * logf(temp - 10.0f) - 305.0447927307f;
    }

    return CRGB(clampByte(red), clampByte(green), clampByte(blue));
}

CRGB currentWhiteColor() {
    uint32_t cyclePosition = (millis() - effectStartedAt) % (WHITE_SHIFT_MS * 2);
    uint32_t rampPosition = cyclePosition;
    if (rampPosition > WHITE_SHIFT_MS) {
        rampPosition = (WHITE_SHIFT_MS * 2) - rampPosition;
    }

    float amount = rampPosition / (float)WHITE_SHIFT_MS;
    uint16_t kelvin = WARM_WHITE_K + (uint16_t)((COLD_WHITE_K - WARM_WHITE_K) * amount);
    return colorTemperature(kelvin);
}

// Network commands addressed to this device or the chandelier group.
void onCommandReceived(const String& id, const String& msg) {
    if (msg == "SEND_UPDATE" && (id == MY_ID || id == "CHANDELIER" || id == "ALL")) {
        prepareUpdateNow();
    }
}

// Create an instance of the Arcanet library
Arcanet arcanet(MY_ID, onCommandReceived);

void setup() {
    Serial.begin(115200);

    randomSeed(esp_random());

    // Initialize the Arcanet network
    arcanet.init();

    if (PIN_BATTERY != DATA_PIN) {
        analogSetPinAttenuation(PIN_BATTERY, ADC_11db); // FS ≈ 3.3 V
    } else {
        Serial.println("Battery monitoring disabled: battery and LED data share a GPIO.");
    }

    Serial.println("Chandelier setup complete");
    Serial.println("My ID is: " + MY_ID);

    pendingUpdate = true;
    updateScheduledAt = millis() + STATUS_UPDATE_PERIOD_MS;

    FastLED.addLeds<LED_TYPE, DATA_PIN, COLOR_ORDER>(leds, NUM_LEDS);
    effectStartedAt = millis();
}

void loop() {
    serviceFor(10);

    fill_solid(leds, NUM_LEDS, currentWhiteColor());
    const float brightness = computeCandleBrightness();
    FastLED.setBrightness(brightness * 255 / 3);
    FastLED.show();
}

void serviceFor(uint32_t ms) {
    const uint32_t serviceStartedAt = millis();
    while (millis() - serviceStartedAt < ms) {
        arcanet.loop();            // processes discovery + queue
        sendUpdate();
        updateControllers();
        delay(1);                  // yield
    }
}

void updateControllers() {
    if (millis() > updateScheduledAt + STATUS_UPDATE_PERIOD_MS) {
        prepareUpdate();
    }
}

int getBatteryLevel() {
    // An ADC read would reconfigure the LED data pin and interrupt its output.
    if (PIN_BATTERY == DATA_PIN) {
        return 0; // Battery reading unavailable; do not touch the LED data GPIO.
    }
    const int mv = analogReadMilliVolts(PIN_BATTERY);
    return mv < 1 ? 1 : mv * 2;
}

void prepareUpdate() {
    pendingUpdate = true;
    updateScheduledAt = millis() + random(0, 2000);
}

void prepareUpdateNow() {
    pendingUpdate = true;
    updateScheduledAt = millis() + 10 + random(0, 10);
}

void sendUpdate() {
    if (pendingUpdate && millis() >= updateScheduledAt) {
        pendingUpdate = false;
        const int batteryMillivolts = getBatteryLevel();
        const String status = MY_ID + "_BLVL_" + String(batteryMillivolts)
            + "_SGNL_" + String(arcanet.getBestRssi())
            + "_STATE_ON";
        arcanet.sendCommand("CONTROLLER", status);
    }
}

// Compute candle brightness from layered noise.
static float noiseSigned(uint32_t x, uint32_t y) {
    return (inoise16(x, y) / 32767.5f) - 1.0f;
}

float computeCandleBrightness() {
    uint32_t t = millis();

    static uint32_t seedSlow = random(0, 65535);
    static uint32_t seedBody = random(0, 65535);
    static uint32_t seedFast = random(0, 65535);

    static uint32_t nextDipAt = 0;
    static uint32_t dipStart = 0;
    static uint16_t dipDuration = 0;
    static float dipDepth = 0.0f;

    static uint32_t nextFlareAt = 0;
    static uint32_t flareStart = 0;
    static uint16_t flareDuration = 0;
    static uint16_t flareAttack = 0;
    static float flareHeight = 0.0f;

    if (nextDipAt == 0) {
        nextDipAt = t + random(700, 6500);
    }

    if (nextFlareAt == 0) {
        nextFlareAt = t + random(500, 4200);
    }

    if ((int32_t)(t - nextDipAt) >= 0) {
        dipStart = t;
        dipDuration = random(80, 360);
        dipDepth = random(5, 22) * 0.01f;
        nextDipAt = t + random(1200, 8000);
    }

    if ((int32_t)(t - nextFlareAt) >= 0) {
        flareStart = t;
        flareDuration = random(130, 520);
        flareAttack = random(12, 55);
        flareHeight = random(7, 24) * 0.01f;
        nextFlareAt = t + random(900, 6500);
    }

    float slow = noiseSigned(t * 2,  seedSlow) * 0.10f;
    float body = noiseSigned(t * 10, seedBody) * 0.055f;
    float fast = noiseSigned(t * 60, seedFast) * 0.035f;

    float dip = 0.0f;
    uint32_t dipAge = t - dipStart;
    if (dipAge < dipDuration) {
        float u = dipAge / (float)dipDuration;
        dip = dipDepth * sinf(u * PI);
    }

    float flare = 0.0f;
    uint32_t flareAge = t - flareStart;
    if (flareAge < flareDuration) {
        float attack = flareAttack / (float)flareDuration;
        float u = flareAge / (float)flareDuration;
        if (u < attack) {
            float v = u / attack;
            flare = flareHeight * v * v * (3.0f - 2.0f * v);
        } else {
            float v = (u - attack) / (1.0f - attack);
            flare = flareHeight * (1.0f - v) * (1.0f - v);
        }
    }

    return constrain(
        BASE_BRIGHTNESS + slow + body + fast + flare - dip,
        MIN_BRIGHTNESS,
        MAX_BRIGHTNESS
    );
}


