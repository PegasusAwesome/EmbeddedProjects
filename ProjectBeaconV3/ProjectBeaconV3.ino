#include <FastLED.h>
#include <WiFi.h>
#include "esp_bt.h"

// --- hardware ---
#define DATA_PIN     1
//#define LED_TYPE          SK6812
#define LED_TYPE          WS2812B
#define COLOR_ORDER       GRB
#define NUM_LEDS          93

// --- effect tuning ---
#define MAX_BRIGHTNESS    255   // global limit (0..255)
#define HEART_MIN_BRIGHTNESS  1
#define HEART_MAX_BRIGHTNESS  255

#define HEART_BPM 60
#define HEART_CYCLE_MS (60000 / HEART_BPM)

CRGB leds[NUM_LEDS];


// --- state ---
uint32_t tLastMove = 0;
uint32_t tLastCol  = 0;

static float smoothStep(float t) {
    if (t <= 0.0f) return 0.0f;
    if (t >= 1.0f) return 1.0f;
    // Quintic smoothing gives a softer start/end than the basic smoothstep.
    return t * t * t * (t * (t * 6.0f - 15.0f) + 10.0f);
}

static float rampSegment(float phase, float startPhase, float endPhase, float from, float to) {
    const float segmentDuration = endPhase - startPhase;
    const float progress = (phase - startPhase) / segmentDuration;
    return from + ((to - from) * smoothStep(progress));
}

static float heartbeatLevel(uint32_t elapsedMs) {
    const float phase = float(elapsedMs) / float(HEART_CYCLE_MS);

    if (phase < 0.1286f) {
        return rampSegment(phase, 0.0f, 0.1286f, 0.0f, 1.0f);
    }
    if (phase < 0.2571f) {
        return rampSegment(phase, 0.1286f, 0.2571f, 1.0f, 0.30f);
    }
    if (phase < 0.3571f) {
        return rampSegment(phase, 0.2571f, 0.3571f, 0.30f, 0.82f);
    }
    if (phase < 0.5429f) {
        return rampSegment(phase, 0.3571f, 0.5429f, 0.82f, 0.0f);
    }
    return 0.0f;
}

static void render() {
    const uint32_t beatPhaseMs = millis() % HEART_CYCLE_MS;
    const float beatLevel = heartbeatLevel(beatPhaseMs);
    const uint8_t brightness = HEART_MIN_BRIGHTNESS + uint8_t((HEART_MAX_BRIGHTNESS - HEART_MIN_BRIGHTNESS) * beatLevel);

    fill_solid(leds, NUM_LEDS, CHSV(0, 255, brightness));

    Serial.print(beatPhaseMs);
    Serial.print(" ");
    Serial.println(brightness);

    FastLED.show();
}


// draw current frame
static void renderTest() {
    CHSV c0(0, 0, MAX_BRIGHTNESS);
    CHSV c1(32, 0, MAX_BRIGHTNESS);
    CHSV c2(64, 0, MAX_BRIGHTNESS);
    CHSV c3(96, 0, MAX_BRIGHTNESS);
    CHSV c4(128, 0, MAX_BRIGHTNESS);
    CHSV c5(160, 0, MAX_BRIGHTNESS);

    fill_solid(leds, NUM_LEDS, CRGB::Black);

    for (int walk = 0; walk < 32; walk++) {
        leds[walk] = c0;
    }

    for (int walk = 32; walk < 32+24; walk++) {
        leds[walk] = c1;
    }

    for (int walk = 32+24; walk < 32+24+16; walk++) {
        leds[walk] = c2;
    }

    for (int walk = 32+24+16; walk < 32+24+16+12; walk++) {
        leds[walk] = c3;
    }

    for (int walk = 32+24+16+12; walk < 32+24+16+12+8; walk++) {
        leds[walk] = c4;
    }

    for (int walk = 32+24+16+12+8; walk < 32+24+16+12+8+1; walk++) {
        leds[walk] = c5;
    }

    FastLED.show();
}







static void sleepMs(uint32_t ms) {
    // // Lock the physical pin state at 0V
    // gpio_hold_en((gpio_num_t)DATA_PIN);

    // esp_sleep_enable_timer_wakeup((uint64_t)ms * 1000ULL);
    // esp_light_sleep_start();

    // // Release the hold so the CPU can control it again
    // gpio_hold_dis((gpio_num_t)DATA_PIN);

    // // Tiny delay for stability
    // delayMicroseconds(50); 

  vTaskDelay(pdMS_TO_TICKS(ms));
}


void setup() {
  Serial.begin(115200);

  FastLED.addLeds<LED_TYPE, DATA_PIN, COLOR_ORDER>(leds, NUM_LEDS);
  FastLED.setBrightness(MAX_BRIGHTNESS);
  FastLED.clear(true);

  tLastMove = millis();
  tLastCol  = millis();

  WiFi.mode(WIFI_OFF); 
  esp_bt_controller_disable();
  setCpuFrequencyMhz(80);
}

void loop() {
    render();
    sleepMs(15);
}
