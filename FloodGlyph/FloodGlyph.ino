#include <FastLED.h>
#include <WiFi.h>
#include "esp_bt.h"

// --- hardware ---
#define DATA_PIN     1
#define LED_TYPE     WS2812B
#define COLOR_ORDER  GRB
#define NUM_LEDS     24

CRGB leds[NUM_LEDS];

// --- effect tuning ---
#define MAX_BRIGHTNESS    255   // global limit (0..255)


// draw current frame
static void render() {
    CHSV c(0, 255, 255);

    fill_solid(leds, NUM_LEDS, CRGB::Red);
    FastLED.show();
    sleepMs(3000);

    fill_solid(leds, NUM_LEDS, CRGB::Yellow);
    FastLED.show();
    sleepMs(3000);

    fill_solid(leds, NUM_LEDS, CRGB::Green);
    FastLED.show();
    sleepMs(3000);

    fill_solid(leds, NUM_LEDS, CRGB::Cyan);
    FastLED.show();
    sleepMs(3000);

    fill_solid(leds, NUM_LEDS, CRGB::Blue);
    FastLED.show();
    sleepMs(3000);

    fill_solid(leds, NUM_LEDS, CRGB::Magenta);
    FastLED.show();
    sleepMs(3000);

    // fill_solid(leds, NUM_LEDS, CRGB::White);
    // FastLED.show();
    // sleepMs(3000);

    fill_solid(leds, NUM_LEDS, CRGB::Black);
    FastLED.show();
    sleepMs(3000);

}

//Someday Ill get the sleep to WORK!
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
//  Serial.begin(115200);

  FastLED.addLeds<LED_TYPE, DATA_PIN, COLOR_ORDER>(leds, NUM_LEDS);
  FastLED.setBrightness(MAX_BRIGHTNESS);
  FastLED.clear(true);

  WiFi.mode(WIFI_OFF); 
  esp_bt_controller_disable();
  setCpuFrequencyMhz(80);

  fill_solid(leds, 5, CRGB::Red);
  FastLED.show();
  sleepMs(1500);

}

void loop() {
    render();
}