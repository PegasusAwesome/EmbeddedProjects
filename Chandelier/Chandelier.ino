#include <Arduino.h>
#include <FastLED.h>
#include "esp_system.h"
#include "src/Arcanet.h"
#include "src/CandleEffect.h"
#include "src/ChandelierControl.h"
#include "src/Ws2812Output.h"

// Your device's unique ID.
const String MY_ID = "CHANDELIER32";

// XIAO ESP32-C6: D1=GPIO1, D2=GPIO2, D3=GPIO21, D4=GPIO22, D5=GPIO23.
const uint8_t CANDLE_PINS[] = {D1, D2, D3, D4, D5};
constexpr uint8_t NUM_CANDLES = sizeof(CANDLE_PINS) / sizeof(CANDLE_PINS[0]);

// Each output drives its own pair of four-LED WS2812B units (GRB order).
#define NUM_LED_UNITS  2
#define LEDS_PER_UNIT  4
#define NUM_LEDS       (NUM_LED_UNITS * LEDS_PER_UNIT)

CandleEffect candles[NUM_CANDLES];
ChandelierControl chandelier;
Ws2812Output<NUM_LEDS> ledOutput;

// Battery sensing needs a separate ADC pin; never read a candle's data pin.
const uint8_t PIN_BATTERY = 1;
constexpr uint32_t STATUS_UPDATE_PERIOD_MS = 10000;
uint32_t updateScheduledAt = 0;
bool pendingUpdate = false;

bool isCandlePin(uint8_t pin) {
    for (uint8_t candlePin : CANDLE_PINS) {
        if (pin == candlePin) return true;
    }
    return false;
}

// Network commands addressed to this device or the chandelier group.
void onCommandReceived(const String& id, const String& msg) {
    if (chandelier.handleCommand(id.c_str(), MY_ID.c_str(), msg.c_str())) {
        prepareUpdateNow();
    }
}

Arcanet arcanet(MY_ID, onCommandReceived, false);

void setup() {
    Serial.begin(115200);
    randomSeed(esp_random());
    arcanet.init();

    if (!isCandlePin(PIN_BATTERY)) {
        analogSetPinAttenuation(PIN_BATTERY, ADC_11db);
    } else {
        Serial.println("Battery monitoring disabled: battery and LED data share a GPIO.");
    }

    for (uint8_t pin : CANDLE_PINS) {
        pinMode(pin, OUTPUT);
        digitalWrite(pin, LOW);
    }
    ledOutput.begin(CANDLE_PINS[0]);

    const uint32_t now = millis();
    for (uint8_t i = 0; i < NUM_CANDLES; ++i) {
        candles[i].begin(now, i, NUM_CANDLES);
        ledOutput.showSolid(CANDLE_PINS[i], CRGB::Black);
    }

    pendingUpdate = true;
    updateScheduledAt = millis() + STATUS_UPDATE_PERIOD_MS;
    Serial.println("Chandelier setup complete: 5 independent candles on D1-D5");
    Serial.println("My ID is: " + MY_ID);
}

void loop() {
    serviceFor(10);

    const uint32_t now = millis();
    for (uint8_t i = 0; i < NUM_CANDLES; ++i) {
        ledOutput.showSolid(CANDLE_PINS[i], chandelier.colorAt(candles[i], now));
    }
}

void serviceFor(uint32_t ms) {
    const uint32_t serviceStartedAt = millis();
    while (millis() - serviceStartedAt < ms) {
        arcanet.loop();
        sendUpdate();
        updateControllers();
        delay(1);
    }
}

void updateControllers() {
    if (millis() > updateScheduledAt + STATUS_UPDATE_PERIOD_MS) {
        prepareUpdate();
    }
}

int getBatteryLevel() {
    if (isCandlePin(PIN_BATTERY)) {
        return 0; // No measurement available; do not reconfigure an LED data GPIO.
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
            + "_STATE_" + (chandelier.isOn() ? "ON" : "OFF");
        arcanet.sendCommand("CONTROLLER", status);
    }
}


