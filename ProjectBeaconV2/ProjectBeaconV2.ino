#include <Arduino.h>
#include "src/Arcanet.h"
#include <FastLED.h>

// Your device's unique ID
const String MY_ID = "LANTERN30";

//GPIO of Popwer (GPIO 0 for reading battery lvl) pin
const uint8_t PIN_BATTERY        = 1;

//GPIO of Lantern control pin
const uint8_t PIN_LANTERN        = 22;
const int     freq               = 500;  // 0.5kHz frequency
const int     resolution         = 8;     // 8-bit resolution (0-255)
const int     PWM_MAX            = (1 << resolution) - 1;

#define DATA_PIN       23
#define LED_TYPE       WS2812B
#define COLOR_ORDER    GRB
#define NUM_LEDS       4

CRGB leds[NUM_LEDS];



unsigned long updateScheduledAt  = 0;
uint32_t tUpdatePeriod = 10000;
boolean pendingUpdate = false;
boolean relicStatus = false;


int16_t  hue            = 0;

uint32_t now            = millis();
uint32_t start          = millis();

const double amplitude = 80.0;
const double period_milli_seconds = 30000.0;


// Callback function to handle received commands
void onCommandReceived(const String& id, const String& msg) {
    if (id == MY_ID || id == "LANTERNALL") {
        if (msg == "LANTERN_ON") {
            ledcWrite(PIN_LANTERN, 128);
            relicStatus = true;
            prepareUpdateNow();

        } else if (msg == "LANTERN_OFF") {
            ledcWrite(PIN_LANTERN, 0);
            relicStatus = false;
            prepareUpdateNow();

        } else if (msg == "SEND_UPDATE") {
            prepareUpdateNow();

        }
    } else if (id == "ALL") {
        if (msg == "SEND_UPDATE") {
            prepareUpdateNow();

        }
    }
}

// Create an instance of the Arcanet library
Arcanet arcanet(MY_ID, onCommandReceived);

void setup() {
    Serial.begin(115200);

    //Optional code to init the led on the XIAO board, so can be used in loop  
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);

    // Init lantern led pin
    ledcAttach(PIN_LANTERN, freq, resolution);

    // Initialize the Arcanet network
    arcanet.init();

    analogSetPinAttenuation(PIN_BATTERY, ADC_11db); // FS ≈ 3.3 V

    Serial.println("####################################");
    Serial.println("### ProjectBeacon setup complete ###");
    Serial.println("####################################");
    Serial.println("My ID is: "+String(MY_ID));

    pendingUpdate = true;
    updateScheduledAt = millis() + 10000;

    FastLED.addLeds<LED_TYPE, DATA_PIN, COLOR_ORDER>(leds, NUM_LEDS);
    fill_solid(leds, NUM_LEDS, CRGB::Red);
    FastLED.setBrightness(100);
    FastLED.show();
    ledcWrite(PIN_LANTERN, 100);
    delay(250);
    fill_solid(leds, NUM_LEDS, CRGB::Black);
    ledcWrite(PIN_LANTERN, 0);
    FastLED.show();
    delay(250);
    fill_solid(leds, NUM_LEDS, CRGB::Red);
    ledcWrite(PIN_LANTERN, 100);
    FastLED.setBrightness(100);
    FastLED.show();
    delay(250);
    fill_solid(leds, NUM_LEDS, CRGB::Black);
    ledcWrite(PIN_LANTERN, 0);
    FastLED.show();
    delay(250);

    start = millis();

}

void loop() {
    now = millis();


    readSerial();//any commands from outside (TODO: put this behind a compile time switch)
    serviceFor(10);

    fill_solid(leds, NUM_LEDS, CRGB::Black);
    float brightness = 0;
    CHSV c(0, 255, 255);

    bool firstPairActive = ((now - start) / 5000) % 2 == 0;//0..4999 ms -> 0, 5000..9999 ms -> 1, so with the % 2 (modulo 2) even numbers are true
    if (((now - start) / 5000) % 4 == 0) {
        leds[0] = c;
        leds[2] = c;
    } else if (((now - start) / 5000) % 4 == 1) {
        leds[1] = c;
        leds[3] = c;
    } else if (((now - start) / 5000) % 4 == 2) {
        brightness = computeCandleBrightness();
    } else {

    }
    
    writeBrightness(brightness);
    

    // double value = 100 + amplitude * std::sin(2.0 * 3.1415926 * (now-start) / period_milli_seconds);
    // FastLED.setBrightness(value);
    FastLED.setBrightness(255);
    FastLED.show();

}


void serviceFor(uint32_t ms) {
    uint32_t start = millis();
    while (millis() - start < ms) {
        now = millis();
        arcanet.loop();            // processes discovery + queue
        sendUpdate();
        updateControllers();
        delay(1);                  // yield
    }                   
}


void updateControllers() {
    if ( now > updateScheduledAt + tUpdatePeriod) {
        prepareUpdate();
    }
}

void readSerial() {
    if (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n');
        input.trim();
        int separator = input.indexOf('_');
        if (separator > 0) {
            String id = input.substring(0, separator);
            String cmd = input.substring(separator + 1);

            if (id == MY_ID) {
                Serial.println("handle command");
                onCommandReceived(id, cmd);
            }

            arcanet.sendCommand(id, cmd);
        }
    }
}

int getBatteryLevel() {
    int mv = analogReadMilliVolts(PIN_BATTERY); 
    mv = mv<1 ? 1 : mv*2;
    return mv;
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
        int v_batt = getBatteryLevel();
        arcanet.sendCommand("CONTROLLER", MY_ID+"_"+"BLVL_"+String(v_batt)+"_SGNL_"+String(arcanet.getBestRssi())+"_STATE_"+( relicStatus ? "ON" : "OFF") );
    }
}


// CANDLE LOGIC

// -------------------- Brightness tuning --------------------

constexpr float BASE_BRIGHTNESS = 0.62f;   // average brightness, 0.0 .. 1.0
constexpr float SLOW_AMOUNT     = 0.22f;   // slow body movement
constexpr float MEDIUM_AMOUNT   = 0.12f;   // main flicker
constexpr float FAST_AMOUNT     = 0.06f;   // tiny shimmer

constexpr float MIN_BRIGHTNESS  = 0.08f;
constexpr float MAX_BRIGHTNESS  = 1.00f;

// -------------------- Time scales --------------------
// Larger values make the noise evolve faster.

constexpr float SLOW_SPEED   = 0.5f;
constexpr float MEDIUM_SPEED = 1.3f;
constexpr float FAST_SPEED   = 4.25f;

float clamp01(float x) {
  if (x < 0.0f) return 0.0f;
  if (x > 1.0f) return 1.0f;
  return x;
}

// Convert normalized brightness [0,1] to PWM output
void writeBrightness(float brightness) {
  brightness = clamp01(brightness);
  uint32_t pwmValue = (uint32_t)(brightness * PWM_MAX);
  ledcWrite(PIN_LANTERN, pwmValue);
}

// Compute candle brightness from layered noise
float computeCandleBrightness() {
  float timeSeconds = millis() / 1000.0f;
//  return constrain(BASE_BRIGHTNESS + SLOW_AMOUNT * sinf(timeSeconds * SLOW_SPEED), MIN_BRIGHTNESS, MAX_BRIGHTNESS);
  return constrain(BASE_BRIGHTNESS + SLOW_AMOUNT * sinf(timeSeconds * SLOW_SPEED) + MEDIUM_AMOUNT * cosf(timeSeconds * MEDIUM_SPEED) + FAST_AMOUNT * tanhf(sinf(timeSeconds * FAST_SPEED) * 2.0f), MIN_BRIGHTNESS, MAX_BRIGHTNESS);
}

