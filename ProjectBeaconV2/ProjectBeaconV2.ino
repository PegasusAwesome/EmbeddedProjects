#include <Arduino.h>
#include "src/Arcanet.h"
#include <FastLED.h>
#include "driver/ledc.h"

// Your device's unique ID
const String MY_ID = "LANTERN30";

//GPIO of Popwer (GPIO 0 for reading battery lvl) pin
const uint8_t PIN_BATTERY        = 1;

//GPIO of Lantern control pin
const uint8_t PIN_LANTERN        = 22;
const int     freq               = 500;  // 0.5kHz frequency
const int     resolution         = 8;     // 8-bit resolution (0-255)
const int     PWM_MAX            = (1 << resolution) - 1;
const uint32_t PHASE_MS          = 5000;
const uint16_t WARM_WHITE_K      = 1600;
const uint16_t COLD_WHITE_K      = 7000;
const uint32_t WHITE_SHIFT_MS    = 30000;
const float RED_FADE_START_TEMP  = 62.0f;
const float RED_FADE_COEFFICIENT = 200.0f;

constexpr ledc_mode_t LANTERN_LEDC_MODE = LEDC_LOW_SPEED_MODE;
constexpr ledc_timer_t LANTERN_LEDC_TIMER = LEDC_TIMER_0;
const uint8_t LANTERN_LEDC_CHANNEL = 0;

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

bool setupLanternPwm() {
    ledcSetClockSource(LEDC_USE_RC_FAST_CLK);
    if (!ledcAttachChannel(PIN_LANTERN, freq, resolution, LANTERN_LEDC_CHANNEL)) {
        return false;
    }

    ledc_channel_config_t channel = {};
    channel.gpio_num = PIN_LANTERN;
    channel.speed_mode = LANTERN_LEDC_MODE;
    channel.channel = (ledc_channel_t)LANTERN_LEDC_CHANNEL;
    channel.timer_sel = LANTERN_LEDC_TIMER;
    channel.intr_type = LEDC_INTR_DISABLE;
    channel.duty = 0;
    channel.hpoint = 0;
    channel.sleep_mode = LEDC_SLEEP_MODE_KEEP_ALIVE;
    return ledc_channel_config(&channel) == ESP_OK;
}

void writeLanternPwm(uint32_t pwmValue) {
    if (pwmValue > PWM_MAX) {
        pwmValue = PWM_MAX;
    }
    ledcWrite(PIN_LANTERN, pwmValue);
}

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
    uint32_t cyclePosition = (millis() - start) % (WHITE_SHIFT_MS * 2);
    uint32_t rampPosition = cyclePosition;
    if (rampPosition > WHITE_SHIFT_MS) {
        rampPosition = (WHITE_SHIFT_MS * 2) - rampPosition;
    }

    float amount = rampPosition / (float)WHITE_SHIFT_MS;
    uint16_t kelvin = WARM_WHITE_K + (uint16_t)((COLD_WHITE_K - WARM_WHITE_K) * amount);
    return colorTemperature(kelvin);
}


// Callback function to handle received commands
void onCommandReceived(const String& id, const String& msg) {
    if (id == MY_ID || id == "LANTERNALL") {
        if (msg == "LANTERN_ON") {
            writeLanternPwm(128);
            relicStatus = true;
            prepareUpdateNow();

        } else if (msg == "LANTERN_OFF") {
            writeLanternPwm(0);
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

void waitForSerial(uint32_t timeoutMs = 3000) {
    uint32_t deadline = millis() + timeoutMs;
    while (!Serial && millis() < deadline) {
        delay(10);
    }
}


// Create an instance of the Arcanet library
Arcanet arcanet(MY_ID, onCommandReceived);

void setup() {
    Serial.begin(115200);

    // Init lantern led pin
    setupLanternPwm();

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
    start = millis();
}

void loop() {
    now = millis();

//    readSerial();//any commands from outside (TODO: put this behind a compile time switch)
    serviceFor(10);

    fill_solid(leds, NUM_LEDS, CRGB::Black);
    float brightness = 0;
//    CHSV c(0, 255, 255);
    CRGB c = currentWhiteColor();
    leds[0] = c;
    leds[1] = c;
    leds[2] = c;
    leds[3] = c;
    brightness = computeCandleBrightness();


    writeBrightness(brightness);
    FastLED.setBrightness(brightness*255/3);
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

constexpr float BASE_BRIGHTNESS = 0.48f;   // average brightness, 0.0 .. 1.0
constexpr float SLOW_AMOUNT_A   = 0.06f;   // slow body movement
constexpr float SLOW_AMOUNT_B   = 0.08f;   // slow body movement
constexpr float MEDIUM_AMOUNT_A = 0.030f;   // main flicker
constexpr float MEDIUM_AMOUNT_B = 0.030f;   // main flicker
constexpr float FAST_AMOUNT     = 0.03f;   // tiny shimmer

constexpr float MIN_BRIGHTNESS  = 0.20f;
constexpr float MAX_BRIGHTNESS  = 1.00f;

// -------------------- Time scales --------------------
// Larger values make the noise evolve faster.

constexpr float SLOW_SPEED_A = 0.115f;
constexpr float SLOW_SPEED_B = 0.12f;
constexpr float MEDIUM_SPEED_A = 0.295f;
constexpr float MEDIUM_SPEED_B = 0.30f;
constexpr float FAST_SPEED   = 1.000f;

float clamp01(float x) {
    if (x < 0.0f) return 0.0f;
    if (x > 1.0f) return 1.0f;
    return x;
}

// Convert normalized brightness [0,1] to PWM output
void writeBrightness(float brightness) {
    brightness = clamp01(brightness);
    uint32_t pwmValue = (uint32_t)(brightness * PWM_MAX);
    writeLanternPwm(pwmValue);
}

// Compute candle brightness from layered noise
float computeCandleBrightness() {
    float timeSeconds = millis() / 1000.0f;
    return constrain(BASE_BRIGHTNESS +
                         SLOW_AMOUNT_A * sinf(timeSeconds * SLOW_SPEED_A) +
                         SLOW_AMOUNT_B * sinf(timeSeconds * SLOW_SPEED_B) +
                         MEDIUM_AMOUNT_A * cosf(timeSeconds * MEDIUM_SPEED_A) +
                         MEDIUM_AMOUNT_B * cosf(timeSeconds * MEDIUM_SPEED_B) +
                         FAST_AMOUNT/2 * tanhf(sinf(timeSeconds * (FAST_SPEED-0.0025) ) * 2.0f) +
                         FAST_AMOUNT/2 * tanhf(sinf(timeSeconds * (FAST_SPEED+0.0025) ) * 2.0f)
                     ,
                     MIN_BRIGHTNESS,
                     MAX_BRIGHTNESS);
}

// writeBrightness(0);
// uint32_t sleepMs = PHASE_MS - (elapsed % PHASE_MS);
// esp_sleep_enable_timer_wakeup((uint64_t)sleepMs * 1000ULL);
// esp_light_sleep_start();

// Serial.end();
// delay(100);
// Serial.begin(115200);
// delay(100);
// return;
