#include <Arduino.h>
#include "src/Arcanet.h"
#include <string>
#include <cstdlib>

// Your device's unique ID
const String MY_ID = "LUX3";

//RGBW PWM Pins
const uint8_t DIM_PIN_WHITE = 2;  //D1 PT4115 DIM WHITE
const uint8_t DIM_PIN_BLUE  = 3;   //D2 PT4115 DIM BLUE
const uint8_t DIM_PIN_GREEN = 4;  //D3 PT4115 DIM GREEN
const uint8_t DIM_PIN_RED   = 5;    //D4 PT4115 DIM RED

const uint8_t DIM_PIN_BATTERY = 6;

float brightness = 0;

//not in use pins on the side of the RGBW pins
// const uint8_t DIM_PIN_D0         = 1;   //
// const uint8_t DIM_PIN_D3         = 4;   //
// const uint8_t DIM_PIN_D6         = 43;  //

//Settings for rgbw driver
const int CHANNEL = 0;
const int FREQ_HZ = 700;  // 1 kHz
static const int RES = 12;

uint32_t tLastBlinkOn = 0;
uint32_t tLastBlinkOff = 0;
uint32_t tBlinkTime = 200;

uint32_t tBlinkPeriod = 3000;
uint32_t now = millis();

unsigned long controllerUpdateScheduledAt = 0;
uint32_t tControllerUpdatePeriod = 60000;
boolean pendingControllerUpdate = false;

typedef struct {
    double r;  // a fraction between 0 and 1
    double g;  // a fraction between 0 and 1
    double b;  // a fraction between 0 and 1
    double w;  // a fraction between 0 and 1
} rgbw;

typedef struct {
    double h;  // angle in degrees
    double s;  // a fraction between 0 and 1
    double v;  // a fraction between 0 and 1
} hsv;


rgbw relicStatus = { 0, 0, 0 };
boolean ledUpdateNeeded = false;


static float parseBrightnessFromCommand(const String& command) {
    unsigned v = 0;
    if (sscanf(command.c_str(), "SET_BRIGHTNESS_%u", &v) != 1) return NAN;
    return (float)v;
}

static float parseHueFromCommand(const String& command) {
    unsigned v = 0;
    if (sscanf(command.c_str(), "SET_HUE_%u", &v) != 1) return NAN;
    return (float)v;
}

static rgbw parseRGBFromCommand(const String& command) {
    rgbw out{0,0,0,0};

    std::string cmd = command.c_str();

    // find the underscore and grab everything after
    auto pos = cmd.find("SET_RGB_");
    if (pos == std::string::npos) return out;

    std::string hex = cmd.substr(pos + 8); // "FF44FF"

    if (hex.size() != 6) return out;

    // parse each pair of hex digits
    int r = std::stoi(hex.substr(0,2), nullptr, 16);
    int g = std::stoi(hex.substr(2,2), nullptr, 16);
    int b = std::stoi(hex.substr(4,2), nullptr, 16);

    // normalize to 0..1
    out.r = r / 255.0;
    out.g = g / 255.0;
    out.b = b / 255.0;
    out.w = 0.0; // or compute white component if you want

    return out;
}



// Callback function to handle received commands
void onCommandReceived(const String& id, const String& command) {
    Serial.printf("Command received for ID: %s, Command: %s\n", id.c_str(), command.c_str());

    if (id == MY_ID || id == "LUXALL") {
        if (command.startsWith("SET_RGB")) {
            rgbw aRgbw = parseRGBFromCommand(command);
            setRgbw(aRgbw);
        } else if (command.startsWith("SET_HUE")) {
            float hue = parseHueFromCommand(command);
            setHue(hue);
        } else if (command.startsWith("SET_BRIGHTNESS")) {
            float value = parseBrightnessFromCommand(command);
            setBrightness(value);
        }
        ledUpdateNeeded = true;
        prepareControllerUpdateNow();
    }
}

// Create an instance of the Arcanet library
Arcanet arcanet(MY_ID, onCommandReceived);


void loop() {
    now = millis();

    arcanet.loop();
    readSerial();
    blink();
    if (ledUpdateNeeded) {
        updateLeds(relicStatus);
        ledUpdateNeeded = false;
    }

    sendUpdate();         //send update if requested
    updateControllers();  //prepare the regular update

    delay(1);
}



void setup() {

    Serial.begin(115200);
    //Optional code to create blinking led in loop
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);

    //init pwm dim pins
    initDimPin(DIM_PIN_RED);
    initDimPin(DIM_PIN_GREEN);
    initDimPin(DIM_PIN_BLUE);
    initDimPin(DIM_PIN_WHITE);

    delay(1500);

    // Initialize the Arcanet network
    arcanet.init();

    analogSetPinAttenuation(DIM_PIN_BATTERY, ADC_6db);  // FS ≈ 3.3 V

    pendingControllerUpdate = true;
    controllerUpdateScheduledAt = millis() + 35000;

    Serial.println("#################################");
    Serial.println("### Luc Arcana setup complete ###");
    Serial.println("#################################");
    Serial.println("My ID is: " + String(MY_ID));
}



void initDimPin(uint8_t pin) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);

    // try to attach
    if (ledcAttach(pin, FREQ_HZ, RES)) {
        uint16_t min = 0;
        ledcWrite(pin, min);
    } else {
        Serial.printf("LEDC attach failed on pin %u %dHz)\n", pin, FREQ_HZ);
    }

}

void blink() {
    if (now - tBlinkPeriod > tLastBlinkOn) {
        tLastBlinkOn = now;
        tLastBlinkOff = now + tBlinkTime;
        digitalWrite(LED_BUILTIN, LOW);
    }
    if (now > tLastBlinkOff) {
        digitalWrite(LED_BUILTIN, HIGH);
    }
}

static void readSerial() {
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




static hsv rgbw2hsv(rgbw in) {
  hsv out;
  double min, max, delta;

  min = in.r < in.g ? in.r : in.g;
  min = min < in.b ? min : in.b;

  max = in.r > in.g ? in.r : in.g;
  max = max > in.b ? max : in.b;

  out.v = max;
  delta = max - min;
  if (delta < 0.00001) {
    out.s = 0;
    out.h = 0;  // undefined, maybe nan?
    return out;
  }
  if (max > 0.0) {  // NOTE: if Max is == 0, this divide would cause a crash
    out.s = (delta / max);
  } else {
    // if max is 0, then r = g = b = 0
    // s = 0, h is undefined
    out.s = 0.0;
    out.h = NAN;
    return out;
  }
  if (in.r >= max)                  // > is bogus, just keeps compilor happy
    out.h = (in.g - in.b) / delta;  // between yellow & magenta
  else if (in.g >= max)
    out.h = 2.0 + (in.b - in.r) / delta;  // between cyan & yellow
  else
    out.h = 4.0 + (in.r - in.g) / delta;  // between magenta & cyan

  out.h *= 60.0;  // degrees

  if (out.h < 0.0)
    out.h += 360.0;

  return out;
}


static rgbw hsv2rgbw(hsv in) {
  double hh, p, q, t, ff;
  long i;
  rgbw out;

  if (in.s <= 0.0) {
    out.r = in.v;
    out.g = in.v;
    out.b = in.v;
    return out;
  }

  hh = in.h;
  if (hh >= 360.0) hh = 0.0;
  hh /= 60.0;
  i = (long)hh;
  ff = hh - i;
  p = in.v * (1.0 - in.s);
  q = in.v * (1.0 - (in.s * ff));
  t = in.v * (1.0 - (in.s * (1.0 - ff)));

  switch (i) {
    case 0:
      out.r = in.v;
      out.g = t;
      out.b = p;
      break;
    case 1:
      out.r = q;
      out.g = in.v;
      out.b = p;
      break;
    case 2:
      out.r = p;
      out.g = in.v;
      out.b = t;
      break;

    case 3:
      out.r = p;
      out.g = q;
      out.b = in.v;
      break;
    case 4:
      out.r = t;
      out.g = p;
      out.b = in.v;
      break;
    case 5:
    default:
      out.r = in.v;
      out.g = p;
      out.b = q;
      break;
  }
  return out;
}


void setHue(double aHue) {
    hsv _hsv = { aHue, 1, 1 };
    relicStatus = hsv2rgbw(_hsv);
}

void setRgbw(rgbw aRgbw) {
    relicStatus = aRgbw;
}


void setBrightness(float value) {
    value = constrain(value, 0.0f, 100.0f);
    brightness = value/100.0f;
}

// set brightness 0..1
void driveLedPWM(uint8_t pin, float value) {
    value = constrain(value, 0.0f, 1.0f);
    value = value * brightness;
    int ledV = (int) (value * ((1 << RES) - 1));
Serial.println(String(value)+" - "+String(ledV));
    ledcWrite(pin, ledV);
}
void updateLeds(rgbw aRgbw) {
    driveLedPWM(DIM_PIN_RED, relicStatus.r);
    driveLedPWM(DIM_PIN_GREEN, relicStatus.g);
    driveLedPWM(DIM_PIN_BLUE, relicStatus.b);
    driveLedPWM(DIM_PIN_WHITE, relicStatus.w);
}


int getBatteryLevel() {
    int mv = analogReadMilliVolts(DIM_PIN_BATTERY);
    mv = mv * 11.3;
    return mv;
}


void updateControllers() {
    if (now > tControllerUpdatePeriod && (now - tControllerUpdatePeriod) > controllerUpdateScheduledAt) {
        prepareControllerUpdate();
    }
}

void prepareControllerUpdate() {
    pendingControllerUpdate = true;
    controllerUpdateScheduledAt = millis() + random(0, 2000);
}

void prepareControllerUpdateNow() {
    pendingControllerUpdate = true;
    controllerUpdateScheduledAt = millis() + 10 + random(0, 10);
}

void sendUpdate() {
    if (pendingControllerUpdate && millis() >= controllerUpdateScheduledAt) {
        pendingControllerUpdate = false;
        int v_batt = getBatteryLevel();
        arcanet.sendCommand("CONTROLLER", MY_ID + "_" + "BLVL_" + String(v_batt) + "_SGNL_" + String(arcanet.getBestRssi()) + "_STATE_" + String(relicStatus.r) + "_" + String(relicStatus.g) + "_" + String(relicStatus.b) + "_" + String(relicStatus.w));
    }
}
