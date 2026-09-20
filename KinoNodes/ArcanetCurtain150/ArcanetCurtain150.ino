#include <Arduino.h>
#include "src\Arcanet.h"
#include <Wire.h>   
#include <FastLED.h>
// #include "driver/ledc.h"
#include "esp_random.h"
#define DATA_PIN_1 D0     //22
#define DATA_PIN_2 D1     //34     
#define LED_TYPE WS2812B
#define COLOR_ORDER GRB
// #define COLOR_ORDER_2 GRB
#define NUM_LEDS 8

// Your device's unique ID
const String MY_ID = "CURTAIN150"; // CHANGE THIS
// 150 XIAO_ESP-32C6 in Arduino IDE

// LED Pin
const int LED_PIN = LED_BUILTIN;
int i=0;

// Declare leds (troubleshooting)
CRGB leds1[NUM_LEDS];
uint8_t gBrightness = 255;

char* validCmd[]={
"AliceBlue",
"Amethyst",
"AntiqueWhite",
"Aqua",
"Aquamarine",
"Azure",
"Beige",
"Bisque",
"Black",
};

//GPIO of Power (GPIO 0 for reading battery lvl) pin
// First pin is analog in
const uint8_t PIN_BATTERY = 2;
int minBatteryLevel = 2900;
unsigned long now = millis();
boolean relicStatus = false;
unsigned long tLastUpdate = 0;
unsigned long tUpdatePeriod = 60000;

uint32_t tLastBlinkOn  = 0;
uint32_t tLastBlinkOff = 0;
uint32_t tBlinkTime    = 200;
uint32_t tBlinkPeriod  = 3000;
uint8_t COMMAND_NUM = 0;

// Variabelen voor Fire1
int fireValue = 0; // No blue in a natural fire

// Variabelen voor HeartBeat
//***************************************************************
// Heart pulse, blood flowing example
// Marc Miller, Jan 2016
//***************************************************************
uint8_t bloodHue = 255;  // Blood color [hue from 0-255]
uint8_t bloodSat = 255;  // Blood staturation [0-255]
int flowDirection = -1;   // Use either 1 or -1 to set flow direction
uint16_t cycleLength = 1500;  // Lover values = continuous flow, higher values = distinct pulses.
uint16_t pulseLength = 150;  // How long the pulse takes to fade out.  Higher value is longer.
uint16_t pulseOffset = 200;  // Delay before second pulse.  Higher value is more delay.
uint8_t baseBrightness = 10;  // Brightness of LEDs when not pulsing. Set to 0 for off.


//***************************************************************
// Breathing effect
// Color shifts from hueA to hueB as it pulses.
// Set A and B to the same hue if you don't want the color to
// change.  Saturation for the high and low can also be set.
// Marc Miller, 2015
// Updated Aug 2020 - removed delay, added dim8_video
//***************************************************************
static float pulseSpeed = 0.5;  // Larger value gives faster pulse.

uint8_t hueA = 160;  // Start hue at valueMin.
uint8_t satA = 230;  // Start saturation at valueMin.
float valueMin = 120.0;  // Pulse minimum value (Should be less then valueMax).

uint8_t hueB = 224;  // End hue at valueMax.
uint8_t satB = 255;  // End saturation at valueMax.
float valueMax = 255.0;  // Pulse maximum value (Should be larger then valueMin).

uint8_t hue = hueA;  // Do Not Edit
uint8_t sat = satA;  // Do Not Edit
float val = valueMin;  // Do Not Edit
uint8_t hueDelta = hueA - hueB;  // Do Not Edit
static float delta = (valueMax - valueMin) / 2.35040238;  // Do Not Edit
float dV = 0;

// Callback function to handle received commands
void onCommandReceived(const String& id, const String& command) {
  Serial.printf("Command received for ID: %s, Command: %s\n", id.c_str(), command.c_str());

  if ((id == MY_ID) || (id == "ALLCURTAINS") || (id == "CURTAINSALL")) {
    FastLED.clear();
    FastLED.setBrightness(gBrightness);  // Reset Brightness
    FastLED.show();
    if (command == "ON") {
      COMMAND_NUM = 9;
    }
    if (command == "OFF") {
      COMMAND_NUM = 0;
    }
    if (command == "FIRE") {
      COMMAND_NUM = 1;
    }
    if (command == "HEARTBEAT") {
      COMMAND_NUM = 2;
    }
    if (command == "BLUEHELL") {
      COMMAND_NUM = 3;
    }
    if (command == "RED") {
      for(i = 0; i< NUM_LEDS; i++) {
        leds1[i] = CRGB::Red;
      }
      COMMAND_NUM = 4;
      FastLED.show();
    }
    if (command == "BLACK") {
      for(i = 0; i< NUM_LEDS; i++) {
        leds1[i] = CRGB::Black;
      }
      COMMAND_NUM = 4;
      FastLED.show();
    }
    if (command == "YELLOW") {
      for(i = 0; i< NUM_LEDS; i++) {
        leds1[i] = CRGB::Yellow;
      }
      COMMAND_NUM = 4;
      FastLED.show();
    }
    if (command == "BLUE") {
      for(i = 0; i< NUM_LEDS; i++) {
        leds1[i] = CRGB::Blue;
      }
      COMMAND_NUM = 4;
      FastLED.show();
    }
    if (command == "GREEN") {
      for(i = 0; i< NUM_LEDS; i++) {
        leds1[i] = CRGB::Green;
      }
      COMMAND_NUM = 4;
      FastLED.show();
    }
    if (command == "PURPLE") {
      for(i = 0; i< NUM_LEDS; i++) {
        leds1[i] = CRGB::Purple;
      }
      COMMAND_NUM = 4;
      FastLED.show();
    }
    if (command == "WHITE") {
      for(i = 0; i< NUM_LEDS; i++) {
        leds1[i] = CRGB::White;
      }
      COMMAND_NUM = 4;
      FastLED.show();
    }
    if (command == "SILVER") {
      for(i = 0; i< NUM_LEDS; i++) {
        leds1[i] = CRGB::Silver;
      }
      COMMAND_NUM = 4;
      FastLED.show();
    }
    if (command == "CRIMSON") {
      for(i = 0; i< NUM_LEDS; i++) {
        leds1[i] = CRGB::Crimson;
      }
      COMMAND_NUM = 4;
      FastLED.show();
    }
  }
}

// Create an instance of the Arcanet library
Arcanet arcanet(MY_ID, onCommandReceived);

// Function to generate random numbers within a specific range
int randomInRange(int min, int max) {
  return min + (esp_random() % (max - min + 1));
}

void setup() {
  Serial.begin(9600);
  pinMode(LED_PIN, OUTPUT);
  COMMAND_NUM = 2;
  // Batterij meten
  analogSetPinAttenuation(PIN_BATTERY, ADC_11db); // FS ≈ 3.3 V

  // Initialize the Arcanet network
  arcanet.init();
  relicStatus = true;
  now = millis();
  tLastUpdate = now;

  // Initieren van de 2 LED modules. Met 1 array hebben ze beide dezelfde kleur.
  FastLED.addLeds<LED_TYPE,DATA_PIN_1,COLOR_ORDER>(leds1,NUM_LEDS);
  delay(50);
  Serial.println("Initialising LED2");
  FastLED.addLeds<LED_TYPE,DATA_PIN_2,COLOR_ORDER>(leds1,NUM_LEDS);

  fill_solid(leds1,NUM_LEDS,CRGB::Purple);
  FastLED.show();
  delay(300);
  fill_solid(leds1,NUM_LEDS,CRGB::Yellow);
  FastLED.show();
  delay(300);
  fill_solid(leds1,NUM_LEDS,CRGB::Red);
  FastLED.setBrightness(gBrightness);
  FastLED.show();
  delay(300);
    fill_solid(leds1,NUM_LEDS,CRGB::White);
  FastLED.show();
  delay(300);
  FastLED.clear();
  FastLED.show();
  delay(300);

  randomSeed(esp_random());

  Serial.println("CURTAIN150 started!");
}

void loop() {
  now = millis();
  // Run the Arcanet loop
  arcanet.loop();

  if (now > (tLastUpdate + tUpdatePeriod)) {
      tLastUpdate = now;
      // Serial.println("Break");
      sendUpdate();
  }
  blink();//show a blinking led so we know this beacon is on

  // Example of sending a command from Serial input
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    int separator = input.indexOf('_');
    if (separator > 0) {
      String id = input.substring(0, separator);
      String cmd = input.substring(separator + 1);

      arcanet.sendCommand(id, cmd);

      //Optional
      if (id == MY_ID) {
        Serial.println("try to handle command on this machine");
        onCommandReceived(id, cmd);
        // Serial.println(COMMAND_NUM);
      }
    }
  }
  delay(1);  //yield
  switch (COMMAND_NUM) {
      case 0: // Command OFF
          FastLED.clear();
          FastLED.show();
          break;
      case 1: // Command Fire!
          for (int i = 0; i < NUM_LEDS; i++) {
              switch(randomInRange(1,5)) {
                case 1:
                  fireValue = 1;
                  break;
                case 2:
                  fireValue = 32;
                  break;
                case 3:
                  fireValue = 64;
                  break;
                case 4:
                  fireValue = 16;
                  break;
                case 5:
                  fireValue = 255;
                  break;
                default:
                  fireValue = 255;
                  break;    
              }
              leds1[i] = CHSV(fireValue, randomInRange(230, 255), 255);
          }
          FastLED.show();
          // Wait a random short time to make the flicker look natural
          serviceFor(randomInRange(1,3)*50);
          break;
      case 2: // Heartbeat
          heartBeat();  // Heart beat function
          FastLED.show();
          break;
      case 3: // The Blue Hell
          dV = ((exp(sin(pulseSpeed * millis()/2000.0*PI)) -0.36787944) * delta);
          val = valueMin + dV;
          hue = map(val, valueMin, valueMax, hueA, hueB);  // Map hue based on current val
          sat = map(val, valueMin, valueMax, satA, satB);  // Map sat based on current val
          for (int i = 0; i < NUM_LEDS; i++) {
            leds1[i] = CHSV(hue, sat, val);
          }
          FastLED.show();
          break;
      case 4: // One color
          FastLED.setBrightness(randomInRange(50,255)); // variable
          FastLED.show();
          // Wait a random short time to make the flicker look natural
          serviceFor(randomInRange(1,20)*150);
          break;
      case 9: // Command ON
          for (int i = 0; i < NUM_LEDS; i++) {
            leds1[i] = CHSV(53, 3, 99);
          }
          FastLED.setBrightness(randomInRange(1,245)+10); // variable
          FastLED.show();
          // Wait a random short time to make the flicker look natural
          serviceFor(randomInRange(1,20)*150);
          break;
      default:
            FastLED.clear();
            FastLED.show();
          break;
  }
}

void blink() {
    now = millis();
    if (now > tBlinkPeriod + tLastBlinkOn) {
        tLastBlinkOn  = now;
        tLastBlinkOff = now + tBlinkTime;
        digitalWrite(LED_BUILTIN, LOW);
    }
    if (now > tLastBlinkOff) {
        digitalWrite(LED_BUILTIN, HIGH);
    }
}

void serviceFor(uint32_t ms) {
  uint32_t start = millis();
  while (millis() - start < ms) {
    // now = millis();
    arcanet.loop();  // processes discovery + queue
    blink();
    // sendUpdate();
    // updateControllers();
    delay(1);  // yield
  }
}

int getBatteryLevel() {
    int mv = analogReadMilliVolts(PIN_BATTERY); 
    // Serial.println("mv: "+String(mv));    
    mv = mv<1 ? 1 : mv*2;
    // Battery level = 2* gemeten waarde (spanningsbrug)
    return mv;
}

void sendUpdate() {
  int v_batt = getBatteryLevel();
  if (v_batt < minBatteryLevel) {
    tBlinkPeriod  = 500;
    // Blink faster if the battery level is low
  } else {
    tBlinkPeriod  = 3000;
  }
  arcanet.sendCommand("CONTROLLER", MY_ID+"_"+"BLVL_"+String(v_batt)+"_SGNL_"+String(arcanet.getBestRssi())+"_STATE_"+( relicStatus ? "ON" : "OFF") );
}


//===============================================================
// Heart Beat Functions
//   The base for all this goodness came from Mark Kriegsman and
//   was initially coded up by Chris Thodey.  I updated it to use
//   HSV and provided all the variables to play with up above.
//   -Marc

void heartBeat(){
  for (int i = 0; i < NUM_LEDS ; i++) {
    uint8_t bloodVal = sumPulse( (5/NUM_LEDS/2) + (NUM_LEDS/2) * i * flowDirection );
    leds1[i] = CHSV( bloodHue, bloodSat, bloodVal );
  }
}

int sumPulse(int time_shift) {
  time_shift = 0;  //Uncomment to heart beat/pulse all LEDs together
  int pulse1 = pulseWave8( millis() + time_shift, cycleLength, pulseLength );
  int pulse2 = pulseWave8( millis() + time_shift + pulseOffset, cycleLength, pulseLength );
  return qadd8( pulse1, pulse2 );  // Add pulses together without overflow
}

uint8_t pulseWave8(uint32_t ms, uint16_t cycleLength, uint16_t pulseLength) {
  uint16_t T = ms % cycleLength;
  if ( T > pulseLength) return baseBrightness;
  uint16_t halfPulse = pulseLength / 2;
  if (T <= halfPulse ) {
    return (T * 255) / halfPulse;  //first half = going up
  } else {
    return((pulseLength - T) * 255) / halfPulse;  //second half = going down
  }
}
//End_heart_beat_functions