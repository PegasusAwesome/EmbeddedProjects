#include <Arduino.h>
#include "src\Arcanet.h"

// Your device's unique ID
const String MY_ID = "REPEATER146"; // CHANGE THIS
// 146 XIAO_ESP-32C6 in Arduino IDE

// LED Pin
const int LED_PIN = LED_BUILTIN;

//GPIO of Power (GPIO 0 for reading battery lvl) pin
// First pin is analog in
const uint8_t PIN_BATTERY = 0;
int minBatteryLevel = 2900;
unsigned long now = millis();
boolean relicStatus = false;
unsigned long tLastUpdate = 0;
unsigned long tUpdatePeriod = 60000;

uint32_t tLastBlinkOn  = 0;
uint32_t tLastBlinkOff = 0;
uint32_t tBlinkTime    = 200;
uint32_t tBlinkPeriod  = 3000;

// Callback function to handle received commands
void onCommandReceived(const String& id, const String& command) {
  Serial.printf("Command received for ID: %s, Command: %s\n", id.c_str(), command.c_str());

  if (id == MY_ID) {
    if (command == "ON") {
      digitalWrite(LED_PIN, HIGH);
    } else if (command == "OFF") {
      digitalWrite(LED_PIN, LOW);
    }
  }
}

// Create an instance of the Arcanet library
Arcanet arcanet(MY_ID, onCommandReceived);

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);

  // Batterij meten
  analogSetPinAttenuation(PIN_BATTERY, ADC_11db); // FS ≈ 3.3 V

  // Initialize the Arcanet network
  arcanet.init();
  relicStatus = true;
  now = millis();
  tLastUpdate = now;

  Serial.println("REPEATER146 started!");
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
      }

    }
  }
  delay(1);  //yield
}

void blink() {
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
    // blink();
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
  } else {
    tBlinkPeriod  = 3000;
  }
  arcanet.sendCommand("CONTROLLER", MY_ID+"_"+"BLVL_"+String(v_batt)+"_SGNL_"+String(arcanet.getBestRssi())+"_STATE_"+( relicStatus ? "ON" : "OFF") );
}

