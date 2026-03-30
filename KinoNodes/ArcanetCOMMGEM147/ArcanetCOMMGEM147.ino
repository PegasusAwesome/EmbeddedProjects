#include <Arduino.h>
#include "src\Arcanet.h"
#include <Wire.h>
// Arduino IDE - device: XIAO_ESP32S3
// Your device's unique ID
const String MY_ID = "COMMGEM147";  // CHANGE THIS to the correct ID

// LED Pin
const int LED_PIN = LED_BUILTIN;  //5

// Input Switches Pin
const int SWITCH1_PIN = D1;
const int SWITCH2_PIN = D2;
int buttonState1 = 0;  // Current state of the button
int buttonState2 = 0;  // Current state of the button
const int SFXOUT_PIN = D3;
const int LEDON_PIN = D4;
const int LEDFIRE_PIN = D5;

  //GPIO of Power (GPIO 0 for reading battery lvl) pin
  // First pin is analog in
const int PIN_BATTERY = A0;
int minBatteryLevel = 2900;
unsigned long now = millis();
boolean relicStatus = false;
unsigned long tLastUpdate = 0;
unsigned long tUpdatePeriod = 60000;
unsigned long tLastButton = 0;
unsigned long tButtonPeriod = 2000;  // Stuur Button berichten maar 1x per 2 seconden

uint32_t tLastBlinkOn = 0;
uint32_t tLastBlinkOff = 0;
uint32_t tBlinkTime = 200;
uint32_t tBlinkPeriod = 3000;

// Callback function to handle received commands
void onCommandReceived(const String& id, const String& command) {
  Serial.printf("Command received for ID: %s, Command: %s\n", id.c_str(), command.c_str());
  if (id == MY_ID) {
    Serial.printf("Command for me ;)");
  }
}

// Create an instance of the Arcanet library
Arcanet arcanet(MY_ID, onCommandReceived);

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);

  // Initialize the Arcanet network
  arcanet.init();

  pinMode(SWITCH1_PIN, INPUT_PULLUP);  // Initialize button pin as input with pull-up resistor
  pinMode(SWITCH2_PIN, INPUT_PULLUP);  // Initialize button pin as input with pull-up resistor

  // Batterij meten
  analogSetPinAttenuation(PIN_BATTERY, ADC_11db);  // FS ≈ 3.3 V

  // SFX out pin naar mosfet
  pinMode(SFXOUT_PIN, OUTPUT);
  digitalWrite(SFXOUT_PIN, LOW);

  pinMode(LEDON_PIN, OUTPUT);
  digitalWrite(LEDON_PIN, LOW);

  pinMode(LEDFIRE_PIN, OUTPUT);
  digitalWrite(LEDFIRE_PIN, LOW);

  relicStatus = true;
  now = millis();
  tLastUpdate = now;
  tLastButton = now;

  Serial.println("COMMGEM147 started. Communicatie Kristal voor spelers!");
  // delay(1000);
  digitalWrite(LED_BUILTIN, LOW);
  digitalWrite(LEDON_PIN, HIGH);
  digitalWrite(LEDFIRE_PIN, HIGH);
  delay(1000);
  digitalWrite(LED_BUILTIN, HIGH);
  // digitalWrite(LEDON_PIN, LOW);
  digitalWrite(LEDFIRE_PIN, LOW);
}

void loop() {
  // Run the Arcanet loop
  arcanet.loop();
  now = millis();

  if (now > (tLastUpdate + tUpdatePeriod)) {
    tLastUpdate = now;
    sendUpdate();
  }

  blink();  //show a blinking led so we know this beacon is on

  // Example of sending a command from Serial input
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    int separator = input.indexOf('_');
    if (separator > 0) {
      String id = input.substring(0, separator);
      String cmd = input.substring(separator + 1);

      arcanet.sendCommand(id, cmd);
    }
    delay(1);
  }

  // Read the state of the button
  buttonState1 = digitalRead(SWITCH1_PIN);

  // Check if the button is pressed (LOW because of the pull-up resistor)
  if (buttonState1 == LOW) {
    buttonState2 = digitalRead(SWITCH2_PIN);
    Serial.println("Button is Pressed!");
    if (arcanet.getBestRssi() != -127) {
      Serial.println("Arcanet found a peer!");
      if (buttonState2 == LOW) {
        Serial.println("Button is Pressed & SFX aangesloten!");
        if (now > (tLastButton + tButtonPeriod)) {
          tLastButton = now;
          arcanet.sendCommand("CONSOLE71", "PRESSEDCOMMGEM");
          Serial.println("Button is Pressed and command is sent: CONSOLE71_PRESSEDCOMMGEM");
          serviceFor(50);  // short delay message
          digitalWrite(SFXOUT_PIN, HIGH);
          digitalWrite(LEDFIRE_PIN, HIGH);
          serviceFor(2000);  // 2 second Fire!
          digitalWrite(SFXOUT_PIN, LOW);
          digitalWrite(LEDFIRE_PIN, LOW);
        }
      }
    }
  } else {
    // No need to print every time the button is not pressed
  }
  serviceFor(50);  // Short delay for debouncing and stability
}

void blink() {
  if (now > tBlinkPeriod + tLastBlinkOn) {
    tLastBlinkOn = now;
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
  mv = mv < 1 ? 1 : mv * 2;
  // Battery level = 2* gemeten waarde (spanningsbrug)
  return mv;
}

void sendUpdate() {
  int v_batt = getBatteryLevel();
  if (v_batt < minBatteryLevel) {
    tBlinkPeriod = 500;
  } else {
    tBlinkPeriod = 3000;
  }
  arcanet.sendCommand("CONTROLLER", MY_ID + "_" + "BLVL_" + String(v_batt) + "_SGNL_" + String(arcanet.getBestRssi()) + "_STATE_" + (relicStatus ? "ON" : "OFF"));
}