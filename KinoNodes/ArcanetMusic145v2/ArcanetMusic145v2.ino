#include <Arduino.h>
#include "src\Arcanet.h"
#include <Wire.h>
// #include <HardwareSerial.h>  // UART

// Arduino IDE - device: XIAO_ESP32C6
// ESP32C6 for playing music remotely

// Your device's unique ID
const String MY_ID = "MUSIC145";  // CHANGE THIS
const String keepalive = "";

// LED Pin
const int LED_PIN = LED_BUILTIN;  //5

// Input Switches Pin -> Output naar MP3 speler
const int SWITCH1_PIN = D0;
const int SWITCH2_PIN = D1;
int buttonState1 = 0;  // Current state of the button - niet meer nodig
int buttonState2 = 0;  // Current state of the button - niet meer nodig
const int RELAIS_PIN = D2;

//GPIO of Popwer (N-Fet) pin
// const uint8_t PIN_POWER          = 1;

//GPIO of Popwer (GPIO 0 for reading battery lvl) pin
// const uint8_t PIN_BATTERY        = 0;

//GPIO of Lantern control pin
// const uint8_t PIN_LANTERN        = 23;

// Define TX and RX pins for UART (change if needed)
// const TXD1 = D6
// const RXD1 = D7
// Use Serial1 for UART communication
HardwareSerial mySerial(1);
int counter = 0;
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
    Serial.printf("Command for me ;)");
    if (command == "PLAYGOLEMIN") {
      playgolemin();
    } else if (command == "PLAYGOLEMOUT") {
      playgolemuit();
    }
  }
}

// Create an instance of the Arcanet library
Arcanet arcanet(MY_ID, onCommandReceived);

void setup() {
  delay(500);
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);

  // Initialize the Arcanet network
  arcanet.init();

  pinMode(SWITCH1_PIN, OUTPUT);     // Initialize button pin as output
  pinMode(SWITCH2_PIN, OUTPUT);     // Initialize button pin as output
  pinMode(RELAIS_PIN, OUTPUT);      // Initialize button pin as output
  digitalWrite(SWITCH1_PIN, HIGH);  // MP3 speler speel NIET Track1
  digitalWrite(SWITCH2_PIN, HIGH);  // MP3 speler speel NIET Track2
  digitalWrite(RELAIS_PIN, LOW);    // relais amplifier uit

  //  mySerial.begin(9600, SERIAL_8N1, D7, D6);  // UART setup
  //  Serial.println("ESP32 UART Transmitter");

  relicStatus = true;
  now = millis();
  tLastUpdate = now;

  digitalWrite(LED_BUILTIN, LOW);
  delay(500);
  digitalWrite(LED_BUILTIN, HIGH);

  Serial.println("MUSIC145 started.");
}

void loop() {
  // Run the Arcanet loop
  arcanet.loop();
  now = millis();

  if (now > (tLastUpdate + tUpdatePeriod)) {
      tLastUpdate = now;
      sendUpdate();
  }
  blink();//show a blinking led so we know this beacon is on

  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    int separator = input.indexOf('_');
    if (separator > 0) {
      String id = input.substring(0, separator);
      String cmd = input.substring(separator + 1);
      arcanet.sendCommand(id, cmd);
    }
  }
  delay(1);
}

void playgolemin() {
  arcanet.sendCommand("BOOK144", "Play_Track1_on_MUSIC145");
  // Send message over UART
  // mySerial.println("PLAYGOLEMIN");
  Serial.println("Sent: Play Golem In track");
  digitalWrite(RELAIS_PIN, HIGH);
  serviceFor(100);  // wait for amplifier
  digitalWrite(SWITCH1_PIN, LOW);
  serviceFor(50);  // Kort en stabile naar Low voor starten Track 1
  digitalWrite(SWITCH1_PIN, HIGH);
  serviceFor(35000);  // 35 seconden
  digitalWrite(RELAIS_PIN, LOW);
}

void playgolemuit() {
  arcanet.sendCommand("BOOK144", "Play_Track2_on_MUSIC145");
  // mySerial.println("PLAYGOLEMOUT");
  Serial.println("Sent: Play Golem Out track");
  digitalWrite(RELAIS_PIN, HIGH);
  serviceFor(100);  // wait for amplifier
  digitalWrite(SWITCH2_PIN, LOW);
  serviceFor(50);  // Kort en stabile naar Low voor starten Track 2
  digitalWrite(SWITCH2_PIN, HIGH);
  serviceFor(35000);  // 35 seconden
  digitalWrite(RELAIS_PIN, LOW);
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

void sendUpdate() {
  int v_batt = 4000;  // No Battery check - USB powerbank
  arcanet.sendCommand("CONTROLLER", MY_ID+"_"+"BLVL_"+String(v_batt)+"_SGNL_"+String(arcanet.getBestRssi())+"_STATE_"+( relicStatus ? "ON" : "OFF") );
}