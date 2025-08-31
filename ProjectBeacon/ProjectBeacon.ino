#include <Arduino.h>
#include "src/Arcanet.h"

// Your device's unique ID
const String MY_ID = "LANTERN12";

//GPIO of Popwer (N-Fet) pin
const uint8_t PIN_POWER          = 1;

//GPIO of Popwer (GPIO 0 for reading battery lvl) pin
const uint8_t PIN_BATTERY        = 0;

//GPIO of Lantern control pin
const uint8_t PIN_LANTERN        = 23;



// Callback function to handle received commands
void onCommandReceived(const String& id, const String& command) {
  Serial.printf("Command received for ID: %s, Command: %s\n", id.c_str(), command.c_str());

    if (id == MY_ID) {
        if (command == "LANTERN_ON") {
            digitalWrite(PIN_LANTERN, HIGH);

        } else if (command == "LANTERN_OFF") {
            digitalWrite(PIN_LANTERN, LOW);

        } else if (command == "POWER_OFF") {
            digitalWrite(PIN_POWER, LOW);

        } else if (command == "BATTERYLEVEL_SHOW") {
            sendBatteryLevelUpdate();

        }
    }
}

// Create an instance of the Arcanet library
Arcanet arcanet(MY_ID, onCommandReceived);

void setup() {
  Serial.begin(115200);
  //Optional code to create blinking led in loop  
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  //Start with latnern off, turn down gpio port (set to 0v)
  pinMode(PIN_LANTERN, OUTPUT);
  digitalWrite(PIN_LANTERN, LOW);

  pinMode(PIN_POWER, OUTPUT);
  digitalWrite(PIN_POWER, HIGH);

  // Initialize the Arcanet network
  arcanet.init();

  analogSetPinAttenuation(PIN_BATTERY, ADC_11db); // FS ≈ 3.3 V

  Serial.println("Application setup complete.");
}

void loop() {
  // Run the Arcanet loop
  arcanet.loop();

  // Example of sending a command from Serial input
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

  //Simple blink, so you know its working
   delay(1750);
   digitalWrite(LED_BUILTIN, LOW);
   delay(500);
   digitalWrite(LED_BUILTIN, HIGH);

}

void sendBatteryLevelUpdate() {
    int mv = analogReadMilliVolts(PIN_BATTERY); 
    float v_adc  = mv / 1000.0;                // 
    float v_batt = v_adc / 0.5;                // divider ratio 1:2
    arcanet.sendCommand("ALL", "BATTERYLEVEL_"+MY_ID+"_"+String(v_batt));
}



