#include <Arduino.h>
#include "src/Arcanet.h"

// Your device's unique ID
const String MY_ID = "14";

//GPIO of Lantern control pin
const uint8_t DIM_LANTERN        = 23;


// Callback function to handle received commands
void onCommandReceived(const String& id, const String& command) {
  Serial.printf("Command received for ID: %s, Command: %s\n", id.c_str(), command.c_str());

  if (id == MY_ID) {
    if (command == "LANTERN_ON") {
        digitalWrite(DIM_LANTERN, HIGH);
    } else if (command == "LANTERN_OFF") {
        digitalWrite(DIM_LANTERN, LOW);
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
  pinMode(DIM_LANTERN, OUTPUT);
  digitalWrite(DIM_LANTERN, LOW);

  // Initialize the Arcanet network
  arcanet.init();

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
