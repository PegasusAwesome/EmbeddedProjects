#include <Arduino.h>
#include "Arcanet.h"

// Your device's unique ID
const String MY_ID = "2"; // CHANGE THIS

// LED Pin
const int LED_PIN = 2;

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

  // Initialize the Arcanet network
  arcanet.init();

  Serial.println("Simple example started.");
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

      arcanet.sendCommand(id, cmd);

      //Optional
      if (id == MY_ID) {
        Serial.println("try to handle command on this machine");
        onCommandReceived(id, cmd);
      }

    }
  }
}
