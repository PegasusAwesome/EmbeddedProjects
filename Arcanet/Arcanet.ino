#include "src/Arcanet.h"

// Your device's unique ID
const String MY_ID = "LANTERN17";


// Callback function to handle received commands
void onCommandReceived(const String& id, const String& command) {
    Serial.printf("Command received for ID: %s, Command: %s\n", id.c_str(), command.c_str());

    if (id == MY_ID) {
        if (command == "ON") {
            //do something cool like: digitalWrite(LED_BUILTIN, LOW);
        } else if (command == "OFF") {
            //stop doing  something cool: digitalWrite(LED_BUILTIN, HIGH);
        }
    }
}

// Create an instance of the Arcanet library
Arcanet arcanet(MY_ID, onCommandReceived);

void setup() {
  Serial.begin(115200);

  // Initialize the Arcanet network
  arcanet.init();

  //Do application setup things

  Serial.println("##################################");
  Serial.println("### Application setup complete ###");
  Serial.println("##################################");
}

void loop() {
  // Run the Arcanet loop
  arcanet.loop();

  readSerial();

  delay(100);

}


//if somewhere in your code you have long delays (>10ms), please use serviceFor in stead of delay
void serviceFor(uint32_t ms) {
    uint32_t start = millis();
    while (millis() - start < ms) {
        readSerial();
        arcanet.loop();            // processes discovery + queue
        delay(1);                  // yield
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



