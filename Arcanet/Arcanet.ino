#include "src/Arcanet.h"

// Your device's unique ID
const String TYPE      = "ARCANET";
const String SERIAL_ID = "18";
const String MY_ID     = TYPE+SERIAL_ID;

const String CONTROLLER_ID = "CONTROLLER";

void sendPeerReport(const String& targetId);


// Callback function to handle received commands
void onCommandReceived(const String& id, const String& command) {
    Serial.printf("Command received for ID: %s, Command: %s\n", id.c_str(), command.c_str());

    if (id == MY_ID) {
        if (command == "REQUESTPEERS") {
            sendPeerReport(CONTROLLER_ID);
        } else if (command == "ON") {
            //do something cool like: digitalWrite(LED_BUILTIN, LOW);
        } else if (command == "OFF") {
            //stop doing  something cool: digitalWrite(LED_BUILTIN, HIGH);
        }
    }
}

// Create an instance of the Arcanet library
Arcanet arcanet(MY_ID, onCommandReceived);

void setup() {
    //Start serial, mainly for console/debugging
    Serial.begin(115200);

    // Initialize the Arcanet network
    arcanet.init();

    //Do application initializition things

    //Tell console we've started
    Serial.println(MY_ID);
    Serial.println("##################################");
    Serial.println("### Application setup complete ###");
    Serial.println("##################################");
}

void loop() {
  // Run the Arcanet loop
  serviceFor(10);

}


//if somewhere in your code you have long delays (>10ms), please use serviceFor instead of delay
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

void sendPeerReport(const String& targetId) {
    Arcanet::PeerInfo peers[5];
    uint8_t peerCount = arcanet.getTopPeersByRssi(peers, 5);

    if (peerCount == 0) {
        arcanet.sendCommand(targetId, "SENDPEERS:0");
        return;
    }

    for (uint8_t i = 0; i < peerCount; ++i) {
        String report = "SENDPEERS:";
        report += String(i + 1);
        report += "/";
        report += String(peerCount);
        report += ":";
        report += peers[i].id;
        report += "=";
        report += String(peers[i].rssi);

        arcanet.sendCommand(targetId, report);
    }
}

