#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include "esp_bt.h"
#include "esp_wifi.h"

//https://protonestiot.medium.com/automating-node-integration-in-esp-now-mesh-networks-with-esp32-73bc9c0baa3f


//IMPORTANT
//Always set "myID" to an unique value from your assigned range

//Mesh ID 0-69 : Marcel
//Mesh 70-139  : Renout 
//Mesh 140-209 : Kino 
//Mesh 210-279 : Bas 

String myID = "4";  // Change this value


#define ID_MAX  32
#define CMD_MAX 32
typedef struct struct_message {
  char type;
  char id[ID_MAX];        // room for multi-alphanumeric IDs like "12" or "kristal_4" or "kristal_vier"
  char command[CMD_MAX];  // room for "LANTERN_ON", etc.
  uint8_t mac[6];
  int msgID;
  int hopCount;
} struct_message;

struct_message myData;
int lastReceivedMsgID = -1;
int msgCount = 0;

#define MAX_PEERS 30  
uint8_t knownPeers[MAX_PEERS][6];  
int peerCount = 0;  

unsigned long lastBroadcastTime = 0;
const unsigned long broadcastInterval = 15000; 
const int MAX_HOPS = MAX_PEERS-1;

//Settings for rgbw driver
const int CHANNEL   = 0;
const int FREQ_HZ   = 5000;  // 5 kHz
const int RES_BITS  = 12;     // 0..4095


//RGBW PWM Pins
const uint8_t DIM_PIN_WHITE    = 2;   // PT4115 DIM WHITE
const uint8_t DIM_PIN_BLUE     = 3;   // PT4115 DIM BLUE
const uint8_t DIM_PIN_GREEN    = 4;   // PT4115 DIM GREEN
const uint8_t DIM_PIN_RED      = 5;   // PT4115 DIM RED


const uint8_t DIM_LANTERN      = 23;   // 


void loop() {
    //This code is to send commands from the CLI
    if (Serial.available() > 0) {

        String line = Serial.readStringUntil('\n');  // read one line
        line.trim();                                 // strip CR/LF/space
        if (line.length() == 0) return;

        int sep = line.indexOf('_');                 // first underscore
        if (sep <= 0 || sep >= line.length()-1) {
            Serial.println("Bad format. Use <id>_<command> (e.g. 12_LANTERN_ON)");
            return;
        }

        String idStr  = line.substring(0, sep);      //ex. "12"
        String cmdStr = line.substring(sep + 1);     //ex. "LANTERN_ON"

        myData.type = 'C';
        idStr.toCharArray(myData.id, sizeof(myData.id));           // NUL-terminated
        cmdStr.toCharArray(myData.command, sizeof(myData.command));
        myData.msgID = msgCount++;
        myData.hopCount = 0;

        Serial.printf("Command from Serial: id='%s' cmd='%s'\n", myData.id, myData.command);

        // Optional: execute locally too if addressed to me
        if (idStr == myID) {
            Serial.println("handle command");
            handleCommand(cmdStr);
        }

        sendCommand(myData);
    }

    unsigned long currentTime = millis();
    
    if (currentTime - lastBroadcastTime >= broadcastInterval) {
        startDiscoveryBroadcast();
        lastBroadcastTime = currentTime;
    }
    delay(1750);
    digitalWrite(LED_BUILTIN, LOW);
    delay(400);
    digitalWrite(LED_BUILTIN, HIGH);

/*
  turnLedOn(DIM_PIN_WHITE, 40);
  delay(1500);
  turnLedOff(DIM_PIN_WHITE, 40);

  turnLedOn(DIM_PIN_BLUE, 50);
  delay(1500);
  turnLedOff(DIM_PIN_BLUE, 50);
  
  turnLedOn(DIM_PIN_GREEN, 50);
  delay(1500);
  turnLedOff(DIM_PIN_GREEN, 50);

  turnLedOn(DIM_PIN_RED, 40);
  delay(1500);
  turnLedOff(DIM_PIN_RED, 40);

  turnLedOn(DIM_PIN_GREEN, 40);
  turnLedOn(DIM_PIN_RED, 50);
  delay(1500);
  turnLedOff(DIM_PIN_RED, 40);
  turnLedOFF(DIM_PIN_GREEN, 50);
*/


}



void handleCommand(const String& command) {
    Serial.println("command        : " + command);

    if (command == "LANTERN_ON") {
        digitalWrite(DIM_LANTERN, HIGH);

    } else if (command == "LANTERN_OFF") {
        digitalWrite(DIM_LANTERN, LOW);

    } else if (command == "WHITE_ON") {
        turnLedOn(DIM_PIN_WHITE, 40);
    
    } else if (command == "WHITE_OFF") {
        turnLedOff(DIM_PIN_WHITE, 40);

    } else if (command == "RED_ON") {
        turnLedOn(DIM_PIN_RED, 40);
    
    } else if (command == "RED_OFF") {
        turnLedOff(DIM_PIN_RED, 40);

    } else if (command == "GREEN_ON") {
        turnLedOn(DIM_PIN_GREEN, 40);
    
    } else if (command == "GREEN_OFF") {
        turnLedOff(DIM_PIN_GREEN, 40);

    } else if (command == "BLUE_ON") {
        turnLedOn(DIM_PIN_BLUE, 40);
    
    } else if (command == "BLUE_OFF") {
        turnLedOff(DIM_PIN_BLUE, 40);

    } else if (command == "ON") {
        digitalWrite(LED_BUILTIN, LOW);

    } else if (command == "OFF") {
        digitalWrite(LED_BUILTIN, HIGH);
    }
}


void setup() {
    //connect serial for debugging
    Serial.begin(115200);

    //turn on board led, by turning down LED gpio (set to 0v, led is wired active-low)
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

    //turn down gpio port (set to 0v)
    pinMode(DIM_LANTERN, OUTPUT);
    digitalWrite(DIM_LANTERN, LOW);

    //init pwm dim pins
    initDimPin(DIM_PIN_RED);
    initDimPin(DIM_PIN_GREEN);
    initDimPin(DIM_PIN_BLUE);
    initDimPin(DIM_PIN_WHITE);

    Serial.println("Pin Init Success");

    //init knownPeers
    memset(knownPeers, 0, sizeof(knownPeers));

    WiFi.mode(WIFI_STA);  
    delay(100);

    //print mac
    uint8_t mac[6];
    WiFi.macAddress(mac);               // STA MAC
    Serial.printf("STA MAC: %02X:%02X:%02X:%02X:%02X:%02X\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    //start wifi for espnow
    esp_wifi_set_ps(WIFI_PS_MIN_MODEM);
    WiFi.disconnect();  
    delay(100);

    if (esp_now_init() == ESP_OK) {
        Serial.println("ESP-NOW Init Success");
        esp_now_register_recv_cb(OnDataRecv);
        esp_now_register_send_cb(OnDataSent);
    } else {
        Serial.println("ESP-NOW Init Failed");
        ESP.restart();
        return;
    }

    //some housekeeping
    esp_bt_controller_mem_release(ESP_BT_MODE_BTDM);//disable bluetooth
//    disableMic();//disable microphone
    delay(3000);

    //turn off board led, by turning up LED gpio (set to 3.3v, led is wired active-low, so not active at 3.3v)
    digitalWrite(LED_BUILTIN, HIGH);
}


void turnLedOn(uint8_t DIM_PIN, uint8_t TO_BRIGHTNESS) {
    for (int i=0;i<=TO_BRIGHTNESS;i++) { 
        setBrightness(i/100.0f, DIM_PIN); 
        delay(10);
    }
}

void turnLedOff(uint8_t DIM_PIN, uint8_t FROM_BRIGHTNESS) {
    for (int i=FROM_BRIGHTNESS;i>=0;i--) { 
        setBrightness(i/100.0f, DIM_PIN); 
        delay(10);
    }
}



void initDimPin(uint8_t DIM_PIN) {
    pinMode(DIM_PIN, OUTPUT);
    digitalWrite(DIM_PIN, LOW);          // hold OFF first (requires pull-down too)

    bool ok = ledcAttach(DIM_PIN, FREQ_HZ, RES_BITS);
    if (!ok) { 
        while (true) { 
            Serial.println("RGBW attach error"+DIM_PIN);
            delay(1000); 
        } 
    }// simple fail-safe

    // start off
    ledcWrite(DIM_PIN, 0);
}


// set brightness 0..1
void setBrightness(float b, uint8_t DIM_PIN) {
    b = constrain(b, 0.0f, 1.0f);
    ledcWrite(DIM_PIN, (int)(b * ((1<<RES_BITS)-1) ) );
}


void setPwmFreq(uint32_t hz, uint8_t bits, uint8_t DIM_PIN) {
    // Change frequency/resolution later if needed
    ledcChangeFrequency(DIM_PIN, hz, bits);
}


void sendCommand(struct_message commandData) {
    Serial.println("Command data is: " + String(commandData.id));
    Serial.println("My ID: " + String(myID));
    if (String(commandData.id) != myID) {
        for (int i = 0; i < peerCount; i++) {
            esp_err_t result = esp_now_send(knownPeers[i], (uint8_t *)&commandData, sizeof(commandData));
            char formattedMac[20];  // Store the formatted MAC address
            formatMacAddress(knownPeers[i], formattedMac, sizeof(formattedMac));
            if (result == ESP_OK) {
                Serial.println("Successfully sent command " + String(commandData.command) + " to MAC: " + String(formattedMac));
            } else {
                Serial.println("ESP-NOW send failed to MAC: " + String(formattedMac));
            }
        }
    } else {
        Serial.println("Its my ID");
    }
}


void startDiscoveryBroadcast() {
    uint8_t mac[6];
    WiFi.macAddress(mac);  
    
    myData.type = 'D';
    memcpy(myData.mac, mac, 6);  

    broadcast(myData);
}


bool isKnownPeer(const uint8_t* mac) {
    for (int i = 0; i < peerCount; i++) {
        if (memcmp(knownPeers[i], mac, 6) == 0) {
            return true;
        }
    }
    return false;
}

bool isBroadcastMac(const uint8_t* mac) {
    for (int i = 0; i < 6; i++) {
        if (mac[i] != 0xFF) {
            return false;
        }
    }
    return true;
}


void addPeer(const uint8_t* mac) {
    if (peerCount < MAX_PEERS) {
        memcpy(knownPeers[peerCount], mac, 6);

        esp_now_peer_info_t peerInfo;
        memcpy(peerInfo.peer_addr, mac, 6);
        peerInfo.channel = 0;
        peerInfo.encrypt = false;
        peerInfo.ifidx = WIFI_IF_STA;

        esp_err_t result = esp_now_add_peer(&peerInfo);
        if (result == ESP_OK) {
            char formattedMac[20];  
            formatMacAddress(mac, formattedMac, sizeof(formattedMac));
            Serial.println("Peer added successfully with MAC: " + String(formattedMac));
            peerCount++;
        } else {
            Serial.println("Failed to add peer.");
        }
    } else {
        Serial.println("Max peer count reached. Can't add new peer.");
    }
}

void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
    // Source MAC is inside the info structure
    const uint8_t *mac = info->src_addr;

    if(len != sizeof(struct_message)) {
        Serial.println("Received data of unexpected size.");
        return;
    }
  
    struct_message receivedData;
    memcpy(&receivedData, incomingData, sizeof(receivedData));
    Serial.print("Received message type: " + String(receivedData.type) + "; ");

    if (receivedData.type == 'D') { 
        char formattedMac[20];  
        formatMacAddress(receivedData.mac, formattedMac, sizeof(formattedMac));

        if (isBroadcastMac(receivedData.mac)) {
            Serial.println("Received MAC is a broadcast MAC. Not adding as peer.");
        } else {
        }
    
        Serial.println("Received Discovery message from MAC: " + String(formattedMac));
        if (!isKnownPeer(receivedData.mac)) {
            addPeer(mac);
        }

    } else if (receivedData.type == 'C') {  
        Serial.print("id: "+String(receivedData.id) + "; ");
        Serial.println("command: " + String(receivedData.command) + "; ");
        String com = String(receivedData.command);

        if (receivedData.msgID == lastReceivedMsgID) {
            Serial.println("same Data");
            return;
        }
        lastReceivedMsgID = receivedData.msgID;

        if (myID.equals(receivedData.id)) {
            Serial.println("ids match, looking up command");
            String command = String(receivedData.command);
            handleCommand(command);

            // if (command == "LANTERN_ON") {
            //     Serial.println("LANTERN_ON");
            //     digitalWrite(DIM_LANTERN, HIGH);

            // } else if (com == "LANTERN_OFF") {
            //     Serial.println("LANTERN_OFF");
            //     digitalWrite(DIM_LANTERN, LOW);

            // } else if (com == "WHITE_ON") {
            //     Serial.println("WHITE_ON");
            //     for (int i=0;i<=40;i++){ setBrightness(i/100.0f, DIM_PIN_WHITE); delay(10);}

            // } else if (com == "WHITE_OFF") {
            //     Serial.println("WHITE_OFF");
            //     for (int i=40;i>=0;i--){ setBrightness(i/100.0f, DIM_PIN_WHITE); delay(10);}

            // } else if (com == "ON") {   
            //     Serial.println("LED_ON");
            //     digitalWrite(LED_BUILTIN, LOW);

            // } else if (com == "OFF") {
            //     Serial.println("LED_OFF");
            //     digitalWrite(LED_BUILTIN, HIGH);
            // }

        }

        if (receivedData.hopCount < MAX_HOPS) {
            Serial.println("Resend");
            receivedData.hopCount++;
            sendCommand(receivedData);
        } else {
            Serial.println("Dropping message due to excessive hops.");
        }

    } else {
        Serial.println("Received unknown message type.");
    }
}

void formatMacAddress(const uint8_t *macAddr, char *buffer, int maxLength) {
    snprintf(buffer, maxLength, "%02x:%02x:%02x:%02x:%02x:%02x", macAddr[0], macAddr[1], macAddr[2], macAddr[3], macAddr[4], macAddr[5]);
}


void broadcast(const struct_message &message) {
    uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    esp_now_peer_info_t peerInfo = {};
    memcpy(&peerInfo.peer_addr, broadcastAddress, 6);
    if (!esp_now_is_peer_exist(broadcastAddress)) {
        esp_now_add_peer(&peerInfo);
    }

    esp_err_t result = esp_now_send(broadcastAddress, (const uint8_t *)&message, sizeof(message));

    if (result == ESP_OK) {
        Serial.println("Broadcast message success");
    } else if (result == ESP_ERR_ESPNOW_NOT_INIT) {
        Serial.println("ESP-NOW not Init.");
    } else if (result == ESP_ERR_ESPNOW_ARG) {
        Serial.println("Invalid Argument");
    } else if (result == ESP_ERR_ESPNOW_INTERNAL) {
        Serial.println("Internal Error");
    } else if (result == ESP_ERR_ESPNOW_NO_MEM) {
        Serial.println("ESP_ERR_ESPNOW_NO_MEM");
    } else if (result == ESP_ERR_ESPNOW_NOT_FOUND) {
        Serial.println("Peer not found.");
    } else {
        Serial.println("Unknown error");
    }
}

void OnDataSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {
    if (status == ESP_NOW_SEND_SUCCESS) {
//        Serial.println("Send success");
    } else {
        Serial.println("Send failed");
    }
}

void disableMic() {
    // const gpio_num_t PDM_CLK  = GPIO_NUM_42;   // Sense board default
    // const gpio_num_t PDM_DATA = GPIO_NUM_41;

    // pinMode(PDM_CLK,  INPUT);      // high-Z, no clock
    // pinMode(PDM_DATA, INPUT);

    // // keep CLK pulled low so the mic cannot see spurious edges
    // gpio_pulldown_en(PDM_CLK);
}

