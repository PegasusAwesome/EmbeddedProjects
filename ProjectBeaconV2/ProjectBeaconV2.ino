#include <Arduino.h>
#include "src/Arcanet.h"

// Your device's unique ID
const String MY_ID = "LANTERN25";

//GPIO of Popwer (N-Fet) pin
const uint8_t PIN_POWER          = 23;

//GPIO of Popwer (GPIO 0 for reading battery lvl) pin
const uint8_t PIN_BATTERY        = 1;

//GPIO of Lantern control pin
const uint8_t PIN_LANTERN        = 22;
 
unsigned long updateScheduledAt  = 0;
uint32_t tUpdatePeriod = 10000;
boolean pendingUpdate = false;
int minBatteryLevel = 2900;
boolean relicStatus = false;

uint32_t tLastBlinkOn  = 0;
uint32_t tLastBlinkOff = 0;
uint32_t tBlinkTime    = 200;
uint32_t tBlinkPeriod  = 3000;

uint32_t tDemoLuxPeriod = 3000;
uint32_t tDemoLux       = 0;
int16_t  hue            = 0;

uint32_t now            = millis();

// Callback function to handle received commands
void onCommandReceived(const String& id, const String& msg) {
    if (id == MY_ID || id == "LANTERNALL") {
        if (msg == "LANTERN_ON") {
            digitalWrite(PIN_LANTERN, HIGH);
            relicStatus = true;
            prepareUpdateNow();

        } else if (msg == "LANTERN_OFF") {
            digitalWrite(PIN_LANTERN, LOW);
            relicStatus = false;
            prepareUpdateNow();

        } else if (msg == "POWER_OFF") {
            digitalWrite(PIN_POWER, LOW);

        } else if (msg == "SEND_UPDATE") {
            prepareUpdateNow();

        }
    } else if (id == "ALL") {
        if (msg == "SEND_UPDATE") {
            prepareUpdateNow();

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

    //Open Fet to power
    pinMode(PIN_POWER, OUTPUT);
    digitalWrite(PIN_POWER, HIGH);

    // Initialize the Arcanet network
    arcanet.init();

    analogSetPinAttenuation(PIN_BATTERY, ADC_11db); // FS ≈ 3.3 V

    randomSeed(esp_random());

    Serial.println("####################################");
    Serial.println("### ProjectBeacon setup complete ###");
    Serial.println("####################################");
    Serial.println("My ID is: "+String(MY_ID));

    //Show lantern is working
    digitalWrite(PIN_LANTERN, HIGH);
    delay(400);
    digitalWrite(PIN_LANTERN, LOW);
    delay(400);
    digitalWrite(PIN_LANTERN, HIGH);
    delay(400);
    digitalWrite(PIN_LANTERN, LOW);
    delay(400);
    digitalWrite(PIN_LANTERN, HIGH);
    delay(400);
    digitalWrite(PIN_LANTERN, LOW);

    pendingUpdate = true;
    updateScheduledAt = millis() + 10000;

}


void loop() {
    now = millis();

    arcanet.loop();//housekeeping our presence in Arcanet
readSerial();//any commands from outside (TODO: put this behind a compile time switch)
    blink();//show a blinking led so we know this beacon is on
    sendUpdate();//send update if requested
    updateControllers();//prepare the regular update
    serviceFor(10);
}
//    demoLux();
//    GolemEntry();

void serviceFor(uint32_t ms) {
    uint32_t start = millis();
    while (millis() - start < ms) {
        now = millis();
        arcanet.loop();            // processes discovery + queue
        blink();
        sendUpdate();
        updateControllers();
        delay(1);                  // yield
    }
}


void demoLux() {
    // if ( now > tDemoLux + tDemoLuxPeriod) {
    //     arcanet.sendCommand("LUX2", "SET_HUE_"+String(hue));
    //     arcanet.sendCommand("LUX3", "SET_HUE_"+String( (hue+120)%360) );
    //     arcanet.sendCommand("LUX4", "SET_HUE_"+String( (hue+240)%360) );
    //     tDemoLux = now;
    //     hue = hue>=360 ? 0 : hue+1;
    // }
}


void updateControllers() {
    if ( now > updateScheduledAt + tUpdatePeriod) {
        prepareUpdate();
        checkBatteryLevel();
    }
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

void checkBatteryLevel() {
    if (getBatteryLevel()<minBatteryLevel) {
        delay(150);
        if (getBatteryLevel()<minBatteryLevel) {
            delay(250);
            if (getBatteryLevel()<minBatteryLevel) {
                pinMode(PIN_POWER, OUTPUT);
                digitalWrite(PIN_POWER, LOW);
            }
        }
    }
}

int getBatteryLevel() {
    int mv = analogReadMilliVolts(PIN_BATTERY); 
Serial.println("mv: "+String(mv));    
    mv = mv<1 ? 1 : mv*2;
    return mv;
}

void prepareUpdate() {
    pendingUpdate = true;
    updateScheduledAt = millis() + random(0, 2000);
}
void prepareUpdateNow() {
    pendingUpdate = true;
    updateScheduledAt = millis() + 10 + random(0, 10);
}

void sendUpdate() {
    if (pendingUpdate && millis() >= updateScheduledAt) {
        pendingUpdate = false;
        int v_batt = getBatteryLevel();
        arcanet.sendCommand("CONTROLLER", MY_ID+"_"+"BLVL_"+String(v_batt)+"_SGNL_"+String(arcanet.getBestRssi())+"_STATE_"+( relicStatus ? "ON" : "OFF") );
    }
}

