#include <Arduino.h>
#include "src/Arcanet.h"

// Your device's unique ID
const String MY_ID = "DOWSER31";

#define MOTOR_PIN     1
 
unsigned long updateScheduledAt  = 0;
uint32_t tUpdatePeriod = 10000;
boolean pendingUpdate = false;

uint32_t tLastBlinkOn  = 0;
uint32_t tLastBlinkOff = 0;
uint32_t tBlinkTime    = 200;
uint32_t tBlinkPeriod  = 3000;

float filteredRssi = -100.0;
unsigned long lastPulse = 0;
bool motorState = false;
unsigned long lastSignal = 0;

uint32_t now            = millis();

//more sinusy
//use onCommandReceived?

// Callback function to handle received commands
void onCommandReceived(const String& id, const String& msg) {

    if ( (id == MY_ID || id == "DOWSERALL" || id == "ALL") && msg == "SEND_UPDATE") {
        prepareUpdateNow();
    } else if (id == MY_ID || id == "DOWSERALL") {
        lastSignal = millis();
    }

}

// Create an instance of the Arcanet library
Arcanet arcanet(MY_ID, onCommandReceived);

void setup() {
    Serial.begin(115200);
    //Optional code to create blinking led in loop  
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);

    ledcAttach(MOTOR_PIN, 200, 8);
    ledcWrite(MOTOR_PIN, 0);//0-255

    // Initialize the Arcanet network
    arcanet.setRssiWindowSize(2);
    arcanet.init();

    randomSeed(esp_random());

    Serial.println("####################################");
    Serial.println("### Dowsers setup complete ###");
    Serial.println("####################################");
    Serial.println("My ID is: "+String(MY_ID));

    pendingUpdate = true;
    updateScheduledAt = millis() + tUpdatePeriod;
}


void loop() {
    now = millis();

    arcanet.loop();//housekeepis from outside (TODO: put this behind a compile time switch)
    blink();//show a blinkinng our presence in Arcanet
    //readSerial();//any commandg led so we know this beacon is on
    sendUpdate();//send update if requested
    updateControllers();//prepare the regular update

    updateMotorFromRssiPulse(arcanet.getBestRssi());

    delay(10);

}

void setMotor(int strength) {
    ledcWrite(MOTOR_PIN, strength);
}

void updateMotorFromRssiPulse(int rssi) {
  filteredRssi = 0.75 * filteredRssi + 0.25 * rssi;

  unsigned long now = millis();

  if (filteredRssi <= -128) {
    setMotor(0);
    return;
  }

  int pulseWidth = 50;
  int strength = 150;
  int interval = 3000;

//  Serial.println(filteredRssi);

  if (filteredRssi<-90) {
    pulseWidth = 40;
    strength = 150;
    interval = 3000;
  } else if (filteredRssi<-83) {
    pulseWidth = 90;
    strength = 255;
    interval = 2200;
  } else if (filteredRssi<-76) {
    pulseWidth = 175;
    strength = 255;
    interval = 1500;
  } else if (filteredRssi<-69) {
    pulseWidth = 160;
    strength = 255;
    interval = 1100;
  } else if (filteredRssi<-62) {
    pulseWidth = 120;
    strength = 255;
    interval = 700;
  } else if (filteredRssi<-55) {
    pulseWidth = 120;
    strength = 255;
    interval = 400;
  } else if (filteredRssi<-48) {
    pulseWidth = 80;
    strength = 255;
    interval = 500;
  } else if (filteredRssi<-41) {
    pulseWidth = 80;
    strength = 255;
    interval = 500;
  } else if (filteredRssi<-34) {
    pulseWidth = 80;
    strength = 255;
    interval = 500;
  } else if (filteredRssi>=-34) {
    pulseWidth = 80;
    strength = 255;
    interval = 80;
  } 


  if (now - lastPulse >= (unsigned long) interval && ((lastSignal+5000) > millis()) ) {

    lastPulse = now;
    motorState = true;
    setMotor(strength);

    if (filteredRssi>-41 && filteredRssi<=-34) {
      setMotor(strength);
      delay(pulseWidth);
      setMotor(0);
      delay(40);
      setMotor(strength);
      delay(pulseWidth);
      setMotor(0);
      delay(40);
      setMotor(strength);
      delay(pulseWidth);
      setMotor(0);
      delay(40);
      setMotor(strength);
      delay(pulseWidth);
      setMotor(0);
      delay(80);
    } else if (filteredRssi>-48 && filteredRssi<=-41) {
      setMotor(strength);
      delay(pulseWidth);
      setMotor(0);
      delay(40);
      setMotor(strength);
      delay(pulseWidth);
      setMotor(0);
      delay(40);
      setMotor(strength);
      delay(pulseWidth);
      setMotor(0);
      delay(80);
    } else if (filteredRssi>-55 && filteredRssi<=-48) {
      setMotor(strength);
      delay(pulseWidth);
      setMotor(0);
      delay(40);
      setMotor(strength);
      delay(pulseWidth);
      setMotor(0);
      delay(60);
    }
  }

  // pulse duration
  if (motorState && now - lastPulse > pulseWidth) {
    motorState = false;
    setMotor(0);
  }
}



void updateControllers() {
    if ( now > updateScheduledAt + tUpdatePeriod) {
        prepareUpdate();
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


void prepareUpdate() {
    pendingUpdate = true;
    updateScheduledAt = millis() + random(0, 50);
}
void prepareUpdateNow() {
    pendingUpdate = true;
    updateScheduledAt = millis() + 10 + random(0, 10);
}

void sendUpdate() {
    if (pendingUpdate && millis() >= updateScheduledAt) {
        pendingUpdate = false;
        int v_batt = 999;
        arcanet.sendCommand("DOWSER32", MY_ID+"_"+"BLVL_"+String(v_batt)+"_SGNL_"+String(arcanet.getBestRssi())+"_STATE_" + "ON" );
    }
}






//////

void updateMotorFromRssiPulse_OLD(int rssi) {
  filteredRssi = 0.8 * filteredRssi + 0.2 * rssi;

  unsigned long now = millis();

  if (filteredRssi <= -128) {
    setMotor(0);
    return;
  }

  int strength = map((int)filteredRssi, -93, -35, 60, 255);
  strength = constrain(strength, 60, 255);

  int interval = map((int)filteredRssi, -93, -35, 4000, 130);
  interval = constrain(interval, 160, 4000);

  if (now - lastPulse >= (unsigned long) interval && ((lastSignal+5000) > millis()) ) {
    lastPulse = now;
    motorState = true;
    setMotor(strength);
  }

  // pulse duration
  if (motorState && now - lastPulse > 100) {
    motorState = false;
    setMotor(0);
  }
}
