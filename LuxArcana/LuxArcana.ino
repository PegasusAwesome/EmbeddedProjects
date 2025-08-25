#include <Arduino.h>
#include "src/Arcanet.h"

// Your device's unique ID
const String MY_ID = "LUX1";

//RGBW PWM Pins
const uint8_t DIM_PIN_WHITE      = 2;   //D1 PT4115 DIM WHITE
const uint8_t DIM_PIN_BLUE       = 3;   //D2 PT4115 DIM BLUE
const uint8_t DIM_PIN_GREEN      = 4;   //D3 PT4115 DIM GREEN
const uint8_t DIM_PIN_RED        = 5;   //D4 PT4115 DIM RED
const uint8_t DIM_PIN_LED_DRIVER = 6;   //D5  PT4115 BOARD

//not in use pins on the side of the RGBW pins
const uint8_t DIM_PIN_D0         = 1;   // 
const uint8_t DIM_PIN_D6         = 43;  // 

//Settings for rgbw driver
const int CHANNEL    = 0;
const int FREQ_HZ    = 5000;  // 5 kHz
static const int RES = 8;

// Track per-pin resolution so we know max duty when writing later
struct PwmInfo { uint8_t pin; uint8_t res; uint16_t max; bool ok; };
PwmInfo pwmInfos[16];  // up to 16 pins; adjust as needed
int pwmInfoCount = 0;



// Callback function to handle received commands
void onCommandReceived(const String& id, const String& command) {
  Serial.printf("Command received for ID: %s, Command: %s\n", id.c_str(), command.c_str());

//  turnAllOff();
  delay(200);

  if (id == MY_ID) {

    if (command == "WHITE_ON") {
        turnLedOn(DIM_PIN_WHITE, 65);
    
    } else if (command == "WHITE_OFF") {
        turnLedOff(DIM_PIN_WHITE, 65);

    } else if (command == "RED_ON") {
        turnLedOn(DIM_PIN_RED, 55);
    
    } else if (command == "RED_OFF") {
        turnLedOff(DIM_PIN_RED, 55);

    } else if (command == "GREEN_ON") {
        turnLedOn(DIM_PIN_GREEN, 65);
    
    } else if (command == "GREEN_OFF") {
        turnLedOff(DIM_PIN_GREEN, 65);

    } else if (command == "BLUE_ON") {
        turnLedOn(DIM_PIN_BLUE, 65);
    
    } else if (command == "BLUE_OFF") {
        turnLedOff(DIM_PIN_BLUE, 65);

    } else if (command == "MAGENTA_ON") {
        turnLedOn(DIM_PIN_BLUE, 65);
        turnLedOn(DIM_PIN_RED, 55);

    } else if (command == "MAGENTA_OFF") {
        turnLedOff(DIM_PIN_BLUE, 65);
        turnLedOff(DIM_PIN_RED, 55);

    } else if (command == "CYAN_ON") {
        turnLedOn(DIM_PIN_BLUE, 65);
        turnLedOn(DIM_PIN_GREEN, 65);

    } else if (command == "CYAN_OFF") {
        turnLedOff(DIM_PIN_BLUE, 65);
        turnLedOff(DIM_PIN_GREEN, 65);

    } else if (command == "YELLOW1_ON") {
        turnLedOn(DIM_PIN_RED, 55);
        turnLedOn(DIM_PIN_GREEN, 65);

    } else if (command == "YELLOW1_OFF") {
        turnLedOff(DIM_PIN_RED, 55);
        turnLedOff(DIM_PIN_GREEN, 65);

    } else if (command == "YELLOW2_ON") {
        turnLedOn(DIM_PIN_RED, 60);
        turnLedOn(DIM_PIN_GREEN, 65);

    } else if (command == "YELLOW2_OFF") {
        turnLedOff(DIM_PIN_RED, 60);
        turnLedOff(DIM_PIN_GREEN, 65);

    } else if (command == "DWHITE_ON") {
        turnLedOn(DIM_PIN_RED, 55);
        turnLedOn(DIM_PIN_GREEN, 65);
        turnLedOn(DIM_PIN_BLUE, 65);
        
    } else if (command == "DWHITE_OFF") {
        turnLedOff(DIM_PIN_RED, 55);
        turnLedOff(DIM_PIN_GREEN, 65);
        turnLedOff(DIM_PIN_BLUE, 65);
    }
  }
}

// Create an instance of the Arcanet library
Arcanet arcanet(MY_ID, onCommandReceived);


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
   delay(500);
   digitalWrite(LED_BUILTIN, LOW);
   delay(500);
   digitalWrite(LED_BUILTIN, HIGH);

//    turnLedOn(DIM_PIN_RED, 45);
//    delay(500);
//    turnLedOff(DIM_PIN_RED, 45);
//    delay(500);

}


void setup() {

  Serial.begin(115200);
  //Optional code to create blinking led in loop  
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  //init pwm dim pins
  initDimPin(DIM_PIN_RED);
  initDimPin(DIM_PIN_GREEN);
  initDimPin(DIM_PIN_BLUE);
  initDimPin(DIM_PIN_WHITE);

  delay(1000);
  //turn on led driver (set to 5v on N-Fet)
  digitalWrite(DIM_PIN_LED_DRIVER, HIGH);
  pinMode(DIM_PIN_LED_DRIVER, OUTPUT);
  digitalWrite(DIM_PIN_LED_DRIVER, HIGH);

  
  // Initialize the Arcanet network
  arcanet.init();

  Serial.println("##################################");
  Serial.println("### Application setup complete ###");
  Serial.println("##################################");
}


void turnLedOn(uint8_t DIM_PIN, uint8_t TO_BRIGHTNESS) {
    for (int i=0;i<=TO_BRIGHTNESS;i++) { 
        setBrightness(DIM_PIN, i/100.0f); 
        delay(10);
    }

}

void turnLedOff(uint8_t DIM_PIN, uint8_t FROM_BRIGHTNESS) {
    for (int i=FROM_BRIGHTNESS; i>=0; i--) { 
        setBrightness(DIM_PIN, i/100.0f); 
        delay(10);
    }
}

void turnAllOff() {
    turnLedOff(DIM_PIN_WHITE, 0);
    turnLedOff(DIM_PIN_RED, 0);
    turnLedOff(DIM_PIN_GREEN, 0);
    turnLedOff(DIM_PIN_BLUE, 0);
}

PwmInfo* initDimPin(uint8_t pin) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, HIGH); // ULN IN = HIGH -> OUT ≈ 0 V (LED OFF) during init

    // try to attach
    if (ledcAttach(pin, FREQ_HZ, RES)) {
        uint16_t max = (1u << RES) - 1u;
        ledcWrite(pin, max); // park at 100% -> IN HIGH -> ULN2003 OUT ≈ 0 V
        PwmInfo info{pin, (uint8_t)RES, max, true};
        pwmInfos[pwmInfoCount++] = info;
        Serial.printf("LEDC attach success on pin %u %d bits at %d Hz\n", pin, RES, FREQ_HZ);
        return &pwmInfos[pwmInfoCount-1];
    }

    Serial.printf("LEDC attach failed on pin %u %dHz)\n", pin, FREQ_HZ);
    static PwmInfo bad{pin,0,0,false};

    return &bad;
}

PwmInfo* findPwm(uint8_t pin) {
    for (int i=0;i<pwmInfoCount;i++) if (pwmInfos[i].pin==pin) return &pwmInfos[i];
    return nullptr;
}

// set brightness 0..1
void setBrightness(uint8_t pin, float brightness) {
    brightness = 1-constrain(brightness, 0.0f, 1.0f);
    ledcWrite(pin, (int) (brightness * ((1<<RES)-1) ) );
}

