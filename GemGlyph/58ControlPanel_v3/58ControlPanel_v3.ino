
#include <esp_now.h>
#include <WiFi.h>
#include "esp_bt.h"
#include <FastLED.h>
#include "Arcanet.h"

#define CLK1_PIN 26  // ESP32 pin GPIO25 connected to the rotary encoder's CLK pin
#define DT1_PIN 25   // ESP32 pin GPIO26 connected to the rotary encoder's DT pin
#define CLK2_PIN 32  // ESP32 pin GPIO32 connected to the rotary encoder's CLK pin
#define DT2_PIN 33   // ESP32 pin GPIO33 connected to the rotary encoder's DT pin
#define CLK3_PIN 34  // ESP32 pin GPIO34 connected to the rotary encoder's CLK pin
#define DT3_PIN 35   // ESP32 pin GPIO35 connected to the rotary encoder's DT pin
#define LOC1_PIN 39  // ESP32 pin for the Potmeter
#define LOC2_PIN 36  // ESP32 pin for the Potmeter

#define DIRECTION_CW 0   // clockwise direction
#define DIRECTION_CCW 1  // counter-clockwise direction

//Definitions to control the rotary encoders
volatile int CRED = 130;    //Red
volatile int CGREEN = 130;  //Green
volatile int CBLUE = 130;   //Blue
int prev_CRED = 130;
int prev_CGREEN = 130;
int prev_CBLUE = 130;
volatile int direction1 = DIRECTION_CW;  //Can be removed later
volatile int direction2 = DIRECTION_CW;
volatile int direction3 = DIRECTION_CW;
volatile unsigned long last_time;  // for debouncing
bool newCLK1;
bool newCLK2;
bool newCLK3;
bool newDT1;
bool newDT2;
bool newDT3;
bool lastCLK1;
bool lastCLK2;
bool lastCLK3;
int STEP = 5;  //step size for rotary encoder

//Definitions to read out the potentiometers
int LocDial1 = 0;
int Loc1;
int LocDial2 = 0;
int Loc2;

//Mapping definition
// --- 1-based lookup table: [row][col] -> bitmask (bit0=L1, bit1=L2, bit2=L3)
const uint8_t LAMP_MAP[8][4] = {
  /* r\c        0     1   2   3  (index 0 unused for convenience) */
  /*0*/ { 0, 0, 0, 0 },
  /*1*/ { 0, 1, 1, 1 },
  /*2*/ { 0, 4, 6, 5 },
  /*3*/ { 0, 2, 3, 3 },
  /*4*/ { 0, 6, 7, 6 },
  /*5*/ { 0, 3, 3, 2 },
  /*6*/ { 0, 5, 6, 4 },
  /*7*/ { 0, 0, 0, 0 },
};

// --- Returns mask for given row (1..7) and col (1..3)
uint8_t getLampMask(uint8_t row, uint8_t col) {
  if (row < 1 || row > 7 || col < 1 || col > 3) return 0;
  return LAMP_MAP[row][col];
}

void debugMask(uint8_t mask) {
  Serial.print("Mask=");
  Serial.print(mask, BIN);
  Serial.print("  (");
  if (mask & 0b001) Serial.print("L1 ");
  if (mask & 0b010) Serial.print("L2 ");
  if (mask & 0b100) Serial.print("L3 ");
  if (mask == 0) Serial.print("none");
  Serial.println(")");
}


//Definitions for the lED strip
#define NUM_LEDS 77  //Later to be 77 3x+1 x=one string R18 G23 (11-1-11) B18 --> color 18
#define LED_PIN 5
CRGB leds[NUM_LEDS];
CHSV hsv;
CRGB rgb;

const String MY_ID = "CONSOLE71";  // CHANGE THIS
//Mesh ID 0-69 : Marcel
//Mesh 70-139  : Renout
//Mesh 140-209 : Kino
//Mesh 210-279 : Bas

bool L1;
bool L2;
bool L3;
const String L1ID = "LUX1";
const String L2ID = "LUX2";
const String L3ID = "LUX3";

unsigned long lastBroadcastTime = 0;
const unsigned long broadcastInterval = 10000;
unsigned long lastLEDTime = 0;
const unsigned long LEDInterval = 1000;
bool GolemAction = false;

uint8_t prev_roundedHue;
uint8_t roundedHue;
String hueName;

// Callback function to handle received commands
void onCommandReceived(const String &id, const String &command) {
  Serial.printf("Command received for ID: %s, Command: %s\n", id.c_str(), command.c_str());

  if (id == MY_ID) {
    //Write command logic here - PRESSEDGOLEMIN = Golem Start PRESSEDGOLEMOUT = Golem Exit
    if (command == "PRESSEDGOLEMIN") {
      GolemEntry();
    } else if (command == "PRESSEDGOLEMOUT") {
      GolemExit();
    }
  }
}

// Create an instance of the Arcanet library
Arcanet arcanet(MY_ID, onCommandReceived);

// IRAM_ATTR ensures the codes is writen into fast RAM at boot and thus ensures seamless execution not holding up other code
void IRAM_ATTR ISR_encoder() {
  if ((millis() - last_time) < 5)  // debounce time is 50ms
    return;

  newCLK1 = digitalRead(CLK1_PIN);
  newDT1 = digitalRead(DT1_PIN);
  newCLK2 = digitalRead(CLK2_PIN);
  newDT2 = digitalRead(DT2_PIN);
  newCLK3 = digitalRead(CLK3_PIN);
  newDT3 = digitalRead(DT3_PIN);

  if (newCLK1 == LOW && lastCLK1 == HIGH) {
    if (newDT1 == HIGH) {
      if (CRED != 255) {
        CRED += STEP;  // Clockwise
      }
      direction1 = DIRECTION_CCW;
    } else {
      if (CRED != 0) {
        CRED -= STEP;  // Counterclockwise
      }
      direction1 = DIRECTION_CW;
    }
  }
  lastCLK1 = newCLK1;

  if (newCLK2 == LOW && lastCLK2 == HIGH) {
    if (newDT2 == HIGH) {
      if (CGREEN != 255) {
        CGREEN += STEP;  // Clockwise
      }
      direction2 = DIRECTION_CCW;
    } else {
      if (CGREEN != 0) {
        CGREEN -= STEP;  // Counterclockwise
      }
      direction2 = DIRECTION_CW;
    }
  }
  lastCLK2 = newCLK2;

  if (newCLK3 == LOW && lastCLK3 == HIGH) {
    if (newDT3 == HIGH) {
      if (CBLUE != 255) {
        CBLUE += STEP;  // Clockwise
      }
      direction3 = DIRECTION_CCW;
    } else {
      if (CBLUE != 0) {
        CBLUE -= STEP;  // Counterclockwise
      }
      direction3 = DIRECTION_CW;
    }
  }
  lastCLK3 = newCLK3;

  last_time = millis();
}

void setup() {
  Serial.begin(115200);
  Serial.println("Going to boot up Arcanet");
  arcanet.init();
  esp_bt_controller_mem_release(ESP_BT_MODE_BTDM);  //disable bluetooth

  FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);  //Update later to WS2815
  FastLED.setBrightness(102);                             //40% test whether more or less is needed

  fill_solid(leds, NUM_LEDS, CRGB::Green);
  FastLED.show();
  delay(3000);
  fill_solid(leds, NUM_LEDS, CRGB::Black);
  FastLED.show();

  // configure encoder pins as inputs
  pinMode(CLK1_PIN, INPUT);
  pinMode(DT1_PIN, INPUT);
  pinMode(CLK2_PIN, INPUT);
  pinMode(DT2_PIN, INPUT);
  pinMode(CLK3_PIN, INPUT);
  pinMode(DT3_PIN, INPUT);

  // use interrupt for CLK pin is enough
  // call ISR_encoder() when CLK pin changes from LOW to HIGH
  attachInterrupt(digitalPinToInterrupt(CLK1_PIN), ISR_encoder, RISING);
  attachInterrupt(digitalPinToInterrupt(CLK2_PIN), ISR_encoder, RISING);
  attachInterrupt(digitalPinToInterrupt(CLK3_PIN), ISR_encoder, RISING);
}

void loop() {
  //Running Arcanet
  arcanet.loop();

  //Starting with the continuous reading of the sensors
  //Rotary encoder run over the interrupt defined above
  LocDial1 = analogRead(LOC1_PIN);
  LocDial2 = analogRead(LOC2_PIN);
  // Case function for the dials -
  Loc1 = map(LocDial1, 0, 4096, 1, 4);  // Dial 1 has 3 options
  Loc2 = map(LocDial2, 0, 4096, 1, 8);  // Dial 2 has 7 options

  //CODE voor de bitmaps en welke lampen aan te sturen
  uint8_t mask = getLampMask(Loc2, Loc1);
  L1 = (mask & 0b001) ? true : false;
  L2 = (mask & 0b010) ? true : false;
  L3 = (mask & 0b100) ? true : false;
  //if (mask & 0b010) L2 = true;


  //Hier moet de gemapte kleur bepaald worden -- Dubbel check op band ranges - nu alleen erboven!
  rgb = CRGB(CRED, CGREEN, CBLUE);
  hsv = rgb2hsv_approximate(rgb);

  uint8_t inputHue = hsv.h;

  if (inputHue >= 240) {
    roundedHue = HUE_RED;
    hueName = "RED";
  } else if (inputHue >= 208) {
    roundedHue = HUE_PINK;
    hueName = "PINK";
  } else if (inputHue >= 176) {
    roundedHue = HUE_PURPLE;
    hueName = "PURPLE";
  } else if (inputHue >= 144) {
    roundedHue = HUE_BLUE;
    hueName = "BLUE";
  } else if (inputHue >= 112) {
    roundedHue = HUE_AQUA;
    hueName = "AQUA";
  } else if (inputHue >= 80) {
    roundedHue = HUE_GREEN;
    hueName = "GREEN";
  } else if (inputHue >= 48) {
    roundedHue = HUE_YELLOW;
    hueName = "YELLOW";
  } else if (inputHue >= 16) {
    roundedHue = HUE_ORANGE;
    hueName = "ORANGE";
  } else {
    roundedHue = HUE_RED;
    hueName = "RED";
  }

  if (roundedHue != prev_roundedHue) {
    Serial.print("Old color: ");
    Serial.println(prev_roundedHue);
    Serial.print("New color: ");
    Serial.print(roundedHue);
    Serial.println(hueName);
    prev_roundedHue = roundedHue;
  }

  // Alle output hieronder - Voor nu als debug, kan later verdwijnen of comment worden
  if (prev_CRED != CRED) {
    Serial.print("Rotary Encoder 1:: direction: ");
    if (direction1 == DIRECTION_CW)
      Serial.print("CLOCKWISE");
    else
      Serial.print("ANTICLOCKWISE");

    Serial.print(" - color: ");
    Serial.println(CRED);

    prev_CRED = CRED;
  }
  if (prev_CGREEN != CGREEN) {
    Serial.print("Rotary Encoder2:: direction: ");
    if (direction2 == DIRECTION_CW)
      Serial.print("CLOCKWISE");
    else
      Serial.print("ANTICLOCKWISE");

    Serial.print(" - count: ");
    Serial.println(CGREEN);

    prev_CGREEN = CGREEN;
  }
  if (prev_CBLUE != CBLUE) {
    Serial.print("Rotary Encoder3:: direction: ");
    if (direction3 == DIRECTION_CW)
      Serial.print("CLOCKWISE");
    else
      Serial.print("ANTICLOCKWISE");

    Serial.print(" - count: ");
    Serial.println(CBLUE);

    prev_CBLUE = CBLUE;
  }

  //Setting the dial indicators
  unsigned long currentTime = millis();
  if (currentTime - lastLEDTime >= LEDInterval) {
    FastLED.clear();
    int ledred = map(CRED, 0, 256, 0, 19);
    int ledgreen = map(CGREEN, 0, 256, 0, 12);
    int ledblue = map(CBLUE, 0, 256, 0, 19);
    fill_solid(leds, ledred, CRGB::Purple);                          //Fill the first 18 based on red dial
    fill_solid(leds + 18, ledgreen, CRGB::Purple);                   //Fill the next 23 based on weird logic
    fill_solid(leds + 18 + 23, ledblue, CRGB::Purple);               //Fill the next 18 based on blue dial
    fill_solid(leds + 18 + 23 + 18, 18, CRGB(CRED, CGREEN, CBLUE));  //Fill the crystal on top with the actual color --> Needs updating to selected 8
    FastLED.show();

    //Temporary code below
    FastLED.clear();
    for (int led = 0; led < 3; led++) {
      leds[led] = CRGB(CRED, CGREEN, CBLUE);
    }
    leds[3] = CRGB::Black;
    leds[4] = CHSV(roundedHue, 255, 255);
    FastLED.show();
  }

  //Regularly broadcasting data if lamps are on
  if (GolemAction != true) {
    if (currentTime - lastBroadcastTime >= broadcastInterval) {
      lastBroadcastTime = currentTime;
      String command(roundedHue);
      //Run through the different lights with a small delay in between them
      if (L1 == true) {
        arcanet.sendCommand(L1ID, command);
        delay(500);
      }
      if (L2 == true) {
        arcanet.sendCommand(L2ID, command);
        delay(500);
      }
      if (L3 == true) {
        arcanet.sendCommand(L3ID, command);
        delay(500);
      }
      Serial.print("Pot 1 :: ");
      Serial.print(LocDial1);
      Serial.print(" :: ");
      Serial.println(Loc1);
      Serial.print("Pot 2 :: ");
      Serial.print(LocDial2);
      Serial.print(" :: ");
      Serial.println(Loc2);
      debugMask(mask);  //To be removed later
      Serial.print("Current color: ");
      Serial.println(roundedHue);
    }
  }


  // Hieronder alles van Marcel ---------------------------------------------------------------------------------------------------------------------------

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

void GolemEntry() {
  GolemAction = true;
  arcanet.sendCommand("LUXALL", "WHITE_ON");
  delay(500);
  arcanet.sendCommand("MUSIC145", "PLAYGOLEMIN");
  delay(500);
  for (int i = 0; i < 22; i++) {
    arcanet.sendCommand("LUXALL", "RED_ON");
    delay(1000);
    arcanet.sendCommand("LUXALL", "BLACK_ON");
    delay(1000);
    Serial.print("Round 1 - ");
    Serial.println(i);
  }
  for (int i = 0; i < 8; i++) {
    arcanet.sendCommand("LUXALL", "PURPLE_ON");
    delay(750);
    arcanet.sendCommand("LUXALL", "BLACK_ON");
    delay(750);
    Serial.print("Round 2 - ");
    Serial.println(i);
  }
  GolemAction = false;
  return;
}

void GolemExit() {
  GolemAction = true;
  arcanet.sendCommand("LUXALL", "WHITE_ON");
  arcanet.sendCommand("MUSIC145", "PLAYGOLEMOUT");
  delay(500);
  GolemAction = false;
}