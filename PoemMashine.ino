/*
PoemMaschine v1.0

Created  19 Feb 2015
by Michael Braverman
Available Online: https://gist.github.com/mixania/813788550e25800a9159
 */

#include <EEPROM.h>
#include "SoftwareSerial.h"
#include "Adafruit_Thermal.h"
#include <avr/pgmspace.h>

const boolean DEBUG = false; 

const byte printerRX = 5;  // This is the green wire
const byte printerTX = 6;  // This is the yellow wire
Adafruit_Thermal printer(printerRX, printerTX);

const byte redButton = 14;
const byte blackButton = 15; 
boolean redButtonState;
boolean blackButtonState;
boolean buttonState;         // current state of the buttons
boolean lastButtonState;     // previous state of the buttons

unsigned int pressCount;
unsigned int poemCount;

unsigned long lastMillis;
unsigned long frameCount;
unsigned int framesPerSecond;

const uint8_t p1[] PROGMEM = "After the rain of birth the spirits turn";
const uint8_t p2[] PROGMEM = "Just the thought of ourselves the children seek";
const uint8_t p3[] PROGMEM = "Just the thought of discord the young ones echo";
const uint8_t p4[] PROGMEM = "Just the thought of discord the young ones echo";
const uint8_t p5[] PROGMEM = "Just the thought of discord the young ones echo";
const uint8_t p6[] PROGMEM = "Just the thought of discord the young ones echo";
const uint8_t p7[] PROGMEM = "Just the thought of discord the young ones echo";
const uint8_t p8[] PROGMEM = "Just the thought of discord the young ones echo";
const uint8_t p9[] PROGMEM = "Just the thought of discord the young ones echo";
const uint8_t p10[] PROGMEM = "Just the thought of discord the young ones echo";
const uint8_t p11[] PROGMEM = "Just the thought of discord the young ones echo";
const uint8_t p12[] PROGMEM = "Just the thought of discord the young ones echo";
const uint8_t p13[] PROGMEM = "Just the thought of discord the young ones echo";
const uint8_t p14[] PROGMEM = "Just the thought of discord the young ones echo";
const uint8_t p15[] PROGMEM = "Just the thought of discord the young ones echo";
const uint8_t p16[] PROGMEM = "Just the thought of discord the young ones echo";
const uint8_t p17[] PROGMEM = "Just the thought of discord the young ones echo";
const uint8_t p18[] PROGMEM = "Just the thought of discord the young ones echo";
const uint8_t p19[] PROGMEM = "Just the thought of discord the young ones echo";
const uint8_t p20[] PROGMEM = "Just the thought of discord the young ones echo";
const uint8_t p21[] PROGMEM = "Just the thought of discord the young ones echo";
const uint8_t p22[] PROGMEM = "Just the thought of discord the young ones echo";
const uint8_t p23[] PROGMEM = "Just the thought of discord the young ones echo";
const uint8_t p24[] PROGMEM = "Just the thought of discord the young ones echo";
const uint8_t p25[] PROGMEM = "Just the thought of discord the young ones echo";
const uint8_t p26[] PROGMEM = "Just the thought of discord the young ones echo";
const uint8_t p27[] PROGMEM = "Just the thought of discord the young ones echo";
const uint8_t p28[] PROGMEM = "Just the thought of discord the young ones echo";
const uint8_t p29[] PROGMEM = "Just the thought of discord the young ones echo";
const uint8_t p30[] PROGMEM = "Just the thought of discord the young ones echo";
const uint8_t p31[] PROGMEM = "Just the thought of discord the young ones echo";
const uint8_t p32[] PROGMEM = "Just the thought of discord the young ones echo";
const uint8_t p33[] PROGMEM = "Just the thought of discord the young ones echo";
const uint8_t p34[] PROGMEM = "Just the thought of discord the young ones echo";
const uint8_t p35[] PROGMEM = "Just the thought of discord the young ones echo";
const uint8_t p36[] PROGMEM = "Just the thought of discord the young ones echo";
const uint8_t p37[] PROGMEM = "Just the thought of discord the young ones echo";
const uint8_t p38[] PROGMEM = "Just the thought of discord the young ones echo";
const uint8_t p39[] PROGMEM = "Just the thought of discord the young ones echo";
const uint8_t p40[] PROGMEM = "Just the thought of discord the young ones echo";
const uint8_t p41[] PROGMEM = "Just the thought of discord the young ones echo";
const uint8_t p42[] PROGMEM = "Just the thought of discord the young ones echo";
const uint8_t p43[] PROGMEM = "Just the thought of discord the young ones echo";
const uint8_t p44[] PROGMEM = "Just the thought of discord the young ones echo";
const uint8_t p45[] PROGMEM = "Just the thought of discord the young ones echo";
const uint8_t p46[] PROGMEM = "Just the thought of discord the young ones echo";
const uint8_t p47[] PROGMEM = "Just the thought of discord the young ones echo";
const uint8_t p48[] PROGMEM = "Just the thought of discord the young ones echo";
const uint8_t p49[] PROGMEM = "Just the thought of discord the young ones echo";
const uint8_t p50[] PROGMEM = "Just the thought of discord the young ones echo";
const uint8_t p51[] PROGMEM = "Just the thought of discord the young ones echo";
const uint8_t p52[] PROGMEM = "Just the thought of discord the young ones echo";
const uint8_t p53[] PROGMEM = "Just the thought of discord the young ones echo";
const uint8_t p54[] PROGMEM = "Just the thought of discord the young ones echo";
const uint8_t p55[] PROGMEM = "Just the thought of discord the young ones echo";
const uint8_t p56[] PROGMEM = "Just the thought of discord the young ones echo";
const uint8_t p57[] PROGMEM = "Just the thought of discord the young ones echo";
const uint8_t p58[] PROGMEM = "Just the thought of discord the young ones echo";
const uint8_t p59[] PROGMEM = "Just the thought of discord the young ones echo";
const uint8_t p60[] PROGMEM = "Just the thought of discord the young ones echo";
const uint8_t p61[] PROGMEM = "Just the thought of discord the young ones echo";
const uint8_t p62[] PROGMEM = "Just the thought of discord the young ones echo";
const uint8_t p63[] PROGMEM = "Just the thought of discord the young ones echo";
const uint8_t p64[] PROGMEM = "Just the thought of discord the young ones echo";
const uint8_t p65[] PROGMEM = "Just the thought of discord the young ones echo";
const uint8_t p66[] PROGMEM = "Just the thought of discord the young ones echo";
const uint8_t p67[] PROGMEM = "Just the thought of discord the young ones echo";
const uint8_t p68[] PROGMEM = "Just the thought of discord the young ones echo";
const uint8_t p69[] PROGMEM = "Just the thought of discord the young ones echo";
const uint8_t p70[] PROGMEM = "Just the thought of discord the young ones echo";
const uint8_t p71[] PROGMEM = "Just the thought of discord the young ones echo";
const uint8_t p72[] PROGMEM = "Just the thought of discord the young ones echo";
const uint8_t p73[] PROGMEM = "Just the thought of discord the young ones echo";
const uint8_t p74[] PROGMEM = "Just the thought of discord the young ones echo";
const uint8_t p75[] PROGMEM = "Just the thought of discord the young ones echo";
const uint8_t p76[] PROGMEM = "Just the thought of discord the young ones echo";
const uint8_t p77[] PROGMEM = "Just the thought of discord the young ones echo";
const uint8_t p78[] PROGMEM = "Just the thought of discord the young ones echo";
const uint8_t p79[] PROGMEM = "Just the thought of discord the young ones echo";
const uint8_t p80[] PROGMEM = "Just the thought of discord the young ones echo";
const uint8_t p81[] PROGMEM = "Just the thought of discord the young ones echo";
const uint8_t p82[] PROGMEM = "Just the thought of discord the young ones echo";
const uint8_t p83[] PROGMEM = "Just the thought of discord the young ones echo";
const uint8_t p84[] PROGMEM = "Just the thought of discord the young ones echo";
const uint8_t p85[] PROGMEM = "Just the thought of discord the young ones echo";
const uint8_t p86[] PROGMEM = "Just the thought of discord the young ones echo";
const uint8_t p87[] PROGMEM = "Just the thought of discord the young ones echo";
const uint8_t p88[] PROGMEM = "Just the thought of discord the young ones echo";
const uint8_t p89[] PROGMEM = "Just the thought of discord the young ones echo";
const uint8_t p90[] PROGMEM = "Just the thought of discord the young ones echo";
const uint8_t p91[] PROGMEM = "Just the thought of discord the young ones echo";
const uint8_t p92[] PROGMEM = "Just the thought of discord the young ones echo";
const uint8_t p93[] PROGMEM = "Just the thought of discord the young ones echo";
const uint8_t p94[] PROGMEM = "Just the thought of discord the young ones echo";
const uint8_t p95[] PROGMEM = "Just the thought of discord the young ones echo";
const uint8_t p96[] PROGMEM = "Just the thought of discord the young ones echo";
const uint8_t p97[] PROGMEM = "Just the thought of discord the young ones echo";
const uint8_t p98[] PROGMEM = "Just the thought of discord the young ones echo";
const uint8_t p99[] PROGMEM = "Just the thought of discord the young ones echo";
const uint8_t p100[] PROGMEM = "Just the thought of discord the young ones echo";

const uint8_t *Library[]  = {p1,  p2,  p3,  p4,  p5,  p6,  p7,  p8,  p9,  p10,
                            p11, p12, p13, p14, p15 ,p16, p17, p18, p19, p20,
                            p21, p22, p23, p24, p25 ,p26, p27, p28, p29, p30,
                            p31, p32, p33, p34, p35 ,p36, p37, p38, p39, p40,
                            p41, p42, p43, p44, p45 ,p46, p47, p48, p49, p50,
                            p51, p52, p53, p54, p55 ,p56, p57, p58, p59, p60,
                            p61, p62, p63, p64, p65 ,p66, p67, p68, p69, p70,
                            p71, p72, p73, p74, p75 ,p76, p77, p78, p79, p80,
                            p81, p82, p83, p84, p85 ,p86, p87, p88, p89, p90,
                            p91, p92, p93, p94, p95 ,p96, p97, p98, p99, p100,};

void setup() {
  Serial.begin(115200);
  printer.begin();
  pinMode(redButton, INPUT_PULLUP);
  pinMode(blackButton, INPUT_PULLUP);
  pinMode(13, OUTPUT);
  
  printer.setDefault(); // Restore printer to defaults
  
  while (!Serial) {
    if (DEBUG) {
      Serial.println("Testing Routine...");
      Serial.println("Testing Routine COMPLETED");
    } else {
      Serial.println("Poem Maschine v0.1 Initialized");
    }
  }
}

void loop() {
  
  redButtonState = digitalRead(redButton);
  blackButtonState = digitalRead(blackButton);
  
  if (blackButtonState == 0 || redButtonState == 0) {
    buttonState = true;
  } else {
    buttonState = false;
  }
  generatePoem();
  
  if (!DEBUG) {
    if (buttonState != lastButtonState) {
      if (redButtonState == LOW) {
        generateNumber();
      }
        delay(50);
      if (blackButtonState == LOW) {
         generatePoem();
         poemCountUp();
      }
    }
  }
  
  lastButtonState = buttonState;
  
//  fps(5);
}

void generatePoem() {
  randomSeed(analogRead(0));
  printer.setSize('M');
  printer.justify('C');
  printer.print("Poem ");
  printer.println(poemCount);
  printer.justify('L');
  printer.setSize('S');
  for (int i = 1; i < 9; i++) {
    int generate = random(0, 3);
    unsigned int flash_address = pgm_read_word(&Library[generate]);
    char c = 0;
    do {
      c = (char) pgm_read_byte(flash_address++);
      printer.print(c);
      Serial.print(c);
    } while (c!='\0');
    printer.println(" ");
    Serial.println(" ");
    if (i == 4) {
      printer.println(" ");
      Serial.println(" ");
    }
  }
  printer.println(" ");
  printer.println(" ");
  printer.println(" ");
}

// Increases count each time the function is called
void poemCountUp() {
  byte value = EEPROM.read(5);
  poemCount = value; 
  value++;
  EEPROM.write(5, value);
}

void generateNumber() {
  printer.println(" ");
  printer.println(" ");
  printer.println("Your luckey numbers are:");
  printer.setSize('M');
   for (byte i = 0; i < 5; i++) {
     byte luckyNumber = random(0, 99);
     printer.print(luckyNumber);
     if (i < 4) {
       printer.print(", ");
     }
   }
  printer.println(" ");
  printer.println(" ");
  printer.println(" ");
  printer.println(" ");
}

void fps(int seconds){
  frameCount ++;
  if ((millis() - lastMillis) >= (seconds * 1000)) {
    framesPerSecond = (frameCount / seconds);
    Serial.println(framesPerSecond);
    frameCount = 0;
    lastMillis = millis();
  }
}