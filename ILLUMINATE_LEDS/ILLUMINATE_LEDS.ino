#include <Wire.h>
#include <Adafruit_MCP23X17.h>

// Custom I2C instance
TwoWire myI2C = TwoWire(0);

// MCP23017 setup
Adafruit_MCP23X17 mcp;

// Button 1: ESP32 GPIO2 (direct to main board, goes to GND when pressed)
const int controlPanelWake = 2; //GND
// Button 2: MCP23017 pin 15
const int button2Pin = 15;  //MCP

const int microswitchClosePin = 16;   // GPIO16
const int microswitchOpenPin = 15;    // GPIO15

// LED pins (MCP23017 side) - same as toilet_kat_change.ino
const int ledPins[] = {1, 9, 13, 14, 10, 6, 11, 12, 8, 0, 2, 3, 4, 5};
const int totalLeds = sizeof(ledPins) / sizeof(ledPins[0]);

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("ILLUMINATE_LEDS - Starting...");

  // Use GPIO6 for SDA and GPIO7 for SCL
  myI2C.begin(6, 7, 100000);

  if (!mcp.begin_I2C(0x20, &myI2C)) {
    Serial.println("Error initializing MCP23017!");
    while (1) delay(1000);
  }
  Serial.println("MCP23017 OK.");

  pinMode(controlPanelWake, INPUT_PULLUP);
  //pinMode(microswitchClosePin, INPUT_PULLUP);
  //pinMode(microswitchOpenPin, INPUT_PULLUP);
  mcp.pinMode(button2Pin, INPUT_PULLUP);

  for (int i = 0; i < totalLeds; i++) {
    mcp.pinMode(ledPins[i], OUTPUT);
  }

  // Illuminate all LEDs on startup
  turnAllLEDsOn();
  Serial.println("All LEDs illuminated. B1=flash(hold to loop), B2=circle(hold to loop), both=fast flash. Runs indefinitely.");
}

void loop() {
  bool button1Pressed = (digitalRead(controlPanelWake) == LOW);
  bool button2Pressed = (mcp.digitalRead(button2Pin) == LOW);

  if (button1Pressed || button2Pressed) {
    while (button1Pressed || button2Pressed) {
      button1Pressed = (digitalRead(controlPanelWake) == LOW);
      button2Pressed = (mcp.digitalRead(button2Pin) == LOW);
      if (button1Pressed && button2Pressed) {
        flashAllLEDsWithDelay(1, 80);
      } else if (button1Pressed) {
        flashAllLEDsWithDelay(3, 200);
      } else if (button2Pressed) {
        circleLEDsOnce();
      }
    }
    turnAllLEDsOn();
  }

  delay(100);
}

void turnAllLEDsOn() {
  for (int i = 0; i < totalLeds; i++) {
    mcp.digitalWrite(ledPins[i], HIGH);
  }
}

void flashAllLEDsWithDelay(int times, int delayMs) {
  for (int t = 0; t < times; t++) {
    for (int i = 0; i < totalLeds; i++) mcp.digitalWrite(ledPins[i], LOW);
    delay(delayMs);
    for (int i = 0; i < totalLeds; i++) mcp.digitalWrite(ledPins[i], HIGH);
    delay(delayMs);
  }
}

void circleLEDsOnce() {
  const int stepDelay = 50;
  for (int i = 0; i < totalLeds; i++) {
    mcp.digitalWrite(ledPins[i], LOW);
  }
  for (int i = 0; i < totalLeds; i++) {
    mcp.digitalWrite(ledPins[i], HIGH);
    delay(stepDelay);
    mcp.digitalWrite(ledPins[i], LOW);
  }
}
