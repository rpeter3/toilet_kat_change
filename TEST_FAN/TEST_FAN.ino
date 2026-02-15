/*
 * Bi-directional PWM Fan Test
 * Tests the amended motor 3 (sealer fan) - now a bi-directional PWM fan.
 * Button 1 (MCP pin 7): advance state | Button 2 (Wake GPIO2): previous state
 */

#include <Wire.h>
#include <Adafruit_MCP23X17.h>

// Custom I2C instance
TwoWire myI2C = TwoWire(0);
Adafruit_MCP23X17 mcp;

// Fan pins (ESP32)
const int sealerFanPWM = 47;      // PWM input, 25 kHz
const int sealerFanPwr = 4;       // Power enable (transistor)
const int sealerFanReverse = 21;   // HIGH = reverse

// PWM config (25 kHz, 8-bit for fanSpeed 0-100)
const int FAN_PWM_FREQ = 25000;
const int FAN_PWM_RESOLUTION = 8;

// Control panel
const int button1Pin = 15;         // MCP pin 15
const int controlPanelWake = 2;   // GPIO2 (Button 2 / Wake)

// LED pins (MCP) - use first 13 for state feedback
const int ledPins[] = {1, 9, 13, 14, 10, 6, 11, 12, 8, 0, 2, 3, 4, 5};
const int totalLeds = sizeof(ledPins) / sizeof(ledPins[0]);
const int NUM_STATES = 13;

// Fan state: power, fanSpeed [0-100], reverse
struct FanState {
  bool power;
  int fanSpeed;
  bool reverse;
};

const FanState fanStates[NUM_STATES] = {
  {true,  0,   false},  // 0:  on, 0%,   forward
  {true,  25,  false},  // 1:  on, 25%,  forward
  {true,  50,  false},  // 2:  on, 50%,  forward
  {true,  75,  false},  // 3:  on, 75%,  forward
  {true,  100, false},  // 4:  on, 100%, forward
  {true,  0,   true},   // 5:  on, 0%,   reverse
  {true,  25,  true},   // 6:  on, 25%,  reverse
  {true,  50,  true},   // 7:  on, 50%,  reverse
  {true,  75,  true},   // 8:  on, 75%,  reverse
  {true,  100, true},   // 9:  on, 100%, reverse
  {true,  100, false},  // 10: on, 100%, forward
  {false, 100, false},  // 11: off, 100%, forward
  {false, 0,   false},  // 12: off, 0%,   forward
};

int stateIndex = 0;

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("Bi-directional PWM Fan Test Starting...");

  // I2C for MCP
  myI2C.begin(6, 7, 100000);
  if (!mcp.begin_I2C(0x20, &myI2C)) {
    Serial.println("Error initializing MCP23017!");
    while (1);
  }
  Serial.println("MCP23017 Initialized.");

  // Buttons
  mcp.pinMode(button1Pin, INPUT_PULLUP);
  pinMode(controlPanelWake, INPUT_PULLUP);

  // LEDs
  for (int i = 0; i < totalLeds; i++) {
    mcp.pinMode(ledPins[i], OUTPUT);
  }

  // Fan pins
  pinMode(sealerFanPwr, OUTPUT);
  pinMode(sealerFanReverse, OUTPUT);
  ledcAttach(sealerFanPWM, FAN_PWM_FREQ, FAN_PWM_RESOLUTION);

  Serial.println("Setup complete.");
  Serial.println("Button 1 (MCP 7): next state | Button 2 (Wake GPIO2): prev state");

  applyFanState(stateIndex);
}

void loop() {
  bool button1Pressed = (mcp.digitalRead(button1Pin) == LOW);
  bool wakePressed = (digitalRead(controlPanelWake) == LOW);

  if (button1Pressed) {
    stateIndex = (stateIndex + 1) % NUM_STATES;
    Serial.print("Button 1 -> State ");
    Serial.println(stateIndex);
    applyFanState(stateIndex);
    delay(150);
  }
  if (wakePressed) {
    stateIndex = (stateIndex + NUM_STATES - 1) % NUM_STATES;
    Serial.print("Button 2 (Wake) -> State ");
    Serial.println(stateIndex);
    applyFanState(stateIndex);
    delay(150);
  }

  delay(50);
}

void applyFanState(int idx) {
  const FanState& s = fanStates[idx];

  // Power
  digitalWrite(sealerFanPwr, s.power ? HIGH : LOW);

  // Reverse
  digitalWrite(sealerFanReverse, s.reverse ? HIGH : LOW);

  // PWM duty: fanSpeed [0,100] -> [0,255]
  int duty = (s.fanSpeed * 255) / 100;
  ledcWrite(sealerFanPWM, duty);

  // LEDs: only current state 
  for (int i = 0; i < totalLeds; i++) {
    mcp.digitalWrite(ledPins[i], (i == idx) ? HIGH : LOW);
  }

  Serial.print("State ");
  Serial.print(idx);
  Serial.print(": power=");
  Serial.print(s.power ? "ON" : "OFF");
  Serial.print(" speed=");
  Serial.print(s.fanSpeed);
  Serial.print("% dir=");
  Serial.println(s.reverse ? "REV" : "FWD");
}
