#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <DualMAX14870MotorShield.h>

// Pins (copied from existing project/test sketches)
const int heaterPin = 17;
const int microswitchClosePin = 16;  // LOW when mechanism is in closed position
const int microswitchOpenPin = 15;
const int controlPanelWake = 2;
const int m1CurrentPin = 8;
const int thermistorPin = 14;
const int batteryVoltagePin = 10;  // Same battery monitor input as main/test sketches

const uint8_t M1DIR_PIN = 1;
const uint8_t M1PWM_PIN = 42;
const uint8_t M1NEN_PIN = 48;
const uint8_t M1NFAULT_PIN = 40;
const uint8_t M2DIR_PIN = 3;
const uint8_t M2PWM_PIN = 13;
const uint8_t M2NEN_PIN = 9;
const uint8_t M2NFAULT_PIN = 11;

DualMAX14870MotorShield motors(M1DIR_PIN, M1PWM_PIN, M2DIR_PIN, M2PWM_PIN, M2NEN_PIN, M2NFAULT_PIN);

// BLE UUIDs (same as existing test programs)
#define SERVICE_UUID "5636340f-afc7-47b1-b0a8-15bc9d7d29a5"
#define SERIAL_CHARACTERISTIC_UUID "c327b077-560f-46a1-8f35-b4ab0332fea1"

BLEServer* blueServer = nullptr;
BLECharacteristic* serialCharacteristic = nullptr;
bool bleConnected = false;
bool serialStreamingEnabled = false;

// Thermistor constants
const float knownResistor = 10000.0f;
const float A = 0.001129148f;
const float B = 0.000234125f;
const float C = 0.0000000876741f;

// Test parameters
const float CUT_MODE_TEMP = 175.0f;
const float COOL_OPEN_TEMP = 90.0f;
const float RESTART_TEMP = 65.0f;
const float HEATER_OVERHEAT_MULTIPLIER = 1.20f;
const float M1_STALL_CURRENT_A = 0.5f;
const int M1_TEST_SPEED = 400;
const unsigned long HOLD_TIME_MS = 5000;
const unsigned long OPEN_EXTRA_MS = 2000;
const unsigned long TEMP_LOG_INTERVAL_MS = 1000;
const unsigned long BATTERY_LOG_INTERVAL_MS = 2000;
const unsigned long CLOSE_TIMEOUT_MS = 30000;
const unsigned long OPEN_TIMEOUT_MS = 30000;
const int MIN_BATTERY_PERCENT = 30;

// Test state
enum TestHeaterStep {
  TEST_CLOSE_TO_STALL,
  TEST_HEAT_TO_TARGET,
  TEST_HOLD_5S,
  TEST_COOL_TO_90,
  TEST_OPEN_WITH_EXTRA_2S,
  TEST_WAIT_TO_65
};

TestHeaterStep testStep = TEST_CLOSE_TO_STALL;
unsigned long stepStartMs = 0;
unsigned long holdStartMs = 0;
unsigned long openExtraStartMs = 0;
unsigned long lastTempLogMs = 0;
unsigned long lastBatteryLogMs = 0;
bool openingExtraPhase = false;
uint32_t cycleCount = 0;
bool heaterOutputOn = false;
bool testHalted = false;

void sendSerialToBLE(const String& message) {
  if (bleConnected && serialCharacteristic) {
    serialCharacteristic->setValue(message.c_str());
    serialCharacteristic->notify();
  }
}

#define SerialBLE_print(x) do { Serial.print(x); if (serialStreamingEnabled) sendSerialToBLE(String(x)); } while (0)
#define SerialBLE_println(x) do { Serial.println(x); if (serialStreamingEnabled) sendSerialToBLE(String(x) + "\n"); } while (0)

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) override {
    bleConnected = true;
    serialStreamingEnabled = true;  // Auto-enable telemetry on connect
    sendSerialToBLE("TEST_HEATER: BLE connected, streaming ON\n");
    (void)pServer;
  }

  void onDisconnect(BLEServer* pServer) override {
    bleConnected = false;
    serialStreamingEnabled = false;
    delay(200);
    pServer->startAdvertising();
  }
};

class SerialCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pCharacteristic) override {
    String command = pCharacteristic->getValue().c_str();
    command.trim();
    if (command == "START_SERIAL") {
      serialStreamingEnabled = true;
      sendSerialToBLE("Serial streaming ENABLED\n");
      Serial.println("Serial streaming ENABLED");
    } else if (command == "STOP_SERIAL") {
      serialStreamingEnabled = false;
      Serial.println("Serial streaming DISABLED");
    }
  }
};

void setupBLE() {
  BLEDevice::init("ESP32 Toilet");
  blueServer = BLEDevice::createServer();
  blueServer->setCallbacks(new ServerCallbacks());

  BLEService* service = blueServer->createService(SERVICE_UUID);
  serialCharacteristic = service->createCharacteristic(
    SERIAL_CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ |
    BLECharacteristic::PROPERTY_WRITE |
    BLECharacteristic::PROPERTY_NOTIFY
  );
  serialCharacteristic->setCallbacks(new SerialCallbacks());
  serialCharacteristic->setValue("TEST_HEATER ready");

  service->start();
  BLEAdvertising* advertising = blueServer->getAdvertising();
  advertising->addServiceUUID(SERVICE_UUID);
  advertising->setScanResponse(true);
  advertising->setMinPreferred(0x06);
  advertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
}

float readTemperature() {
  int analogValue = analogRead(thermistorPin);
  float voltage = analogValue * (3.3f / 4095.0f);
  if (voltage <= 0.01f || voltage >= 3.29f) {
    return -100.0f;
  }
  float resistance = (voltage * knownResistor) / (3.3f - voltage);
  float logR = log(resistance);
  float tempKelvin = 1.0f / (A + B * logR + C * logR * logR * logR);
  return tempKelvin - 273.15f;
}

float readM1Current() {
  int analogValue = analogRead(m1CurrentPin);
  float voltage = analogValue * (3.3f / 4095.0f);
  return voltage / 2.0f;
}

float readBatteryVoltage() {
  int analogValue = analogRead(batteryVoltagePin);
  float vmon = analogValue * (3.3f / 4095.0f);
  return vmon * 7.317f;
}

int getBatteryChargeLevel() {
  float batteryVoltage = readBatteryVoltage();
  if (batteryVoltage >= 12.6f) {
    return 100;
  }
  if (batteryVoltage <= 11.0f) {
    return 0;
  }
  return (int)(((batteryVoltage - 11.0f) / 1.6f) * 100.0f);
}

void setHeater(bool on) {
  analogWrite(heaterPin, on ? 255 : 0);
  heaterOutputOn = on;
}

void updateHeaterControl(float currentTemp) {
  const float lower = CUT_MODE_TEMP - 2.0f;
  const float upper = CUT_MODE_TEMP;
  if (!heaterOutputOn && currentTemp < lower) {
    setHeater(true);
  } else if (heaterOutputOn && currentTemp >= upper) {
    setHeater(false);
  }
}

void enterStep(TestHeaterStep nextStep, const String& stateName) {
  testStep = nextStep;
  stepStartMs = millis();
  SerialBLE_print("TEST_HEATER: STATE=");
  SerialBLE_println(stateName);
}

void haltTest(const String& reason) {
  motors.setM1Speed(0);
  motors.setM2Speed(0);
  setHeater(false);
  testHalted = true;
  SerialBLE_print("TEST_HEATER: HALTED=");
  SerialBLE_println(reason);
}

void setup() {
  Serial.begin(115200);
  delay(300);

  pinMode(heaterPin, OUTPUT);
  pinMode(microswitchClosePin, INPUT_PULLUP);
  pinMode(microswitchOpenPin, INPUT_PULLUP);
  pinMode(controlPanelWake, INPUT_PULLUP);
  pinMode(m1CurrentPin, INPUT);
  pinMode(thermistorPin, INPUT);
  pinMode(batteryVoltagePin, INPUT);
  pinMode(M1NFAULT_PIN, INPUT);
  pinMode(M2NFAULT_PIN, INPUT);
  pinMode(M1NEN_PIN, OUTPUT);
  pinMode(M2NEN_PIN, OUTPUT);

  motors.enableDrivers();
  motors.setM1Speed(0);
  motors.setM2Speed(0);
  setHeater(false);

  setupBLE();

  stepStartMs = millis();
  lastTempLogMs = 0;
  SerialBLE_println("TEST_HEATER: START");
  SerialBLE_print("TEST_HEATER: TARGET_TEMP=");
  SerialBLE_println(CUT_MODE_TEMP);
  SerialBLE_println("TEST_HEATER: MIN_BATTERY_PERCENT=30");
}

void logTempPeriodic(const char* prefix, float tempNow) {
  unsigned long now = millis();
  if (now - lastTempLogMs >= TEMP_LOG_INTERVAL_MS) {
    lastTempLogMs = now;
    SerialBLE_print(prefix);
    SerialBLE_print(" TEMP_C=");
    SerialBLE_println(tempNow);
  }
}

void loop() {
  if (testHalted) {
    delay(20);
    return;
  }

  unsigned long now = millis();
  float tempNow = readTemperature();

  // Safety: hard overheat cutout
  if (tempNow > (CUT_MODE_TEMP * HEATER_OVERHEAT_MULTIPLIER)) {
    haltTest("OVERHEAT");
    return;
  }

  switch (testStep) {
    case TEST_CLOSE_TO_STALL: {
      if (stepStartMs == 0) {
        stepStartMs = now;
      }
      int batteryPercent = getBatteryChargeLevel();
      if (batteryPercent < MIN_BATTERY_PERCENT) {
        motors.setM1Speed(0);
        if (now - lastBatteryLogMs >= BATTERY_LOG_INTERVAL_MS) {
          lastBatteryLogMs = now;
          SerialBLE_print("TEST_HEATER: BATTERY_LOW_WAIT PCT=");
          SerialBLE_println(batteryPercent);
        }
        break;
      }
      motors.setM1Speed(M1_TEST_SPEED);
      float m1Current = readM1Current();
      logTempPeriodic("TEST_HEATER:CLOSE", tempNow);

      if (digitalRead(microswitchClosePin) == LOW && m1Current > M1_STALL_CURRENT_A) {
        motors.setM1Speed(0);
        enterStep(TEST_HEAT_TO_TARGET, "HEAT_TO_TARGET");
      } else if (now - stepStartMs > CLOSE_TIMEOUT_MS) {
        haltTest("CLOSE_TIMEOUT");
      }
      break;
    }

    case TEST_HEAT_TO_TARGET: {
      updateHeaterControl(tempNow);
      logTempPeriodic("TEST_HEATER:HEATING", tempNow);
      if (tempNow >= CUT_MODE_TEMP) {
        holdStartMs = now;
        enterStep(TEST_HOLD_5S, "HOLD_5S");
      }
      break;
    }

    case TEST_HOLD_5S: {
      updateHeaterControl(tempNow);
      logTempPeriodic("TEST_HEATER:HOLD", tempNow);
      if (now - holdStartMs >= HOLD_TIME_MS) {
        setHeater(false);
        enterStep(TEST_COOL_TO_90, "COOL_TO_90");
      }
      break;
    }

    case TEST_COOL_TO_90: {
      setHeater(false);
      logTempPeriodic("TEST_HEATER:COOLING_TO_90", tempNow);
      if (tempNow < COOL_OPEN_TEMP) {
        openingExtraPhase = false;
        openExtraStartMs = 0;
        enterStep(TEST_OPEN_WITH_EXTRA_2S, "OPEN_WITH_EXTRA_2S");
      }
      break;
    }

    case TEST_OPEN_WITH_EXTRA_2S: {
      motors.setM1Speed(-M1_TEST_SPEED);
      logTempPeriodic("TEST_HEATER:OPENING", tempNow);

      if (!openingExtraPhase) {
        if (digitalRead(microswitchClosePin) == HIGH) {
          openingExtraPhase = true;
          openExtraStartMs = now;
          SerialBLE_println("TEST_HEATER: OPEN_SWITCH_DETECTED, EXTRA_2S");
        } else if (now - stepStartMs > OPEN_TIMEOUT_MS) {
          haltTest("OPEN_TIMEOUT");
        }
      } else {
        if (now - openExtraStartMs >= OPEN_EXTRA_MS) {
          motors.setM1Speed(0);
          enterStep(TEST_WAIT_TO_65, "WAIT_TO_65");
        }
      }
      break;
    }

    case TEST_WAIT_TO_65: {
      setHeater(false);
      motors.setM1Speed(0);
      logTempPeriodic("TEST_HEATER:WAIT_TO_65", tempNow);
      if (tempNow < RESTART_TEMP) {
        cycleCount++;
        SerialBLE_print("TEST_HEATER: CYCLE_COUNT=");
        SerialBLE_println(cycleCount);
        enterStep(TEST_CLOSE_TO_STALL, "CLOSE_TO_STALL");
      }
      break;
    }
  }

  delay(10);
}
