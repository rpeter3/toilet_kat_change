/*
 * QC Test Program
 * Modified from toilet_kat_change.ino for QC testing with Bluetooth app.
 * Sequence: locate -> thermistor -> heater -> motor -> feed (BLE confirm) -> fan (BLE confirm) -> modified flush
 */

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <DualMAX14870MotorShield.h>

// Pin definitions (from toilet_kat_change.ino)
const int heaterPin = 17;
const int microswitchClosePin = 16;
const int microswitchOpenPin = 15;
const int m1CurrentPin = 8;
const int heaterCurrentPin = 12;
const int thermistorPin = 14;

// Motor shield pins
const uint8_t M1DIR_PIN = 1;
const uint8_t M1PWM_PIN = 42;
const uint8_t M1NEN_PIN = 48;
const uint8_t M1NFAULT_PIN = 40;
const uint8_t M2DIR_PIN = 3;
const uint8_t M2PWM_PIN = 13;
const uint8_t M2NEN_PIN = 9;
const uint8_t M2NFAULT_PIN = 11;

// PWM fan
const int sealerFanPWM = 47;
const int sealerFanPwr = 4;
const int sealerFanReverse = 21;
const int FAN_PWM_FREQ = 25000;
const int FAN_PWM_RESOLUTION = 8;

DualMAX14870MotorShield motors(M1DIR_PIN, M1PWM_PIN, M2DIR_PIN, M2PWM_PIN, M2NEN_PIN, M2NFAULT_PIN);

// BLE UUIDs (same as main firmware for app compatibility)
#define SERVICE_UUID "5636340f-afc7-47b1-b0a8-15bc9d7d29a5"
#define CHARACTERISTIC_UUID "c327b077-560f-46a1-8f35-b4ab0332fea0"
#define RESPONSE_CHARACTERISTIC_UUID "c327b077-560f-46a1-8f35-b4ab0332fea4"
#define SERIAL_CHARACTERISTIC_UUID "c327b077-560f-46a1-8f35-b4ab0332fea1"

BLECharacteristic* blue_characteristic = nullptr;
BLECharacteristic* response_characteristic = nullptr;
BLECharacteristic* serial_characteristic = nullptr;
BLEServer* blue_server = nullptr;
bool bleConnected = false;
bool serialStreamingEnabled = false;

// Thermistor (from main firmware)
const float knownResistor = 10000.0f;
const float A = 2.7807099250E-03f;
const float B = 2.4294397063E-04f;
const float C = 1.0810891036E-06f;
const float THERMISTOR_VOLTAGE_GUARD_V = 0.01f;
const float HARDWARE_DISCONNECT_RESISTANCE_OHMS = 100000.0f;

// QC constants
const unsigned long TIMEOUT_MS = 15000;
const float M1_STALL_CURRENT_A = 0.5f;
const float QC_HEAT_TARGET_C = 80.0f;
const float QC_COOL_TARGET_C = 60.0f;
const int HEATER_ADC_THRESHOLD = 200;

// QC state machine
enum QCStep {
  QC_IDLE,        // Waiting for START_TEST over BLE
  QC_LOCATE,
  QC_THERMISTOR,
  QC_HEATER,
  QC_MOTOR,
  QC_FEED_WAIT,
  QC_FAN_WAIT,
  QC_FLUSH_CLOSE,
  QC_FLUSH_EXTRA_1S,
  QC_FLUSH_HEAT,
  QC_FLUSH_COOL,
  QC_FLUSH_OPEN,
  QC_DONE,
  QC_HALTED
};

QCStep qcStep = QC_IDLE;
bool qcHalted = false;
String qcHaltReason = "";
unsigned long stepStartMs = 0;
bool heaterOutputOn = false;

void sendSerialToBLE(const String& msg) {
  if (bleConnected && serial_characteristic) {
    serial_characteristic->setValue(msg.c_str());
    serial_characteristic->notify();
  }
}

void setBleResponse(const String& rsp) {
  if (response_characteristic) {
    response_characteristic->setValue(rsp.c_str());
    response_characteristic->notify();
  }
}

#define SerialBLE_print(x) do { Serial.print(x); if (serialStreamingEnabled) sendSerialToBLE(String(x)); } while (0)
#define SerialBLE_println(x) do { Serial.println(x); if (serialStreamingEnabled) sendSerialToBLE(String(x) + "\n"); } while (0)

// Temperature
float readTemperature() {
  int analogValue = analogRead(thermistorPin);
  float voltage = analogValue * (3.3f / 4095.0f);
  if (voltage <= THERMISTOR_VOLTAGE_GUARD_V || voltage >= (3.3f - THERMISTOR_VOLTAGE_GUARD_V)) {
    return -100.0f;
  }
  float resistance = (voltage * knownResistor) / (3.3f - voltage);
  float logR = log(resistance / 1000.0f);
  float tempKelvin = 1.0f / (A + B * logR + C * logR * logR * logR);
  return tempKelvin - 273.15f;
}

float readMainThermistorResistanceOhms() {
  int analogValue = analogRead(thermistorPin);
  float voltage = analogValue * (3.3f / 4095.0f);
  if (voltage <= THERMISTOR_VOLTAGE_GUARD_V) return 0.0f;
  if (voltage >= (3.3f - THERMISTOR_VOLTAGE_GUARD_V)) return 1000000000.0f;
  return (voltage * knownResistor) / (3.3f - voltage);
}

float readM1Current() {
  int analogValue = analogRead(m1CurrentPin);
  float voltage = analogValue * (3.3f / 4095.0f);
  return voltage / 2.0f;
}

void setFanSpeed(int speed) {
  if (speed > 0) {
    digitalWrite(sealerFanPwr, HIGH);
    digitalWrite(sealerFanReverse, LOW);
    ledcWrite(sealerFanPWM, 255);
  } else if (speed < 0) {
    digitalWrite(sealerFanPwr, HIGH);
    digitalWrite(sealerFanReverse, HIGH);
    ledcWrite(sealerFanPWM, 255);
  } else {
    digitalWrite(sealerFanPwr, LOW);
    ledcWrite(sealerFanPWM, 0);
    digitalWrite(sealerFanReverse, LOW);
  }
}

void heaterOff() {
  digitalWrite(heaterPin, LOW);
  heaterOutputOn = false;
}

void haltQC(const String& reason) {
  qcHalted = true;
  qcHaltReason = reason;
  qcStep = QC_HALTED;
  motors.setM1Speed(0);
  motors.setM2Speed(0);
  setFanSpeed(0);
  heaterOff();
  SerialBLE_print("QC_HALTED: ");
  SerialBLE_println(reason);
  setBleResponse("QC_HALTED:" + reason);
}

bool testHeaterCurrent() {
  SerialBLE_println("QC: Heater current test...");
  digitalWrite(heaterPin, HIGH);
  heaterOutputOn = true;

  unsigned long start = millis();
  unsigned long lastCheck = 0;
  const unsigned long duration = 500;
  const unsigned long interval = 50;
  bool detected = false;

  while (millis() - start < duration) {
    if (millis() - lastCheck >= interval) {
      lastCheck = millis();
      int adc = analogRead(heaterCurrentPin);
      if (adc > HEATER_ADC_THRESHOLD) {
        detected = true;
        break;
      }
    }
    delay(10);
  }

  heaterOff();
  if (!detected) {
    SerialBLE_println("QC: Heater current FAILED");
    return false;
  }
  SerialBLE_println("QC: Heater current PASSED");
  return true;
}

void locateMotorPos() {
  SerialBLE_println("QC: Locating motor position...");
  bool motorOpenSwitchClosed = (digitalRead(microswitchOpenPin) == LOW);

  if (motorOpenSwitchClosed) {
    motors.setM1Speed(400);
    unsigned long start = millis();
    while (digitalRead(microswitchOpenPin) == LOW) {
      if (millis() - start > TIMEOUT_MS) {
        motors.setM1Speed(0);
        haltQC("LOCATE_CLOSE_TIMEOUT");
        return;
      }
      delay(10);
    }
    delay(500);
    motors.setM1Speed(0);
  }

  motors.setM1Speed(-400);
  unsigned long start = millis();
  while (digitalRead(microswitchOpenPin) == HIGH) {
    if (millis() - start > TIMEOUT_MS) {
      motors.setM1Speed(0);
      haltQC("LOCATE_OPEN_TIMEOUT");
      return;
    }
    delay(10);
  }
  motors.setM1Speed(0);
  SerialBLE_println("QC: Locate complete");
}

void updateHeaterControlQC(float currentTemp, float targetC) {
  const float lower = targetC - 2.0f;
  const float upper = targetC;
  if (!heaterOutputOn && currentTemp < lower) {
    digitalWrite(heaterPin, HIGH);
    heaterOutputOn = true;
  } else if (heaterOutputOn && currentTemp >= upper) {
    digitalWrite(heaterPin, LOW);
    heaterOutputOn = false;
  }
}

const char* qcStepName(QCStep s) {
  switch (s) {
    case QC_IDLE: return "IDLE";
    case QC_LOCATE: return "LOCATE";
    case QC_THERMISTOR: return "THERMISTOR";
    case QC_HEATER: return "HEATER";
    case QC_MOTOR: return "MOTOR";
    case QC_FEED_WAIT: return "FEED_WAIT";
    case QC_FAN_WAIT: return "FAN_WAIT";
    case QC_FLUSH_CLOSE: return "FLUSH_CLOSE";
    case QC_FLUSH_EXTRA_1S: return "FLUSH_EXTRA_1S";
    case QC_FLUSH_HEAT: return "FLUSH_HEAT";
    case QC_FLUSH_COOL: return "FLUSH_COOL";
    case QC_FLUSH_OPEN: return "FLUSH_OPEN";
    case QC_DONE: return "DONE";
    case QC_HALTED: return "HALTED";
    default: return "?";
  }
}

// BLE command handler
class CharacteristicCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pCh) override {
    if (pCh->getUUID().toString() != CHARACTERISTIC_UUID) return;
    int len = pCh->getLength();
    if (len <= 0) return;
    String cmd = String((char*)pCh->getData(), (unsigned int)len);
    cmd.trim();

    if (cmd == "START_TEST") {
      if (qcStep != QC_IDLE) {
        setBleResponse("QC_ERR:TEST_ALREADY_RUNNING");
        return;
      }
      qcStep = QC_LOCATE;
      setBleResponse("QC_STATUS:LOCATE");
      locateMotorPos();
      if (qcHalted) return;
      qcStep = QC_THERMISTOR;
      stepStartMs = millis();
      SerialBLE_println("QC: Thermistor check...");
      return;
    }
    if (cmd == "QC_CONFIRM_FEED") {
      if (qcStep == QC_FEED_WAIT) {
        motors.setM2Speed(0);
        qcStep = QC_FAN_WAIT;
        stepStartMs = millis();
        setFanSpeed(-400);
        setBleResponse("QC_CONFIRM_FEED_ACK");
        SerialBLE_println("QC: Feed confirmed, starting fan reverse");
      } else {
        setBleResponse("QC_ERR:NOT_IN_FEED_STEP");
      }
      return;
    }
    if (cmd == "QC_CONFIRM_FAN") {
      if (qcStep == QC_FAN_WAIT) {
        setFanSpeed(0);
        qcStep = QC_FLUSH_CLOSE;
        stepStartMs = millis();
        motors.setM1Speed(400);
        setBleResponse("QC_CONFIRM_FAN_ACK");
        SerialBLE_println("QC: Fan confirmed, starting flush close");
      } else {
        setBleResponse("QC_ERR:NOT_IN_FAN_STEP");
      }
      return;
    }
    if (cmd == "QC_STATUS") {
      String rsp = String("QC_STATUS:") + qcStepName(qcStep);
      if (qcHalted) rsp += ",HALTED:" + qcHaltReason;
      setBleResponse(rsp);
      return;
    }
    if (cmd == "START_SERIAL") {
      serialStreamingEnabled = true;
      setBleResponse("SERIAL_ON");
      return;
    }
    if (cmd == "STOP_SERIAL") {
      serialStreamingEnabled = false;
      setBleResponse("SERIAL_OFF");
      return;
    }
    setBleResponse("CMD_ERR:UNKNOWN");
  }
};

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) override {
    bleConnected = true;
    serialStreamingEnabled = true;
    sendSerialToBLE("QC_TEST: BLE connected\n");
  }
  void onDisconnect(BLEServer* pServer) override {
    bleConnected = false;
    serialStreamingEnabled = false;
    delay(200);
    pServer->startAdvertising();
  }
};

void setupBLE() {
  BLEDevice::init("QC_Test");
  blue_server = BLEDevice::createServer();
  blue_server->setCallbacks(new ServerCallbacks());

  BLEService* svc = blue_server->createService(SERVICE_UUID);

  blue_characteristic = svc->createCharacteristic(
    CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE
  );
  blue_characteristic->setCallbacks(new CharacteristicCallbacks());

  response_characteristic = svc->createCharacteristic(
    RESPONSE_CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
  );
  response_characteristic->setValue("QC_READY");

  serial_characteristic = svc->createCharacteristic(
    SERIAL_CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY
  );
  serial_characteristic->setValue("QC_TEST ready");

  svc->start();
  BLEAdvertising* adv = blue_server->getAdvertising();
  adv->addServiceUUID(SERVICE_UUID);
  adv->setScanResponse(true);
  adv->setMinPreferred(0x06);
  adv->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("QC_TEST starting");

  pinMode(heaterPin, OUTPUT);
  pinMode(microswitchClosePin, INPUT_PULLUP);
  pinMode(microswitchOpenPin, INPUT_PULLUP);
  pinMode(m1CurrentPin, INPUT);
  pinMode(heaterCurrentPin, INPUT);
  pinMode(thermistorPin, INPUT);
  pinMode(sealerFanPwr, OUTPUT);
  pinMode(sealerFanReverse, OUTPUT);
  pinMode(M1NFAULT_PIN, INPUT);
  pinMode(M2NFAULT_PIN, INPUT);
  pinMode(M1NEN_PIN, OUTPUT);
  pinMode(M2NEN_PIN, OUTPUT);

  ledcAttach(sealerFanPWM, FAN_PWM_FREQ, FAN_PWM_RESOLUTION);
  setFanSpeed(0);
  heaterOff();

  setupBLE();

  qcStep = QC_IDLE;
  setBleResponse("QC_READY");
  Serial.println("QC_TEST: Waiting for START_TEST over BLE");
}

void loop() {
  if (qcHalted) {
    delay(100);
    return;
  }
  if (qcStep == QC_IDLE) {
    delay(100);
    return;
  }

  unsigned long now = millis();

  switch (qcStep) {
    case QC_THERMISTOR: {
      float r = readMainThermistorResistanceOhms();
      if (r > HARDWARE_DISCONNECT_RESISTANCE_OHMS) {
        haltQC("THERMISTOR_DISCONNECTED");
        return;
      }
      SerialBLE_println("QC: Thermistor OK");
      qcStep = QC_HEATER;
      break;
    }

    case QC_HEATER: {
      if (!testHeaterCurrent()) {
        haltQC("HEATER_CURRENT_FAIL");
        return;
      }
      qcStep = QC_MOTOR;
      break;
    }

    case QC_MOTOR: {
      motors.enableDrivers();
      delay(100);
      if (motors.getFault()) {
        haltQC("MOTOR_FAULT");
        return;
      }
      SerialBLE_println("QC: Motor OK");
      qcStep = QC_FEED_WAIT;
      stepStartMs = now;
      motors.setM2Speed(-400);
      SerialBLE_println("QC: Feed motor ON - confirm via BLE QC_CONFIRM_FEED");
      break;
    }

    case QC_FEED_WAIT: {
      if (now - stepStartMs > TIMEOUT_MS) {
        motors.setM2Speed(0);
        haltQC("FEED_CONFIRM_TIMEOUT");
      }
      break;
    }
    case QC_FAN_WAIT: {
      if (now - stepStartMs > TIMEOUT_MS) {
        setFanSpeed(0);
        haltQC("FAN_CONFIRM_TIMEOUT");
      }
      break;
    }

    case QC_FLUSH_CLOSE: {
      float m1 = readM1Current();
      if (digitalRead(microswitchClosePin) == LOW && m1 > M1_STALL_CURRENT_A) {
        motors.setM1Speed(0);
        qcStep = QC_FLUSH_EXTRA_1S;
        stepStartMs = now;
        SerialBLE_println("QC: Close complete, extra 1s");
      } else if (now - stepStartMs > TIMEOUT_MS) {
        motors.setM1Speed(0);
        haltQC("FLUSH_CLOSE_TIMEOUT");
      }
      break;
    }

    case QC_FLUSH_EXTRA_1S: {
      motors.setM1Speed(400);
      if (now - stepStartMs >= 1000) {
        motors.setM1Speed(0);
        qcStep = QC_FLUSH_HEAT;
        stepStartMs = now;
        SerialBLE_println("QC: Heating to 80C");
      }
      break;
    }

    case QC_FLUSH_HEAT: {
      float temp = readTemperature();
      updateHeaterControlQC(temp, QC_HEAT_TARGET_C);
      if (temp >= QC_HEAT_TARGET_C) {
        heaterOff();
        qcStep = QC_FLUSH_COOL;
        stepStartMs = now;
        SerialBLE_println("QC: Cooling to 60C");
      }
      break;
    }

    case QC_FLUSH_COOL: {
      heaterOff();
      float temp = readTemperature();
      if (temp <= QC_COOL_TARGET_C) {
        qcStep = QC_FLUSH_OPEN;
        stepStartMs = now;
        motors.setM1Speed(-400);
        SerialBLE_println("QC: Opening");
      }
      break;
    }

    case QC_FLUSH_OPEN: {
      if (digitalRead(microswitchOpenPin) == LOW) {
        motors.setM1Speed(0);
        qcStep = QC_DONE;
        SerialBLE_println("QC: DONE - all tests passed");
        setBleResponse("QC_DONE");
      } else if (now - stepStartMs > TIMEOUT_MS) {
        motors.setM1Speed(0);
        haltQC("FLUSH_OPEN_TIMEOUT");
      }
      break;
    }

    case QC_DONE:
      break;

    default:
      break;
  }

  delay(10);
}
