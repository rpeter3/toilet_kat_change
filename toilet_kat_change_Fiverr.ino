#include <BLEServer.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <Wire.h>
#include <Adafruit_MCP23X17.h>
#include <DualMAX14870MotorShield.h> // Include the motor driver library
#include <PID_v1_bc.h>  // Include the PID library
#include <EEPROM.h>     // Include EEPROM library for parameter persistence
#include <esp_ota_ops.h>  // Include OTA operations for updates
#include <nvs_flash.h>   // Include NVS for rollback state storage
#include <esp_system.h>  // Include for reboot functionality
#include <mbedtls/md5.h> // Include for MD5 validation


// Pin and component definitions
const int heaterPin = 17;        // GPIO17 (HEATER)

const int buzzerPin = 38;        // GPIO38 (BUZ)
const int microswitchClosePin = 19;   // GPIO42 (RXD0)
const int microswitchOpenPin = 20;    // GPIO43 (TXD0)

// Motor current monitoring
const int m1CurrentPin = 8;     // GPIO8 (M1_A) - Current sense output

// Heater current monitoring
const int heaterCurrentPin = 12; // GPIO12 (HS_OUT) - Heater current sense output from INA169

// Battery voltage monitoring
const int batteryVoltagePin = 10;  // GPIO10 (VMON) - Battery voltage monitoring
const int batteryTempPin = 11;     // GPIO11 (B_TEMP) - Battery temperature monitoring

// Custom I2C instance
TwoWire myI2C = TwoWire(0);

// MCP23017 setup
Adafruit_MCP23X17 mcp;

#define SERVICE_UUID "5636340f-afc7-47b1-b0a8-15bc9d7d29a5"
#define CHARACTERISTIC_UUID "c327b077-560f-46a1-8f35-b4ab0332fea0"
#define SERIAL_CHARACTERISTIC_UUID "c327b077-560f-46a1-8f35-b4ab0332fea1"
#define VERSION_CHARACTERISTIC_UUID "c327b077-560f-46a1-8f35-b4ab0332fea2"
#define UPDATE_SERVICE_UUID "5636340f-afc7-47b1-b0a8-15bcb9d7d29a6"
#define UPDATE_CHARACTERISTIC_UUID "c327b077-560f-46a1-8f35-b4ab0332fea3"

// BLE Server global variables
BLECharacteristic * blue_characteristic;
BLECharacteristic * serial_characteristic;
BLECharacteristic * version_characteristic;
BLECharacteristic * update_characteristic;
BLEServer * blue_server;
BLEService * update_service;
bool is_device_connected, old_device_connect = false;
bool serial_streaming_enabled = false;

// EEPROM configuration
#define EEPROM_SIZE 512
#define PARAM_START_ADDR 0
#define PARAM_MAGIC_NUMBER 0x1234
#define HW_VERSION_ADDR 200
#define HW_VERSION_MAGIC 0xFADE

// Version information
struct VersionInfo {
  uint16_t magic;
  uint16_t hardware_version;
  char hardware_description[32];
};

const char* SOFTWARE_VERSION = "2.1.0";
const char* SOFTWARE_BUILD_DATE = __DATE__ " " __TIME__;

// Forward declarations
void sendSerialToBLE(const String& message);
void sendSerialToBLE(float value);
void sendSerialToBLE(int value);
void sendSerialToBLE(unsigned long value);
void saveParametersToEEPROM();
void loadParametersFromEEPROM();
void initializeHardwareVersion();
VersionInfo readHardwareVersion();
void writeHardwareVersion(uint16_t version, const char* description);
String getVersionString();
bool prepareForOTA();
void notifyUpdateProgress(int percentage);
bool rollbackOTAUpdate();
bool validateFirmware();
void handleOTAChunk(uint8_t* data, size_t length);
void resetOTAState();
void checkBootFailure();
void saveRollbackInfo();
void enableOTA();
void disableOTA();
void restartBLEServer();
void slowCircleLeds();

// Macros to automatically forward Serial output to BLE when streaming is enabled
#define SerialBLE_print(x) do { Serial.print(x); if(serial_streaming_enabled) sendSerialToBLE(x); } while(0)
#define SerialBLE_println(x) do { Serial.println(x); if(serial_streaming_enabled) sendSerialToBLE(String(x) + "\n"); } while(0)
#define SerialBLE_println_empty() do { Serial.println(); if(serial_streaming_enabled) sendSerialToBLE("\n"); } while(0)

// Button pins (MCP23017 side)
const int button1Pin = 7;
const int button2Pin = 15;

// LED pins
const int ledPins[] = {9, 13, 14, 10, 1, 6, 11, 12, 8, 0, 2, 3, 4, 5};
const int totalLeds = sizeof(ledPins) / sizeof(ledPins[0]);
int ledIndex = 0;
bool clockwise = true; // Direction flag for LED cycling

// Timing variables
unsigned long previousMillis = 0;
unsigned long flushStartMillis = 0;
unsigned long processStartMillis = 0;
unsigned long ledLastUpdateMillis = 0;
unsigned long motorStartMillis = 0;
bool mechanismMotorRunning = false;
bool heaterOn = false;
unsigned long stepStartMillis = 0;

// Temperature setup
const int thermistorPin = 14;    // GPIO14 (IO14)
float knownResistor = 10000.0;  // Will be initialized from thermistorResistance parameter
//float knownResistor = 33000.0;  // Will be initialized from thermistorResistance parameter

const float A = 0.001129148; //for 10k Ohm
const float B = 0.000234125;   // for 10k ohm
const float C = 0.0000000876741;//for 10K ohm
//const float A = 0.003306387; //for 33k Ohm
//const float B = 0.000305051;   // for 3k ohm
//const float C = 0.000000010;//for 10K ohm


/* Parameters 
// I think these are the things Richard wants changed
const int batteryThreshold = -1; //parameters_list[0]
const float K = 60.0; //parameters_list[1]
const int F = 5; //parameters_list[2]
const long T = 50; //parameters_list[3]
const int r1 = 5; //parameters_list[4]
const int r2 = 2; //parameters_list[5]
const int backupTime = 2; //parameters_list[6]
const int r4 = 2; //parameters_list[7]
const int fanDuration = 5; //parameters_list[8]
const long H = 50; //parameters_list[9]
const float continueFeeder = 6; //parameters_list[10]
const int maxOpeningTime = 10; //parameters_list[11]
const int typicalOpeningTime = 5; //parameters_list[12] */

// Parameters
// Can't have global constant variables in C
int batteryThreshold = 5; //parameters_list[0]
float K = 120.0; //parameters_list[1] - Updated to match 1mil High Barrier Plastic
int F = 6; //parameters_list[2]
long T = 60; //parameters_list[3] - Cooling Time (Seconds) - Updated to match 1mil High Barrier Plastic
float thermistorResistance = 10000.0; //parameters_list[4] - Thermistor Resistance (Ohms)
int r2 = 2; //parameters_list[5]
float backupTime = 1.7; //parameters_list[6]
int r4 = 2; //parameters_list[7]
int fanDuration = 5; //parameters_list[8]
long H = 40; //parameters_list[9] - Heater On time (Seconds) - Updated to match 1mil High Barrier Plastic
float continueFeeder = 7.0; //parameters_list[10]
int maxOpeningTime = 12; //parameters_list[11]
int typicalOpeningTime = 10; //parameters_list[12]
float MOTOR_CUT_TIME = 0.5; //parameters_list[13]
float CUT_MODE_HEAT_TIME = 25.0; //parameters_list[14] - Updated to match 1mil High Barrier Plastic
float postCoolingBagDuration = 5.0; //parameters_list[15]
float preFeedFan = 3.0; //parameters_list[16]
float fanReverseTime = 3.0; //parameters_list[17]
float fanReverseStartTime = 0.0; //parameters_list[18]
float backupTimeAfterReopen = 1.7; //parameters_list[19]

bool isFlushing = false;
int flushStep = 0;
bool case5FeedExecuted = false; // Flag to prevent multiple M2 activations in case 5
bool case1FeedStarted = false; // Flag to track if M2 has started in case 1
bool button2FeedStarted = false; // Flag to track if M2 has started when button 2 is pressed
unsigned long button2FanStartTime = 0; // Timestamp when fan starts for button 2
bool case10FanStarted = false; // Flag to track if fan has started in case 10
bool case10BackupStarted = false; // Flag to track if backup has started in case 10
unsigned long case10BackupStartTime = 0; // Track when backup starts in case 10
unsigned long m1CloseStartTime = 0; // Track when M1 starts closing
unsigned long m3ReverseStartTime = 0; // Track when M3 reverse starts
bool m3ReverseActive = false; // Track if M3 reverse is running
bool m3ReverseCompleted = false; // Track if M3 reverse has completed its cycle

// Cut bag functionality
bool cutBag = false;
bool button1Held = false;
unsigned long button1HoldStartTime = 0;
const int BUTTON_HOLD_TIME = 1500; // 1.5 seconds
const int BUTTON_DELAY = 200; // 200ms delay before acting on single button press
unsigned long cutStartTime = 0;
unsigned long fanStartTime = 0;  // Timer for fan duration after feed button release
bool fanRunning = false;         // Flag to track if fan is running
bool cutMotorRunning = false;

// Button state tracking
bool button2WasPressed = false;
bool button1WasPressed = false;
unsigned long button1PressStartTime = 0;
unsigned long button2PressStartTime = 0;
bool button1DelayActive = false;
bool button2DelayActive = false;
bool bothButtonsPressed = false;
bool batteryDisplayMode = false;
unsigned long batteryDisplayStartTime = 0;

// BLE auto-shutdown variables
unsigned long bleStartupTime = 0;
const unsigned long BLE_TIMEOUT = 10 * 60 * 1000; // 10 minutes in milliseconds
  bool bleEnabled = true;

// OTA timing variables
bool otaEnabled = false;
unsigned long batteryMonitorStartTime = 0;
unsigned long otaWindowStartTime = 0;
bool batteryMonitoringActive = false;
const unsigned long BATTERY_MONITOR_DURATION = 10000; // 10 seconds
const unsigned long OTA_WINDOW_DURATION = 60000; // 1 minute
unsigned long lastBatteryCheckTime = 0;
const unsigned long BATTERY_CHECK_INTERVAL = 1000; // Check battery every 1 second

// Update system variables
bool updateInProgress = false;
int updateProgress = 0;

// OTA State Management
enum OTAState {
  OTA_IDLE,
  OTA_PREPARING,
  OTA_RECEIVING,
  OTA_VALIDATING,
  OTA_FINALIZING,
  OTA_ERROR,
  OTA_ROLLBACK
};

OTAState otaState = OTA_IDLE;
esp_ota_handle_t ota_handle = 0;
const esp_partition_t *ota_partition = NULL;
const esp_partition_t *running_partition = NULL;
const esp_partition_t *update_partition = NULL;
size_t firmware_size = 0;
size_t bytes_received = 0;
uint8_t expected_md5[16] = {0};
uint8_t calculated_md5[16] = {0};
bool rollback_required = false;
String ota_error_message = "";

// OTA Chunk buffer (BLE MTU is typically 20-512 bytes, we'll use 512 for safety)
#define OTA_CHUNK_SIZE 512
uint8_t ota_chunk_buffer[OTA_CHUNK_SIZE];
size_t chunk_sequence = 0;
bool md5_received = false;
mbedtls_md5_context md5_ctx;
bool md5_initialized = false;

// LED timing variables
unsigned long totalSequenceTime = 0;
unsigned long ledUpdateInterval = 0;
unsigned long slowCircleLastUpdate = 0;
int slowCircleLedIndex = 0;
const unsigned long SLOW_CIRCLE_INTERVAL = 300; // 300ms per LED for slow circle

const unsigned int TIMEOUT = 15000;

int ERROR_CODE = 0;
//ERROR_CODE = 1 ==> Mechanism Motor Timeout
//ERROR_CODE = 2 ==> Low battery
//ERROR_CODE = 3 ==> Heater too hot 
//ERROR_CODE = 4 ==> Motor Fault Detected
//ERROR_CODE = 5 ==> Heater current detection failure

// Motor Driver Definitions
const uint8_t M1DIR_PIN = 1;     // GPIO1 (M1DIR)
const uint8_t M1PWM_PIN = 42;    // GPIO42 (M1PWM)
const uint8_t M1NEN_PIN = 25;    // GPIO25 (M1NEN)
const uint8_t M1NFAULT_PIN = 40; // GPIO40 (M1NFAULT)

const uint8_t M2DIR_PIN = 3;     // GPIO3 (M2DIR)
const uint8_t M2PWM_PIN = 13;    // GPIO13 (M2PWM)
const uint8_t M2NEN_PIN = 9;     // GPIO9 (M2NEN)
const uint8_t M2NFAULT_PIN = 11; // GPIO11 (M2NFAULT)

const uint8_t M3DIR_PIN = 2;     // GPIO2 (M3DIR)
const uint8_t M3PWM_PIN = 41;    // GPIO41 (M3PWM)
const uint8_t M3NEN_PIN = 47;    // GPIO47 (M3NEN)
const uint8_t M3NFAULT_PIN = 39; // GPIO39 (M3NFAULT)

// Motor shield instances - M1&M2 on first shield, M3 on second shield
DualMAX14870MotorShield motors(M1DIR_PIN, M1PWM_PIN, M2DIR_PIN, M2PWM_PIN, M2NEN_PIN, M2NFAULT_PIN);
DualMAX14870MotorShield motors3(M3DIR_PIN, M3PWM_PIN, M3DIR_PIN, M3PWM_PIN, M3NEN_PIN, M3NFAULT_PIN);

// PID
double setpoint = K;
double input, output;
double Kp = 100.0, Ki = 0.01, Kd = 0.1;
PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

// Setup BLE callbacks called onConnect and onDisconnect
class server_callbacks: public BLEServerCallbacks {
  void onConnect(BLEServer * blue_server) {
    is_device_connected = true;
    Serial.println("Device connected!");
    sendSerialToBLE("BLE Device Connected!");
    
    // Update characteristic with current parameters
    String paramString = String(batteryThreshold) + "," +
                        String(K) + "," +
                        String(F) + "," +
                        String(T) + "," +
                        String(thermistorResistance) + "," +
                        String(r2) + "," +
                        String(backupTime) + "," +
                        String(r4) + "," +
                        String(fanDuration) + "," +
                        String(H) + "," +
                        String(continueFeeder) + "," +
                        String(maxOpeningTime) + "," +
                        String(typicalOpeningTime) + "," +
                        String(MOTOR_CUT_TIME) + "," +
                        String(CUT_MODE_HEAT_TIME) + "," +
                        String(postCoolingBagDuration) + "," +
                        String(preFeedFan) + "," +
                        String(fanReverseTime) + "," +
                        String(fanReverseStartTime) + "," +
                        String(backupTimeAfterReopen);
    blue_characteristic->setValue(paramString.c_str());
    Serial.printf("Characteristic updated on connect: %s\n", paramString.c_str());
    SerialBLE_println("Characteristic updated on connect");
  }

  void onDisconnect(BLEServer * blue_server) {
    is_device_connected = false;
    serial_streaming_enabled = false;
    Serial.println("Device disconnected!");

    // Receive message
    int message_length = blue_characteristic->getLength();
    Serial.printf("Length of message: %d\n", message_length);
    unsigned char* message = blue_characteristic->getData();
    Serial.printf("Message: ");
    for (int i = 0; i < message_length; i++) {
      Serial.printf("%c", message[i]);
    }

    // Decode message by splitting the string to get the delay time (as an INTEGER) for each LED
    // Splits the string on the first comma and keeps all the characters BEFORE the comma
    char * parameters_string = strtok((char*) message, ",");
    int parameters_list[100];
    int k = 0;
    while (parameters_string != NULL) {
      bool float_check = false;
      for (int i = 0; i <= strlen(parameters_string); i++) {
        if (parameters_string[i] == '.') {
          const float float_parameter = atof(parameters_string);
          Serial.printf("\n%f", float_parameter);
          parameters_list[k] = float_parameter;
          float_check = true;
        }
      }
      if (float_check == false) {
        char *endptr;
        // Converts a char to an int
        const int new_parameter = strtol(parameters_string, &endptr, 10);
        if ((new_parameter < 0) && (k != 0)) {
          Serial.printf("Delay time is not a valid integer. This time will not be accepted.");
        }
        else {
          // Appends each integer delay_time to a list
          parameters_list[k] = new_parameter;
        }
      }
      // Splits the remaining string on the next comma keeping the first segment again
      parameters_string = strtok(NULL, ",");
      k++;
      float_check = false;
    }
    for (int j = 0; j < k; j++) {
      Serial.printf("\n%d", parameters_list[j]);
    }
    batteryThreshold = parameters_list[0];
    K = parameters_list[1];
    F = parameters_list[2];
    T = parameters_list[3];
    thermistorResistance = parameters_list[4];
    knownResistor = thermistorResistance;  // Update knownResistor when parameter changes
    r2 = parameters_list[5];
    backupTime = parameters_list[6];
    r4 = parameters_list[7];
    fanDuration = parameters_list[8];
    H = parameters_list[9];
    continueFeeder = parameters_list[10];
    maxOpeningTime = parameters_list[11];
    typicalOpeningTime = parameters_list[12];
    MOTOR_CUT_TIME = parameters_list[13];
    CUT_MODE_HEAT_TIME = parameters_list[14];
    postCoolingBagDuration = parameters_list[15];
    preFeedFan = parameters_list[16];
    fanReverseTime = parameters_list[17];
    fanReverseStartTime = parameters_list[18];
    backupTimeAfterReopen = parameters_list[19];
    
    // Update PID setpoint when K changes
    setpoint = K;
    
    // Save parameters to EEPROM for persistence
    saveParametersToEEPROM();
    
    Serial.printf("Parameters updated! H=%ld, K=%.1f\n", H, K);
  }
};

// BLE Characteristic callback for OTA updates
class update_characteristic_callbacks: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    if (pCharacteristic->getUUID().toString() == UPDATE_CHARACTERISTIC_UUID) {
      // Check if OTA is enabled - reject all operations if disabled
      if (!otaEnabled) {
        update_characteristic->setValue("OTA_DISABLED");
        update_characteristic->notify();
        Serial.println("OTA request rejected - OTA mode not enabled");
        return;
      }
      
      int message_length = pCharacteristic->getLength();
      if (message_length > 0) {
        uint8_t* data = pCharacteristic->getData();
        
        // Handle OTA chunks
        if (otaState == OTA_RECEIVING) {
          handleOTAChunk(data, message_length);
          return;
        }
        
        // Handle commands
        String command = String((char*)data);
        command.trim();
        
        Serial.print("DEBUG: Received update command: '");
        Serial.print(command);
        Serial.println("'");
        
        if (command == "CHECK_VERSION") {
          String versionInfo = getVersionString();
          version_characteristic->setValue(versionInfo.c_str());
          version_characteristic->notify();
          SerialBLE_println("Version check requested");
        } else if (command == "PREPARE_UPDATE") {
          if (prepareForOTA()) {
            update_characteristic->setValue("UPDATE_PREPARED");
            update_characteristic->notify();
            SerialBLE_println("Device prepared for update");
          } else {
            update_characteristic->setValue("UPDATE_BLOCKED");
            update_characteristic->notify();
            SerialBLE_println("Update preparation blocked");
          }
        } else if (command == "START_UPDATE") {
          if (otaState == OTA_PREPARING) {
            // Initialize OTA
            esp_err_t err = esp_ota_begin(update_partition, OTA_SIZE_UNKNOWN, &ota_handle);
            if (err != ESP_OK) {
              Serial.printf("ERROR: esp_ota_begin failed: %s\n", esp_err_to_name(err));
              otaState = OTA_ERROR;
              ota_error_message = "OTA begin failed: " + String(esp_err_to_name(err));
              update_characteristic->setValue("UPDATE_ERROR");
              update_characteristic->notify();
              return;
            }
            
            otaState = OTA_RECEIVING;
            update_characteristic->setValue("UPDATE_STARTED");
            update_characteristic->notify();
            SerialBLE_println("OTA update started");
            notifyUpdateProgress(0);
          } else {
            update_characteristic->setValue("UPDATE_NOT_PREPARED");
            update_characteristic->notify();
            SerialBLE_println("Update not prepared");
          }
        } else if (command == "FINALIZE_UPDATE") {
          if (otaState == OTA_VALIDATING) {
            // Validate firmware
            if (!validateFirmware()) {
              SerialBLE_println("Firmware validation failed, rolling back...");
              otaState = OTA_ERROR;
              rollback_required = true;
              
              // Abort OTA
              esp_ota_abort(ota_handle);
              ota_handle = 0;
              
              update_characteristic->setValue("UPDATE_VALIDATION_FAILED");
              update_characteristic->notify();
              
              // Attempt rollback
              if (rollbackOTAUpdate()) {
                SerialBLE_println("Rollback successful, rebooting...");
                delay(1000);
                esp_restart();
              }
              return;
            }
            
            // Finalize OTA
            otaState = OTA_FINALIZING;
            esp_err_t err = esp_ota_end(ota_handle);
            if (err != ESP_OK) {
              Serial.printf("ERROR: esp_ota_end failed: %s\n", esp_err_to_name(err));
              otaState = OTA_ERROR;
              ota_error_message = "OTA end failed: " + String(esp_err_to_name(err));
              update_characteristic->setValue("UPDATE_ERROR");
              update_characteristic->notify();
              return;
            }
            
            // Set boot partition
            err = esp_ota_set_boot_partition(update_partition);
            if (err != ESP_OK) {
              Serial.printf("ERROR: esp_ota_set_boot_partition failed: %s\n", esp_err_to_name(err));
              otaState = OTA_ERROR;
              ota_error_message = "Set boot partition failed: " + String(esp_err_to_name(err));
              update_characteristic->setValue("UPDATE_ERROR");
              update_characteristic->notify();
              return;
            }
            
            update_characteristic->setValue("UPDATE_COMPLETE");
            update_characteristic->notify();
            SerialBLE_println("OTA update completed successfully, rebooting...");
            notifyUpdateProgress(100);
            
            delay(1000);
            esp_restart();
          } else {
            update_characteristic->setValue("UPDATE_NOT_READY");
            update_characteristic->notify();
            SerialBLE_println("Update not ready for finalization");
          }
        } else {
          Serial.print("DEBUG: Unknown update command: '");
          Serial.print(command);
          Serial.println("'");
        }
      }
    }
  }
};

// BLE Characteristic callback for immediate parameter updates
class characteristic_callbacks: public BLECharacteristicCallbacks {
  void onRead(BLECharacteristic *pCharacteristic) {
    if (pCharacteristic->getUUID().toString() == CHARACTERISTIC_UUID) {
      // Return current parameters as comma-separated string
      String paramString = String(batteryThreshold) + "," +
                          String(K) + "," +
                          String(F) + "," +
                          String(T) + "," +
                          String(thermistorResistance) + "," +
                          String(r2) + "," +
                          String(backupTime) + "," +
                          String(r4) + "," +
                          String(fanDuration) + "," +
                          String(H) + "," +
                          String(continueFeeder) + "," +
                          String(maxOpeningTime) + "," +
                          String(typicalOpeningTime) + "," +
                          String(MOTOR_CUT_TIME) + "," +
                          String(CUT_MODE_HEAT_TIME) + "," +
                          String(postCoolingBagDuration) + "," +
                          String(preFeedFan) + "," +
                          String(fanReverseTime) + "," +
                          String(fanReverseStartTime) + "," +
                          String(backupTimeAfterReopen);
      
      pCharacteristic->setValue(paramString.c_str());
      Serial.printf("Read request - returning parameters: %s\n", paramString.c_str());
      SerialBLE_println("Read request - returning parameters:");
      SerialBLE_println(paramString);
    }
  }
  
  void onWrite(BLECharacteristic *pCharacteristic) {
    if (pCharacteristic->getUUID().toString() == CHARACTERISTIC_UUID) {
      // Process parameters immediately when written
      int message_length = pCharacteristic->getLength();
      if (message_length > 0) {
        unsigned char* message = pCharacteristic->getData();
        Serial.printf("Immediate parameter update received, length: %d\n", message_length);
        
        // Parse and apply parameters immediately
        char * parameters_string = strtok((char*) message, ",");
        float parameters_list[100];
        int k = 0;
        while (parameters_string != NULL) {
          bool float_check = false;
          for (int i = 0; i <= strlen(parameters_string); i++) {
            if (parameters_string[i] == '.') {
              const float float_parameter = atof(parameters_string);
              parameters_list[k] = float_parameter;
              float_check = true;
            }
          }
          if (float_check == false) {
            char *endptr;
            const float new_parameter = strtof(parameters_string, &endptr);
            parameters_list[k] = new_parameter;
          }
          parameters_string = strtok(NULL, ",");
          k++;
        }
        
        // Apply parameters immediately
        batteryThreshold = (int)parameters_list[0];
        K = parameters_list[1];
        F = (int)parameters_list[2];
        T = (long)parameters_list[3];
        thermistorResistance = parameters_list[4];
        knownResistor = thermistorResistance;  // Update knownResistor when parameter changes
        r2 = (int)parameters_list[5];
        backupTime = parameters_list[6];
        r4 = (int)parameters_list[7];
        fanDuration = (int)parameters_list[8];
        H = (long)parameters_list[9];
        continueFeeder = parameters_list[10];
        maxOpeningTime = (int)parameters_list[11];
        typicalOpeningTime = (int)parameters_list[12];
        MOTOR_CUT_TIME = parameters_list[13];
        CUT_MODE_HEAT_TIME = parameters_list[14];
        postCoolingBagDuration = parameters_list[15];
        preFeedFan = parameters_list[16];
        fanReverseTime = parameters_list[17];
        fanReverseStartTime = parameters_list[18];
        backupTimeAfterReopen = parameters_list[19];
        
        // Update PID setpoint immediately
        setpoint = K;
        
        // Save parameters to EEPROM for persistence
        saveParametersToEEPROM();
        
        // Update characteristic value with new parameters
        String paramString = String(batteryThreshold) + "," +
                            String(K) + "," +
                            String(F) + "," +
                            String(T) + "," +
                            String(thermistorResistance) + "," +
                            String(r2) + "," +
                            String(backupTime) + "," +
                            String(r4) + "," +
                            String(fanDuration) + "," +
                            String(H) + "," +
                            String(continueFeeder) + "," +
                            String(maxOpeningTime) + "," +
                            String(typicalOpeningTime) + "," +
                            String(MOTOR_CUT_TIME) + "," +
                            String(CUT_MODE_HEAT_TIME) + "," +
                            String(postCoolingBagDuration) + "," +
                            String(preFeedFan) + "," +
                            String(fanReverseTime) + "," +
                            String(fanReverseStartTime) + "," +
                            String(backupTimeAfterReopen);
        pCharacteristic->setValue(paramString.c_str());
        
        Serial.printf("Parameters updated immediately! H=%ld, K=%.1f\n", H, K);
        Serial.printf("Characteristic value updated to: %s\n", paramString.c_str());
        Serial.printf("DEBUG: About to save to EEPROM - H=%ld, K=%.1f\n", H, K);
        
        // Send debug info via BLE
        SerialBLE_print("Parameters updated immediately! H=");
        SerialBLE_print((int)H);
        SerialBLE_print(", K=");
        SerialBLE_print(K);
        SerialBLE_println();
        SerialBLE_print("DEBUG: About to save to EEPROM - H=");
        SerialBLE_print((int)H);
        SerialBLE_print(", K=");
        SerialBLE_print(K);
        SerialBLE_println();
      }
    }
  }
};

// Function to initialize the MCP23017 for output and input
void mcp_setup() {
  // ✅ Use GPIO6 for SDA and GPIO7 for SCL
  myI2C.begin(6, 7, 100000);

  // Init MCP23017 with custom I2C bus
  if (!mcp.begin_I2C(0x20, &myI2C)) {
    Serial.println("Error initializing MCP23017!");
    while (1);
  } else {
    Serial.println("MCP23017 Initialized Successfully.");
  }

  // Setup button pins on expander
  mcp.pinMode(button1Pin, INPUT_PULLUP);
  mcp.pinMode(button2Pin, INPUT_PULLUP);

  // Setup LED pins on expander
  for (int i = 0; i < totalLeds; i++) {
    mcp.pinMode(ledPins[i], OUTPUT);
  }
}

// Function to write a value to a specific MCP23017 pin
void mcp_digitalWrite(int pin, int value) {
  mcp.digitalWrite(pin, value);
}

// Function to read from a specific MCP23017 pin
int mcp_digitalRead(int pin) {
  return mcp.digitalRead(pin);
}

// Function to initialize the BLE Server
void server_setup(bool includeOTA = false) {
  Serial.println("Setting up Bluetooth Low Energy.");
  // Create a BLE Device
  BLEDevice::init("ESP32 Toilet");
  
  // Set up the BLE Device as a server
  blue_server = BLEDevice::createServer();
  
  // Create a new server_callbacks() object
  blue_server->setCallbacks(new server_callbacks());
  // Set up a service for the server
  BLEService * blue_service = blue_server->createService(SERVICE_UUID);
  // Set the characteristics for the service - a client can both READ and WRITE to the server
  blue_characteristic = blue_service->createCharacteristic(
                                                          CHARACTERISTIC_UUID, 
                                                          BLECharacteristic::PROPERTY_READ |
                                                          BLECharacteristic::PROPERTY_WRITE
                                                        );
  // Set callback for immediate parameter updates
  blue_characteristic->setCallbacks(new characteristic_callbacks());
  
  // Set initial value to current parameters
  String initialParams = String(batteryThreshold) + "," +
                        String(K) + "," +
                        String(F) + "," +
                        String(T) + "," +
                        String(thermistorResistance) + "," +
                        String(r2) + "," +
                        String(backupTime) + "," +
                        String(r4) + "," +
                        String(fanDuration) + "," +
                        String(H) + "," +
                        String(continueFeeder) + "," +
                        String(maxOpeningTime) + "," +
                        String(typicalOpeningTime) + "," +
                        String(MOTOR_CUT_TIME) + "," +
                        String(CUT_MODE_HEAT_TIME) + "," +
                        String(postCoolingBagDuration) + "," +
                        String(preFeedFan) + "," +
                        String(fanReverseTime) + "," +
                        String(fanReverseStartTime) + "," +
                        String(backupTimeAfterReopen);
  blue_characteristic->setValue(initialParams.c_str());
  Serial.printf("Initial characteristic value set to: %s\n", initialParams.c_str());
  Serial.printf("DEBUG: H value at BLE init: %ld\n", H);
  //SerialBLE_print("DEBUG: H value at BLE init: ");
  //SerialBLE_print((int)H);
  //SerialBLE_println();
  
  // Create serial streaming characteristic
  serial_characteristic = blue_service->createCharacteristic(
                                                             SERIAL_CHARACTERISTIC_UUID,
                                                             BLECharacteristic::PROPERTY_READ |
                                                             BLECharacteristic::PROPERTY_WRITE |
                                                             BLECharacteristic::PROPERTY_NOTIFY
                                                           );
  serial_characteristic->setValue("Serial streaming ready");
  
  // Create version characteristic with dummy value
  version_characteristic = blue_service->createCharacteristic(
                                                             VERSION_CHARACTERISTIC_UUID,
                                                             BLECharacteristic::PROPERTY_READ |
                                                             BLECharacteristic::PROPERTY_NOTIFY
                                                           );
  version_characteristic->setValue("v1.0");
  Serial.println("Version characteristic set to dummy value");
  
  // DON'T create update service at startup - create it dynamically in enableOTA()
  update_service = NULL;
  update_characteristic = NULL;
  
  // Starts the service on the server
  blue_service->start();
  
  // Starts broadcasting so any devices can now find it and connect
  // Only advertise main service - OTA service will be added in enableOTA()
  BLEAdvertising * blue_advert = blue_server->getAdvertising();
  blue_advert->addServiceUUID(SERVICE_UUID);
  // Don't add UPDATE_SERVICE_UUID here - it will be added in enableOTA()
  blue_advert->setScanResponse(true);
  // Functions that help with iPhone connections issue
  blue_advert->setMinPreferred(0x06);
  blue_advert->setMinPreferred(0x12);


  // Once the server starts advertising, the client can now see the server as it's visible to everyone
  BLEDevice::startAdvertising();


  Serial.print("ESP32 MAC ADDRESS: ");
  // Outputs the MAC Address of the ESP32 Board
  Serial.println(BLEDevice::getAddress().toString());
  Serial.println("Characteristic defined! Now your client can read it!");
}

// Function to save parameters to EEPROM
void saveParametersToEEPROM() {
  EEPROM.begin(EEPROM_SIZE);
  
  // Write magic number to verify valid data
  EEPROM.put(PARAM_START_ADDR, PARAM_MAGIC_NUMBER);
  
  // Write parameters in order
  int addr = sizeof(PARAM_MAGIC_NUMBER);
  EEPROM.put(addr, batteryThreshold); addr += sizeof(batteryThreshold);
  EEPROM.put(addr, K); addr += sizeof(K);
  EEPROM.put(addr, F); addr += sizeof(F);
  EEPROM.put(addr, T); addr += sizeof(T);
  EEPROM.put(addr, thermistorResistance); addr += sizeof(thermistorResistance);
  EEPROM.put(addr, r2); addr += sizeof(r2);
  EEPROM.put(addr, backupTime); addr += sizeof(backupTime);
  EEPROM.put(addr, r4); addr += sizeof(r4);
  EEPROM.put(addr, fanDuration); addr += sizeof(fanDuration);
  EEPROM.put(addr, H); addr += sizeof(H);
  EEPROM.put(addr, continueFeeder); addr += sizeof(continueFeeder);
  EEPROM.put(addr, maxOpeningTime); addr += sizeof(maxOpeningTime);
  EEPROM.put(addr, typicalOpeningTime); addr += sizeof(typicalOpeningTime);
  EEPROM.put(addr, MOTOR_CUT_TIME); addr += sizeof(MOTOR_CUT_TIME);
  EEPROM.put(addr, CUT_MODE_HEAT_TIME); addr += sizeof(CUT_MODE_HEAT_TIME);
  EEPROM.put(addr, postCoolingBagDuration); addr += sizeof(postCoolingBagDuration);
  EEPROM.put(addr, preFeedFan); addr += sizeof(preFeedFan);
  EEPROM.put(addr, fanReverseTime); addr += sizeof(fanReverseTime);
  EEPROM.put(addr, fanReverseStartTime); addr += sizeof(fanReverseStartTime);
  EEPROM.put(addr, backupTimeAfterReopen); addr += sizeof(backupTimeAfterReopen);
  
  EEPROM.commit();
  
  // Verify the write by reading back the magic number
  uint16_t verifyMagic;
  EEPROM.get(PARAM_START_ADDR, verifyMagic);
  
  if (verifyMagic == PARAM_MAGIC_NUMBER) {
    Serial.println("Parameters saved to EEPROM - verification successful");
    SerialBLE_println("Parameters saved to EEPROM - verification successful");
  } else {
    Serial.printf("ERROR: EEPROM write verification failed! Expected: 0x%04X, Read: 0x%04X\n", 
                  PARAM_MAGIC_NUMBER, verifyMagic);
    SerialBLE_print("ERROR: EEPROM write verification failed! Expected: 0x");
    SerialBLE_print(String(PARAM_MAGIC_NUMBER, HEX));
    SerialBLE_print(", Read: 0x");
    SerialBLE_print(String(verifyMagic, HEX));
    SerialBLE_println();
  }
  
  EEPROM.end();
}

// Function to load parameters from EEPROM
void loadParametersFromEEPROM() {
  EEPROM.begin(EEPROM_SIZE);
  
  // Check magic number
  uint16_t magic;
  EEPROM.get(PARAM_START_ADDR, magic);
  Serial.printf("DEBUG: EEPROM magic number check - read: 0x%04X, expected: 0x%04X\n", magic, PARAM_MAGIC_NUMBER);
  SerialBLE_print("DEBUG: EEPROM magic number check - read: 0x");
  SerialBLE_print(String(magic, HEX));
  SerialBLE_print(", expected: 0x");
  SerialBLE_print(String(PARAM_MAGIC_NUMBER, HEX));
  SerialBLE_println();
  
  if (magic == PARAM_MAGIC_NUMBER) {
    // Valid data found, load parameters
    int addr = sizeof(PARAM_MAGIC_NUMBER);
    EEPROM.get(addr, batteryThreshold); addr += sizeof(batteryThreshold);
    EEPROM.get(addr, K); addr += sizeof(K);
    EEPROM.get(addr, F); addr += sizeof(F);
    EEPROM.get(addr, T); addr += sizeof(T);
    EEPROM.get(addr, thermistorResistance); addr += sizeof(thermistorResistance);
    knownResistor = thermistorResistance;  // Initialize knownResistor from loaded parameter
    EEPROM.get(addr, r2); addr += sizeof(r2);
    EEPROM.get(addr, backupTime); addr += sizeof(backupTime);
    EEPROM.get(addr, r4); addr += sizeof(r4);
    EEPROM.get(addr, fanDuration); addr += sizeof(fanDuration);
    EEPROM.get(addr, H); addr += sizeof(H);
    EEPROM.get(addr, continueFeeder); addr += sizeof(continueFeeder);
    EEPROM.get(addr, maxOpeningTime); addr += sizeof(maxOpeningTime);
    EEPROM.get(addr, typicalOpeningTime); addr += sizeof(typicalOpeningTime);
    EEPROM.get(addr, MOTOR_CUT_TIME); addr += sizeof(MOTOR_CUT_TIME);
    EEPROM.get(addr, CUT_MODE_HEAT_TIME); addr += sizeof(CUT_MODE_HEAT_TIME);
    EEPROM.get(addr, postCoolingBagDuration); addr += sizeof(postCoolingBagDuration);
    EEPROM.get(addr, preFeedFan); addr += sizeof(preFeedFan);
    EEPROM.get(addr, fanReverseTime); addr += sizeof(fanReverseTime);
    EEPROM.get(addr, fanReverseStartTime); addr += sizeof(fanReverseStartTime);
    EEPROM.get(addr, backupTimeAfterReopen); addr += sizeof(backupTimeAfterReopen);
    
    Serial.println("Parameters loaded from EEPROM");
    Serial.printf("Loaded: H=%ld, K=%.1f, F=%d, T=%ld, backupTime=%.1f\n", H, K, F, T, backupTime);
    SerialBLE_println("Parameters loaded from EEPROM");
    SerialBLE_print("Loaded: H=");
    SerialBLE_print((int)H);
    SerialBLE_print(", K=");
    SerialBLE_print(K);
    SerialBLE_print(", F=");
    SerialBLE_print(F);
    SerialBLE_print(", T=");
    SerialBLE_print((int)T);
    SerialBLE_print(", backupTime=");
    SerialBLE_print(backupTime);
    SerialBLE_println();
  } else {
    Serial.println("No valid parameters in EEPROM, using defaults");
    SerialBLE_println("No valid parameters in EEPROM, using defaults");
    
    // Flash LEDs 5 times in 1 second to indicate EEPROM error
    for (int i = 0; i < 5; i++) {
      // Turn on all LEDs
      for (int j = 0; j < totalLeds; j++) {
        mcp_digitalWrite(ledPins[j], HIGH);
      }
      delay(100); // 100ms on
      
      // Turn off all LEDs
      for (int j = 0; j < totalLeds; j++) {
        mcp_digitalWrite(ledPins[j], LOW);
      }
      delay(100); // 100ms off
    }
  }
  
  EEPROM.end();
}

// Initialize hardware version (write once when device is first programmed)
void initializeHardwareVersion() {
  EEPROM.begin(EEPROM_SIZE);
  
  // Check if hardware version already exists
  uint16_t magic;
  EEPROM.get(HW_VERSION_ADDR, magic);
  
  if (magic != HW_VERSION_MAGIC) {
    // First time programming - write hardware version
    Serial.println("Initializing hardware version for first time");
    writeHardwareVersion(1, "ESP32 Toilet System v1.0");
    SerialBLE_println("Hardware version initialized");
  }
  
  EEPROM.end();
}

// Read hardware version from EEPROM
VersionInfo readHardwareVersion() {
  VersionInfo hwInfo = {};
  EEPROM.begin(EEPROM_SIZE);
  
  uint16_t magic;
  EEPROM.get(HW_VERSION_ADDR, magic);
  
  if (magic == HW_VERSION_MAGIC) {
    EEPROM.get(HW_VERSION_ADDR + sizeof(magic), hwInfo.hardware_version);
    EEPROM.get(HW_VERSION_ADDR + sizeof(magic) + sizeof(hwInfo.hardware_version), hwInfo.hardware_description);
    hwInfo.magic = magic;
  } else {
    // Hardware version not initialized - set defaults
    hwInfo.magic = 0;
    hwInfo.hardware_version = 0;
    strcpy(hwInfo.hardware_description, "Uninitialized");
    Serial.println("WARNING: Hardware version not found in EEPROM - using defaults");
  }
  
  EEPROM.end();
  return hwInfo;
}

// Write hardware version to EEPROM (one-time operation)
void writeHardwareVersion(uint16_t version, const char* description) {
  EEPROM.begin(EEPROM_SIZE);
  
  uint16_t magic = HW_VERSION_MAGIC;
  EEPROM.put(HW_VERSION_ADDR, magic);
  EEPROM.put(HW_VERSION_ADDR + sizeof(magic), version);
  EEPROM.put(HW_VERSION_ADDR + sizeof(magic) + sizeof(version), description);
  
  EEPROM.commit();
  EEPROM.end();
  
  Serial.printf("Hardware version written: %d - %s\n", version, description);
}

// Get combined version string for BLE transmission
String getVersionString() {
  VersionInfo hwInfo = readHardwareVersion();
  String versionString = "";
  
  // Only add hardware version if it's properly initialized
  if (hwInfo.magic == HW_VERSION_MAGIC) {
    versionString += "HW:";
    versionString += hwInfo.hardware_version;
    versionString += "|";
  } else {
    versionString += "HW:Uninitialized|";
  }
  
  versionString += "SW:";
  versionString += SOFTWARE_VERSION;
  versionString += "|Build:";
  versionString += SOFTWARE_BUILD_DATE;
  
  if (hwInfo.magic == HW_VERSION_MAGIC) {
    versionString += "|Desc:";
    versionString += hwInfo.hardware_description;
  }
  
  return versionString;
}

// Prepare device for OTA update
bool prepareForOTA() {
  if (isFlushing || updateInProgress) {
    SerialBLE_println("Update preparation blocked - flush in progress");
    return false;
  }
  
  // Get current running partition
  running_partition = esp_ota_get_running_partition();
  if (running_partition == NULL) {
    SerialBLE_println("ERROR: Cannot get running partition");
    return false;
  }
  
  // Determine update partition (opposite of current)
  if (running_partition->subtype == ESP_PARTITION_SUBTYPE_APP_OTA_0) {
    update_partition = esp_partition_find_first(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_OTA_1, NULL);
  } else if (running_partition->subtype == ESP_PARTITION_SUBTYPE_APP_OTA_1) {
    update_partition = esp_partition_find_first(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_OTA_0, NULL);
  } else {
    // Running from factory, use ota_0
    update_partition = esp_partition_find_first(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_OTA_0, NULL);
  }
  
  if (update_partition == NULL) {
    SerialBLE_println("ERROR: Cannot find update partition");
    return false;
  }
  
  // Verify we're not trying to overwrite the running partition
  if (update_partition == running_partition) {
    SerialBLE_println("ERROR: Cannot update running partition");
    return false;
  }
  
  // Save rollback information before starting
  saveRollbackInfo();
  
  // Stop all operations
  stopEverything();
  
  // Disable heater and motors
  heaterOff();
  motors.setM1Speed(0);
  motors.setM2Speed(0);
  setM3Speed(0);
  
  // Reset OTA state
  resetOTAState();
  otaState = OTA_PREPARING;
  updateInProgress = true;
  
  // Indicate update mode with LEDs
  flashLeds(); // Flash LEDs to indicate update mode
  
  SerialBLE_println("Device prepared for OTA update");
  Serial.printf("Current partition: %s, Update partition: %s\n", 
                running_partition->label, update_partition->label);
  return true;
}

// Notify update progress via BLE
void notifyUpdateProgress(int percentage) {
  if (version_characteristic && is_device_connected) {
    String progressMsg = "UPDATE_PROGRESS:" + String(percentage);
    version_characteristic->setValue(progressMsg.c_str());
    version_characteristic->notify();
    Serial.println("Update progress: " + String(percentage) + "%");
    SerialBLE_print("Update progress: ");
    SerialBLE_print(percentage);
    SerialBLE_println("%");
  }
  updateProgress = percentage;
}

// Reset OTA state variables
void resetOTAState() {
  otaState = OTA_IDLE;
  ota_handle = 0;
  firmware_size = 0;
  bytes_received = 0;
  chunk_sequence = 0;
  md5_received = false;
  rollback_required = false;
  ota_error_message = "";
  memset(expected_md5, 0, 16);
  memset(calculated_md5, 0, 16);
  
  // Clean up MD5 context if initialized
  if (md5_initialized) {
    mbedtls_md5_free(&md5_ctx);
    md5_initialized = false;
  }
}

// Save rollback information to NVS
void saveRollbackInfo() {
  nvs_handle_t nvs_handle;
  esp_err_t err = nvs_open("ota", NVS_READWRITE, &nvs_handle);
  if (err != ESP_OK) {
    Serial.println("ERROR: Failed to open NVS for rollback info");
    return;
  }
  
  // Save current partition subtype
  uint8_t current_subtype = running_partition->subtype;
  nvs_set_u8(nvs_handle, "rollback_subtype", current_subtype);
  
  // Save boot count (will be incremented on next boot)
  uint8_t boot_count = 0;
  nvs_get_u8(nvs_handle, "boot_count", &boot_count);
  boot_count++;
  nvs_set_u8(nvs_handle, "boot_count", boot_count);
  
  // Set rollback flag to false initially
  nvs_set_u8(nvs_handle, "rollback_flag", 0);
  
  nvs_commit(nvs_handle);
  nvs_close(nvs_handle);
  
  Serial.printf("Rollback info saved: subtype=%d, boot_count=%d\n", current_subtype, boot_count);
}

// Check for boot failures and rollback if needed
void checkBootFailure() {
  nvs_handle_t nvs_handle;
  esp_err_t err = nvs_open("ota", NVS_READWRITE, &nvs_handle);
  if (err != ESP_OK) {
    Serial.println("WARNING: Failed to open NVS for boot check");
    return;
  }
  
  uint8_t boot_count = 0;
  nvs_get_u8(nvs_handle, "boot_count", &boot_count);
  
  // If boot count is high, it means we've been rebooting repeatedly (boot failure)
  if (boot_count > 3) {
    Serial.println("WARNING: High boot count detected, may need rollback");
    rollback_required = true;
    
    // Reset boot count
    nvs_set_u8(nvs_handle, "boot_count", 0);
    nvs_commit(nvs_handle);
    
    // Attempt rollback
    if (rollbackOTAUpdate()) {
      Serial.println("Rollback successful, rebooting...");
      delay(1000);
      esp_restart();
    }
  } else {
    // Successful boot, reset boot count
    nvs_set_u8(nvs_handle, "boot_count", 0);
    nvs_commit(nvs_handle);
  }
  
  nvs_close(nvs_handle);
}

// Rollback to previous partition
bool rollbackOTAUpdate() {
  Serial.println("Attempting OTA rollback...");
  
  nvs_handle_t nvs_handle;
  esp_err_t err = nvs_open("ota", NVS_READWRITE, &nvs_handle);
  if (err != ESP_OK) {
    Serial.println("ERROR: Failed to open NVS for rollback");
    return false;
  }
  
  uint8_t rollback_subtype;
  err = nvs_get_u8(nvs_handle, "rollback_subtype", &rollback_subtype);
  if (err != ESP_OK) {
    Serial.println("ERROR: Failed to get rollback partition info");
    nvs_close(nvs_handle);
    return false;
  }
  
  // Find the rollback partition
  const esp_partition_t *rollback_partition = NULL;
  if (rollback_subtype == ESP_PARTITION_SUBTYPE_APP_OTA_0) {
    rollback_partition = esp_partition_find_first(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_OTA_0, NULL);
  } else if (rollback_subtype == ESP_PARTITION_SUBTYPE_APP_OTA_1) {
    rollback_partition = esp_partition_find_first(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_OTA_1, NULL);
  } else {
    rollback_partition = esp_partition_find_first(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_FACTORY, NULL);
  }
  
  if (rollback_partition == NULL) {
    Serial.println("ERROR: Cannot find rollback partition");
    nvs_close(nvs_handle);
    return false;
  }
  
  // Set boot partition to rollback partition
  err = esp_ota_set_boot_partition(rollback_partition);
  if (err != ESP_OK) {
    Serial.printf("ERROR: Failed to set boot partition: %s\n", esp_err_to_name(err));
    nvs_close(nvs_handle);
    return false;
  }
  
  // Clear rollback flag
  nvs_set_u8(nvs_handle, "rollback_flag", 0);
  nvs_set_u8(nvs_handle, "boot_count", 0);
  nvs_commit(nvs_handle);
  nvs_close(nvs_handle);
  
  Serial.printf("Rollback successful to partition: %s\n", rollback_partition->label);
  return true;
}

// Validate firmware using MD5
bool validateFirmware() {
  if (!md5_received) {
    Serial.println("ERROR: MD5 hash not received");
    return false;
  }
  
  // Compare calculated MD5 with expected MD5
  if (memcmp(calculated_md5, expected_md5, 16) != 0) {
    Serial.println("ERROR: MD5 validation failed");
    Serial.print("Expected MD5: ");
    for (int i = 0; i < 16; i++) {
      Serial.printf("%02x", expected_md5[i]);
    }
    Serial.println();
    Serial.print("Calculated MD5: ");
    for (int i = 0; i < 16; i++) {
      Serial.printf("%02x", calculated_md5[i]);
    }
    Serial.println();
    return false;
  }
  
  Serial.println("MD5 validation successful");
  return true;
}

// Handle OTA chunk reception
void handleOTAChunk(uint8_t* data, size_t length) {
  if (otaState != OTA_RECEIVING) {
    Serial.println("ERROR: Received chunk but not in RECEIVING state");
    return;
  }
  
  // Check if this is a metadata chunk (firmware size or MD5)
  if (length >= 4 && data[0] == 'S' && data[1] == 'I' && data[2] == 'Z' && data[3] == 'E') {
    // Firmware size chunk: "SIZE:1234567"
    String sizeStr = String((char*)data + 5);
    firmware_size = sizeStr.toInt();
    Serial.printf("Firmware size received: %d bytes\n", firmware_size);
    return;
  }
  
  if (length >= 3 && data[0] == 'M' && data[1] == 'D' && data[2] == '5') {
    // MD5 chunk: "MD5:abcdef123456..."
    if (length >= 20) { // "MD5:" + 32 hex chars = 36 bytes
      String md5Str = String((char*)data + 4);
      // Convert hex string to bytes
      for (int i = 0; i < 16; i++) {
        char hex[3] = {md5Str[i*2], md5Str[i*2+1], 0};
        expected_md5[i] = strtol(hex, NULL, 16);
      }
      md5_received = true;
      Serial.println("MD5 hash received");
    }
    return;
  }
  
  // Regular firmware data chunk
  if (ota_handle == 0) {
    Serial.println("ERROR: OTA handle not initialized");
    otaState = OTA_ERROR;
    ota_error_message = "OTA handle not initialized";
    return;
  }
  
  // Initialize MD5 context for first chunk
  if (!md5_initialized) {
    mbedtls_md5_init(&md5_ctx);
    mbedtls_md5_starts(&md5_ctx);
    md5_initialized = true;
  }
  
  // Update MD5 hash
  mbedtls_md5_update(&md5_ctx, data, length);
  
  // Write chunk to OTA partition
  esp_err_t err = esp_ota_write(ota_handle, data, length);
  if (err != ESP_OK) {
    Serial.printf("ERROR: Failed to write OTA chunk: %s\n", esp_err_to_name(err));
    otaState = OTA_ERROR;
    ota_error_message = "OTA write failed: " + String(esp_err_to_name(err));
    mbedtls_md5_free(&md5_ctx);
    md5_initialized = false;
    return;
  }
  
  bytes_received += length;
  chunk_sequence++;
  
  // Calculate and notify progress
  if (firmware_size > 0) {
    int progress = (bytes_received * 100) / firmware_size;
    notifyUpdateProgress(progress);
  }
  
  // Finalize MD5 if this is the last chunk (indicated by bytes_received >= firmware_size)
  if (firmware_size > 0 && bytes_received >= firmware_size) {
    mbedtls_md5_finish(&md5_ctx, calculated_md5);
    mbedtls_md5_free(&md5_ctx);
    md5_initialized = false;
    Serial.println("Firmware reception complete, validating...");
    otaState = OTA_VALIDATING;
  }
}

// Test heater current detection at startup
void testHeaterCurrent() {
  Serial.println("Starting heater current detection test...");
  SerialBLE_println("Starting heater current detection test...");
  
  // Set heater to 10% PWM (26 out of 255)
  const int testPWM = 26; // 10% of 255
  analogWrite(heaterPin, testPWM);
  Serial.printf("Heater set to 10%% PWM (%d/255)\n", testPWM);
  SerialBLE_print("Heater set to 10% PWM (");
  SerialBLE_print(testPWM);
  SerialBLE_println("/255)");
  
  // Monitor for 0.5 seconds, checking every 50ms
  unsigned long testStartTime = millis();
  const unsigned long testDuration = 500; // 0.5 seconds
  const unsigned long checkInterval = 50; // Check every 50ms
  const int threshold = 200; // ADC threshold for current detection
  bool currentDetected = false;
  unsigned long lastCheckTime = 0;
  
  while ((millis() - testStartTime) < testDuration) {
    // Check every 50ms
    if ((millis() - lastCheckTime) >= checkInterval) {
      lastCheckTime = millis();
      
      int adcReading = analogRead(heaterCurrentPin);
      Serial.printf("Heater current ADC reading: %d\n", adcReading);
      SerialBLE_print("Heater current ADC: ");
      SerialBLE_println(adcReading);
      
      if (adcReading > threshold) {
        currentDetected = true;
        Serial.println("Heater current detected - test PASSED");
        SerialBLE_println("Heater current detected - test PASSED");
        break; // Exit loop early if current detected
      }
    }
    delay(10); // Small delay to prevent tight loop
  }
  
  // Turn off heater
  analogWrite(heaterPin, 0);
  Serial.println("Heater turned off after test");
  
  // Check result
  if (!currentDetected) {
    Serial.println("ERROR: Heater current detection FAILED - No current detected after 0.5s");
    SerialBLE_println("ERROR: Heater current detection FAILED");
    if (ERROR_CODE == 0) {
      ERROR_CODE = 5;
      LEDErrorCode(ERROR_CODE);
    }
    Serial.println("System halted due to heater current detection failure");
    // Don't return - let the system continue but with error state
  } else {
    Serial.println("Heater current test completed successfully");
    SerialBLE_println("Heater current test PASSED");
  }
}

void setup() {
  Serial.begin(115200); // Increased baud rate for faster output
  delay(500); // Longer delay to ensure Serial is ready
  Serial.println("\n\n=== SETUP STARTING ===");
  Serial.printf("Free heap at startup: %d bytes\n", ESP.getFreeHeap());
  Serial.printf("Largest free block: %d bytes\n", ESP.getMaxAllocHeap());
  Serial.println("Beginning Setup");
  
  // Also send to BLE if available (will be sent after BLE is initialized)
  // Store in a buffer or send later after BLE setup
  
  // Initialize NVS for OTA rollback
  esp_err_t err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    // NVS partition was truncated and needs to be erased
    ESP_ERROR_CHECK(nvs_flash_erase());
    err = nvs_flash_init();
  }
  ESP_ERROR_CHECK(err);
  
  // Check for boot failures and rollback if needed
  checkBootFailure();

  mcp_setup(); // Initialize the MCP23017
  Serial.println("MCP23017 Initialized");
  circleLeds();
  Serial.println("LED startup test complete");

  // Initialize hardware version first 
  //initializeHardwareVersion();

  // Record BLE startup time  // Record BLE startup time
  bleStartupTime = millis();
  
  Serial.println("=== MEMORY BEFORE BLE SERVER SETUP ===");
  Serial.printf("Free heap: %d bytes\n", ESP.getFreeHeap());
  Serial.printf("Largest free block: %d bytes\n", ESP.getMaxAllocHeap());
  Serial.printf("Min free heap ever: %d bytes\n", ESP.getMinFreeHeap());
  
  server_setup(false); // Initialize the Bluetooth Low Energy Server without OTA
  
  Serial.println("=== MEMORY AFTER BLE SERVER SETUP ===");
  Serial.printf("Free heap: %d bytes\n", ESP.getFreeHeap());
  Serial.printf("Largest free block: %d bytes\n", ESP.getMaxAllocHeap());
  Serial.printf("Memory used by BLE: ~%d bytes\n", 
                (ESP.getFreeHeap() < ESP.getMaxAllocHeap()) ? 
                (ESP.getMaxAllocHeap() - ESP.getFreeHeap()) : 0);
  
  // Send memory info to BLE
  sendSerialToBLE("\n=== MEMORY AFTER BLE SETUP ===");
  sendSerialToBLE("Free heap: " + String(ESP.getFreeHeap()) + " bytes");
  sendSerialToBLE("Largest free block: " + String(ESP.getMaxAllocHeap()) + " bytes");
  sendSerialToBLE("Min free heap: " + String(ESP.getMinFreeHeap()) + " bytes");
  
  Serial.println("BLE enabled for 10 minutes to save power (OTA disabled initially)");

  
  // Load parameters from EEPROM 
  Serial.println("DEBUG: About to load parameters from EEPROM");
  loadParametersFromEEPROM();
  // Initialize knownResistor from thermistorResistance parameter
  knownResistor = thermistorResistance;
  Serial.printf("DEBUG: After EEPROM load - H=%ld, K=%.1f\n", H, K);
  SerialBLE_println("DEBUG: About to load parameters from EEPROM");
  SerialBLE_print("DEBUG: After EEPROM load - H=");
  SerialBLE_print((int)H);
  SerialBLE_print(", K=");
  SerialBLE_print(K);
  SerialBLE_println();
  
  // Update PID setpoint to match loaded K value
  setpoint = K;

  pinMode(heaterPin, OUTPUT);
  pinMode(buzzerPin, OUTPUT);
  pinMode(microswitchClosePin, INPUT_PULLUP);
  pinMode(microswitchOpenPin, INPUT_PULLUP);
  
  // Motor fault pins
  pinMode(M1NFAULT_PIN, INPUT);
  pinMode(M2NFAULT_PIN, INPUT);
  pinMode(M3NFAULT_PIN, INPUT);
  
  // Motor enable pins
  pinMode(M1NEN_PIN, OUTPUT);
  pinMode(M2NEN_PIN, OUTPUT);
  pinMode(M3NEN_PIN, OUTPUT);
  
  // Motor direction pins
  pinMode(M1DIR_PIN, OUTPUT);
  pinMode(M2DIR_PIN, OUTPUT);
  pinMode(M3DIR_PIN, OUTPUT);
  
  // Motor PWM pins
  pinMode(M1PWM_PIN, OUTPUT);
  pinMode(M2PWM_PIN, OUTPUT);
  pinMode(M3PWM_PIN, OUTPUT);
  
  // Current monitoring pins
  pinMode(m1CurrentPin, INPUT);
  pinMode(heaterCurrentPin, INPUT);
  
  // Battery voltage monitoring pin
  pinMode(batteryVoltagePin, INPUT);
  pinMode(batteryTempPin, INPUT);
 
  // Test heater current detection
  testHeaterCurrent();
  
  motors.enableDrivers();
  motors3.enableDrivers();  // Enable M3 motor shield
  locateMotorPos();
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0, 255);
}

void loop() {
  // OTA mode handling
  if (otaEnabled) {
    // OTA mode is active - run slow circle animation
    slowCircleLeds();
    
    // Check if OTA window has expired (1 minute)
    unsigned long currentMillis = millis();
    if (currentMillis - otaWindowStartTime >= OTA_WINDOW_DURATION) {
      // Check if there's an active OTA connection
      bool hasOTAConnection = (is_device_connected && update_characteristic != NULL);
      
      if (!hasOTAConnection) {
        Serial.println("OTA window expired with no connections - disabling OTA and restarting BLE");
        restartBLEServer();
        batteryMonitoringActive = false; // Reset monitoring state
        lastBatteryCheckTime = millis(); // Reset battery check timer
      } else {
        // OTA connection exists, extend window or keep it open
        Serial.println("OTA connection active - keeping OTA mode enabled");
      }
    }
  }
  
  // Check for BLE timeout (10 minutes from startup)
  if (bleEnabled && (millis() - bleStartupTime > BLE_TIMEOUT)) {
    Serial.println("BLE shutting down after 10 minutes to save power");
    SerialBLE_println("BLE shutting down after 10 minutes to save power");
    
    // Disconnect all clients and stop advertising
    if (blue_server) {
      blue_server->getAdvertising()->stop();
      BLEDevice::deinit();
    }
    
    bleEnabled = false;
    batteryMonitoringActive = false; // Reset monitoring when BLE shuts down
    Serial.println("BLE disabled - manually restart device to re-enable");
    // Continue with control panel operations even when BLE is disabled
  }
  
  // BLE-specific operations - only run if BLE is enabled and OTA is not active
  if (bleEnabled && !otaEnabled) {
    if (!is_device_connected) {
    Serial.println("hi");
  } else {
    // Send periodic status when connected
    static unsigned long lastStatusPrint = 0;
    if (millis() - lastStatusPrint > 5000) { // Every 5 seconds
      lastStatusPrint = millis();
      Serial.println("BLE Connected - System Running");
      sendSerialToBLE("System Status: Running - " + String(millis() / 1000) + "s");
    }
  }
  if (!is_device_connected && old_device_connect) {
    delay(500);
    blue_server->startAdvertising();
    old_device_connect = is_device_connected;
  }
  if (is_device_connected && !old_device_connect) {
    old_device_connect = is_device_connected;
  }
  
  // Handle serial streaming commands
  if (is_device_connected && serial_characteristic) {
    int serial_message_length = serial_characteristic->getLength();
    Serial.print("DEBUG: serial_message_length = ");
    Serial.println(serial_message_length);
    
    if (serial_message_length > 0) {
      unsigned char* serial_message = serial_characteristic->getData();
      String command = String((char*)serial_message);
      command.trim();
      
      Serial.print("DEBUG: Received command: '");
      Serial.print(command);
      Serial.println("'");
      
      if (command == "START_SERIAL") {
        Serial.println("DEBUG: Processing START_SERIAL command");
        serial_streaming_enabled = true;
        Serial.println("Serial streaming enabled via BLE");
        sendSerialToBLE("Serial streaming ENABLED via BLE");
      } else if (command == "STOP_SERIAL") {
        Serial.println("DEBUG: Processing STOP_SERIAL command");
        serial_streaming_enabled = false;
        Serial.println("Serial streaming disabled via BLE");
        sendSerialToBLE("Serial streaming DISABLED via BLE");
      } else {
        Serial.print("DEBUG: Unknown command: '");
        Serial.print(command);
        Serial.println("'");
      }
    }
  }
  } // End of BLE-specific operations
  
  // OTA update handling is now done via update_characteristic_callbacks class
  // Read button states
  bool button1Pressed = (mcp_digitalRead(button1Pin) == LOW);
  bool button2Pressed = (mcp_digitalRead(button2Pin) == LOW);
  
  // Determine if both buttons are pressed
  bool bothButtonsPressedNow = button1Pressed && button2Pressed;
  
  // Check for both buttons pressed simultaneously
  if (bothButtonsPressedNow && !bothButtonsPressed) {
    SerialBLE_println("Both buttons pressed - Stopping all operations and displaying battery charge level");
    
    // Stop all ongoing operations immediately
    Serial.println("DEBUG: Stopping all motors for battery display");
    motors.setM2Speed(0);  // Stop feed motor
    setM3Speed(0);         // Stop fan
    isFlushing = false;    // Stop any flush sequence
    fanRunning = false;    // Stop fan timer
    
    // Reset all button state tracking
    button1WasPressed = false;
    button2WasPressed = false;
    button1Held = false;
    button1DelayActive = false;
    button2DelayActive = false;
    
    // Enter battery display mode and start monitoring for OTA trigger
    bothButtonsPressed = true;
    batteryDisplayMode = true;
    batteryDisplayStartTime = millis();
    batteryMonitorStartTime = millis(); // Start OTA monitoring timer
    batteryMonitoringActive = true;
    lastBatteryCheckTime = millis();
    displayBatteryChargeLevel();
  } else if (bothButtonsPressedNow && bothButtonsPressed && !otaEnabled) {
    // Both buttons still pressed - continue battery monitoring for OTA trigger
    unsigned long currentMillis = millis();
    if (currentMillis - lastBatteryCheckTime >= BATTERY_CHECK_INTERVAL) {
      lastBatteryCheckTime = currentMillis;
      
      // Read battery level (this is the monitoring action)
      int batteryLevel = getBatteryChargeLevel();
      
      // Check if we've been monitoring for 10 seconds continuously
      if (batteryMonitoringActive && (currentMillis - batteryMonitorStartTime >= BATTERY_MONITOR_DURATION)) {
        Serial.println("\n\n=== BATTERY MONITORED FOR 10 SECONDS - TRIGGERING OTA MODE ===");
        sendSerialToBLE("\n\n=== BATTERY MONITORED FOR 10 SECONDS - TRIGGERING OTA MODE ===");
        
        Serial.println("=== MEMORY DIAGNOSTICS BEFORE OTA ===");
        sendSerialToBLE("=== MEMORY DIAGNOSTICS BEFORE OTA ===");
        
        size_t freeHeap = ESP.getFreeHeap();
        size_t maxAlloc = ESP.getMaxAllocHeap();
        size_t minFree = ESP.getMinFreeHeap();
        size_t heapSize = ESP.getHeapSize();
        
        Serial.printf("Free heap: %d bytes\n", freeHeap);
        sendSerialToBLE("Free heap: " + String(freeHeap) + " bytes");
        
        Serial.printf("Largest free block: %d bytes\n", maxAlloc);
        sendSerialToBLE("Largest free block: " + String(maxAlloc) + " bytes");
        
        Serial.printf("Min free heap ever: %d bytes\n", minFree);
        sendSerialToBLE("Min free heap ever: " + String(minFree) + " bytes");
        
        Serial.printf("Heap size: %d bytes\n", heapSize);
        sendSerialToBLE("Heap size: " + String(heapSize) + " bytes");
        
        Serial.printf("Free PSRAM: %d bytes\n", ESP.getFreePsram());
        sendSerialToBLE("Free PSRAM: " + String(ESP.getFreePsram()) + " bytes");
        
        Serial.printf("PSRAM size: %d bytes\n", ESP.getPsramSize());
        sendSerialToBLE("PSRAM size: " + String(ESP.getPsramSize()) + " bytes");
        
        // Check for memory fragmentation
        float fragmentation = ((float)(freeHeap - maxAlloc) / freeHeap) * 100.0;
        Serial.printf("Memory fragmentation: %.1f%% (lower is better)\n", fragmentation);
        sendSerialToBLE("Memory fragmentation: " + String(fragmentation, 1) + "% (lower is better)");
        
        // Check stack usage (approximate)
        UBaseType_t stackHighWater = uxTaskGetStackHighWaterMark(NULL);
        Serial.printf("Stack high water mark: %d words (lower = more stack used)\n", stackHighWater);
        sendSerialToBLE("Stack high water mark: " + String(stackHighWater) + " words");
        
        Serial.println("=== CALLING enableOTA() ===");
        sendSerialToBLE("=== CALLING enableOTA() ===");
        enableOTA();
        batteryMonitoringActive = false; // Reset monitoring state
        batteryDisplayMode = false; // Exit battery display mode
        bothButtonsPressed = false; // Reset button state
      }
    }
  } else if (!bothButtonsPressedNow) {
    bothButtonsPressed = false;
    batteryMonitoringActive = false; // Reset monitoring when buttons released
    if (batteryDisplayMode && (millis() - batteryDisplayStartTime > 3000)) {
      // Turn off battery display after 3 seconds
      batteryDisplayMode = false;
      for (int i = 0; i < totalLeds; i++) {
        mcp_digitalWrite(ledPins[i], LOW);
      }
    }
  }
  
  // Handle individual button presses only if not in battery display mode
  if (!batteryDisplayMode) {
    // Handle button 1 (Flush) - check for hold to enable cutBag
    if (button1Pressed && !button1WasPressed) {
      SerialBLE_println("Button 1 just pressed - start delay timer");
      // Button 1 just pressed - start delay timer
      button1PressStartTime = millis();
      button1DelayActive = true;
      Serial.println("Button 1 pressed - waiting for delay/hold detection");
    } else if (button1Pressed && button1DelayActive && (millis() - button1PressStartTime >= BUTTON_DELAY)) {
      // Delay passed, now start timing for hold detection
      button1DelayActive = false;
      button1HoldStartTime = millis();
      button1Held = true;
      SerialBLE_println("Flushing Pressed");
      Serial.printf("DEBUG: Button 1 delay passed, starting hold timer at %lu\n", button1HoldStartTime);
      // Don't start flush sequence immediately - wait to see if it's a hold
    } else if (button1Pressed && button1Held && (millis() - button1HoldStartTime >= BUTTON_HOLD_TIME)) {
      // Button 1 held for 1.5 seconds - enable cutBag and start flush sequence
      if (!isFlushing) {  // Only start if not already flushing
        cutBag = true;
        SerialBLE_println("Cut Bag mode enabled!");
        Serial.printf("DEBUG: cutBag set to true, hold time: %lu ms\n", millis() - button1HoldStartTime);
        cutModeLEDAnimation();
        
        // Start flush sequence with cut bag enabled
        SerialBLE_println("Starting flush sequence with cut bag enabled");
        
        // Calculate sequence timing and reset LED state
        calculateSequenceTiming();
        ledIndex = 0;
        clockwise = true; // Start clockwise when flushing begins.
        ledLastUpdateMillis = millis();  // Reset the timer
        
        // Turn off all LEDs first, then turn on the first one
        for (int i = 0; i < totalLeds; i++) {
          mcp_digitalWrite(ledPins[i], LOW);
        }
        mcp_digitalWrite(ledPins[0], HIGH);
        
        isFlushing = true;
        flushStartMillis = millis();
      }
    } else if (!button1Pressed && button1Held) {
      // Button 1 released - check if it was a short press
      unsigned long holdDuration = millis() - button1HoldStartTime;
      button1Held = false;
      
      if (holdDuration < BUTTON_HOLD_TIME) {
        // Short press - start flush sequence
        SerialBLE_println("Short press - starting flush sequence");
        
        // Calculate sequence timing and reset LED state
        calculateSequenceTiming();
        ledIndex = 0;
        clockwise = true; // Start clockwise when flushing begins.
        ledLastUpdateMillis = millis();  // Reset the timer
        
        // Turn off all LEDs first, then turn on the first one
        for (int i = 0; i < totalLeds; i++) {
          mcp_digitalWrite(ledPins[i], LOW);
        }
        mcp_digitalWrite(ledPins[0], HIGH);
        
        isFlushing = true;
        flushStartMillis = millis();
      }
    }
    
    // Handle button 2 (Feed Down) and motor3 (fan) - single else-if chain to prevent conflicts
    /* One sequential else-if chain handling:
    Button2 just pressed → start delay timer
    Button2 pressed and delay passed → start M2 feed motor and M3 fan
    Button2 pressed continuously → keep both motors running
    Button2 just released → stop M2, start fan timer, keep M3 running
    Fan timer active (< fanDuration) → keep M3 running
    Fan timer completed (>= fanDuration) → stop M3
    Default → ensure M3 is stopped (only if not flushing) */
    if (button2Pressed && !button2WasPressed) {
      // Button 2 just pressed - start delay timer
      button2PressStartTime = millis();
      button2DelayActive = true;
      SerialBLE_println("Button 2 pressed - waiting for delay before starting feed");
      // Don't change motors yet - wait for delay
    } else if (button2Pressed && button2DelayActive && (millis() - button2PressStartTime >= BUTTON_DELAY)) {
      // Delay passed while button still pressed - start fan first
      button2DelayActive = false;
      button2FeedStarted = false;
      button2FanStartTime = millis();
      SerialBLE_println("Feed Down Delay Passed - Starting fan, waiting before feed");
      setM3Speed(400);  // Start M3 fan immediately
      fanRunning = false;  
    } else if (button2Pressed && !button2DelayActive && !button2FeedStarted) {
      // Button 2 pressed, delay passed, but M2 not started yet - wait for preFeedFan
      if (millis() - button2FanStartTime >= preFeedFan * 1000) {
        SerialBLE_println("Pre-feed fan delay complete, starting feed motor");
        motors.setM2Speed(400);
        button2FeedStarted = true;
      }
    } else if (button2Pressed && !button2DelayActive && button2FeedStarted) {
      SerialBLE_println("b2 pressed, feed running - keep motors running");
      // Button 2 is pressed, delay has passed, and feed has started - keep motors running
      //motors.setM2Speed(400);  // Ensure M2 is running
      //setM3Speed(400);  // Continuously ensure M3 at full power
      fanRunning = false;  // Prevent fan timer from interfering
    } else if (!button2Pressed && button2WasPressed) {
      // Button 2 just released - stop feed motor, start fan timer
      SerialBLE_println("Feed Down Released - Stopping feed, starting fan timer");
      motors.setM2Speed(0);
      button2FeedStarted = false;  // Reset flag for next button press
      button2FanStartTime = 0;
      fanStartTime = millis();  // Start fan timer
      fanRunning = true;        // Set fan running flag
      //setM3Speed(255);  // Keep fan running during timer
    } else if (!button2Pressed && fanRunning && (millis() - fanStartTime < fanDuration * 1000)) {
      SerialBLE_println("b2 not pressed, fan running, no time out - keep running ");
      // Fan timer is active - keep M3 running
      //setM3Speed(255);  // Keep fan running
    } else if (!button2Pressed && fanRunning && (millis() - fanStartTime >= fanDuration * 1000)) {
      // Fan timer completed - stop fan
      SerialBLE_println("b2 not pressed, fan running, time out ");
      Serial.println("Fan timer completed - stopping fan");
      setM3Speed(0);  // Stop M3 motor/fan
      fanRunning = false;  // Clear fan running flag
    } else {
      // Default: button not pressed, no active fan - ensure M3 (fan) is stopped
      // Only if not flushing (flush sequence controls M3 during flush)
      if (!isFlushing) {
        setM3Speed(0);
        //SerialBLE_println("stopping fan - no conditions met and not flushing");
        fanRunning = false;
      }
      
    }
  }  // Close if (!batteryDisplayMode)
  
  // Clear delay flags if buttons released before delay completes
  if (!button1Pressed && button1DelayActive) {
    button1DelayActive = false;
    Serial.println("Button 1 released before delay - cancelled");
  }
  if (!button2Pressed && button2DelayActive) {
    button2DelayActive = false;
    Serial.println("Button 2 released before delay - cancelled");
  }
  
  button1WasPressed = button1Pressed;
  button2WasPressed = button2Pressed;
  if (mechanismMotorRunning && (millis() - motorStartMillis > TIMEOUT) && ERROR_CODE == 0) {
    SerialBLE_println("Mechanism Motor Running Timeout in loop");
    ERROR_CODE = 1;
    stopEverything();
    LEDErrorCode(ERROR_CODE);
  }
  if (heaterOn) {
    updateHeaterPID();
    //SerialBLE_print("DEBUG: K value = ");
    //SerialBLE_println(K);
    //SerialBLE_print("DEBUG: Overheat threshold = ");
    //SerialBLE_println(K * 1.2);
    if (readTemperature() >= K * 1.2 && ERROR_CODE == 0) {
      SerialBLE_println("Heater Too Hot - stopping");
      ERROR_CODE = 3;
      stopEverything();
      LEDErrorCode(ERROR_CODE);
    }
  }
  if (isFlushing) {
    flushSequence();
    updateLEDs();
  }
  
  // Stream serial data to BLE if enabled (only when BLE is active)
  if (bleEnabled) {
    streamSerialToBLE();
  }
  
  // Check buttons for direction change
  if (mcp_digitalRead(button1Pin) == LOW) { // Button 1 pressed (LOW due to pull-up)
    Serial.println("Button 1 pressed: Clockwise");
    clockwise = true;
    ledIndex = 0;
    ledLastUpdateMillis = millis();
  }
  if (mcp_digitalRead(button2Pin) == LOW) { // Button 2 pressed
    Serial.println("Button 2 pressed: Anticlockwise");
    clockwise = false;
    ledIndex = 0;
    ledLastUpdateMillis = millis();
  }
}

void flushSequence() {
  unsigned long currentMillis = millis();
  
  // Calculate total heater time (H + CUT_MODE_HEAT_TIME if in cut mode)
  long totalHeaterTime = H;
  if (cutBag) {
    totalHeaterTime += (long)CUT_MODE_HEAT_TIME;
    Serial.printf("DEBUG: Cut mode - Total heater time: %ld seconds (H=%ld + CUT_MODE_HEAT_TIME=%.1f)\n", 
                 totalHeaterTime, H, CUT_MODE_HEAT_TIME);
  }
 
  switch (flushStep) {
    case 0:
      ledLastUpdateMillis = millis();
      Serial.println("Checking battery voltage");
      if (getBatteryChargeLevel() < batteryThreshold) { 
       stopEverything();
       flushStep = 12;
       ERROR_CODE = 2;
       LEDErrorCode(ERROR_CODE);
       return;
      }
      checkAllMotorFaults();
      if (ERROR_CODE != 0) {
        return;
      }
      // Test heater current detection at start of flush cycle
      testHeaterCurrent();
      if (ERROR_CODE != 0) {
        return;
      }
      case1FeedStarted = false;  // Reset flag for case 1
      flushStep++;
      SerialBLE_println("Moving to Case1:");
      SerialBLE_println(millis());
      SerialBLE_println("Starting fan, waiting before feed");
      stepStartMillis = currentMillis;
      setM3Speed(400);  // Start M3 fan immediately
      break;

    case 1:
      currentMillis = millis();
      
      // Start M2 feed motor after preFeedFan delay
      if (!case1FeedStarted && (currentMillis - stepStartMillis >= preFeedFan * 1000)) {
        SerialBLE_println("Pre-feed fan delay complete, starting feed motor");
        motors.setM2Speed(400);
        case1FeedStarted = true;
      }
      
      // Check if feed time (F) has elapsed (counted from when M2 starts, not from case 1 start)
      if (case1FeedStarted && (currentMillis - stepStartMillis >= (preFeedFan + F) * 1000)) {
        flushStep++;
        SerialBLE_print("Moving to Case2:");
        SerialBLE_println(millis());
        motors.setM2Speed(0);
        // Don't stop M3 here - case 2 will set it to reverse immediately
        stepStartMillis = currentMillis;
      }
      break;

    case 2:
      Serial.println("Close motor on");
      mechanismMotorRunning = true;
      motorStartMillis = millis();
      m1CloseStartTime = millis(); // Record when M1 starts closing
      motors.setM1Speed(-400);
      // M3 reverse will start in case 3 after fanReverseStartTime delay
      m3ReverseActive = false; // Reset M3 reverse state
      m3ReverseCompleted = false; // Reset completion flag
      Serial.print("MOTOR START TIME: ");
      Serial.println(motorStartMillis);
      flushStep++;
      SerialBLE_print("Moving to Case3:");
      Serial.println(millis());
      SerialBLE_print("MOTOR START MILLIS: ");
      SerialBLE_println(motorStartMillis);
      break;

    case 3:
      SerialBLE_println("Heater on");
      mcp_digitalWrite(ledPins[0], HIGH);
      heaterOn = true;
      updateHeaterPID();
      
      // Check if it's time to start M3 reverse (based on fanReverseStartTime percentage of typicalOpeningTime)
      if (!m3ReverseActive && !m3ReverseCompleted && m1CloseStartTime > 0) {
        unsigned long delayMs = (unsigned long)((fanReverseStartTime / 100.0) * typicalOpeningTime * 1000);
        if (currentMillis - m1CloseStartTime >= delayMs) {
          SerialBLE_println("Starting M3 reverse");
          setM3Speed(-400);  // M3 in reverse at full speed
          m3ReverseStartTime = currentMillis;
          m3ReverseActive = true;
        }
      }
      
      // Check if M3 reverse time has elapsed
      if (m3ReverseActive && (currentMillis - m3ReverseStartTime >= fanReverseTime * 1000)) {
        SerialBLE_println("Stopping M3 reverse - time elapsed");
        setM3Speed(0);
        m3ReverseActive = false;
        m3ReverseCompleted = true; // Mark as completed to prevent restart
      }
      
      stepStartMillis = currentMillis;
      flushStep++;
      SerialBLE_print("Moving to Case4:");
      Serial.println(millis());
      break;

    case 4:
        // Continue checking M3 reverse timing in case 4
        // Check if it's time to start M3 reverse (if not started yet)
        if (!m3ReverseActive && !m3ReverseCompleted && m1CloseStartTime > 0) {
          unsigned long delayMs = (unsigned long)((fanReverseStartTime / 100.0) * typicalOpeningTime * 1000);
          if (currentMillis - m1CloseStartTime >= delayMs) {
            SerialBLE_println("Starting M3 reverse (delayed start)");
            setM3Speed(-400);  // M3 in reverse at full speed
            m3ReverseStartTime = currentMillis;
            m3ReverseActive = true;
          }
        }
        
        // Check if M3 reverse time has elapsed
        if (m3ReverseActive && (currentMillis - m3ReverseStartTime >= fanReverseTime * 1000)) {
          SerialBLE_println("Stopping M3 reverse - time elapsed");
          setM3Speed(0);
          m3ReverseActive = false;
          m3ReverseCompleted = true; // Mark as completed to prevent restart
        }
        
        case5FeedExecuted = false; // Reset flag when entering case 5
        flushStep++;
        SerialBLE_print("Moving to Case5:");
        Serial.println(millis());
      break;

    case 5: {
      // Check M3 reverse timing in case 5 (where we wait for mechanism to close)
      // Check if it's time to start M3 reverse (if not started yet and not completed)
      if (!m3ReverseActive && !m3ReverseCompleted && m1CloseStartTime > 0) {
        unsigned long delayMs = (unsigned long)((fanReverseStartTime / 100.0) * typicalOpeningTime * 1000);
        if (currentMillis - m1CloseStartTime >= delayMs) {
          SerialBLE_println("Starting M3 reverse (delayed start in case 5)");
          setM3Speed(-400);  // M3 in reverse at full speed
          m3ReverseStartTime = currentMillis;
          m3ReverseActive = true;
        }
      }
      
      // Check if M3 reverse time has elapsed
      if (m3ReverseActive && (currentMillis - m3ReverseStartTime >= fanReverseTime * 1000)) {
        SerialBLE_println("Stopping M3 reverse - time elapsed (in case 5)");
        setM3Speed(0);
        m3ReverseActive = false;
        m3ReverseCompleted = true; // Mark as completed to prevent restart
      }
      
      // Primary check: microswitch closed
        float m1Current = readM1Current();
        SerialBLE_print("M1 Current: ");
        SerialBLE_print(m1Current);
        SerialBLE_println(" A");
      
      if (digitalRead(microswitchClosePin) == LOW) {
        SerialBLE_print("Microswitch closed - M1 Current: ");
        SerialBLE_print(m1Current);
        SerialBLE_println(" A");
        
        // Only execute feed once per case 5
        if (!case5FeedExecuted) {
        motors.setM2Speed(400);
          SerialBLE_println("mechanism closed,feed to relieve strain ");
        delay(300); // feed a bit longer
        motors.setM2Speed(0);
          case5FeedExecuted = true; // Mark as executed
        }
        
        // Normal mode: current > 0.5A is acceptable
        if (m1Current > 0.5) {
          SerialBLE_println("Normal mode: Current > 0.5A confirmed");
        motors.setM1Speed(0);
        setM3Speed(0);  // Stop M3 motor
        m3ReverseActive = false;  // Clear M3 reverse flag
        mechanismMotorRunning = false; 
        } else {
          SerialBLE_println("Normal mode: Low current detected, waiting for current > 0.5A");
          return; // Stay in case 5 until current threshold met
        }
        
       
        
        SerialBLE_println("switch closed, stop feeding and closing");
        flushStep++;
      }
      break;
    }

    case 6:
      
      if ((currentMillis - stepStartMillis >= totalHeaterTime * 1000)) {
        SerialBLE_println("Heater time complete - Begin cooling");
        
        // If cut bag mode is enabled, run cut motor now
        if (cutBag) {
          SerialBLE_println("Cut bag mode: Running cut motor after heating");
          Serial.printf("DEBUG: MOTOR_CUT_TIME = %.3f seconds\n", MOTOR_CUT_TIME);
          Serial.printf("DEBUG: delay = %d ms\n", int(MOTOR_CUT_TIME * 1000));
          motors.setM1Speed(-400); // Full speed for cutting
          delay(int(MOTOR_CUT_TIME * 1000)); // Run for MOTOR_CUT_TIME seconds
          motors.setM1Speed(0); // Stop cut motor
          SerialBLE_println("Cut motor stopped");
        } else {
          SerialBLE_println("DEBUG: cutBag is false, skipping cut motor");
        }
        
        flushStep++;
        SerialBLE_print("Moving to Case7 from case 6: ");
        SerialBLE_println(millis());
        
        SerialBLE_println("Heater off");
        heaterOn = false;
        heaterOff();
        stepStartMillis = currentMillis;
      }
      break;

    case 7:
      if (currentMillis - stepStartMillis >= T * 1000) {
        Serial.print(currentMillis);
        SerialBLE_println("  Cooling complete opening sealer");
        mechanismMotorRunning = true;
        motorStartMillis = millis();
        motors.setM1Speed(400);
        SerialBLE_print("MOTOR START TIME: ");
        Serial.println(motorStartMillis);
        motors.setM2Speed(-400);
        flushStep++;
        SerialBLE_print("Moving to Case 8: ");
        SerialBLE_println(millis());
        stepStartMillis = currentMillis;
      }
      break;

    case 8:
      if (currentMillis - stepStartMillis >= backupTime * 1000) {
        Serial.println("Stop backing bag up");
        motors.setM2Speed(0);
        flushStep++;
        SerialBLE_print("Moving to Case9: ");
        SerialBLE_println(millis());
      }
      break;

    case 9:
      case10FanStarted = false;  // Reset flag for case 10
      case10BackupStarted = false;  // Reset flag for case 10 backup
      flushStep++;
      SerialBLE_print("Moving to Case10: ");
      Serial.println(millis());
      break;

    case 10:
      if ((digitalRead(microswitchOpenPin) == LOW)) {
        motors.setM1Speed(0);
        mechanismMotorRunning = false;
        
        // First, start backup if not started yet
        if (!case10BackupStarted) {
          SerialBLE_println("Stop opening, starting backup");
          motors.setM2Speed(-400);  // M2 in reverse for backup
          case10BackupStartTime = currentMillis;
          case10BackupStarted = true;
        }
        
        // Wait for backupTimeAfterReopen before proceeding to fan
        if (case10BackupStarted && (currentMillis - case10BackupStartTime >= backupTimeAfterReopen * 1000)) {
          // Stop backup and start fan (only once)
          if (!case10FanStarted) {
            SerialBLE_println("Backup complete, stopping backup and activating fan");
            motors.setM2Speed(0);  // Stop backup
            setM3Speed(400);  // M3 at full power
            stepStartMillis = currentMillis;
            case10FanStarted = true;
          }
        }
        
        // Wait for postCoolingBagDuration before starting feed motors (only after backup and fan started)
        if (case10BackupStarted && case10FanStarted && (currentMillis - stepStartMillis >= postCoolingBagDuration * 1000)) {
          SerialBLE_println("Fan delay complete, starting feed motors");
          motors.setM2Speed(400);
          stepStartMillis = currentMillis;
          case10FanStarted = false;  // Reset for next cycle
          case10BackupStarted = false;  // Reset for next cycle
          flushStep++;
          SerialBLE_print("Moving to Case11:");
          Serial.println(millis());
        }
      }
      break;

    case 11:
      if (currentMillis - stepStartMillis >= continueFeeder * 1000) {
        SerialBLE_println("STOP Feeding bag, keep fan running");
        motors.setM2Speed(0);
        // Keep M3 running - don't stop it here, case 12 will handle fan duration
        stepStartMillis = currentMillis;
        flushStep++;
        SerialBLE_print("Moving to Case12:");
        SerialBLE_println(millis());
      }
      break;

    case 12:
      if (currentMillis - stepStartMillis >= fanDuration * 1000) {
        SerialBLE_println("STOP fan after fanDuration");
        setM3Speed(0);// Stop M3 motor/fan
        flushStep++;
        SerialBLE_print("Moving to Case13:");
        SerialBLE_println(millis());
      }
      break;

    case 13:
      Serial.println("All LEDs off");
      for (int i = 0; i < totalLeds; i++) {
        mcp_digitalWrite(ledPins[i], LOW);
      }
      isFlushing = false;
      flushStep = 0;
      cutBag = false;  // Reset cutBag for next cycle
      SerialBLE_println("Moving to Case0 from 13 - end of sequence:");
      Serial.println(millis());
      break;
  }
}

void flashLowBattLeds(int n) {
  for (int i = 0; i < n; i++) {
    mcp_digitalWrite(5, HIGH);
    delay(200);
    mcp_digitalWrite(5, LOW);
    delay(200);
  }
}

void flashLeds() {
  unsigned int duration = 200;
  for (int j = 0; j < 5; j++) {
    for (int i = 0; i < totalLeds; i++) {
      mcp_digitalWrite(ledPins[i], HIGH);
    }
    delay(duration);
    for (int i = 0; i < totalLeds; i++) {
      mcp_digitalWrite(ledPins[i], LOW);
    }
    delay(duration);
  }
}

void circleLeds() {
  unsigned int duration = 40;
  for (int j = 0; j < 4; j++) {
    for (int i = 0; i < totalLeds; i++) {
      mcp_digitalWrite(ledPins[i], HIGH);
      delay(duration);
      mcp_digitalWrite(ledPins[i], LOW);
    }
  }
}

// Slow circle LED animation for OTA mode (continuous, slow)
void slowCircleLeds() {
  unsigned long currentMillis = millis();
  if (currentMillis - slowCircleLastUpdate >= SLOW_CIRCLE_INTERVAL) {
    slowCircleLastUpdate = currentMillis;
    
    // Turn off previous LED
    mcp_digitalWrite(ledPins[slowCircleLedIndex], LOW);
    
    // Move to next LED (forward direction)
    slowCircleLedIndex = (slowCircleLedIndex + 1) % totalLeds;
    
    // Turn on current LED
    mcp_digitalWrite(ledPins[slowCircleLedIndex], HIGH);
  }
}

// Enable OTA mode - create service dynamically
void enableOTA() {
  if (otaEnabled) {
    Serial.println("OTA already enabled - returning");
    return; // Already enabled
  }
  
  Serial.println("\n=== ENABLING OTA MODE ===");
  sendSerialToBLE("\n=== ENABLING OTA MODE ===");
  delay(10);
  
  Serial.println("=== MEMORY DIAGNOSTICS ===");
  sendSerialToBLE("=== MEMORY DIAGNOSTICS ===");
  
  size_t freeHeap = ESP.getFreeHeap();
  size_t maxAlloc = ESP.getMaxAllocHeap();
  size_t minFree = ESP.getMinFreeHeap();
  size_t heapSize = ESP.getHeapSize();
  
  Serial.printf("Free heap: %d bytes\n", freeHeap);
  sendSerialToBLE("Free heap: " + String(freeHeap) + " bytes");
  
  Serial.printf("Largest free block: %d bytes\n", maxAlloc);
  sendSerialToBLE("Largest free block: " + String(maxAlloc) + " bytes");
  
  Serial.printf("Min free heap ever: %d bytes\n", minFree);
  sendSerialToBLE("Min free heap ever: " + String(minFree) + " bytes");
  
  Serial.printf("Heap size: %d bytes\n", heapSize);
  sendSerialToBLE("Heap size: " + String(heapSize) + " bytes");
  
  // Check for memory fragmentation
  float fragmentation = ((float)(freeHeap - maxAlloc) / freeHeap) * 100.0;
  Serial.printf("Memory fragmentation: %.1f%%\n", fragmentation);
  sendSerialToBLE("Memory fragmentation: " + String(fragmentation, 1) + "%");
  
  // Check if we have enough memory (BLE services typically need ~10-20KB)
  if (maxAlloc < 20000) {
    Serial.println("WARNING: Largest free block is less than 20KB - may cause issues!");
    sendSerialToBLE("WARNING: Largest free block is less than 20KB - may cause issues!");
  }
  
  // Stop advertising temporarily
  if (bleEnabled && blue_server) {
    blue_server->getAdvertising()->stop();
  }
  
  // Create update service dynamically
  if (update_service == NULL) {
    Serial.println("Creating OTA service...");
    sendSerialToBLE("Creating OTA service...");
    update_service = blue_server->createService(UPDATE_SERVICE_UUID);
    
    freeHeap = ESP.getFreeHeap();
    Serial.printf("Free heap AFTER createService: %d bytes\n", freeHeap);
    sendSerialToBLE("Free heap AFTER createService: " + String(freeHeap) + " bytes");
    
    // Create update characteristic
    update_characteristic = update_service->createCharacteristic(
      UPDATE_CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_READ |
      BLECharacteristic::PROPERTY_WRITE |
      BLECharacteristic::PROPERTY_NOTIFY
    );
    
    freeHeap = ESP.getFreeHeap();
    Serial.printf("Free heap AFTER createCharacteristic: %d bytes\n", freeHeap);
    sendSerialToBLE("Free heap AFTER createCharacteristic: " + String(freeHeap) + " bytes");
    
    update_characteristic->setValue("OTA_READY");
    
    Serial.println("Creating callback object...");
    sendSerialToBLE("Creating callback object...");
    update_characteristic->setCallbacks(new update_characteristic_callbacks());
    
    freeHeap = ESP.getFreeHeap();
    Serial.printf("Free heap AFTER setCallbacks: %d bytes\n", freeHeap);
    sendSerialToBLE("Free heap AFTER setCallbacks: " + String(freeHeap) + " bytes");
    
    // Start update service
    update_service->start();
    
    freeHeap = ESP.getFreeHeap();
    Serial.printf("Free heap AFTER start service: %d bytes\n", freeHeap);
    sendSerialToBLE("Free heap AFTER start service: " + String(freeHeap) + " bytes");
  }
  
  // Add OTA service to advertising
  BLEAdvertising * blue_advert = blue_server->getAdvertising();
  blue_advert->addServiceUUID(UPDATE_SERVICE_UUID);
  blue_advert->setScanResponse(true);
  blue_advert->setMinPreferred(0x06);
  blue_advert->setMinPreferred(0x12);
  
  // Restart advertising
  Serial.println("Restarting advertising...");
  sendSerialToBLE("Restarting advertising...");
  BLEDevice::startAdvertising();
  
  freeHeap = ESP.getFreeHeap();
  maxAlloc = ESP.getMaxAllocHeap();
  Serial.printf("Free heap AFTER startAdvertising: %d bytes\n", freeHeap);
  sendSerialToBLE("Free heap AFTER startAdvertising: " + String(freeHeap) + " bytes");
  
  // Final memory check
  fragmentation = ((float)(freeHeap - maxAlloc) / freeHeap) * 100.0;
  Serial.printf("Final memory fragmentation: %.1f%%\n", fragmentation);
  sendSerialToBLE("Final memory fragmentation: " + String(fragmentation, 1) + "%");
  
  Serial.println("=== OTA MODE ENABLED ===");
  sendSerialToBLE("=== OTA MODE ENABLED ===");
  
  otaEnabled = true;
  otaWindowStartTime = millis();
  slowCircleLedIndex = 0;
  slowCircleLastUpdate = millis();
  
  // Turn off all LEDs first
  for (int i = 0; i < totalLeds; i++) {
    mcp_digitalWrite(ledPins[i], LOW);
  }
  
  Serial.println("OTA mode enabled - window open for 1 minute");
}

// Disable OTA mode
void disableOTA() {
  if (!otaEnabled) {
    return; // Already disabled
  }
  
  Serial.println("Disabling OTA mode");
  
  // Stop advertising
  if (blue_server) {
    blue_server->getAdvertising()->stop();
  }
  
  // Note: We can't easily remove the service, but we can stop using it
  // The service will remain in memory but won't be advertised
  otaEnabled = false;
  updateInProgress = false;
  otaState = OTA_IDLE;
  
  // Turn off all LEDs
  for (int i = 0; i < totalLeds; i++) {
    mcp_digitalWrite(ledPins[i], LOW);
  }
  
  Serial.println("OTA mode disabled");
}

// Restart BLE server without OTA
void restartBLEServer() {
  Serial.println("Restarting BLE server (without OTA)");
  
  disableOTA();
  
  // Stop current advertising
  if (blue_server) {
    blue_server->getAdvertising()->stop();
  }
  
  // Restart advertising - OTA service exists but won't be advertised
  // (we can't easily remove it from advertising once added)
  BLEDevice::startAdvertising();
  
  bleEnabled = true;
  bleStartupTime = millis();
  is_device_connected = false;
  serial_streaming_enabled = false;
  
  Serial.println("BLE server restarted - ready for normal connections");
}

float readTemperature() {
  int analogValue = analogRead(thermistorPin);
  //SerialBLE_print("analog temperature value:");
  //SerialBLE_println(analogValue);
  float voltage = analogValue * (3.3 / 4095.0);
  //SerialBLE_print("voltage:");
  //SerialBLE_println(voltage);
  float resistance = (voltage * knownResistor) / (3.3 - voltage);
  //SerialBLE_print("resistance:");
  //SerialBLE_println(resistance);
  float logR = log(resistance);
  float tempKelvin = 1.0 / (A + B * logR + C * logR * logR * logR);
  return tempKelvin - 273.15;
}

float readBatteryVoltage() {
  int analogValue = analogRead(batteryVoltagePin);
  // Convert ADC reading to voltage (0-3.3V)
  float voltage = analogValue * (3.3 / 4095.0);
  
  // Voltage divider: 12V battery -> VMON pin
  // R29 = 10kΩ, R28 = 2.2kΩ
  // Battery voltage = VMON voltage * (R29 + R28) / R28
  // Battery = VMON * (10k + 2.2k) / 2.2k = VMON * 5.545
  float batteryVoltage = voltage * 7.317; //Changed to 7.317, from 5.545
  
  // Debug output to troubleshoot
  
  SerialBLE_print("Battery Debug analog val: ");
  SerialBLE_println(analogValue);
  SerialBLE_print("Battery Debug voltage: ");
  SerialBLE_println(voltage);
  SerialBLE_print("Battery voltage: ");
  SerialBLE_println(batteryVoltage);
  
  //Serial.printf("Battery Debug: ADC=%d, VMON=%.3fV, Battery=%.2fV\n", analogValue, voltage, batteryVoltage);
  
  return batteryVoltage;
}

float readBatteryTemperature() {
  int analogValue = analogRead(batteryTempPin);
  
  // Step 1: Convert ADC reading to voltage (0-3.3V)
  float voltage = analogValue * (3.3 / 4095.0);
  
  // Step 2: Calculate thermistor resistance using voltage divider formula
  // Voltage divider: 3.3V -> 10kΩ pull-up -> NTC thermistor -> GND
  // Formula: R_thermistor = R_pullup * (V_supply - V_measured) / V_measured
  float thermistorResistance = 10000.0 * (3.3 - voltage) / voltage;
  
  // Step 3: Steinhart-Hart equation for NTC thermistor temperature calculation
  // Using typical 10kΩ NTC thermistor coefficients (B25/85 = 3950K)
  // Formula: 1/T = 1/B * ln(R/R0) + 1/T0
  // Where: B = 3950K, R0 = 10000Ω, T0 = 25°C = 298.15K
  float ln_ratio = log(thermistorResistance / 10000.0);
  float steinhart = ln_ratio / 3950.0 + 1.0 / (25.0 + 273.15);
  float temperature = (1.0 / steinhart) - 273.15;
  
  // Detailed debug output to Serial
  Serial.println("=== BATTERY TEMPERATURE CALCULATION DEBUG ===");
  Serial.printf("Step 1 - ADC Reading: %d (0-4095)\n", analogValue);
  Serial.printf("Step 2 - Voltage: %.3fV (ADC * 3.3/4095)\n", voltage);
  Serial.printf("Step 3 - Thermistor Resistance: %.0fΩ\n", thermistorResistance);
  Serial.printf("Step 4 - ln(R/R0): %.4f\n", ln_ratio);
  Serial.printf("Step 5 - Steinhart: %.6f\n", steinhart);
  Serial.printf("Step 6 - Temperature: %.1f°C\n", temperature);
  Serial.println("=============================================");
  
  // Debug output to Bluetooth
  SerialBLE_println("=== BATTERY TEMP DEBUG ===");
  SerialBLE_print("ADC: ");
  SerialBLE_print(analogValue);
  SerialBLE_print(", Voltage: ");
  SerialBLE_print(voltage);
  SerialBLE_print("V, Resistance: ");
  SerialBLE_print(thermistorResistance);
  SerialBLE_print("Ω, Temp: ");
  SerialBLE_print(temperature);
  SerialBLE_println("°C");
  
  return temperature;
}

int getBatteryChargeLevel() {
  float batteryVoltage = readBatteryVoltage();
  
  // Linear percentage calculation: 11.0V = 0%, 12.6V = 100%
  if (batteryVoltage >= 12.6) return 100;        // Cap at 100%
  else if (batteryVoltage <= 11.0) return 0;    // Cap at 0%
  else {
    // Linear interpolation: ((voltage - 11.0) / 1.6) * 100
    int percentage = (int)((batteryVoltage - 11.0) / 1.6 * 100);
    return percentage;
  }
}

void displayBatteryChargeLevel() {
  int chargeLevel = getBatteryChargeLevel();
  float batteryVoltage = readBatteryVoltage();
  float batteryTemp = readBatteryTemperature();
  
  SerialBLE_print("Battery Voltage: ");
  SerialBLE_print(batteryVoltage);
  SerialBLE_print("V, Charge Level: ");
  SerialBLE_print(chargeLevel);
  SerialBLE_print("%, Temperature: ");
  SerialBLE_print(batteryTemp);
  SerialBLE_println("°C");
  
  // Additional debug info
  Serial.printf("DEBUG: Charge level = %d%%, Battery voltage = %.2fV, Temperature = %.1f°C\n", 
                chargeLevel, batteryVoltage, batteryTemp);
  
  // Turn off all LEDs first
  for (int i = 0; i < totalLeds; i++) {
    mcp_digitalWrite(ledPins[i], LOW);
  }
  
  // Calculate how many LEDs to light based on charge level
  // LED 1 (index 0) = 100%, LED 14 (index 13) = 0%
  // For 50% charge, LEDs 7-14 (indices 6-13) should be illuminated
  int ledsToLight = (chargeLevel * totalLeds) / 100;
  
  // Light up LEDs from the bottom (LED 14 = index 13) upwards
  // Start from the highest index and work down
  for (int i = totalLeds - ledsToLight; i < totalLeds; i++) {
    mcp_digitalWrite(ledPins[i], HIGH);
  }
  
  // Flash the last LED if charge is very low (0-20%)
  if (chargeLevel <= 20) {
    for (int flash = 0; flash < 3; flash++) {
      mcp_digitalWrite(ledPins[totalLeds - 1], HIGH);
      delay(200);
      mcp_digitalWrite(ledPins[totalLeds - 1], LOW);
      delay(200);
    }
  }
}

void flashLEDsAcknowledgment() {
  // Flash all LEDs once to acknowledge cutBag command
  for (int i = 0; i < totalLeds; i++) {
    mcp_digitalWrite(ledPins[i], HIGH);
  }
  delay(500);
  for (int i = 0; i < totalLeds; i++) {
    mcp_digitalWrite(ledPins[i], LOW);
  }
}

void cutModeLEDAnimation() {
  // LED arrangement: left to right with paired LEDs
  // Pattern: 4 -> (5,3) -> (6,2) -> (7,1) -> (8,14) -> (9,13) -> (10,12) -> 11
  // Then reverse: 11 -> (10,12) -> (9,13) -> (8,14) -> (7,1) -> (6,2) -> (5,3) -> 4
  // Run 3 times, then 200ms delay before flush sequence
  
  const int animationDelay = 75; // 600ms / 8 steps = 75ms per step
  
  // Define LED pairs for left-to-right animation
  int leftLEDs[] = {3, 2, 1, 0, 13, 12, 11, 10};
  int rightLEDs[] = {-1, 4, 5, 6, 7, 8, 9, -1}; // -1 means no paired LED
  
  for (int cycle = 0; cycle < 3; cycle++) {
    // Turn on LEDs from left to right
    for (int i = 0; i < 8; i++) {
      mcp_digitalWrite(ledPins[leftLEDs[i]], HIGH);
      if (rightLEDs[i] != -1) {
        mcp_digitalWrite(ledPins[rightLEDs[i]], HIGH);
      }
      delay(animationDelay);
    }
    
    // Turn off LEDs from right to left
    for (int i = 7; i >= 0; i--) {
      mcp_digitalWrite(ledPins[leftLEDs[i]], LOW);
      if (rightLEDs[i] != -1) {
        mcp_digitalWrite(ledPins[rightLEDs[i]], LOW);
      }
      delay(animationDelay);
    }
  }
  
  // 200ms delay before flush sequence starts
  delay(200);
}


void stopEverything() {
  
  SerialBLE_println("stop everything");
  motors.setM2Speed(0);
  motors.setM1Speed(0);
  setM3Speed(0);  // Stop M3 motor
  heaterOff();
  flushStep = 12;
  isFlushing = false;
}

void calculateSequenceTiming() {
  // Calculate total sequence time from parameters
  // Case 1: F seconds (feed down)
  // Case 2: Immediate (stop feed)
  // Case 3: Immediate (start close motor)
  // Case 4: Immediate (heater on)
  // Case 5: Variable (wait for microswitch + current)
  // Case 6: H seconds (heater time)
  // Case 7: T seconds (cooling time)
  // Case 8: backupTime seconds (back up bag)
  // Case 9: Immediate
  // Case 10: Variable (wait for microswitch)
  // Case 11: F seconds (feed down)
  // Case 12: fanDuration seconds (fan time)
  // Case 13: Immediate (end)
  
  // Calculate total heater time (H + CUT_MODE_HEAT_TIME if in cut mode)
  long totalHeaterTime = H;
  if (cutBag) {
    totalHeaterTime += (long)CUT_MODE_HEAT_TIME;
  }
  
  totalSequenceTime = F + totalHeaterTime + T + backupTime + F + fanDuration;
  
  // Calculate LED update interval to evenly distribute across sequence
  // Use totalLeds - 1 to ensure last LED turns on near the end
  ledUpdateInterval = (totalSequenceTime * 1000) / (totalLeds - 1);
  
  Serial.printf("Total sequence time: %lu seconds\n", totalSequenceTime);
  Serial.printf("LED update interval: %lu ms\n", ledUpdateInterval);
}

void updateLEDs() {
  unsigned long currentMillis = millis();
  if (currentMillis - ledLastUpdateMillis >= ledUpdateInterval) {
    ledLastUpdateMillis = currentMillis;
    Serial.print("Turning on LED index: ");
    Serial.println(ledIndex);
    Serial.print("millis: ");
    Serial.println(currentMillis);
    
    // Update the ledIndex based on direction
    if (clockwise) {
      ledIndex = (ledIndex + 1) % totalLeds;
    } else {
      ledIndex = (ledIndex - 1 + totalLeds) % totalLeds; // Corrected for underflow
    }
    // Turn on the current LED (keep all previous LEDs on)
    mcp_digitalWrite(ledPins[ledIndex], HIGH);
  }
}

void locateMotorPos() {
  Serial.println("Ensuring Closing motor is at a known position...");
  bool motorOPEN_switchClosed = (digitalRead(microswitchOpenPin) == LOW);
  Serial.print("motor open switch closed (LOW): ");
  Serial.println(motorOPEN_switchClosed);

  mechanismMotorRunning = false; // Initialize motor state
///mechanism open - partially close it
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  if (motorOPEN_switchClosed) {
    Serial.println("Motor is OPEN (switch closed), partially closing to confirm...");
    unsigned long motorStartMillis = millis();
    motors.setM1Speed(-400); // Start closing motor
    mechanismMotorRunning = true;
    Serial.print("MOTOR START TIME (closing): ");
    Serial.println(motorStartMillis);
    Serial.println("Partially closing mechanism...");
    while (digitalRead(microswitchOpenPin) == LOW) {
      if (millis() - motorStartMillis > TIMEOUT) {
        if (ERROR_CODE == 0) {
          ERROR_CODE = 1;
          flashLeds();
          stopEverything();
          LEDErrorCode(ERROR_CODE);
        }
        motors.setM1Speed(0);
        mechanismMotorRunning = false;
        return;
      }
      delay(10);
    }
    delay(500); // make it go a bit further
    motors.setM1Speed(0); // Stop closing motor
    mechanismMotorRunning = false;

    Serial.println("Motor partially closed (open switch is now open).");
  }
    
  // Mecahnism now partially closed
    Serial.println("Motor is NOT fully OPEN (switch open), opening fully...");
    unsigned long motorStartMillis = millis();
    motors.setM1Speed(400); // Start opening
    mechanismMotorRunning = true;
    Serial.print("MOTOR START TIME (opening fully): ");
    Serial.println(motorStartMillis);

    while (digitalRead(microswitchOpenPin) == HIGH) {
      if (millis() - motorStartMillis > TIMEOUT) {
        if (ERROR_CODE == 0) {
          ERROR_CODE = 3;
          flashLeds();
          stopEverything();
          LEDErrorCode(ERROR_CODE);
        }
        motors.setM1Speed(0);
        mechanismMotorRunning = false;
        return;
      }
      delay(10);
    }
    motors.setM1Speed(0); // Stop opening
    mechanismMotorRunning = false;
    Serial.println("Motor opened fully and positioned (open switch closed).");
  
  Serial.println("Motor positioning complete.");
}



void LEDErrorCode(int errorCode) { // Modified to accept errorCode
  Serial.print("Flashing Error Code: ");
  Serial.println(errorCode);
  for (int j = 0; j < totalLeds; j++) {
    mcp_digitalWrite(ledPins[j], LOW);
  }
  for (int j = 0; j < 10; j++) {
    mcp_digitalWrite(ledPins[errorCode], HIGH);

    digitalWrite(buzzerPin, HIGH);
    delay(300);
    mcp_digitalWrite(ledPins[errorCode], LOW);
    digitalWrite(buzzerPin, LOW);
  }
}

void checkMotorFaults() {
  if (motors.getFault() && ERROR_CODE == 0) {
        SerialBLE_println("Motor Fault Detected!");
    ERROR_CODE = 4;
    stopEverything();
    LEDErrorCode(ERROR_CODE);
  }
}

void updateHeaterPID() {
  input = readTemperature();
  myPID.Compute();
  analogWrite(heaterPin, (int)output);
  
  // Read analog voltage from thermistor pin
  int analogValue = analogRead(thermistorPin);
  float voltage = analogValue * (3.3 / 4095.0);
  
  SerialBLE_print("temperature: ");
  SerialBLE_println(input);
  SerialBLE_print("PWM Output: ");
  SerialBLE_println(output);
  SerialBLE_print("Analog Voltage: ");
  SerialBLE_print(voltage);
  SerialBLE_println(" V");
  Serial.printf("Heater Debug: GPIO%d PWM=%d, Setpoint=%.1f, Input=%.1f, Analog=%d, Voltage=%.3fV\n", 
                heaterPin, (int)output, setpoint, input, analogValue, voltage);
}

void heaterOff() {
  analogWrite(heaterPin, 0);
  Serial.print("PWM Output: ");
  Serial.println(output);
  Serial.printf("Heater OFF: GPIO%d PWM=0\n", heaterPin);
  heaterOn = false;
}

// Motor 3 control functions using DualMAX14870MotorShield
void setM3Speed(int speed) {
  motors3.setM1Speed(speed);  // Use M1 channel of second shield for M3
}

void enableM3() {
  motors3.enableDrivers();  // Enable M3 motor shield
}

void disableM3() {
  motors3.disableDrivers(); // Disable M3 motor shield
}

bool getM3Fault() {
  return motors3.getFault();  // Check M3 fault status
}

// Current monitoring function
float readM1Current() {
  int analogValue = analogRead(m1CurrentPin);
  SerialBLE_print("analog current value:"); 
  SerialBLE_println(analogValue);

  // Convert ADC reading to voltage (0-3.3V)
  float voltage = analogValue * (3.3 / 4095.0);
  SerialBLE_print("voltage:");
  SerialBLE_println(voltage);

  // Based on INA169 circuit: Current = (Voltage - 0.5) / (0.1 * Rshunt)
  // Rshunt = 200mΩ, so Current = (Voltage - 0.5) / 0.02
  // Assuming 0.5V offset and 0.1V/A sensitivity
  float current = (voltage) / 2.0; //(no offset observed, 80 A at 0.02)
  
  SerialBLE_print("M1 current:");
  SerialBLE_println(current);
  return current;
}

// Heater current monitoring function
float readHeaterCurrent() {
  int analogValue = analogRead(heaterCurrentPin);
  SerialBLE_print("analog heater current value:"); 
  SerialBLE_println(analogValue);

  // Convert ADC reading to voltage (0-3.3V)
  float voltage = analogValue * (3.3 / 4095.0);
  SerialBLE_print("heater voltage:");
  SerialBLE_println(voltage);

  // Based on INA169 circuit: Current = (Voltage - 0.5) / (0.1 * Rshunt)
  // Rshunt = 200mΩ, so Current = (Voltage - 0.5) / 0.02
  // Assuming 0.5V offset and 0.1V/A sensitivity
  float current = (voltage) / 2.0; //(no offset observed, similar to M1 current)
  
  SerialBLE_print("Heater current:");
  SerialBLE_println(current);
  return current;
}

// Enhanced motor fault checking for all 3 motors
void checkAllMotorFaults() {
  // Add a small delay to ensure motors are properly initialized
  static unsigned long lastCheck = 0;
  if (millis() - lastCheck < 100) {  // Only check every 100ms
    return;
  }
  lastCheck = millis();
  
  if (motors.getFault() && ERROR_CODE == 0) {
    Serial.println("DEBUG: Motor 1 or 2 fault detected - checking motor state");
    SerialBLE_println("Motor 1 or 2 Fault Detected!");
    ERROR_CODE = 4;
    stopEverything();
    LEDErrorCode(ERROR_CODE);
  }
  
  if (getM3Fault() && ERROR_CODE == 0) {
    Serial.println("DEBUG: Motor 3 fault detected - checking motor state");
    SerialBLE_println("Motor 3 Fault Detected!");
    ERROR_CODE = 4;
    stopEverything();
    LEDErrorCode(ERROR_CODE);
  }
}

// Global buffer for serial streaming
String serialBuffer = "";
unsigned long lastSerialFlush = 0;

// Function to send serial data to BLE
void sendSerialToBLE(const String& message) {
  if (is_device_connected && serial_characteristic) {
    serial_characteristic->setValue(message.c_str());
    serial_characteristic->notify();
  }
}

// Overloaded functions for different data types
void sendSerialToBLE(float value) {
  sendSerialToBLE(String(value));
}

void sendSerialToBLE(int value) {
  sendSerialToBLE(String(value));
}

void sendSerialToBLE(unsigned long value) {
  sendSerialToBLE(String(value));
}

// Function to capture and forward Serial output to BLE
void captureSerialOutput() {
  // This function will be called periodically to capture Serial output
  // We'll add explicit BLE forwarding to key Serial.println() calls
}

// Function to stream serial data over BLE
void streamSerialToBLE() {
  if (is_device_connected && serial_streaming_enabled && serial_characteristic) {
    
    // Send any buffered serial data
    if (serialBuffer.length() > 0 && millis() - lastSerialFlush > 100) {
      sendSerialToBLE(serialBuffer);
      serialBuffer = "";
      lastSerialFlush = millis();
    }
  }
}