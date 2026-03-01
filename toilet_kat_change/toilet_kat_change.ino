#include <BLEServer.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <Wire.h>
#include <Adafruit_MCP23X17.h>
#include <DualMAX14870MotorShield.h> // Include the motor driver library
#include <EEPROM.h>     // Include EEPROM library for parameter persistence
#include <esp_ota_ops.h>  // Include OTA operations for updates
#include <nvs_flash.h>   // Include NVS for rollback state storage
#include <nvs.h>
#include <esp_system.h>  // Include for reboot functionality
#include <esp_sleep.h>    // Include for deep sleep and wake
#include <driver/rtc_io.h> // RTC GPIO for wake pin pull-up in deep sleep
#include <driver/gpio.h>   // GPIO hold during deep sleep (fan PWM pin)
#include <mbedtls/md5.h> // Include for MD5 validation
#define LED_PHASE_COUNT 5
//########################################################################
///THIS IS IN THE BLE UPDATED FOLDER FROM FIVERR
//########################################################################
// Pin and component definitions
const int heaterPin = 17;        // GPIO17 (HEATER)

const int buzzerPin = 38;        // GPIO38 (BUZ)
const int microswitchClosePin = 16;   // GPIO16
const int microswitchOpenPin = 15;    // GPIO15
const int controlPanelWake = 2;       // GPIO2 - wake/button1 (direct to main board)

// Motor current monitoring
const int m1CurrentPin = 8;     // GPIO8 (M1_A) - Current sense output

// Heater current monitoring
const int heaterCurrentPin = 12; // GPIO12 (HS_OUT) - Heater current sense output from INA169

// Battery voltage monitoring
const int batteryVoltagePin = 10;  // GPIO10 (VMON) - Battery voltage monitoring
const int batteryTempPin = 5;      // GPIO5 (B_TEMP) - Battery temperature monitoring

// Custom I2C instance
TwoWire myI2C = TwoWire(0);

// MCP23017 setup
Adafruit_MCP23X17 mcp;
bool mcpInitialized = false;
unsigned long lastMcpUnavailableLogMillis = 0;
const unsigned long MCP_UNAVAILABLE_LOG_INTERVAL_MS = 5000;

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
#define FLUSH_COUNT_MAGIC_ADDR 300
#define FLUSH_COUNT_ADDR (FLUSH_COUNT_MAGIC_ADDR + sizeof(uint16_t))
#define FLUSH_COUNT_MAGIC 0xF1C5

// Hardware matrix persistence (NVS)
#define HW_MATRIX_MAGIC 0x484D4154UL  // "HMAT"
#define HW_MATRIX_SCHEMA_VERSION 1
#define HW_COMPONENT_VERSION_LEN 24
#define HW_COMPONENT_DESC_LEN 96
#define HW_COMPONENT_DATE_LEN 11  // YYYY-MM-DD + '\0'

// HWCFG transactional profile storage
#define HWCFG_MAGIC 0x48434647UL  // "HCFG"
#define HWCFG_SCHEMA_VERSION 1
#define HWCFG_PROFILE_MAX 24
#define HWCFG_PROFILE_ID_LEN 24
#define HWCFG_PROFILE_PARAM_BLOB_LEN 320
#define HWCFG_CONFIG_NAMESPACE "hwcfg"
#define HWCFG_ACTIVE_KEY "active"
#define HWCFG_LAST_GOOD_KEY "lkg"

// Version information
struct VersionInfo {
  uint16_t magic;
  uint16_t hardware_version;
  char hardware_description[32];
};

enum HardwareComponentId {
  HW_CONTROL_PANEL = 0,
  HW_HEATING_ELEMENT,
  HW_MAIN_CIRCUIT_BOARD,
  HW_VACUUM_FAN,
  HW_FEED_MOTOR,
  HW_MECHANISM_MOTOR,
  HW_THERMISTOR,
  HW_BATTERY,
  HW_FACTORY_SOFTWARE_DATE,
  HW_FACTORY_SOFTWARE_VERSION_NUMBER,
  HW_COMPONENT_COUNT
};

struct HardwareComponentEntry {
  char current_version[HW_COMPONENT_VERSION_LEN];
  char current_description[HW_COMPONENT_DESC_LEN];
  char install_date[HW_COMPONENT_DATE_LEN];
  char previous_version[HW_COMPONENT_VERSION_LEN];
  char previous_description[HW_COMPONENT_DESC_LEN];
  char previous_install_date[HW_COMPONENT_DATE_LEN];
};

struct HardwareMatrix {
  uint32_t matrix_magic;
  uint16_t matrix_schema_version;
  uint16_t component_count;
  HardwareComponentEntry components[HW_COMPONENT_COUNT];
  uint32_t crc32;
};

struct HWCFGProfileEntry {
  uint8_t in_use;
  char profile_id[HWCFG_PROFILE_ID_LEN];
  char component_name[HW_COMPONENT_DESC_LEN];
  char component_version[HW_COMPONENT_VERSION_LEN];
  char params_blob[HWCFG_PROFILE_PARAM_BLOB_LEN];
};

struct HWCFGConfigStore {
  uint32_t magic;
  uint16_t schema_version;
  uint16_t profile_count;
  char active_profile_id[HWCFG_PROFILE_ID_LEN];
  char last_good_profile_id[HWCFG_PROFILE_ID_LEN];
  uint8_t active_validated;
  HWCFGProfileEntry profiles[HWCFG_PROFILE_MAX];
  uint32_t crc32;
};

const char* SOFTWARE_VERSION = "2.4.3"; //HOPEFULLY FIX MD5
const char* SOFTWARE_BUILD_DATE = __DATE__ " " __TIME__;

// Forward declarations
void sendSerialToBLE(const String& message);
void sendSerialToBLE(float value);
void sendSerialToBLE(int value);
void sendSerialToBLE(unsigned long value);
String buildMotorFaultStatusSnapshot();
void saveParametersToEEPROM();
void loadParametersFromEEPROM();
void loadFlushCountFromEEPROM();
bool saveFlushCountToEEPROM();
void incrementFlushCount();
void initializeHardwareVersion();
VersionInfo readHardwareVersion();
void writeHardwareVersion(uint16_t version, const char* description);
bool initializeHardwareMatrix();
String getVersionString();
String getHardwareComponentsListString();
String getHardwareComponentString(HardwareComponentId componentId);
bool setHardwareComponentByName(const String& componentName, const String& version, const String& installDate, const String& description, String& errorCode);
bool lookupHardwareComponentId(const String& componentName, HardwareComponentId& outId);
bool initializeHWCFGStore();
String handleHWCFGCommand(const String& cmd);
void enterEEPROMInvalidErrorState(const char* reason);
void maintainEEPROMErrorIndicator();
void startEEPROMWakeAlert();
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
bool loadDevModeSetting();
bool saveDevModeSetting(bool enabled);
bool setDevModeEnabled(bool enabled);
String buildDevModeStatusMessage();
void logMotorFaultDebug(const char* context);
float readMainThermistorResistanceOhms();
bool isHardwareLikelyDisconnectedForUserAction();
void playHardwareNotConnectedAlert();
bool enforceHeaterToleranceGap(const char* sourceTag, bool notifyBle = true);

// Macros to automatically forward Serial output to BLE when streaming is enabled
#define SerialBLE_print(x) do { Serial.print(x); if(serial_streaming_enabled) sendSerialToBLE(x); } while(0)
#define SerialBLE_println(x) do { Serial.println(x); if(serial_streaming_enabled) sendSerialToBLE(String(x) + "\n"); } while(0)
#define SerialBLE_println_empty() do { Serial.println(); if(serial_streaming_enabled) sendSerialToBLE("\n"); } while(0)

// Button pins (MCP23017 side)
const int button1Pin = 7;
const int button2Pin = 15;

// LED pins
const int ledPins[] = {1, 9, 13, 14, 10, 6, 11, 12, 8, 0, 2, 3, 4, 5};
const int totalLeds = sizeof(ledPins) / sizeof(ledPins[0]);
int ledIndex = 0;
bool clockwise = true; // Direction flag for LED cycling
const float HARDWARE_DISCONNECT_RESISTANCE_OHMS = 100000.0;
const float THERMISTOR_VOLTAGE_GUARD_V = 0.01;

// Timing variables
unsigned long previousMillis = 0;
unsigned long flushStartMillis = 0;
unsigned long processStartMillis = 0;
unsigned long ledLastUpdateMillis = 0;
unsigned long motorStartMillis = 0;
bool mechanismMotorRunning = false;
bool heaterOn = false;
unsigned long stepStartMillis = 0;
unsigned long timeAboveSetpointMillis = 0;   // Cumulative ms thermistor has been above K this heating phase
unsigned long lastHeaterCheckMillis = 0;     // Last millis() when we updated the time-above-K accumulator
unsigned long timeAboveCutModeTempMillis = 0;   // After cut motor: cumulative ms temp >= CUT_MODE_TEMP
unsigned long lastCutModeTempCheckMillis = 0;   // Last millis() for time-above-CUT_MODE_TEMP accumulator
unsigned long lastHeaterControlLogMillis = 0;  // Last millis() when heater control state was logged
const unsigned long HEATER_CONTROL_LOG_INTERVAL_MS = 1000;  // 1s heater debug log cadence
unsigned long lastCoolingTempLogMillis = 0;  // Last millis() when cooling temp was sent to BLE
const unsigned long COOLING_TEMP_LOG_INTERVAL_MS = 1000;  // 1s cooling temperature log cadence
const unsigned long BLE_STATUS_LOG_INTERVAL_MS = 2000;  // 2s status log cadence to avoid BLE spam
const unsigned long M1_CURRENT_LOG_INTERVAL_MS = 3000;  // 3s max-current log window
unsigned long m1CurrentLogWindowStartMillis = 0;
float m1CurrentMaxInWindow = 0.0;
bool heaterOutputOn = false;  // Physical heater output state (independent from heater phase state)
unsigned long maxHeaterWallTimeMs = 0;  // Computed once at flush start (case 0) and reused for that cycle

// Temperature setup
const int thermistorPin = 14;    // GPIO14 (IO14)
float knownResistor = 10000.0;  // Will be initialized from thermistorResistance parameter
//float knownResistor = 33000.0;  // Will be initialized from thermistorResistance parameter

const float A = 2.7807099250E-03;
const float B = 2.4294397063E-04;
const float C = 1.0810891036E-06;
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

// Parameters (defaults: 1.5mil High Barrier Plastic from material_parameters.csv)
// Can't have global constant variables in C
int batteryThreshold = 5; //parameters_list[0]
float K = 150.0; //parameters_list[1]
int F = 6; //parameters_list[2]
long T = 60; //parameters_list[3] - Estimated cooling time for LED pacing (seconds)
float thermistorResistance = 10000.0; // constant, not in BLE
int r2 = 2; // constant, not in BLE
float backupTime = 1.0; //parameters_list[4]
int r4 = 2; // constant, not in BLE
int fanDuration = 5; //parameters_list[5]
long H = 30; //parameters_list[6] - Time (seconds) thermistor must be above K
float continueFeeder = 6.0; //parameters_list[7]
int maxOpeningTime = 12; //parameters_list[8]
int typicalOpeningTime = 10; //parameters_list[9]
float MOTOR_CUT_TIME = 0.5; //parameters_list[10]
float CUT_MODE_HEAT_TIME = 15.0; //parameters_list[11] - Additional time (seconds) above K in cut mode
float postCoolingFanDuration = 5.0; //parameters_list[12]
float preFeedFan = 2.0; //parameters_list[13]
float fanReverseTime = 12.0; //parameters_list[14]
float fanReverseStartTime = 0.0; //parameters_list[15]
float backupTimeAfterReopen = 1.7; //parameters_list[16]
float CUT_MODE_TEMP = 150.0;      //parameters_list[17] - Temp to maintain for CUT_MODE_HEAT_TIME after cut
float heaterLowerToleranceC = 0.0; //parameters_list[18] - Turn heater ON at target-lower
float heaterUpperToleranceC = 2.0; //parameters_list[19] - Turn heater OFF at target+upper (can be negative)
float COOL_OPEN_TEMP_C = 80.0f; //parameters_list[20] - Open sealer below this thermistor temp
long MAX_COOL_WAIT_S = 180; //parameters_list[21] - Safety fallback max cooling wait (seconds)
float heaterTargetTemp = 150.0;    // Active heater target (K or CUT_MODE_TEMP)

bool isFlushing = false;
int flushStep = 0;
bool case5FeedExecuted = false; // Flag to prevent multiple M2 activations in case 5
bool case1FeedStarted = false; // Flag to track if M2 has started in case 1
bool case6CutMotorRun = false;  // Flag: cut motor run after H (in cut mode); then heat continues for CUT_MODE_HEAT_TIME
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
bool button1DisconnectAlertedForCurrentPress = false;
bool button2DisconnectAlertedForCurrentPress = false;

// BLE auto-shutdown variables
unsigned long bleStartupTime = 0;
unsigned long bleIdleStartTime = 0;
const unsigned long BLE_TIMEOUT = 10 * 60 * 1000; // 10 minutes in milliseconds
const unsigned long INACTIVITY_SLEEP_MS = 2 * 60 * 1000;  // 2 minutes - then light sleep

// Runtime DEV mode (persisted in NVS): when enabled, BLE stays on and inactivity sleep is disabled.
const char* DEV_MODE_NAMESPACE = "system";
const char* DEV_MODE_KEY = "dev_mode";
const char* HW_MATRIX_NAMESPACE = "hwmeta";
const char* HW_MATRIX_ACTIVE_KEY = "matrix";
const char* HW_MATRIX_LAST_GOOD_KEY = "matrix_lkg";
bool devModeEnabled = true;
// M1/M2 fault handling controls.
bool ignoreM12Faults = true;
bool hasIgnoredM12Fault = false;
unsigned long ignoredM12FaultCount = 0;
unsigned long lastIgnoredM12FaultMillis = 0;
unsigned long lastIgnoredM12FaultLogMillis = 0;
unsigned long suppressedIgnoredM12FaultLogs = 0;
const unsigned long M12_FAULT_LOG_THROTTLE_MS = 5000;
const bool latchM12FaultAfterRecoveryFailure = true;
const uint8_t M12_FAULT_RECOVERY_MAX_ATTEMPTS = 3;
const unsigned long M12_FAULT_RECOVERY_DISABLE_MS = 25;
const unsigned long M12_FAULT_RECOVERY_SETTLE_MS = 60;
const unsigned long M12_FAULT_RECOVERY_COOLDOWN_MS = 2000;
unsigned long lastM12RecoveryCycleMillis = 0;
unsigned long m12RecoverySuccessCount = 0;
unsigned long m12RecoveryFailureCount = 0;
unsigned long suppressedM12RecoveryCycles = 0;

unsigned long lastActivityMillis = 0;
bool bleEnabled = true;

// OTA timing variables
bool otaEnabled = false;
unsigned long batteryMonitorStartTime = 0;
unsigned long otaWindowStartTime = 0;
bool batteryMonitoringActive = false;
const unsigned long BATTERY_MONITOR_DURATION = 10000; // 10 seconds
const unsigned long OTA_WINDOW_DURATION = 60000; // 1 minute
const unsigned long OTA_MODE_MAX_DURATION = 2 * 60 * 1000; // 2 minutes - then always exit OTA and stop circle
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
inline bool otaTransferInProgress() {
  return (otaState == OTA_PREPARING || otaState == OTA_RECEIVING ||
          otaState == OTA_VALIDATING || otaState == OTA_FINALIZING);
}
void publishOTAStatus(const String& status, bool notify = true);
void publishOTAErrorStatus(const char* reasonCode);
bool setOTAState(OTAState nextState, const char* reason);
bool isKnownOTACommand(const String& command);
void handleOTACommand(const String& command);
void checkOTATimeouts();
String last_update_status = "UPDATE_IDLE";
unsigned long otaStateEnteredAt = 0;
unsigned long otaLastChunkMillis = 0;
const unsigned long OTA_PREPARE_TIMEOUT_MS = 15000;
const unsigned long OTA_RECEIVE_INACTIVITY_TIMEOUT_MS = 20000;
const unsigned long OTA_FINALIZE_TIMEOUT_MS = 15000;
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

// LED timing variables (flush progress pacing)
unsigned long totalSequenceTime = 0;   // Kept for debug telemetry
unsigned long ledUpdateInterval = 0;   // Kept for debug telemetry
unsigned long slowCircleLastUpdate = 0;
int slowCircleLedIndex = 0;
const unsigned long SLOW_CIRCLE_INTERVAL = 300; // 300ms per LED for slow circle
const unsigned long LED_FLUSH_VISUAL_INTERVAL_MS = 90;

enum LedPhaseSection {
  LED_PHASE_INITIAL = 0,
  LED_PHASE_HEAT_UP = 1,
  LED_PHASE_HEAT_HOLD = 2,
  LED_PHASE_COOLING = 3,
  LED_PHASE_FINAL = 4
};

int ledSectionSteps[LED_PHASE_COUNT] = {0};
int ledSectionStartIndex[LED_PHASE_COUNT] = {0};
int ledSectionEndIndex[LED_PHASE_COUNT] = {0};
unsigned long ledSectionEstimateMs[LED_PHASE_COUNT] = {0};
int ledLastSectionSeen = -1;
int ledLastFlushStepSeen = -1;
unsigned long ledHeatStartMillis = 0;
unsigned long ledCoolingStartMillis = 0;
unsigned long ledFinalStartMillis = 0;
float ledHeatStartTempC = 0.0f;
float ledHeatTargetTempC = 0.0f;
float ledCoolStartTempC = 0.0f;
float ledCoolStartRefTempC = 0.0f;

const unsigned int TIMEOUT = 15000;

int ERROR_CODE = 0;
//ERROR_CODE = 1 ==> Mechanism Motor Timeout
//ERROR_CODE = 2 ==> Low battery
//ERROR_CODE = 3 ==> Heater too hot 
//ERROR_CODE = 4 ==> Motor Fault Detected
//ERROR_CODE = 5 ==> Heater current detection failure
//ERROR_CODE = 6 ==> Heater max wall time - time above K not reached
//ERROR_CODE = 7 ==> EEPROM invalid (reflash/parameter recovery required)
const int EEPROM_INVALID_ERROR_CODE = 7;
const uint16_t VIRGIN_EEPROM_MAGIC = 0xFFFF;
bool eepromErrorState = false;
bool lastEEPROMWriteVerified = false;
bool eepromWakeAlertActive = false;
unsigned long eepromWakeAlertStartMillis = 0;
const unsigned long EEPROM_WAKE_ALERT_MS = 10000;
uint32_t lifetimeFlushCount = 0;
HardwareMatrix hardwareMatrix = {};
bool hardwareMatrixInitialized = false;
HWCFGConfigStore hwcfgStore = {};
bool hwcfgStoreInitialized = false;
bool hwcfgSafeFault = false;
HardwareMatrix hardwareMatrixScratchActive = {};
HardwareMatrix hardwareMatrixScratchLastGood = {};
// HWCFG scratch/snapshot buffers kept at file scope to avoid large loopTask stack frames.
HWCFGConfigStore hwcfgScratchActive = {};
HWCFGConfigStore hwcfgScratchLastGood = {};
HWCFGConfigStore hwcfgSnapshotStore = {};
HardwareMatrix hwcfgSnapshotMatrix = {};
const char* HARDWARE_COMPONENT_NAMES[HW_COMPONENT_COUNT] = {
  "CONTROL_PANEL",
  "HEATING_ELEMENT",
  "MAIN_CIRCUIT_BOARD",
  "VACUUM_FAN",
  "FEED_MOTOR",
  "MECHANISM_MOTOR",
  "THERMISTOR",
  "BATTERY",
  "FACTORY_SOFTWARE_DATE",
  "FACTORY_SOFTWARE_VERSION_NUMBER"
};

// Motor Driver Definitions
const uint8_t M1DIR_PIN = 1;     // GPIO1 (M1DIR)
const uint8_t M1PWM_PIN = 42;    // GPIO42 (M1PWM)
const uint8_t M1NEN_PIN = 48;    // GPIO48 (M1NEN)
const uint8_t M1NFAULT_PIN = 40; // GPIO40 (M1NFAULT)

const uint8_t M2DIR_PIN = 3;     // GPIO3 (M2DIR)
const uint8_t M2PWM_PIN = 13;    // GPIO13 (M2PWM)
const uint8_t M2NEN_PIN = 9;     // GPIO9 (M2NEN)
const uint8_t M2NFAULT_PIN = 11; // GPIO11 (M2NFAULT)

// PWM fan (replaces M3 sealer motor)
const int sealerFanPWM = 47;       // PWM input, 25 kHz
const int sealerFanPwr = 4;        // Power enable (transistor)
const int sealerFanReverse = 21;   // HIGH = reverse
const int FAN_PWM_FREQ = 25000;
const int FAN_PWM_RESOLUTION = 8;

// Motor shield instance - M1&M2 only
DualMAX14870MotorShield motors(M1DIR_PIN, M1PWM_PIN, M2DIR_PIN, M2PWM_PIN, M2NEN_PIN, M2NFAULT_PIN);

bool enforceHeaterToleranceGap(const char* sourceTag, bool notifyBle) {
  if ((heaterLowerToleranceC != heaterLowerToleranceC) || (heaterUpperToleranceC != heaterUpperToleranceC)) {
    return false;
  }

  float toleranceGap = heaterUpperToleranceC - heaterLowerToleranceC;
  if (toleranceGap >= 2.0f) {
    return false;
  }

  float originalLower = heaterLowerToleranceC;
  float originalUpper = heaterUpperToleranceC;
  heaterLowerToleranceC = heaterUpperToleranceC - 2.0f;

  Serial.printf(
      "Adjusted heater tolerances (%s): lower %.3f -> %.3f, upper %.3f (enforced >= 2.0C gap)\n",
      sourceTag,
      originalLower,
      heaterLowerToleranceC,
      originalUpper);
  if (notifyBle) {
    sendSerialToBLE(
        "Adjusted heater tolerances (" + String(sourceTag) +
        "): lower set to upper-2.0C to enforce minimum 2.0C gap");
  }
  return true;
}

// Setup BLE callbacks called onConnect and onDisconnect
class server_callbacks: public BLEServerCallbacks {
  void onConnect(BLEServer * blue_server) {
    is_device_connected = true;
    Serial.println("Device connected!");
    sendSerialToBLE("BLE Device Connected!");
    
    // Update characteristic with current parameters (22 BLE values; thermistorResistance, r2, r4 are constants)
    String paramString = String(batteryThreshold) + "," +
                        String(K) + "," +
                        String(F) + "," +
                        String(T) + "," +
                        String(backupTime) + "," +
                        String(fanDuration) + "," +
                        String(H) + "," +
                        String(continueFeeder) + "," +
                        String(maxOpeningTime) + "," +
                        String(typicalOpeningTime) + "," +
                        String(MOTOR_CUT_TIME) + "," +
                        String(CUT_MODE_HEAT_TIME) + "," +
                        String(postCoolingFanDuration) + "," +
                        String(preFeedFan) + "," +
                        String(fanReverseTime) + "," +
                        String(fanReverseStartTime) + "," +
                        String(backupTimeAfterReopen) + "," +
                        String(CUT_MODE_TEMP) + "," +
                        String(heaterLowerToleranceC) + "," +
                        String(heaterUpperToleranceC) + "," +
                        String(COOL_OPEN_TEMP_C) + "," +
                        String(MAX_COOL_WAIT_S);
    blue_characteristic->setValue(paramString.c_str());
    Serial.printf("Characteristic updated on connect: %s\n", paramString.c_str());
    SerialBLE_println("Characteristic updated on connect");
  }

  void onDisconnect(BLEServer * blue_server) {
    is_device_connected = false;
    serial_streaming_enabled = false;
    Serial.println("Device disconnected!");
    if (isFlushing) {
      Serial.println("Parameter update on disconnect blocked - flush in progress");
      SerialBLE_println("Parameter update blocked - flush in progress");
      return;
    }

    // Receive message
    int message_length = blue_characteristic->getLength();
    Serial.printf("Length of message: %d\n", message_length);
    unsigned char* message = blue_characteristic->getData();
    Serial.printf("Message: ");
    for (int i = 0; i < message_length; i++) {
      Serial.printf("%c", message[i]);
    }

    // Parse BLE comma-separated float values (20 legacy or 22 current; thermistorResistance, r2, r4 are constants)
    char * parameters_string = strtok((char*) message, ",");
    float parameters_list[100];
    int k = 0;
    while (parameters_string != NULL) {
      char *endptr;
      parameters_list[k] = strtof(parameters_string, &endptr);
      Serial.printf("\n%f", parameters_list[k]);
      parameters_string = strtok(NULL, ",");
      k++;
    }
    for (int j = 0; j < k; j++) {
      Serial.printf("\n%f", parameters_list[j]);
    }
    if (k >= 20) {
      batteryThreshold = (int)parameters_list[0];
      K = parameters_list[1];
      F = (int)parameters_list[2];
      T = (long)parameters_list[3];
      backupTime = parameters_list[4];
      fanDuration = (int)parameters_list[5];
      H = (long)parameters_list[6];
      continueFeeder = parameters_list[7];
      maxOpeningTime = (int)parameters_list[8];
      typicalOpeningTime = (int)parameters_list[9];
      MOTOR_CUT_TIME = parameters_list[10];
      CUT_MODE_HEAT_TIME = parameters_list[11];
      postCoolingFanDuration = parameters_list[12];
      preFeedFan = parameters_list[13];
      fanReverseTime = parameters_list[14];
      fanReverseStartTime = parameters_list[15];
      backupTimeAfterReopen = parameters_list[16];
      CUT_MODE_TEMP = parameters_list[17];
      if (k >= 22) {
        COOL_OPEN_TEMP_C = parameters_list[20];
        MAX_COOL_WAIT_S = (long)parameters_list[21];
      }
      if (k >= 20) {
        heaterLowerToleranceC = parameters_list[18];
        heaterUpperToleranceC = parameters_list[19];
        enforceHeaterToleranceGap("server_disconnect_write");
      }
      if (!(isFlushing && cutBag && case6CutMotorRun)) {
        heaterTargetTemp = K;
      }
    }
    
    // Save parameters to EEPROM for persistence
    saveParametersToEEPROM();
    // Keep EEPROM latch logic tied to explicit BLE parameter writes only.
    
    Serial.printf("Parameters updated! H=%ld, K=%.1f\n", H, K);
  }
};

// BLE Characteristic callback for OTA updates
class update_characteristic_callbacks: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    if (pCharacteristic->getUUID().toString() == UPDATE_CHARACTERISTIC_UUID) {
      // Check if OTA is enabled - reject all operations if disabled
      if (!otaEnabled) {
        publishOTAStatus("OTA_DISABLED");
        Serial.println("OTA request rejected - OTA mode not enabled");
        return;
      }
      
      int message_length = pCharacteristic->getLength();
      if (message_length > 0) {
        uint8_t* data = pCharacteristic->getData();

        // Use length-aware parsing (BLE payload is not guaranteed null-terminated)
        String command = String((char*)data, (unsigned int)message_length);
        command.trim();

        if (isKnownOTACommand(command)) {
          handleOTACommand(command);
          return;
        }

        // Handle OTA chunks
        if (otaState == OTA_RECEIVING) {
          handleOTAChunk(data, message_length);
          return;
        }
        
        Serial.print("DEBUG: Received update command: '");
        Serial.print(command);
        Serial.println("'");
        SerialBLE_println("Unknown OTA payload while not receiving");
      }
    }
  }
};

// BLE Characteristic callback for immediate parameter updates
class characteristic_callbacks: public BLECharacteristicCallbacks {
  void onRead(BLECharacteristic *pCharacteristic) {
    if (pCharacteristic->getUUID().toString() == CHARACTERISTIC_UUID) {
      // Return current parameters as comma-separated string (22 values)
      String paramString = String(batteryThreshold) + "," +
                          String(K) + "," +
                          String(F) + "," +
                          String(T) + "," +
                          String(backupTime) + "," +
                          String(fanDuration) + "," +
                          String(H) + "," +
                          String(continueFeeder) + "," +
                          String(maxOpeningTime) + "," +
                          String(typicalOpeningTime) + "," +
                          String(MOTOR_CUT_TIME) + "," +
                          String(CUT_MODE_HEAT_TIME) + "," +
                          String(postCoolingFanDuration) + "," +
                          String(preFeedFan) + "," +
                          String(fanReverseTime) + "," +
                          String(fanReverseStartTime) + "," +
                          String(backupTimeAfterReopen) + "," +
                          String(CUT_MODE_TEMP) + "," +
                          String(heaterLowerToleranceC) + "," +
                          String(heaterUpperToleranceC) + "," +
                          String(COOL_OPEN_TEMP_C) + "," +
                          String(MAX_COOL_WAIT_S);
      
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
        if (isFlushing) {
          pCharacteristic->setValue("PARAM_UPDATE_BLOCKED_FLUSH");
          Serial.println("Immediate parameter update blocked - flush in progress");
          SerialBLE_println("Parameter update blocked - flush in progress");
          return;
        }
        unsigned char* message = pCharacteristic->getData();
        // Allow remote clients to request OTA mode via BLE write
        String cmd = String((char*)message, (unsigned int)message_length);
        cmd.trim();
        if (cmd.startsWith("HWCFG_")) {
          String hwcfgResponse = handleHWCFGCommand(cmd);
          pCharacteristic->setValue(hwcfgResponse.c_str());
          Serial.printf("Processed HWCFG command '%s' -> '%s'\n", cmd.c_str(), hwcfgResponse.c_str());
          sendSerialToBLE("Processed HWCFG command");
          return;
        }
        if (cmd == "ENABLE_OTA") {
          Serial.println("Received ENABLE_OTA command via BLE");
          sendSerialToBLE("Received ENABLE_OTA command via BLE");
          enableOTA();
          pCharacteristic->setValue("ENABLE_OTA_ACK");
          return;
        }
        if (cmd == "GET_DEV_MODE") {
          String statusMessage = buildDevModeStatusMessage();
          pCharacteristic->setValue(statusMessage.c_str());
          Serial.printf("Processed GET_DEV_MODE, returned %s\n", statusMessage.c_str());
          sendSerialToBLE("Processed GET_DEV_MODE");
          return;
        }
        if (cmd == "GET_FLUSH_COUNT") {
          String flushCountMessage = String("FLUSH_COUNT:") + String((unsigned long)lifetimeFlushCount);
          pCharacteristic->setValue(flushCountMessage.c_str());
          Serial.printf("Processed GET_FLUSH_COUNT, returned %s\n", flushCountMessage.c_str());
          sendSerialToBLE("Processed GET_FLUSH_COUNT");
          return;
        }
        if (cmd == "GET_HW_MATRIX") {
          if (!hardwareMatrixInitialized && !initializeHardwareMatrix()) {
            pCharacteristic->setValue("HW_MATRIX_ERR:INIT_FAIL");
            sendSerialToBLE("GET_HW_MATRIX failed: init");
            return;
          }
          String response = getHardwareComponentsListString();
          pCharacteristic->setValue(response.c_str());
          Serial.printf("Processed GET_HW_MATRIX, returned %s\n", response.c_str());
          sendSerialToBLE("Processed GET_HW_MATRIX");
          return;
        }
        if (cmd.startsWith("GET_HW_COMPONENT:")) {
          if (!hardwareMatrixInitialized && !initializeHardwareMatrix()) {
            pCharacteristic->setValue("HW_COMPONENT_ERR:INIT_FAIL");
            sendSerialToBLE("GET_HW_COMPONENT failed: init");
            return;
          }
          String componentName = cmd.substring(String("GET_HW_COMPONENT:").length());
          componentName.trim();
          HardwareComponentId componentId;
          if (!lookupHardwareComponentId(componentName, componentId)) {
            pCharacteristic->setValue("HW_COMPONENT_ERR:UNKNOWN_COMPONENT");
            Serial.printf("Rejected GET_HW_COMPONENT with unknown component: %s\n", componentName.c_str());
            sendSerialToBLE("GET_HW_COMPONENT rejected: unknown component");
            return;
          }
          String response = getHardwareComponentString(componentId);
          pCharacteristic->setValue(response.c_str());
          sendSerialToBLE("Processed GET_HW_COMPONENT");
          return;
        }
        if (cmd.startsWith("SET_HW_COMPONENT:")) {
          String payload = cmd.substring(String("SET_HW_COMPONENT:").length());
          int firstSep = payload.indexOf(':');
          int secondSep = (firstSep >= 0) ? payload.indexOf(':', firstSep + 1) : -1;
          int thirdSep = (secondSep >= 0) ? payload.indexOf(':', secondSep + 1) : -1;
          if (firstSep < 0 || secondSep < 0 || thirdSep < 0) {
            pCharacteristic->setValue("SET_HW_COMPONENT_ERR:BAD_FORMAT");
            sendSerialToBLE("SET_HW_COMPONENT rejected: bad format");
            return;
          }

          String componentName = payload.substring(0, firstSep);
          String version = payload.substring(firstSep + 1, secondSep);
          String installDate = payload.substring(secondSep + 1, thirdSep);
          String description = payload.substring(thirdSep + 1);
          componentName.trim();

          String errorCode;
          if (!setHardwareComponentByName(componentName, version, installDate, description, errorCode)) {
            String errResponse = String("SET_HW_COMPONENT_ERR:") + errorCode;
            pCharacteristic->setValue(errResponse.c_str());
            Serial.printf("SET_HW_COMPONENT failed for %s: %s\n", componentName.c_str(), errorCode.c_str());
            sendSerialToBLE("SET_HW_COMPONENT failed");
            return;
          }

          String ack = String("SET_HW_COMPONENT_ACK:") + componentName;
          pCharacteristic->setValue(ack.c_str());
          sendSerialToBLE("SET_HW_COMPONENT applied");
          return;
        }
        if (cmd.startsWith("SET_DEV_MODE:")) {
          String valueString = cmd.substring(String("SET_DEV_MODE:").length());
          valueString.trim();

          if (valueString != "0" && valueString != "1") {
            pCharacteristic->setValue("SET_DEV_MODE_ERR:INVALID_VALUE");
            Serial.printf("Rejected SET_DEV_MODE with invalid value: '%s'\n", valueString.c_str());
            sendSerialToBLE("SET_DEV_MODE rejected: invalid value");
            return;
          }

          bool requestedValue = (valueString == "1");
          if (!setDevModeEnabled(requestedValue)) {
            pCharacteristic->setValue("SET_DEV_MODE_ERR:PERSIST_FAIL");
            sendSerialToBLE("SET_DEV_MODE failed: persist error");
            return;
          }

          String ack = String("SET_DEV_MODE_ACK:") + String(devModeEnabled ? 1 : 0);
          pCharacteristic->setValue(ack.c_str());
          sendSerialToBLE("SET_DEV_MODE applied");
          return;
        }
        Serial.printf("Immediate parameter update received, length: %d\n", message_length);
        // Parse BLE comma-separated float values (20 legacy or 22 current)
        char * parameters_string = strtok((char*) message, ",");
        float parameters_list[100];
        int k = 0;
        while (parameters_string != NULL) {
          char *endptr;
          parameters_list[k] = strtof(parameters_string, &endptr);
          parameters_string = strtok(NULL, ",");
          k++;
        }
        
        // Apply parameters immediately (20 legacy or 22 current values)
        if (k >= 20) {
          batteryThreshold = (int)parameters_list[0];
          K = parameters_list[1];
          F = (int)parameters_list[2];
          T = (long)parameters_list[3];
          backupTime = parameters_list[4];
          fanDuration = (int)parameters_list[5];
          H = (long)parameters_list[6];
          continueFeeder = parameters_list[7];
          maxOpeningTime = (int)parameters_list[8];
          typicalOpeningTime = (int)parameters_list[9];
          MOTOR_CUT_TIME = parameters_list[10];
          CUT_MODE_HEAT_TIME = parameters_list[11];
          postCoolingFanDuration = parameters_list[12];
          preFeedFan = parameters_list[13];
          fanReverseTime = parameters_list[14];
          fanReverseStartTime = parameters_list[15];
          backupTimeAfterReopen = parameters_list[16];
          CUT_MODE_TEMP = parameters_list[17];
          if (k >= 20) {
            heaterLowerToleranceC = parameters_list[18];
            heaterUpperToleranceC = parameters_list[19];
            enforceHeaterToleranceGap("characteristic_write");
          }
          if (k >= 22) {
            COOL_OPEN_TEMP_C = parameters_list[20];
            MAX_COOL_WAIT_S = (long)parameters_list[21];
          }
          if (!(isFlushing && cutBag && case6CutMotorRun)) {
            heaterTargetTemp = K;
          }
        }
        
        // Save parameters to EEPROM for persistence
        saveParametersToEEPROM();
        if (eepromErrorState) {
          if (lastEEPROMWriteVerified) {
            eepromErrorState = false;
            eepromWakeAlertActive = false;
            if (ERROR_CODE == EEPROM_INVALID_ERROR_CODE) {
              ERROR_CODE = 0;
            }
            Serial.println("EEPROM recovery successful from BLE parameter write. Clearing latched EEPROM error.");
            sendSerialToBLE("EEPROM RECOVERED - latched error cleared");
          } else {
            Serial.println("EEPROM recovery attempt failed - still in EEPROM error state");
            sendSerialToBLE("EEPROM RECOVERY FAILED - write not verified");
          }
        }
        
        // Update characteristic value with new parameters (22 values)
        String paramString = String(batteryThreshold) + "," +
                            String(K) + "," +
                            String(F) + "," +
                            String(T) + "," +
                            String(backupTime) + "," +
                            String(fanDuration) + "," +
                            String(H) + "," +
                            String(continueFeeder) + "," +
                            String(maxOpeningTime) + "," +
                            String(typicalOpeningTime) + "," +
                            String(MOTOR_CUT_TIME) + "," +
                            String(CUT_MODE_HEAT_TIME) + "," +
                            String(postCoolingFanDuration) + "," +
                            String(preFeedFan) + "," +
                            String(fanReverseTime) + "," +
                            String(fanReverseStartTime) + "," +
                            String(backupTimeAfterReopen) + "," +
                            String(CUT_MODE_TEMP) + "," +
                            String(heaterLowerToleranceC) + "," +
                            String(heaterUpperToleranceC) + "," +
                            String(COOL_OPEN_TEMP_C) + "," +
                            String(MAX_COOL_WAIT_S);
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

  // Retry MCP startup to survive first-boot rail/I2C bring-up timing.
  const int maxAttempts = 20;
  const unsigned long retryDelayMs = 100;
  mcpInitialized = false;
  for (int attempt = 1; attempt <= maxAttempts; attempt++) {
    if (mcp.begin_I2C(0x20, &myI2C)) {
      mcpInitialized = true;
      Serial.printf("MCP23017 Initialized Successfully on attempt %d/%d.\n", attempt, maxAttempts);
      break;
    }
    Serial.printf("WARNING: MCP23017 init attempt %d/%d failed.\n", attempt, maxAttempts);
    delay(retryDelayMs);
  }

  if (!mcpInitialized) {
    Serial.println("ERROR: MCP23017 unavailable after retries - continuing without expander.");
    return;
  }

  // Setup button pins on expander
  mcp.pinMode(button2Pin, INPUT_PULLUP);

  // Setup LED pins on expander
  for (int i = 0; i < totalLeds; i++) {
    mcp.pinMode(ledPins[i], OUTPUT);
  }
}

// Function to write a value to a specific MCP23017 pin
void mcp_digitalWrite(int pin, int value) {
  if (!mcpInitialized) {
    return;
  }
  mcp.digitalWrite(pin, value);
}

// Function to read from a specific MCP23017 pin
int mcp_digitalRead(int pin) {
  if (!mcpInitialized) {
    unsigned long now = millis();
    if (lastMcpUnavailableLogMillis == 0 || (now - lastMcpUnavailableLogMillis >= MCP_UNAVAILABLE_LOG_INTERVAL_MS)) {
      Serial.println("WARNING: MCP23017 read while unavailable; returning HIGH.");
      lastMcpUnavailableLogMillis = now;
    }
    return HIGH;
  }
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
  
  // Set initial value to current parameters (22 values)
  String initialParams = String(batteryThreshold) + "," +
                        String(K) + "," +
                        String(F) + "," +
                        String(T) + "," +
                        String(backupTime) + "," +
                        String(fanDuration) + "," +
                        String(H) + "," +
                        String(continueFeeder) + "," +
                        String(maxOpeningTime) + "," +
                        String(typicalOpeningTime) + "," +
                        String(MOTOR_CUT_TIME) + "," +
                        String(CUT_MODE_HEAT_TIME) + "," +
                        String(postCoolingFanDuration) + "," +
                        String(preFeedFan) + "," +
                        String(fanReverseTime) + "," +
                        String(fanReverseStartTime) + "," +
                        String(backupTimeAfterReopen) + "," +
                        String(CUT_MODE_TEMP) + "," +
                        String(heaterLowerToleranceC) + "," +
                        String(heaterUpperToleranceC) + "," +
                        String(COOL_OPEN_TEMP_C) + "," +
                        String(MAX_COOL_WAIT_S);
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

  // Create update characteristic as part of main service (OTA commands; Python client uses it after ENABLE_OTA)
  Serial.println("Creating update characteristic in main service...");
  update_characteristic = blue_service->createCharacteristic(
    UPDATE_CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ |
    BLECharacteristic::PROPERTY_WRITE
  );
  update_characteristic->setCallbacks(new update_characteristic_callbacks());
  update_characteristic->setValue("READY");
  Serial.println("Update characteristic created and callback set");
  update_service = NULL;

  // Starts the service on the server
  blue_service->start();

  // Starts broadcasting so any devices can now find it and connect
  BLEAdvertising * blue_advert = blue_server->getAdvertising();
  blue_advert->addServiceUUID(SERVICE_UUID);
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
  lastEEPROMWriteVerified = false;
  
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
  EEPROM.put(addr, postCoolingFanDuration); addr += sizeof(postCoolingFanDuration);
  EEPROM.put(addr, preFeedFan); addr += sizeof(preFeedFan);
  EEPROM.put(addr, fanReverseTime); addr += sizeof(fanReverseTime);
  EEPROM.put(addr, fanReverseStartTime); addr += sizeof(fanReverseStartTime);
  EEPROM.put(addr, backupTimeAfterReopen); addr += sizeof(backupTimeAfterReopen);
  EEPROM.put(addr, CUT_MODE_TEMP); addr += sizeof(CUT_MODE_TEMP);
  EEPROM.put(addr, heaterLowerToleranceC); addr += sizeof(heaterLowerToleranceC);
  EEPROM.put(addr, heaterUpperToleranceC); addr += sizeof(heaterUpperToleranceC);
  EEPROM.put(addr, COOL_OPEN_TEMP_C); addr += sizeof(COOL_OPEN_TEMP_C);
  EEPROM.put(addr, MAX_COOL_WAIT_S); addr += sizeof(MAX_COOL_WAIT_S);
  
  bool commitOk = EEPROM.commit();
  
  // Verify the write by reading back the magic number
  uint16_t verifyMagic;
  EEPROM.get(PARAM_START_ADDR, verifyMagic);
  
  if (commitOk && verifyMagic == PARAM_MAGIC_NUMBER) {
    lastEEPROMWriteVerified = true;
    Serial.println("Parameters saved to EEPROM - verification successful");
    SerialBLE_println("Parameters saved to EEPROM - verification successful");
  } else {
    Serial.printf("ERROR: EEPROM write verification failed! Commit OK: %s, Expected: 0x%04X, Read: 0x%04X\n", 
                  commitOk ? "true" : "false", PARAM_MAGIC_NUMBER, verifyMagic);
    SerialBLE_print("ERROR: EEPROM write verification failed! Expected: 0x");
    SerialBLE_print(String(PARAM_MAGIC_NUMBER, HEX));
    SerialBLE_print(", Read: 0x");
    SerialBLE_print(String(verifyMagic, HEX));
    SerialBLE_println();
  }
  
  EEPROM.end();
}

void enterEEPROMInvalidErrorState(const char* reason) {
  Serial.printf("EEPROM INVALID: %s\n", reason);
  SerialBLE_print("EEPROM INVALID: ");
  SerialBLE_println(reason);

  // Flash all LEDs 5 times before showing dedicated EEPROM error code.
  for (int i = 0; i < 5; i++) {
    for (int j = 0; j < totalLeds; j++) {
      mcp_digitalWrite(ledPins[j], HIGH);
    }
    delay(100);
    for (int j = 0; j < totalLeds; j++) {
      mcp_digitalWrite(ledPins[j], LOW);
    }
    delay(100);
  }

  ERROR_CODE = EEPROM_INVALID_ERROR_CODE;
  eepromErrorState = true;
  LEDErrorCode(ERROR_CODE);
}

void maintainEEPROMErrorIndicator() {
  if (!eepromErrorState || ERROR_CODE != EEPROM_INVALID_ERROR_CODE) {
    return;
  }
  // Keep the EEPROM error LED latched high whenever the device is awake.
  mcp_digitalWrite(ledPins[EEPROM_INVALID_ERROR_CODE], HIGH);
}

void startEEPROMWakeAlert() {
  if (!eepromErrorState) {
    return;
  }
  eepromWakeAlertActive = true;
  eepromWakeAlertStartMillis = millis();
  Serial.println("EEPROM error wake alert active for 10 seconds.");
  sendSerialToBLE("EEPROM error wake alert active for 10 seconds");
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
    EEPROM.get(addr, postCoolingFanDuration); addr += sizeof(postCoolingFanDuration);
    EEPROM.get(addr, preFeedFan); addr += sizeof(preFeedFan);
    EEPROM.get(addr, fanReverseTime); addr += sizeof(fanReverseTime);
    EEPROM.get(addr, fanReverseStartTime); addr += sizeof(fanReverseStartTime);
    EEPROM.get(addr, backupTimeAfterReopen); addr += sizeof(backupTimeAfterReopen);
    EEPROM.get(addr, CUT_MODE_TEMP); addr += sizeof(CUT_MODE_TEMP);

    // New heater hysteresis parameters (may be absent in older EEPROM layouts)
    EEPROM.get(addr, heaterLowerToleranceC); addr += sizeof(heaterLowerToleranceC);
    EEPROM.get(addr, heaterUpperToleranceC); addr += sizeof(heaterUpperToleranceC);
    EEPROM.get(addr, COOL_OPEN_TEMP_C); addr += sizeof(COOL_OPEN_TEMP_C);
    EEPROM.get(addr, MAX_COOL_WAIT_S); addr += sizeof(MAX_COOL_WAIT_S);

    // Guard invalid values and preserve a valid hysteresis band.
    if ((heaterLowerToleranceC != heaterLowerToleranceC) || heaterLowerToleranceC < 0.0f) {
      heaterLowerToleranceC = 0.0f;
    }
    if (heaterUpperToleranceC != heaterUpperToleranceC) {
      heaterUpperToleranceC = 2.0f;
    }
    if (heaterUpperToleranceC < 0.0f) {
      heaterUpperToleranceC = 0.0f;
    }
    enforceHeaterToleranceGap("eeprom_load", false);
    if ((COOL_OPEN_TEMP_C != COOL_OPEN_TEMP_C) || COOL_OPEN_TEMP_C < 20.0f || COOL_OPEN_TEMP_C > 150.0f) {
      COOL_OPEN_TEMP_C = 80.0f;
    }
    if (MAX_COOL_WAIT_S <= 0 || MAX_COOL_WAIT_S > 1800) {
      MAX_COOL_WAIT_S = 180;
    }
    heaterTargetTemp = K;
    
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
    EEPROM.end();
    return;
  }

  if (magic == VIRGIN_EEPROM_MAGIC) {
    Serial.println("Virgin EEPROM detected (0xFFFF). Writing default parameters.");
    SerialBLE_println("Virgin EEPROM detected (0xFFFF). Writing default parameters.");
    EEPROM.end();
    saveParametersToEEPROM();
    if (!lastEEPROMWriteVerified) {
      enterEEPROMInvalidErrorState("failed to initialize defaults");
    } else {
      Serial.println("Virgin EEPROM initialized with defaults.");
      SerialBLE_println("Virgin EEPROM initialized with defaults.");
    }
    return;
  } else {
    Serial.printf("EEPROM magic invalid/corrupt (0x%04X). Entering EEPROM error state.\n", magic);
    SerialBLE_print("EEPROM magic invalid/corrupt: 0x");
    SerialBLE_print(String(magic, HEX));
    SerialBLE_println();
    EEPROM.end();
    // Keep toilet operational: rewrite defaults, but latch EEPROM error until BLE update succeeds.
    saveParametersToEEPROM();
    if (!lastEEPROMWriteVerified) {
      Serial.println("WARNING: Failed to rewrite defaults after EEPROM corruption.");
      SerialBLE_println("WARNING: Failed to rewrite defaults after EEPROM corruption.");
    } else {
      Serial.println("Defaults rewritten after EEPROM corruption.");
      SerialBLE_println("Defaults rewritten after EEPROM corruption.");
    }
    enterEEPROMInvalidErrorState("magic mismatch");
    return;
  }
}

void loadFlushCountFromEEPROM() {
  EEPROM.begin(EEPROM_SIZE);

  uint16_t magic = 0;
  EEPROM.get(FLUSH_COUNT_MAGIC_ADDR, magic);

  if (magic == FLUSH_COUNT_MAGIC) {
    EEPROM.get(FLUSH_COUNT_ADDR, lifetimeFlushCount);
    if (lifetimeFlushCount == 0xFFFFFFFFUL) {
      lifetimeFlushCount = 0;
    }
    Serial.printf("Lifetime flush count loaded: %lu\n", lifetimeFlushCount);
    SerialBLE_print("Lifetime flush count loaded: ");
    SerialBLE_println((unsigned long)lifetimeFlushCount);
    EEPROM.end();
    return;
  }

  EEPROM.end();

  // First-time initialization (or invalid data): start from 0 and create record.
  lifetimeFlushCount = 0;
  if (saveFlushCountToEEPROM()) {
    Serial.println("Initialized lifetime flush counter in EEPROM.");
    SerialBLE_println("Initialized lifetime flush counter in EEPROM.");
  } else {
    Serial.println("WARNING: Failed to initialize lifetime flush counter in EEPROM.");
    SerialBLE_println("WARNING: Failed to initialize lifetime flush counter in EEPROM.");
  }
}

bool saveFlushCountToEEPROM() {
  EEPROM.begin(EEPROM_SIZE);
  EEPROM.put(FLUSH_COUNT_MAGIC_ADDR, (uint16_t)FLUSH_COUNT_MAGIC);
  EEPROM.put(FLUSH_COUNT_ADDR, lifetimeFlushCount);
  bool commitOk = EEPROM.commit();

  uint16_t verifyMagic = 0;
  uint32_t verifyCount = 0;
  EEPROM.get(FLUSH_COUNT_MAGIC_ADDR, verifyMagic);
  EEPROM.get(FLUSH_COUNT_ADDR, verifyCount);
  EEPROM.end();

  bool verified = commitOk && (verifyMagic == FLUSH_COUNT_MAGIC) && (verifyCount == lifetimeFlushCount);
  if (!verified) {
    Serial.printf("ERROR: Failed to persist lifetime flush counter. commit=%s magic=0x%04X count=%lu expected=%lu\n",
                  commitOk ? "true" : "false", verifyMagic, (unsigned long)verifyCount, (unsigned long)lifetimeFlushCount);
    SerialBLE_println("ERROR: Failed to persist lifetime flush counter.");
  }
  return verified;
}

void incrementFlushCount() {
  if (lifetimeFlushCount == 0xFFFFFFFFUL) {
    Serial.println("WARNING: Lifetime flush counter reached max value. Not incrementing.");
    SerialBLE_println("WARNING: Lifetime flush counter reached max value.");
    return;
  }

  lifetimeFlushCount++;
  if (saveFlushCountToEEPROM()) {
    Serial.printf("Lifetime flush count: %lu\n", lifetimeFlushCount);
    SerialBLE_print("Lifetime flush count: ");
    SerialBLE_println((unsigned long)lifetimeFlushCount);
  } else {
    // Roll back RAM value to reflect persisted value.
    lifetimeFlushCount--;
  }
}

void copyBoundedString(char* dest, size_t destSize, const char* src) {
  if (destSize == 0) {
    return;
  }
  if (src == NULL) {
    dest[0] = '\0';
    return;
  }
  strncpy(dest, src, destSize - 1);
  dest[destSize - 1] = '\0';
}

bool isIso8601DateOrEmpty(const char* dateValue) {
  if (dateValue == NULL || dateValue[0] == '\0') {
    return true;
  }
  if (strlen(dateValue) != 10) {
    return false;
  }
  for (int i = 0; i < 10; i++) {
    char c = dateValue[i];
    if (i == 4 || i == 7) {
      if (c != '-') {
        return false;
      }
    } else if (c < '0' || c > '9') {
      return false;
    }
  }
  int month = (dateValue[5] - '0') * 10 + (dateValue[6] - '0');
  int day = (dateValue[8] - '0') * 10 + (dateValue[9] - '0');
  if (month < 1 || month > 12) {
    return false;
  }
  if (day < 1 || day > 31) {
    return false;
  }
  return true;
}

bool isSafeFieldString(const char* text, bool allowEmpty) {
  if (text == NULL) {
    return allowEmpty;
  }
  if (!allowEmpty && text[0] == '\0') {
    return false;
  }
  for (size_t i = 0; text[i] != '\0'; i++) {
    char c = text[i];
    if (c < 0x20 || c > 0x7E) {
      return false;
    }
    // Keep BLE payload parser simple by rejecting delimiters used in protocol.
    if (c == ':' || c == '|') {
      return false;
    }
  }
  return true;
}

uint32_t computeHardwareMatrixCRC(const HardwareMatrix& matrix) {
  const uint8_t* bytes = reinterpret_cast<const uint8_t*>(&matrix);
  size_t len = offsetof(HardwareMatrix, crc32);
  uint32_t crc = 0xFFFFFFFFUL;
  for (size_t i = 0; i < len; i++) {
    crc ^= bytes[i];
    for (int bit = 0; bit < 8; bit++) {
      uint32_t mask = static_cast<uint32_t>(-(int32_t)(crc & 1U));
      crc = (crc >> 1) ^ (0xEDB88320UL & mask);
    }
  }
  return ~crc;
}

void refreshHardwareMatrixCRC(HardwareMatrix& matrix) {
  matrix.crc32 = computeHardwareMatrixCRC(matrix);
}

void setHardwareComponentDefaults(HardwareComponentEntry& entry, const char* version, const char* description, const char* installDate) {
  copyBoundedString(entry.current_version, sizeof(entry.current_version), version);
  copyBoundedString(entry.current_description, sizeof(entry.current_description), description);
  copyBoundedString(entry.install_date, sizeof(entry.install_date), installDate);
  entry.previous_version[0] = '\0';
  entry.previous_description[0] = '\0';
  entry.previous_install_date[0] = '\0';
}

void initializeDefaultHardwareMatrix(HardwareMatrix& matrix) {
  memset(&matrix, 0, sizeof(matrix));
  matrix.matrix_magic = HW_MATRIX_MAGIC;
  matrix.matrix_schema_version = HW_MATRIX_SCHEMA_VERSION;
  matrix.component_count = HW_COMPONENT_COUNT;
  setHardwareComponentDefaults(matrix.components[HW_CONTROL_PANEL], "5", "CONTROL INTERFACE FOR DEVICE", "2026-02-28");
  setHardwareComponentDefaults(matrix.components[HW_HEATING_ELEMENT], "1", "VERSION OF THE HEATING ELEMENT IN THE SEALING MECHANISM", "2026-02-28");
  setHardwareComponentDefaults(matrix.components[HW_MAIN_CIRCUIT_BOARD], "5", "MAIN CONROL PANEL FOR LOGIC", "2026-02-28");
  setHardwareComponentDefaults(matrix.components[HW_VACUUM_FAN], "1", "FAN IN THE SEALER HARDWARE TO ADJUST INTERNAL PRESSURE", "2026-02-28");
  setHardwareComponentDefaults(matrix.components[HW_FEED_MOTOR], "1", "MOTOR DESIGN FOR THE BAG FEED MECHANISM", "2026-02-28");
  setHardwareComponentDefaults(matrix.components[HW_MECHANISM_MOTOR], "1", "MOTOR DESIGN FOR THE OPEN & CLOSE CLAMPING MECANISM", "2026-02-28");
  setHardwareComponentDefaults(matrix.components[HW_THERMISTOR], "1", "VERSION OF THE THERMISTOR IN THE HEATING ELEMENT", "2026-02-28");
  setHardwareComponentDefaults(matrix.components[HW_BATTERY], "1", "VERSION OF THE BATTERY POWER SUPPLY", "2026-02-28");
  setHardwareComponentDefaults(matrix.components[HW_FACTORY_SOFTWARE_DATE], "2026-02-28", "DATE OF THE FACTORY SOFTWARE", "2026-02-28");
  setHardwareComponentDefaults(matrix.components[HW_FACTORY_SOFTWARE_VERSION_NUMBER], "1", "SOFTWARE VERSION NUMBER", "2026-02-28");
  refreshHardwareMatrixCRC(matrix);
}

bool validateHardwareMatrix(const HardwareMatrix& matrix, String* errorCode) {
  if (matrix.matrix_magic != HW_MATRIX_MAGIC) {
    if (errorCode != NULL) {
      *errorCode = "BAD_MAGIC";
    }
    return false;
  }
  if (matrix.matrix_schema_version != HW_MATRIX_SCHEMA_VERSION) {
    if (errorCode != NULL) {
      *errorCode = "BAD_SCHEMA";
    }
    return false;
  }
  if (matrix.component_count != HW_COMPONENT_COUNT) {
    if (errorCode != NULL) {
      *errorCode = "BAD_COUNT";
    }
    return false;
  }

  uint32_t expectedCRC = computeHardwareMatrixCRC(matrix);
  if (expectedCRC != matrix.crc32) {
    if (errorCode != NULL) {
      *errorCode = "BAD_CRC";
    }
    return false;
  }

  for (int i = 0; i < HW_COMPONENT_COUNT; i++) {
    const HardwareComponentEntry& entry = matrix.components[i];
    if (!isSafeFieldString(entry.current_version, false)) {
      if (errorCode != NULL) {
        *errorCode = "BAD_CUR_VER";
      }
      return false;
    }
    if (!isSafeFieldString(entry.current_description, false)) {
      if (errorCode != NULL) {
        *errorCode = "BAD_CUR_DESC";
      }
      return false;
    }
    if (!isIso8601DateOrEmpty(entry.install_date) || entry.install_date[0] == '\0') {
      if (errorCode != NULL) {
        *errorCode = "BAD_CUR_DATE";
      }
      return false;
    }
    if (!isSafeFieldString(entry.previous_version, true) ||
        !isSafeFieldString(entry.previous_description, true) ||
        !isIso8601DateOrEmpty(entry.previous_install_date)) {
      if (errorCode != NULL) {
        *errorCode = "BAD_PREV";
      }
      return false;
    }
  }
  return true;
}

bool loadHardwareMatrixBlob(const char* key, HardwareMatrix& outMatrix) {
  nvs_handle_t nvsHandle;
  esp_err_t err = nvs_open(HW_MATRIX_NAMESPACE, NVS_READONLY, &nvsHandle);
  if (err != ESP_OK) {
    return false;
  }

  size_t blobSize = 0;
  err = nvs_get_blob(nvsHandle, key, NULL, &blobSize);
  if (err != ESP_OK || blobSize != sizeof(HardwareMatrix)) {
    nvs_close(nvsHandle);
    return false;
  }

  err = nvs_get_blob(nvsHandle, key, &outMatrix, &blobSize);
  nvs_close(nvsHandle);
  return err == ESP_OK;
}

bool saveHardwareMatrixBlob(const HardwareMatrix& matrix, bool updateLastKnownGood) {
  nvs_handle_t nvsHandle;
  esp_err_t err = nvs_open(HW_MATRIX_NAMESPACE, NVS_READWRITE, &nvsHandle);
  if (err != ESP_OK) {
    Serial.printf("Failed to open hardware matrix NVS namespace: %s\n", esp_err_to_name(err));
    return false;
  }

  err = nvs_set_blob(nvsHandle, HW_MATRIX_ACTIVE_KEY, &matrix, sizeof(HardwareMatrix));
  if (err == ESP_OK && updateLastKnownGood) {
    err = nvs_set_blob(nvsHandle, HW_MATRIX_LAST_GOOD_KEY, &matrix, sizeof(HardwareMatrix));
  }
  if (err == ESP_OK) {
    err = nvs_commit(nvsHandle);
  }
  nvs_close(nvsHandle);

  if (err != ESP_OK) {
    Serial.printf("Failed to persist hardware matrix: %s\n", esp_err_to_name(err));
    return false;
  }
  return true;
}

bool lookupHardwareComponentId(const String& componentName, HardwareComponentId& outId) {
  for (int i = 0; i < HW_COMPONENT_COUNT; i++) {
    if (componentName.equals(HARDWARE_COMPONENT_NAMES[i])) {
      outId = static_cast<HardwareComponentId>(i);
      return true;
    }
  }
  return false;
}

String getHardwareComponentsListString() {
  String response = "HW_COMPONENTS:";
  for (int i = 0; i < HW_COMPONENT_COUNT; i++) {
    if (i > 0) {
      response += ",";
    }
    response += HARDWARE_COMPONENT_NAMES[i];
  }
  return response;
}

String getHardwareComponentString(HardwareComponentId componentId) {
  if (!hardwareMatrixInitialized) {
    return "HW_COMPONENT_ERR:NOT_INITIALIZED";
  }
  const HardwareComponentEntry& entry = hardwareMatrix.components[componentId];
  String response = "HW_COMPONENT:";
  response += HARDWARE_COMPONENT_NAMES[componentId];
  response += "|";
  response += entry.current_version;
  response += "|";
  response += entry.current_description;
  response += "|";
  response += entry.install_date;
  response += "|";
  response += entry.previous_version;
  response += "|";
  response += entry.previous_description;
  response += "|";
  response += entry.previous_install_date;
  return response;
}

bool initializeHardwareMatrix() {
  String validationError = "";
  memset(&hardwareMatrixScratchActive, 0, sizeof(hardwareMatrixScratchActive));
  if (loadHardwareMatrixBlob(HW_MATRIX_ACTIVE_KEY, hardwareMatrixScratchActive) &&
      validateHardwareMatrix(hardwareMatrixScratchActive, &validationError)) {
    hardwareMatrix = hardwareMatrixScratchActive;
    hardwareMatrixInitialized = true;
    Serial.println("Hardware matrix loaded from active NVS record");
    return true;
  }

  if (validationError.length() > 0) {
    Serial.printf("Active hardware matrix invalid (%s), trying last-known-good\n", validationError.c_str());
  }

  validationError = "";
  memset(&hardwareMatrixScratchLastGood, 0, sizeof(hardwareMatrixScratchLastGood));
  if (loadHardwareMatrixBlob(HW_MATRIX_LAST_GOOD_KEY, hardwareMatrixScratchLastGood) &&
      validateHardwareMatrix(hardwareMatrixScratchLastGood, &validationError)) {
    hardwareMatrix = hardwareMatrixScratchLastGood;
    hardwareMatrixInitialized = true;
    saveHardwareMatrixBlob(hardwareMatrix, false);
    Serial.println("Recovered hardware matrix from last-known-good record");
    return true;
  }

  initializeDefaultHardwareMatrix(hardwareMatrix);
  hardwareMatrixInitialized = true;
  bool saved = saveHardwareMatrixBlob(hardwareMatrix, true);
  if (saved) {
    Serial.println("Initialized default hardware matrix");
  } else {
    Serial.println("WARNING: hardware matrix defaults loaded in RAM but persistence failed");
  }
  return saved;
}

bool setHardwareComponentByName(const String& componentName, const String& version, const String& installDate, const String& description, String& errorCode) {
  if (!hardwareMatrixInitialized) {
    if (!initializeHardwareMatrix()) {
      errorCode = "INIT_FAIL";
      return false;
    }
  }

  HardwareComponentId componentId;
  if (!lookupHardwareComponentId(componentName, componentId)) {
    errorCode = "UNKNOWN_COMPONENT";
    return false;
  }

  String versionTrim = version;
  String installTrim = installDate;
  String descriptionTrim = description;
  versionTrim.trim();
  installTrim.trim();
  descriptionTrim.trim();

  if (versionTrim.length() == 0 || versionTrim.length() >= HW_COMPONENT_VERSION_LEN) {
    errorCode = "BAD_VERSION";
    return false;
  }
  if (descriptionTrim.length() == 0 || descriptionTrim.length() >= HW_COMPONENT_DESC_LEN) {
    errorCode = "BAD_DESCRIPTION";
    return false;
  }
  if (installTrim.length() >= HW_COMPONENT_DATE_LEN || !isIso8601DateOrEmpty(installTrim.c_str()) || installTrim.length() == 0) {
    errorCode = "BAD_DATE";
    return false;
  }
  if (!isSafeFieldString(versionTrim.c_str(), false) || !isSafeFieldString(descriptionTrim.c_str(), false)) {
    errorCode = "BAD_CHARS";
    return false;
  }

  HardwareMatrix candidate = hardwareMatrix;
  HardwareComponentEntry& entry = candidate.components[componentId];
  copyBoundedString(entry.previous_version, sizeof(entry.previous_version), entry.current_version);
  copyBoundedString(entry.previous_description, sizeof(entry.previous_description), entry.current_description);
  copyBoundedString(entry.previous_install_date, sizeof(entry.previous_install_date), entry.install_date);
  copyBoundedString(entry.current_version, sizeof(entry.current_version), versionTrim.c_str());
  copyBoundedString(entry.current_description, sizeof(entry.current_description), descriptionTrim.c_str());
  copyBoundedString(entry.install_date, sizeof(entry.install_date), installTrim.c_str());
  refreshHardwareMatrixCRC(candidate);

  String validationError;
  if (!validateHardwareMatrix(candidate, &validationError)) {
    errorCode = String("INVALID_MATRIX_") + validationError;
    return false;
  }

  if (!saveHardwareMatrixBlob(candidate, true)) {
    errorCode = "PERSIST_FAIL";
    return false;
  }

  hardwareMatrix = candidate;
  errorCode = "";
  Serial.printf("Hardware matrix component updated: %s -> version=%s date=%s\n",
                HARDWARE_COMPONENT_NAMES[componentId],
                hardwareMatrix.components[componentId].current_version,
                hardwareMatrix.components[componentId].install_date);
  return true;
}

uint32_t computeCRC32Bytes(const uint8_t* bytes, size_t len) {
  uint32_t crc = 0xFFFFFFFFUL;
  for (size_t i = 0; i < len; i++) {
    crc ^= bytes[i];
    for (int bit = 0; bit < 8; bit++) {
      uint32_t mask = static_cast<uint32_t>(-(int32_t)(crc & 1U));
      crc = (crc >> 1) ^ (0xEDB88320UL & mask);
    }
  }
  return ~crc;
}

uint32_t computeHWCFGCRC(const HWCFGConfigStore& store) {
  const uint8_t* bytes = reinterpret_cast<const uint8_t*>(&store);
  return computeCRC32Bytes(bytes, offsetof(HWCFGConfigStore, crc32));
}

void refreshHWCFGCRC(HWCFGConfigStore& store) {
  store.crc32 = computeHWCFGCRC(store);
}

bool parseStrictFloat(const String& input, float& out) {
  String trimmed = input;
  trimmed.trim();
  if (trimmed.length() == 0) {
    return false;
  }
  char buffer[32];
  if (trimmed.length() >= (int)sizeof(buffer)) {
    return false;
  }
  strncpy(buffer, trimmed.c_str(), sizeof(buffer) - 1);
  buffer[sizeof(buffer) - 1] = '\0';
  char* endPtr = nullptr;
  float value = strtof(buffer, &endPtr);
  if (endPtr == buffer || *endPtr != '\0') {
    return false;
  }
  out = value;
  return true;
}

bool isKnownParameterKey(const String& key) {
  return key == "batteryThreshold" || key == "K" || key == "F" || key == "T" ||
         key == "backupTime" || key == "fanDuration" || key == "H" || key == "continueFeeder" ||
         key == "maxOpeningTime" || key == "typicalOpeningTime" || key == "MOTOR_CUT_TIME" ||
         key == "CUT_MODE_HEAT_TIME" || key == "postCoolingFanDuration" || key == "preFeedFan" ||
         key == "fanReverseTime" || key == "fanReverseStartTime" || key == "backupTimeAfterReopen" ||
         key == "CUT_MODE_TEMP" || key == "heaterLowerToleranceC" || key == "heaterUpperToleranceC" ||
         key == "COOL_OPEN_TEMP_C" || key == "MAX_COOL_WAIT_S";
}

bool validateParameterBlob(const String& componentName, const String& paramsBlob, String& errorCode) {
  if (paramsBlob.length() == 0) {
    errorCode = "EMPTY_PARAMS";
    return false;
  }
  bool hasK = false;
  bool hasCutTemp = false;
  bool hasLower = false;
  bool hasUpper = false;

  int start = 0;
  while (start < paramsBlob.length()) {
    int sep = paramsBlob.indexOf(';', start);
    if (sep < 0) {
      sep = paramsBlob.length();
    }
    String pair = paramsBlob.substring(start, sep);
    pair.trim();
    if (pair.length() > 0) {
      int eq = pair.indexOf('=');
      if (eq <= 0 || eq >= pair.length() - 1) {
        errorCode = "BAD_FORMAT";
        return false;
      }
      String key = pair.substring(0, eq);
      String value = pair.substring(eq + 1);
      key.trim();
      value.trim();
      if (!isKnownParameterKey(key)) {
        errorCode = "UNKNOWN_PARAM";
        return false;
      }
      float numericValue = 0.0f;
      if (!parseStrictFloat(value, numericValue)) {
        errorCode = "BAD_VALUE";
        return false;
      }
      if (key == "K" || key == "CUT_MODE_TEMP") {
        if (numericValue < 20.0f || numericValue > 250.0f) {
          errorCode = "OUT_OF_RANGE";
          return false;
        }
      }
      if (key == "heaterLowerToleranceC" || key == "heaterUpperToleranceC") {
        if (numericValue < -30.0f || numericValue > 30.0f) {
          errorCode = "OUT_OF_RANGE";
          return false;
        }
      }
      if (key == "MAX_COOL_WAIT_S") {
        if (numericValue < 1.0f || numericValue > 1800.0f) {
          errorCode = "OUT_OF_RANGE";
          return false;
        }
      }
      if (key == "K") hasK = true;
      if (key == "CUT_MODE_TEMP") hasCutTemp = true;
      if (key == "heaterLowerToleranceC") hasLower = true;
      if (key == "heaterUpperToleranceC") hasUpper = true;
    }
    start = sep + 1;
  }

  if (componentName == "HEATING_ELEMENT" && (!hasK || !hasCutTemp || !hasLower || !hasUpper)) {
    errorCode = "INCOMPATIBLE";
    return false;
  }

  errorCode = "";
  return true;
}

bool applyParameterBlobToRuntime(const String& paramsBlob, String& errorCode) {
  int start = 0;
  while (start < paramsBlob.length()) {
    int sep = paramsBlob.indexOf(';', start);
    if (sep < 0) {
      sep = paramsBlob.length();
    }
    String pair = paramsBlob.substring(start, sep);
    pair.trim();
    if (pair.length() > 0) {
      int eq = pair.indexOf('=');
      if (eq <= 0 || eq >= pair.length() - 1) {
        errorCode = "BAD_FORMAT";
        return false;
      }
      String key = pair.substring(0, eq);
      String value = pair.substring(eq + 1);
      key.trim();
      value.trim();
      float numericValue = 0.0f;
      if (!parseStrictFloat(value, numericValue)) {
        errorCode = "BAD_VALUE";
        return false;
      }

      if (key == "batteryThreshold") batteryThreshold = (int)numericValue;
      else if (key == "K") K = numericValue;
      else if (key == "F") F = (int)numericValue;
      else if (key == "T") T = (long)numericValue;
      else if (key == "backupTime") backupTime = numericValue;
      else if (key == "fanDuration") fanDuration = (int)numericValue;
      else if (key == "H") H = (long)numericValue;
      else if (key == "continueFeeder") continueFeeder = numericValue;
      else if (key == "maxOpeningTime") maxOpeningTime = (int)numericValue;
      else if (key == "typicalOpeningTime") typicalOpeningTime = (int)numericValue;
      else if (key == "MOTOR_CUT_TIME") MOTOR_CUT_TIME = numericValue;
      else if (key == "CUT_MODE_HEAT_TIME") CUT_MODE_HEAT_TIME = numericValue;
      else if (key == "postCoolingFanDuration") postCoolingFanDuration = numericValue;
      else if (key == "preFeedFan") preFeedFan = numericValue;
      else if (key == "fanReverseTime") fanReverseTime = numericValue;
      else if (key == "fanReverseStartTime") fanReverseStartTime = numericValue;
      else if (key == "backupTimeAfterReopen") backupTimeAfterReopen = numericValue;
      else if (key == "CUT_MODE_TEMP") CUT_MODE_TEMP = numericValue;
      else if (key == "heaterLowerToleranceC") heaterLowerToleranceC = numericValue;
      else if (key == "heaterUpperToleranceC") heaterUpperToleranceC = numericValue;
      else if (key == "COOL_OPEN_TEMP_C") COOL_OPEN_TEMP_C = numericValue;
      else if (key == "MAX_COOL_WAIT_S") MAX_COOL_WAIT_S = (long)numericValue;
      else {
        errorCode = "UNKNOWN_PARAM";
        return false;
      }
    }
    start = sep + 1;
  }

  enforceHeaterToleranceGap("hwcfg_apply", false);
  heaterTargetTemp = K;
  saveParametersToEEPROM();
  if (!lastEEPROMWriteVerified) {
    errorCode = "EEPROM_WRITE_FAIL";
    return false;
  }
  errorCode = "";
  return true;
}

int findHWCFGProfileIndex(const String& componentName, const String& componentVersion) {
  for (int i = 0; i < HWCFG_PROFILE_MAX; i++) {
    if (hwcfgStore.profiles[i].in_use == 1 &&
        componentName.equals(hwcfgStore.profiles[i].component_name) &&
        componentVersion.equals(hwcfgStore.profiles[i].component_version)) {
      return i;
    }
  }
  return -1;
}

int allocateHWCFGProfileSlot() {
  for (int i = 0; i < HWCFG_PROFILE_MAX; i++) {
    if (hwcfgStore.profiles[i].in_use == 0) {
      return i;
    }
  }
  return -1;
}

bool loadHWCFGBlob(const char* key, HWCFGConfigStore& outStore) {
  nvs_handle_t nvsHandle;
  esp_err_t err = nvs_open(HWCFG_CONFIG_NAMESPACE, NVS_READONLY, &nvsHandle);
  if (err != ESP_OK) {
    return false;
  }
  size_t blobSize = 0;
  err = nvs_get_blob(nvsHandle, key, NULL, &blobSize);
  if (err != ESP_OK || blobSize != sizeof(HWCFGConfigStore)) {
    nvs_close(nvsHandle);
    return false;
  }
  err = nvs_get_blob(nvsHandle, key, &outStore, &blobSize);
  nvs_close(nvsHandle);
  return err == ESP_OK;
}

bool saveHWCFGBlob(const HWCFGConfigStore& store, bool updateLastGood) {
  nvs_handle_t nvsHandle;
  esp_err_t err = nvs_open(HWCFG_CONFIG_NAMESPACE, NVS_READWRITE, &nvsHandle);
  if (err != ESP_OK) {
    Serial.printf("HWCFG nvs_open failed: %s\n", esp_err_to_name(err));
    return false;
  }
  err = nvs_set_blob(nvsHandle, HWCFG_ACTIVE_KEY, &store, sizeof(HWCFGConfigStore));
  if (err == ESP_OK && updateLastGood) {
    err = nvs_set_blob(nvsHandle, HWCFG_LAST_GOOD_KEY, &store, sizeof(HWCFGConfigStore));
  }
  if (err == ESP_OK) {
    err = nvs_commit(nvsHandle);
  }
  nvs_close(nvsHandle);
  if (err != ESP_OK) {
    Serial.printf("HWCFG persist failed: %s\n", esp_err_to_name(err));
    return false;
  }
  return true;
}

bool validateHWCFGStore(const HWCFGConfigStore& store, String& errorCode) {
  if (store.magic != HWCFG_MAGIC) {
    errorCode = "BAD_MAGIC";
    return false;
  }
  if (store.schema_version != HWCFG_SCHEMA_VERSION) {
    errorCode = "BAD_SCHEMA";
    return false;
  }
  if (store.profile_count > HWCFG_PROFILE_MAX) {
    errorCode = "BAD_COUNT";
    return false;
  }
  if (computeHWCFGCRC(store) != store.crc32) {
    errorCode = "BAD_CRC";
    return false;
  }
  for (int i = 0; i < HWCFG_PROFILE_MAX; i++) {
    const HWCFGProfileEntry& entry = store.profiles[i];
    if (entry.in_use == 0) {
      continue;
    }
    if (!isSafeFieldString(entry.profile_id, false) ||
        !isSafeFieldString(entry.component_name, false) ||
        !isSafeFieldString(entry.component_version, false) ||
        !isSafeFieldString(entry.params_blob, false)) {
      errorCode = "BAD_FIELDS";
      return false;
    }
    HardwareComponentId dummyId;
    if (!lookupHardwareComponentId(String(entry.component_name), dummyId)) {
      errorCode = "BAD_COMPONENT";
      return false;
    }
    String blobErr;
    if (!validateParameterBlob(String(entry.component_name), String(entry.params_blob), blobErr)) {
      errorCode = String("BAD_PROFILE_") + blobErr;
      return false;
    }
  }
  errorCode = "";
  return true;
}

void initializeDefaultHWCFG(HWCFGConfigStore& store) {
  memset(&store, 0, sizeof(store));
  store.magic = HWCFG_MAGIC;
  store.schema_version = HWCFG_SCHEMA_VERSION;
  store.profile_count = 0;
  store.active_profile_id[0] = '\0';
  store.last_good_profile_id[0] = '\0';
  store.active_validated = 0;
  refreshHWCFGCRC(store);
}

bool initializeHWCFGStore() {
  String err;
  memset(&hwcfgScratchActive, 0, sizeof(hwcfgScratchActive));
  if (loadHWCFGBlob(HWCFG_ACTIVE_KEY, hwcfgScratchActive) && validateHWCFGStore(hwcfgScratchActive, err)) {
    hwcfgStore = hwcfgScratchActive;
    hwcfgStoreInitialized = true;
    hwcfgSafeFault = false;
    return true;
  }

  memset(&hwcfgScratchLastGood, 0, sizeof(hwcfgScratchLastGood));
  err = "";
  if (loadHWCFGBlob(HWCFG_LAST_GOOD_KEY, hwcfgScratchLastGood) && validateHWCFGStore(hwcfgScratchLastGood, err)) {
    hwcfgStore = hwcfgScratchLastGood;
    hwcfgStoreInitialized = true;
    hwcfgSafeFault = false;
    saveHWCFGBlob(hwcfgStore, false);
    return true;
  }

  initializeDefaultHWCFG(hwcfgStore);
  hwcfgStoreInitialized = true;
  if (!saveHWCFGBlob(hwcfgStore, true)) {
    hwcfgSafeFault = true;
    return false;
  }
  hwcfgSafeFault = false;
  return true;
}

String buildHWCFGActiveSummary(bool lastGood) {
  String response = lastGood ? "HWCFG_LAST_GOOD:" : "HWCFG_ACTIVE:";
  for (int i = 0; i < HW_COMPONENT_COUNT; i++) {
    if (i > 0) {
      response += ";";
    }
    response += HARDWARE_COMPONENT_NAMES[i];
    response += "=";
    const HardwareComponentEntry& entry = hardwareMatrix.components[i];
    if (lastGood && entry.previous_version[0] != '\0') {
      response += entry.previous_version;
    } else {
      response += entry.current_version;
    }
  }
  response += "|profile_id=";
  response += lastGood ? hwcfgStore.last_good_profile_id : hwcfgStore.active_profile_id;
  if (!lastGood) {
    response += "|validated=";
    response += String(hwcfgStore.active_validated ? 1 : 0);
  }
  return response;
}

bool validateCandidateChange(const String& componentName, const String& newVersion, int& outProfileIndex, String& reason) {
  HardwareComponentId componentId;
  if (!lookupHardwareComponentId(componentName, componentId)) {
    reason = "UNKNOWN_COMPONENT";
    return false;
  }
  int profileIndex = findHWCFGProfileIndex(componentName, newVersion);
  if (profileIndex < 0) {
    reason = "NO_PROFILE";
    return false;
  }
  String blobErr;
  if (!validateParameterBlob(componentName, String(hwcfgStore.profiles[profileIndex].params_blob), blobErr)) {
    reason = blobErr;
    return false;
  }
  outProfileIndex = profileIndex;
  reason = "";
  return true;
}

String handleHWCFGCommand(const String& cmd) {
  if (!hwcfgStoreInitialized && !initializeHWCFGStore()) {
    return "HWCFG_APPLY_ERR:INIT_FAIL";
  }
  if (!hardwareMatrixInitialized && !initializeHardwareMatrix()) {
    return "HWCFG_APPLY_ERR:MATRIX_INIT_FAIL";
  }

  if (cmd == "HWCFG_GET_CAPS") {
    return "HWCFG_CAPS:V1|PROFILE_STORE|TXN_APPLY|ROLLBACK";
  }
  if (cmd == "HWCFG_GET_ACTIVE_CONFIG") {
    return buildHWCFGActiveSummary(false);
  }
  if (cmd == "HWCFG_GET_LAST_GOOD_CONFIG") {
    return buildHWCFGActiveSummary(true);
  }
  if (cmd == "HWCFG_PROFILE_LIST") {
    String response = "HWCFG_PROFILE_LIST:";
    bool first = true;
    for (int i = 0; i < HWCFG_PROFILE_MAX; i++) {
      if (hwcfgStore.profiles[i].in_use == 0) {
        continue;
      }
      if (!first) response += ",";
      first = false;
      response += hwcfgStore.profiles[i].profile_id;
      response += "@";
      response += hwcfgStore.profiles[i].component_name;
      response += ":";
      response += hwcfgStore.profiles[i].component_version;
    }
    return response;
  }

  if (cmd.startsWith("HWCFG_PROFILE_GET:")) {
    String payload = cmd.substring(String("HWCFG_PROFILE_GET:").length());
    int sep = payload.indexOf('|');
    if (sep < 0) {
      return "HWCFG_VALIDATE_ERR:BAD_FORMAT";
    }
    String component = payload.substring(0, sep);
    String versionPart = payload.substring(sep + 1);
    component.trim();
    versionPart.trim();
    if (!versionPart.startsWith("version=")) {
      return "HWCFG_VALIDATE_ERR:BAD_FORMAT";
    }
    String version = versionPart.substring(String("version=").length());
    int idx = findHWCFGProfileIndex(component, version);
    if (idx < 0) {
      return "HWCFG_VALIDATE_ERR:NO_PROFILE";
    }
    const HWCFGProfileEntry& profile = hwcfgStore.profiles[idx];
    String response = "HWCFG_PROFILE:";
    response += profile.profile_id;
    response += "|component=";
    response += profile.component_name;
    response += "|version=";
    response += profile.component_version;
    response += "|params=";
    response += profile.params_blob;
    return response;
  }

  if (cmd.startsWith("HWCFG_PROFILE_PUT:")) {
    String payload = cmd.substring(String("HWCFG_PROFILE_PUT:").length());
    int sep1 = payload.indexOf('|');
    int sep2 = (sep1 >= 0) ? payload.indexOf('|', sep1 + 1) : -1;
    int sep3 = (sep2 >= 0) ? payload.indexOf('|', sep2 + 1) : -1;
    if (sep1 < 0 || sep2 < 0 || sep3 < 0) {
      return "HWCFG_VALIDATE_ERR:BAD_FORMAT";
    }
    String profileId = payload.substring(0, sep1);
    String componentPart = payload.substring(sep1 + 1, sep2);
    String versionPart = payload.substring(sep2 + 1, sep3);
    String paramsBlob = payload.substring(sep3 + 1);
    profileId.trim();
    componentPart.trim();
    versionPart.trim();
    paramsBlob.trim();
    if (!componentPart.startsWith("component=") || !versionPart.startsWith("version=")) {
      return "HWCFG_VALIDATE_ERR:BAD_FORMAT";
    }
    String component = componentPart.substring(String("component=").length());
    String version = versionPart.substring(String("version=").length());
    component.trim();
    version.trim();
    if (profileId.length() == 0 || component.length() == 0 || version.length() == 0 || paramsBlob.length() == 0) {
      return "HWCFG_VALIDATE_ERR:BAD_FORMAT";
    }
    if (profileId.length() >= HWCFG_PROFILE_ID_LEN || component.length() >= HW_COMPONENT_DESC_LEN ||
        version.length() >= HW_COMPONENT_VERSION_LEN || paramsBlob.length() >= HWCFG_PROFILE_PARAM_BLOB_LEN) {
      return "HWCFG_VALIDATE_ERR:TOO_LONG";
    }
    HardwareComponentId id;
    if (!lookupHardwareComponentId(component, id)) {
      return "HWCFG_VALIDATE_ERR:UNKNOWN_COMPONENT";
    }
    String blobErr;
    if (!validateParameterBlob(component, paramsBlob, blobErr)) {
      return String("HWCFG_VALIDATE_ERR:") + blobErr;
    }
    int idx = findHWCFGProfileIndex(component, version);
    if (idx < 0) {
      idx = allocateHWCFGProfileSlot();
      if (idx < 0) {
        return "HWCFG_VALIDATE_ERR:PROFILE_FULL";
      }
      hwcfgStore.profiles[idx].in_use = 1;
      hwcfgStore.profile_count++;
    }
    copyBoundedString(hwcfgStore.profiles[idx].profile_id, sizeof(hwcfgStore.profiles[idx].profile_id), profileId.c_str());
    copyBoundedString(hwcfgStore.profiles[idx].component_name, sizeof(hwcfgStore.profiles[idx].component_name), component.c_str());
    copyBoundedString(hwcfgStore.profiles[idx].component_version, sizeof(hwcfgStore.profiles[idx].component_version), version.c_str());
    copyBoundedString(hwcfgStore.profiles[idx].params_blob, sizeof(hwcfgStore.profiles[idx].params_blob), paramsBlob.c_str());
    refreshHWCFGCRC(hwcfgStore);
    if (!saveHWCFGBlob(hwcfgStore, true)) {
      return "HWCFG_VALIDATE_ERR:PERSIST_FAIL";
    }
    return String("HWCFG_VALIDATE_OK:") + component + "|version=" + version + "|profile_id=" + profileId;
  }

  if (cmd.startsWith("HWCFG_VALIDATE_CHANGE:")) {
    String payload = cmd.substring(String("HWCFG_VALIDATE_CHANGE:").length());
    int sep = payload.indexOf('|');
    if (sep < 0) {
      return "HWCFG_VALIDATE_ERR:BAD_FORMAT";
    }
    String component = payload.substring(0, sep);
    String versionPart = payload.substring(sep + 1);
    component.trim();
    versionPart.trim();
    if (!versionPart.startsWith("new_version=")) {
      return "HWCFG_VALIDATE_ERR:BAD_FORMAT";
    }
    String version = versionPart.substring(String("new_version=").length());
    int profileIndex = -1;
    String reason;
    if (!validateCandidateChange(component, version, profileIndex, reason)) {
      return String("HWCFG_VALIDATE_ERR:") + reason;
    }
    String response = "HWCFG_VALIDATE_OK:";
    response += component;
    response += "|version=";
    response += version;
    response += "|profile_id=";
    response += hwcfgStore.profiles[profileIndex].profile_id;
    return response;
  }

  if (cmd.startsWith("HWCFG_APPLY_CHANGE:")) {
    if (hwcfgSafeFault) {
      return "HWCFG_APPLY_ERR:SAFE_FAULT";
    }
    String payload = cmd.substring(String("HWCFG_APPLY_CHANGE:").length());
    int sep1 = payload.indexOf('|');
    int sep2 = (sep1 >= 0) ? payload.indexOf('|', sep1 + 1) : -1;
    int sep3 = (sep2 >= 0) ? payload.indexOf('|', sep2 + 1) : -1;
    if (sep1 < 0 || sep2 < 0 || sep3 < 0) {
      return "HWCFG_APPLY_ERR:BAD_FORMAT";
    }
    String component = payload.substring(0, sep1);
    String versionPart = payload.substring(sep1 + 1, sep2);
    String datePart = payload.substring(sep2 + 1, sep3);
    String descPart = payload.substring(sep3 + 1);
    component.trim();
    versionPart.trim();
    datePart.trim();
    descPart.trim();
    if (!versionPart.startsWith("new_version=") || !datePart.startsWith("install_date=") || !descPart.startsWith("desc=")) {
      return "HWCFG_APPLY_ERR:BAD_FORMAT";
    }
    String version = versionPart.substring(String("new_version=").length());
    String installDate = datePart.substring(String("install_date=").length());
    String description = descPart.substring(String("desc=").length());
    int profileIndex = -1;
    String reason;
    if (!validateCandidateChange(component, version, profileIndex, reason)) {
      return String("HWCFG_APPLY_ERR:") + reason;
    }
    if (!isIso8601DateOrEmpty(installDate.c_str()) || installDate.length() == 0) {
      return "HWCFG_APPLY_ERR:BAD_DATE";
    }
    if (!isSafeFieldString(description.c_str(), false)) {
      return "HWCFG_APPLY_ERR:BAD_DESC";
    }

    // Snapshot current runtime parameters for rollback in case of apply failure.
    int oldBatteryThreshold = batteryThreshold;
    float oldK = K;
    int oldF = F;
    long oldT = T;
    float oldBackupTime = backupTime;
    int oldFanDuration = fanDuration;
    long oldH = H;
    float oldContinueFeeder = continueFeeder;
    int oldMaxOpeningTime = maxOpeningTime;
    int oldTypicalOpeningTime = typicalOpeningTime;
    float oldMotorCutTime = MOTOR_CUT_TIME;
    float oldCutModeHeatTime = CUT_MODE_HEAT_TIME;
    float oldPostCooling = postCoolingFanDuration;
    float oldPreFeedFan = preFeedFan;
    float oldFanReverseTime = fanReverseTime;
    float oldFanReverseStartTime = fanReverseStartTime;
    float oldBackupAfterReopen = backupTimeAfterReopen;
    float oldCutModeTemp = CUT_MODE_TEMP;
    float oldLowerTol = heaterLowerToleranceC;
    float oldUpperTol = heaterUpperToleranceC;
    float oldCoolOpen = COOL_OPEN_TEMP_C;
    long oldMaxCoolWait = MAX_COOL_WAIT_S;
    hwcfgSnapshotMatrix = hardwareMatrix;
    hwcfgSnapshotStore = hwcfgStore;

    String applyError;
    if (!applyParameterBlobToRuntime(String(hwcfgStore.profiles[profileIndex].params_blob), applyError)) {
      return String("HWCFG_APPLY_ERR:") + applyError;
    }

    if (!setHardwareComponentByName(component, version, installDate, description, applyError)) {
      // Roll back runtime params and matrix.
      batteryThreshold = oldBatteryThreshold;
      K = oldK;
      F = oldF;
      T = oldT;
      backupTime = oldBackupTime;
      fanDuration = oldFanDuration;
      H = oldH;
      continueFeeder = oldContinueFeeder;
      maxOpeningTime = oldMaxOpeningTime;
      typicalOpeningTime = oldTypicalOpeningTime;
      MOTOR_CUT_TIME = oldMotorCutTime;
      CUT_MODE_HEAT_TIME = oldCutModeHeatTime;
      postCoolingFanDuration = oldPostCooling;
      preFeedFan = oldPreFeedFan;
      fanReverseTime = oldFanReverseTime;
      fanReverseStartTime = oldFanReverseStartTime;
      backupTimeAfterReopen = oldBackupAfterReopen;
      CUT_MODE_TEMP = oldCutModeTemp;
      heaterLowerToleranceC = oldLowerTol;
      heaterUpperToleranceC = oldUpperTol;
      COOL_OPEN_TEMP_C = oldCoolOpen;
      MAX_COOL_WAIT_S = oldMaxCoolWait;
      hardwareMatrix = hwcfgSnapshotMatrix;
      saveParametersToEEPROM();
      saveHardwareMatrixBlob(hardwareMatrix, false);
      return String("HWCFG_APPLY_ERR:") + applyError;
    }

    copyBoundedString(hwcfgStore.last_good_profile_id, sizeof(hwcfgStore.last_good_profile_id), hwcfgSnapshotStore.active_profile_id);
    copyBoundedString(hwcfgStore.active_profile_id, sizeof(hwcfgStore.active_profile_id), hwcfgStore.profiles[profileIndex].profile_id);
    hwcfgStore.active_validated = 1;
    refreshHWCFGCRC(hwcfgStore);
    if (!saveHWCFGBlob(hwcfgStore, true)) {
      hwcfgStore = hwcfgSnapshotStore;
      hardwareMatrix = hwcfgSnapshotMatrix;
      saveHardwareMatrixBlob(hardwareMatrix, false);
      return "HWCFG_APPLY_ERR:PERSIST_FAIL";
    }

    String ack = "HWCFG_APPLY_ACK:";
    ack += component;
    ack += "|version=";
    ack += version;
    return ack;
  }

  if (cmd == "HWCFG_ROLLBACK_LAST_GOOD") {
    memset(&hwcfgScratchLastGood, 0, sizeof(hwcfgScratchLastGood));
    String err;
    if (!loadHWCFGBlob(HWCFG_LAST_GOOD_KEY, hwcfgScratchLastGood) || !validateHWCFGStore(hwcfgScratchLastGood, err)) {
      return "HWCFG_ROLLBACK_ERR:NO_LAST_GOOD";
    }
    hwcfgStore = hwcfgScratchLastGood;
    refreshHWCFGCRC(hwcfgStore);
    if (!saveHWCFGBlob(hwcfgStore, false)) {
      return "HWCFG_ROLLBACK_ERR:PERSIST_FAIL";
    }
    return "HWCFG_ROLLBACK_ACK";
  }

  return "HWCFG_VALIDATE_ERR:UNKNOWN_COMMAND";
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
    // Sanitize description: only append printable ASCII so BLE payload is valid UTF-8
    for (size_t i = 0; i < sizeof(hwInfo.hardware_description) && hwInfo.hardware_description[i] != '\0'; i++) {
      char c = hwInfo.hardware_description[i];
      versionString += (c >= 0x20 && c <= 0x7E) ? c : '?';
    }
  }
  return versionString;
}

// Prepare device for OTA update
bool prepareForOTA() {
  // When OTA mode is enabled, allow preparation (we already idled in enableOTA())
  if (otaEnabled) {
    isFlushing = false;
    updateInProgress = false;
  }
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
  setFanSpeed(0);
  
  // Reset OTA state
  resetOTAState();
  if (!setOTAState(OTA_PREPARING, "prepare command")) {
    publishOTAErrorStatus("INVALID_PREPARE_TRANSITION");
    return false;
  }
  updateInProgress = true;
  otaLastChunkMillis = millis();
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

String buildDevModeStatusMessage() {
  return String("DEV_MODE:") + String(devModeEnabled ? 1 : 0);
}

bool saveDevModeSetting(bool enabled) {
  nvs_handle_t nvsHandle;
  esp_err_t err = nvs_open(DEV_MODE_NAMESPACE, NVS_READWRITE, &nvsHandle);
  if (err != ESP_OK) {
    Serial.printf("Failed to open NVS for DEV mode write: %s\n", esp_err_to_name(err));
    return false;
  }

  err = nvs_set_u8(nvsHandle, DEV_MODE_KEY, enabled ? 1 : 0);
  if (err == ESP_OK) {
    err = nvs_commit(nvsHandle);
  }
  nvs_close(nvsHandle);

  if (err != ESP_OK) {
    Serial.printf("Failed to persist DEV mode setting: %s\n", esp_err_to_name(err));
    return false;
  }
  return true;
}

bool loadDevModeSetting() {
  nvs_handle_t nvsHandle;
  esp_err_t err = nvs_open(DEV_MODE_NAMESPACE, NVS_READONLY, &nvsHandle);
  if (err == ESP_ERR_NVS_NOT_FOUND) {
    devModeEnabled = false;
    Serial.println("DEV mode setting namespace not found. Defaulting to NORMAL mode.");
    return true;
  }
  if (err != ESP_OK) {
    Serial.printf("Failed to open NVS for DEV mode read: %s\n", esp_err_to_name(err));
    return false;
  }

  uint8_t storedValue = 0;
  err = nvs_get_u8(nvsHandle, DEV_MODE_KEY, &storedValue);
  nvs_close(nvsHandle);

  if (err == ESP_ERR_NVS_NOT_FOUND) {
    devModeEnabled = false;
    Serial.println("DEV mode key missing. Defaulting to NORMAL mode.");
    return true;
  }
  if (err != ESP_OK) {
    Serial.printf("Failed to read DEV mode setting: %s\n", esp_err_to_name(err));
    return false;
  }

  devModeEnabled = (storedValue == 1);
  Serial.printf("Loaded DEV mode from NVS: %d\n", devModeEnabled ? 1 : 0);
  return true;
}

bool setDevModeEnabled(bool enabled) {
  if (!saveDevModeSetting(enabled)) {
    return false;
  }
  devModeEnabled = enabled;
  // Restart power-saving timers so mode transitions are predictable.
  bleStartupTime = millis();
  bleIdleStartTime = millis();
  lastActivityMillis = millis();
  Serial.printf("DEV mode updated: %d\n", devModeEnabled ? 1 : 0);
  return true;
}

// Reset OTA state variables
void resetOTAState() {
  setOTAState(OTA_IDLE, "reset");
  ota_handle = 0;
  firmware_size = 0;
  bytes_received = 0;
  chunk_sequence = 0;
  md5_received = false;
  rollback_required = false;
  ota_error_message = "";
  last_update_status = "UPDATE_IDLE";
  otaLastChunkMillis = 0;
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
  otaLastChunkMillis = millis();
  
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
      setOTAState(OTA_VALIDATING, "md5 received");
      publishOTAStatus("UPDATE_VALIDATING");
    }
    return;
  }
  
  // Regular firmware data chunk
  if (ota_handle == 0) {
    Serial.println("ERROR: OTA handle not initialized");
    setOTAState(OTA_ERROR, "chunk received without handle");
    ota_error_message = "OTA handle not initialized";
    publishOTAErrorStatus("HANDLE_NOT_INITIALIZED");
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
    setOTAState(OTA_ERROR, "esp_ota_write failed");
    ota_error_message = "OTA write failed: " + String(esp_err_to_name(err));
    publishOTAErrorStatus("WRITE_FAILED");
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
    Serial.println("Firmware reception complete, awaiting MD5.");
  }
}

// Test heater current detection at startup
void testHeaterCurrent() {
  Serial.println("Starting heater current detection test...");
  SerialBLE_println("Starting heater current detection test...");
  
  // Set heater fully ON for current detection.
  digitalWrite(heaterPin, HIGH);
  heaterOutputOn = true;
  Serial.println("Heater set to ON for startup current test");
  SerialBLE_println("Heater set to ON for startup current test");
  
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
  digitalWrite(heaterPin, LOW);
  heaterOutputOn = false;
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
  if (ignoreM12Faults) {
    Serial.println("WARNING: M1/M2 motor faults are set to LOG-ONLY (ignoreM12Faults=true)");
  }

  esp_sleep_wakeup_cause_t wakeCause = esp_sleep_get_wakeup_cause();
  bool wokeFromDeepSleep = (wakeCause == ESP_SLEEP_WAKEUP_EXT0);

  // PWM fan pins - configure early to ensure fan is off before BLE (hardware requirement)
  pinMode(sealerFanPwr, OUTPUT);
  pinMode(sealerFanReverse, OUTPUT);
  ledcAttach(sealerFanPWM, FAN_PWM_FREQ, FAN_PWM_RESOLUTION);
  setFanSpeed(0);  // Ensure fan is off at startup

  if (wokeFromDeepSleep) {
    // Fast wake path: no checkBootFailure, no BLE, no locateMotorPos
    bleEnabled = false;
    Serial.println("\n\n=== WOKE FROM DEEP SLEEP (GPIO2) ===");

    // Release GPIO2 from RTC IO so it can be used as normal digital input again
    rtc_gpio_deinit((gpio_num_t)controlPanelWake);

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
    loadDevModeSetting();
    initializeHardwareMatrix();
    if (!initializeHWCFGStore()) {
      Serial.println("WARNING: HWCFG store failed to initialize on wake path");
      SerialBLE_println("WARNING: HWCFG store init failed");
    }
    if (ignoreM12Faults) {
      SerialBLE_println("WARNING: M1/M2 motor faults are LOG-ONLY");
    }

    mcp_setup();
    Serial.println("MCP23017 Initialized");
    circleLeds();

    loadParametersFromEEPROM();
    loadFlushCountFromEEPROM();
    knownResistor = thermistorResistance;
    heaterTargetTemp = K;
    startEEPROMWakeAlert();
    maintainEEPROMErrorIndicator();

    pinMode(heaterPin, OUTPUT);
    pinMode(buzzerPin, OUTPUT);
    pinMode(microswitchClosePin, INPUT_PULLUP);
    pinMode(microswitchOpenPin, INPUT_PULLUP);
    pinMode(controlPanelWake, INPUT_PULLUP);
    pinMode(M1NFAULT_PIN, INPUT);
    pinMode(M2NFAULT_PIN, INPUT);
    pinMode(M1NEN_PIN, OUTPUT);
    pinMode(M2NEN_PIN, OUTPUT);
    pinMode(M1DIR_PIN, OUTPUT);
    pinMode(M2DIR_PIN, OUTPUT);
    pinMode(M1PWM_PIN, OUTPUT);
    pinMode(M2PWM_PIN, OUTPUT);
    pinMode(m1CurrentPin, INPUT);
    pinMode(heaterCurrentPin, INPUT);
    pinMode(batteryVoltagePin, INPUT);
    pinMode(batteryTempPin, INPUT);

    testHeaterCurrent();
    motors.enableDrivers();
    // Skip locateMotorPos() on wake - mechanism has not moved
    return;
  }

  // Cold boot: full startup with BLE and motor homing
  Serial.println("\n\n=== SETUP STARTING ===");
  Serial.printf("Free heap at startup: %d bytes\n", ESP.getFreeHeap());
  Serial.printf("Largest free block: %d bytes\n", ESP.getMaxAllocHeap());
  Serial.println("Beginning Setup");

  esp_err_t err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    err = nvs_flash_init();
  }
  ESP_ERROR_CHECK(err);
  loadDevModeSetting();
  initializeHardwareMatrix();
  if (!initializeHWCFGStore()) {
    Serial.println("WARNING: HWCFG store failed to initialize on cold boot");
    SerialBLE_println("WARNING: HWCFG store init failed");
  }
  if (ignoreM12Faults) {
    SerialBLE_println("WARNING: M1/M2 motor faults are LOG-ONLY");
  }

  checkBootFailure();

  mcp_setup();
  Serial.println("MCP23017 Initialized");
  circleLeds();

  Serial.println("LED startup test complete");

  bleStartupTime = millis();
  bleIdleStartTime = millis();
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

  sendSerialToBLE("\n=== MEMORY AFTER BLE SETUP ===");
  sendSerialToBLE("Free heap: " + String(ESP.getFreeHeap()) + " bytes");
  sendSerialToBLE("Largest free block: " + String(ESP.getMaxAllocHeap()) + " bytes");
  sendSerialToBLE("Min free heap: " + String(ESP.getMinFreeHeap()) + " bytes");

  if (devModeEnabled) {
    Serial.println("DEV mode is ON: BLE stays enabled and inactivity sleep is disabled.");
  } else {
    Serial.println("DEV mode is OFF: BLE timeout and inactivity sleep are active.");
  }

  Serial.println("DEBUG: About to load parameters from EEPROM");
  loadParametersFromEEPROM();
  loadFlushCountFromEEPROM();
  knownResistor = thermistorResistance;
  Serial.printf("DEBUG: After EEPROM load - H=%ld, K=%.1f\n", H, K);
  SerialBLE_println("DEBUG: About to load parameters from EEPROM");
  SerialBLE_print("DEBUG: After EEPROM load - H=");
  SerialBLE_print((int)H);
  SerialBLE_print(", K=");
  SerialBLE_print(K);
  SerialBLE_println();
  maintainEEPROMErrorIndicator();

  heaterTargetTemp = K;

  pinMode(heaterPin, OUTPUT);
  pinMode(buzzerPin, OUTPUT);
  pinMode(microswitchClosePin, INPUT_PULLUP);
  pinMode(microswitchOpenPin, INPUT_PULLUP);
  pinMode(controlPanelWake, INPUT_PULLUP);
  pinMode(M1NFAULT_PIN, INPUT);
  pinMode(M2NFAULT_PIN, INPUT);
  pinMode(M1NEN_PIN, OUTPUT);
  pinMode(M2NEN_PIN, OUTPUT);
  pinMode(M1DIR_PIN, OUTPUT);
  pinMode(M2DIR_PIN, OUTPUT);
  pinMode(M1PWM_PIN, OUTPUT);
  pinMode(M2PWM_PIN, OUTPUT);
  pinMode(m1CurrentPin, INPUT);
  pinMode(heaterCurrentPin, INPUT);
  pinMode(batteryVoltagePin, INPUT);
  pinMode(batteryTempPin, INPUT);

  testHeaterCurrent();
  motors.enableDrivers();
  locateMotorPos();
}

void loop() {
  if (lastActivityMillis == 0) {
    lastActivityMillis = millis();
  }
  maintainEEPROMErrorIndicator();

  // OTA mode handling
  if (otaEnabled) {
    // OTA mode is active - run slow circle animation
    slowCircleLeds();
    checkOTATimeouts();
    
    // Check if OTA window has expired - never expire while transfer in progress
    unsigned long currentMillis = millis();
    if (!otaTransferInProgress()) {
      if (currentMillis - otaWindowStartTime >= OTA_MODE_MAX_DURATION) {
        // 2 minutes reached - always exit OTA and stop circle
        Serial.println("OTA mode max duration (2 min) reached - disabling OTA and restarting BLE");
        restartBLEServer();
        batteryMonitoringActive = false;
        lastBatteryCheckTime = millis();
      } else if (currentMillis - otaWindowStartTime >= OTA_WINDOW_DURATION) {
        // 1 minute - expire only if no OTA connection
        bool hasOTAConnection = (is_device_connected && update_characteristic != NULL);
        if (!hasOTAConnection) {
          Serial.println("OTA window expired with no connections - disabling OTA and restarting BLE");
          restartBLEServer();
          batteryMonitoringActive = false;
          lastBatteryCheckTime = millis();
        } else {
          Serial.println("OTA connection active - keeping OTA mode enabled");
        }
      }
    }
  }
  
  // Keep BLE alive while serial streaming is active. When idle (not streaming),
  // allow timeout after BLE_TIMEOUT even if a client remains connected.
  if (serial_streaming_enabled) {
    bleIdleStartTime = millis();
  }

  // Check for BLE timeout during idle (no serial streaming) - never shut down during OTA transfer.
  if (!devModeEnabled && bleEnabled && !otaTransferInProgress() && !serial_streaming_enabled &&
      (millis() - bleIdleStartTime > BLE_TIMEOUT)) {
    Serial.println("BLE shutting down after 10 minutes to save power");
    SerialBLE_println("BLE shutting down after 10 minutes to save power");
    
    // Disconnect all clients and stop advertising
    if (blue_server) {
      is_device_connected = false;
      serial_streaming_enabled = false;
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
    //Serial.println("hi");
  } else {
    // Send periodic status when connected
    static unsigned long lastStatusPrint = 0;
    if (millis() - lastStatusPrint > 5000) { // Every 5 seconds
      lastStatusPrint = millis();
      Serial.println("BLE Connected - System Running");
      sendSerialToBLE("System Status: Running - " + String(millis() / 1000) + "s");
      String motorFaultStatus = "Motor Fault Status: " + buildMotorFaultStatusSnapshot();
      Serial.println(motorFaultStatus);
      sendSerialToBLE(motorFaultStatus);
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

  if (eepromWakeAlertActive) {
    if (millis() - eepromWakeAlertStartMillis < EEPROM_WAKE_ALERT_MS) {
      // Show latched EEPROM error indicator while holding controls for user awareness.
      maintainEEPROMErrorIndicator();
      return;
    }
    eepromWakeAlertActive = false;
    Serial.println("EEPROM wake alert ended. Normal operation resumed.");
    sendSerialToBLE("EEPROM wake alert ended. Normal operation resumed");
  }
  
  // OTA update handling is now done via update_characteristic_callbacks class
  // Read button states
  bool button1Pressed = (digitalRead(controlPanelWake) == LOW);
  bool button2Pressed = (mcp_digitalRead(button2Pin) == LOW);
  if (button1Pressed || button2Pressed) {
    lastActivityMillis = millis();
  }

  // Determine if both buttons are pressed
  bool bothButtonsPressedNow = button1Pressed && button2Pressed;
  
  // Check for both buttons pressed simultaneously
  if (bothButtonsPressedNow && isHardwareLikelyDisconnectedForUserAction()) {
    if (!bothButtonsPressed) {
      SerialBLE_println("Hardware not connected (thermistor open) - battery read blocked");
      playHardwareNotConnectedAlert();
    }
    bothButtonsPressed = true;
    batteryMonitoringActive = false;
    batteryDisplayMode = false;
  } else if (bothButtonsPressedNow && !bothButtonsPressed && !otaEnabled) {
    SerialBLE_println("Both buttons pressed - Stopping all operations and displaying battery charge level");
    
    // Stop all ongoing operations immediately
    Serial.println("DEBUG: Stopping all motors for battery display");
    motors.setM2Speed(0);  // Stop feed motor
    setFanSpeed(0);         // Stop fan
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
      button1DisconnectAlertedForCurrentPress = false;
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
        if (isHardwareLikelyDisconnectedForUserAction()) {
          if (!button1DisconnectAlertedForCurrentPress) {
            SerialBLE_println("Hardware not connected (thermistor open) - flush blocked");
            playHardwareNotConnectedAlert();
            button1DisconnectAlertedForCurrentPress = true;
          }
        } else {
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
      }
    } else if (!button1Pressed && button1Held) {
      // Button 1 released - check if it was a short press
      unsigned long holdDuration = millis() - button1HoldStartTime;
      button1Held = false;
      
      if (holdDuration < BUTTON_HOLD_TIME) {
        if (isHardwareLikelyDisconnectedForUserAction()) {
          SerialBLE_println("Hardware not connected (thermistor open) - flush blocked");
          playHardwareNotConnectedAlert();
          button1DisconnectAlertedForCurrentPress = true;
        } else {
        // Short press - also enable cutBag mode before starting flush
        cutBag = true;
        SerialBLE_println("Cut Bag mode enabled!");
        SerialBLE_println("Short press - starting flush sequence with cut bag enabled");
        Serial.printf("DEBUG: cutBag set to true, short press duration: %lu ms\n", holdDuration);
        cutModeLEDAnimation();
        
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
      button2DisconnectAlertedForCurrentPress = false;
      SerialBLE_println("Button 2 pressed - waiting for delay before starting feed");
      // Don't change motors yet - wait for delay
    } else if (button2Pressed && button2DelayActive && (millis() - button2PressStartTime >= BUTTON_DELAY)) {
      // Delay passed while button still pressed - start fan first
      button2DelayActive = false;
      button2FeedStarted = false;
      button2FanStartTime = millis();
      SerialBLE_println("Feed Down Delay Passed - Starting fan, waiting before feed");
      setFanSpeed(400);  // Start M3 fan immediately
      fanRunning = false;  
    } else if (button2Pressed && !button2DelayActive && !button2FeedStarted) {
      // Button 2 pressed, delay passed, but M2 not started yet - wait for preFeedFan
      if (millis() - button2FanStartTime >= preFeedFan * 1000) {
        if (isHardwareLikelyDisconnectedForUserAction()) {
          if (!button2DisconnectAlertedForCurrentPress) {
            SerialBLE_println("Hardware not connected (thermistor open) - feed blocked");
            playHardwareNotConnectedAlert();
            button2DisconnectAlertedForCurrentPress = true;
          }
          motors.setM2Speed(0);
          setFanSpeed(0);
          fanRunning = false;
          button2FeedStarted = false;
        } else {
          SerialBLE_println("Pre-feed fan delay complete, starting feed motor");
          motors.setM2Speed(-400);
          button2FeedStarted = true;
        }
      }
    } else if (button2Pressed && !button2DelayActive && button2FeedStarted) {
      static unsigned long lastFeedKeepRunningLog = 0;
      if (millis() - lastFeedKeepRunningLog >= BLE_STATUS_LOG_INTERVAL_MS) {
        SerialBLE_println("b2 pressed, feed running - keep motors running");
        lastFeedKeepRunningLog = millis();
      }
      // Button 2 is pressed, delay has passed, and feed has started - keep motors running
      //motors.setM2Speed(-400);  // Ensure M2 is running
      //setFanSpeed(400);  // Continuously ensure M3 at full power
      fanRunning = false;  // Prevent fan timer from interfering
    } else if (!button2Pressed && button2WasPressed) {
      // Button 2 just released - stop feed motor, start fan timer
      SerialBLE_println("Feed Down Released - Stopping feed, starting fan timer");
      motors.setM2Speed(0);
      button2FeedStarted = false;  // Reset flag for next button press
      button2DisconnectAlertedForCurrentPress = false;
      button2FanStartTime = 0;
      fanStartTime = millis();  // Start fan timer
      fanRunning = true;        // Set fan running flag
      //setFanSpeed(255);  // Keep fan running during timer
    } else if (!button2Pressed && fanRunning && (millis() - fanStartTime < fanDuration * 1000)) {
      // Throttle keep-running status logs to avoid BLE spam in the loop.
      static unsigned long lastFanKeepRunningLog = 0;
      if (millis() - lastFanKeepRunningLog >= BLE_STATUS_LOG_INTERVAL_MS) {
        SerialBLE_println("b2 not pressed, fan running, no time out - keep running ");
        lastFanKeepRunningLog = millis();
      }
      // Fan timer is active - keep M3 running
      //setFanSpeed(255);  // Keep fan running
    } else if (!button2Pressed && fanRunning && (millis() - fanStartTime >= fanDuration * 1000)) {
      // Fan timer completed - stop fan
      SerialBLE_println("b2 not pressed, fan running, time out ");
      Serial.println("Fan timer completed - stopping fan");
      setFanSpeed(0);  // Stop M3 motor/fan
      fanRunning = false;  // Clear fan running flag
    } else {
      // Default: button not pressed, no active fan - ensure M3 (fan) is stopped
      // Only if not flushing (flush sequence controls M3 during flush)
      if (!isFlushing) {
        setFanSpeed(0);
        //SerialBLE_println("stopping fan - no conditions met and not flushing");
        fanRunning = false;
      }
      
    }
  }  // Close if (!batteryDisplayMode)
  
  // Clear delay flags if buttons released before delay completes
  if (!button1Pressed && button1DelayActive) {
    button1DelayActive = false;
    button1DisconnectAlertedForCurrentPress = false;
    Serial.println("Button 1 released before delay - cancelled");
  }
  if (!button2Pressed && button2DelayActive) {
    button2DelayActive = false;
    button2DisconnectAlertedForCurrentPress = false;
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

  // Inactivity light sleep: 2 min with no activity, wake on GPIO2 (controlPanelWake)
  // Only enter when wake button is released (HIGH) so pin does not trigger immediate wake.
  // Light sleep keeps GPIO state so fan pins stay LOW (fan off) without hold logic.
  if (!devModeEnabled && !otaEnabled && !batteryDisplayMode && !isFlushing && !mechanismMotorRunning && !fanRunning &&
      (millis() - lastActivityMillis >= INACTIVITY_SLEEP_MS) &&
      (digitalRead(controlPanelWake) == HIGH)) {
    motors.setM2Speed(0);
    setFanSpeed(0);
    motors.setM1Speed(0);
    // RTC GPIO pull-up so pin stays HIGH in sleep and only wakes when button pulls LOW
    rtc_gpio_init((gpio_num_t)controlPanelWake);
    rtc_gpio_pullup_en((gpio_num_t)controlPanelWake);
    rtc_gpio_pulldown_dis((gpio_num_t)controlPanelWake);
    esp_sleep_enable_ext0_wakeup((gpio_num_t)controlPanelWake, 0);  // LOW = button pressed
    esp_light_sleep_start();
    circleLeds();
    startEEPROMWakeAlert();
    maintainEEPROMErrorIndicator();
    lastActivityMillis = millis();
  }

  if (heaterOn) {
    float heaterTemp = readTemperature();
    updateHeaterControl(heaterTemp);
    unsigned long now = millis();
    if (heaterTemp > K) {
      timeAboveSetpointMillis += (now - lastHeaterCheckMillis);
    }
    lastHeaterCheckMillis = now;
    // After cut motor (cut mode): count time above CUT_MODE_TEMP for CUT_MODE_HEAT_TIME
    if (isFlushing && flushStep == 6 && cutBag && case6CutMotorRun) {
      if (heaterTemp >= CUT_MODE_TEMP) {
        timeAboveCutModeTempMillis += (now - lastCutModeTempCheckMillis);
      }
      lastCutModeTempCheckMillis = now;
    }
    // In cut-bag flushes, allow headroom relative to the higher cut-mode target.
    // This prevents false overheat trips when K is intentionally lower than CUT_MODE_TEMP.
    float overheatBaseTemp = heaterTargetTemp;
    if (isFlushing && cutBag) {
      overheatBaseTemp = max(K, CUT_MODE_TEMP);
    }
    if (heaterTemp >= overheatBaseTemp * 1.2 && ERROR_CODE == 0) {
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
  
  // Check buttons for direction change only while idle.
  // During flush, LED progress is phase-driven and monotonic.
  if (!isFlushing) {
    if (digitalRead(controlPanelWake) == LOW) { // Button 1 / wake switch (GPIO2)
      lastActivityMillis = millis();
      Serial.println("Button 1 pressed: Clockwise");
      clockwise = true;
      ledIndex = 0;
      ledLastUpdateMillis = millis();
    }
    if (mcp_digitalRead(button2Pin) == LOW) { // Button 2 pressed
      lastActivityMillis = millis();
      Serial.println("Button 2 pressed: Anticlockwise");
      clockwise = false;
      ledIndex = 0;
      ledLastUpdateMillis = millis();
    }
  }
  maintainEEPROMErrorIndicator();
}

void flushSequence() {
  unsigned long currentMillis = millis();
  
  // Required time above K in seconds (H + CUT_MODE_HEAT_TIME if in cut mode)
  long totalHeaterTime = H;
  if (cutBag) {
    totalHeaterTime += (long)CUT_MODE_HEAT_TIME;
    Serial.printf("DEBUG: Cut mode - Total time above K: %ld seconds (H=%ld + CUT_MODE_HEAT_TIME=%.1f)\n", 
                 totalHeaterTime, H, CUT_MODE_HEAT_TIME);
  }
  switch (flushStep) {
    case 0: {
      ledLastUpdateMillis = millis();
      maxHeaterWallTimeMs = 0;  // Reset at start of each flush cycle before recomputing once.
      Serial.println("Checking battery voltage");
      int batteryLevel = getBatteryChargeLevel();
      if (batteryLevel < batteryThreshold) {
       String lowBatteryMsg = "LOW BATTERY STOP: level=" + String(batteryLevel) +
                              "% < threshold=" + String(batteryThreshold) + "%";
       Serial.println(lowBatteryMsg);
       SerialBLE_println(lowBatteryMsg);
       stopEverything();
       flushStep = 12;
       ERROR_CODE = 2;
       LEDErrorCode(ERROR_CODE);
       return;
      }
      logMotorFaultDebug("flush case0 before fault check");
      checkAllMotorFaults();
      logMotorFaultDebug("flush case0 after fault check");
      if (ERROR_CODE != 0) {
        return;
      }
      // Test heater current detection at start of flush cycle
      testHeaterCurrent();
      if (ERROR_CODE != 0) {
        return;
      }
      case1FeedStarted = false;  // Reset flag for case 1
      case6CutMotorRun = false;   // Reset flag for case 6 cut motor (run after H, then heat for CUT_MODE_HEAT_TIME)
      timeAboveCutModeTempMillis = 0;
      // Compute timeout once per flush cycle from starting temperature.
      {
        float timeoutCurrentTemp = readTemperature();
        float timeoutTargetTemp = cutBag ? CUT_MODE_TEMP : K;
        float rampSecondsF = (timeoutTargetTemp - timeoutCurrentTemp) / 1.0f;  // 1 C/s ramp assumption
        if (rampSecondsF < 0.0f) {
          rampSecondsF = 0.0f;
        }
        float holdSecondsF = (float)totalHeaterTime;
        maxHeaterWallTimeMs = (unsigned long)((rampSecondsF + holdSecondsF) * 1.2f * 1000.0f);
        Serial.printf("DEBUG: Heater timeout calc (cycle start): current=%.1f C, target=%.1f C, ramp=%.1f s, hold=%.1f s, maxWall=%lu ms\n",
                      timeoutCurrentTemp, timeoutTargetTemp, rampSecondsF, holdSecondsF, maxHeaterWallTimeMs);
      }
      flushStep++;
      SerialBLE_println("Moving to Case1:");
      SerialBLE_println(millis());
      SerialBLE_println("Starting fan, waiting before feed");
      stepStartMillis = currentMillis;
      setFanSpeed(400);  // Start M3 fan immediately
      break;
    }

    case 1:
      currentMillis = millis();
      
      // Start M2 feed motor after preFeedFan delay
      if (!case1FeedStarted && (currentMillis - stepStartMillis >= preFeedFan * 1000)) {
        SerialBLE_println("Pre-feed fan delay complete, starting feed motor");
        motors.setM2Speed(-400);
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
      motors.setM1Speed(400);
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
      heaterTargetTemp = K;  // Target K until cut motor runs (then CUT_MODE_TEMP in cut mode)
      timeAboveSetpointMillis = 0;
      lastHeaterCheckMillis = currentMillis;
      lastHeaterControlLogMillis = 0;  // Force immediate first heater control log for this heating phase
      updateHeaterControl(readTemperature());
      
      // Check if it's time to start M3 reverse (based on fanReverseStartTime percentage of typicalOpeningTime)
      if (!m3ReverseActive && !m3ReverseCompleted && m1CloseStartTime > 0) {
        unsigned long delayMs = (unsigned long)((fanReverseStartTime / 100.0) * typicalOpeningTime * 1000);
        if (currentMillis - m1CloseStartTime >= delayMs) {
          SerialBLE_println("Starting M3 reverse");
          setFanSpeed(-400);  // M3 in reverse at full speed
          m3ReverseStartTime = currentMillis;
          m3ReverseActive = true;
        }
      }
      
      // Check if M3 reverse time has elapsed
      if (m3ReverseActive && (currentMillis - m3ReverseStartTime >= fanReverseTime * 1000)) {
        SerialBLE_println("Stopping M3 reverse - time elapsed");
        setFanSpeed(0);
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
            setFanSpeed(-400);  // M3 in reverse at full speed
            m3ReverseStartTime = currentMillis;
            m3ReverseActive = true;
          }
        }
        
        // Check if M3 reverse time has elapsed
        if (m3ReverseActive && (currentMillis - m3ReverseStartTime >= fanReverseTime * 1000)) {
          SerialBLE_println("Stopping M3 reverse - time elapsed");
          setFanSpeed(0);
          m3ReverseActive = false;
          m3ReverseCompleted = true; // Mark as completed to prevent restart
        }
        
        case5FeedExecuted = false; // Reset flag when entering case 5
        m1CurrentLogWindowStartMillis = currentMillis;
        m1CurrentMaxInWindow = 0.0;
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
          setFanSpeed(-400);  // M3 in reverse at full speed
          m3ReverseStartTime = currentMillis;
          m3ReverseActive = true;
        }
      }
      
      // Check if M3 reverse time has elapsed
      if (m3ReverseActive && (currentMillis - m3ReverseStartTime >= fanReverseTime * 1000)) {
        SerialBLE_println("Stopping M3 reverse - time elapsed (in case 5)");
        setFanSpeed(0);
        m3ReverseActive = false;
        m3ReverseCompleted = true; // Mark as completed to prevent restart
      }
      
      // Primary check: microswitch closed
        float m1Current = readM1Current();
        if (m1Current > m1CurrentMaxInWindow) {
          m1CurrentMaxInWindow = m1Current;
        }
        if (m1CurrentLogWindowStartMillis == 0) {
          m1CurrentLogWindowStartMillis = currentMillis;
        } else if (currentMillis - m1CurrentLogWindowStartMillis >= M1_CURRENT_LOG_INTERVAL_MS) {
          SerialBLE_print("M1 max current (");
          SerialBLE_print(M1_CURRENT_LOG_INTERVAL_MS / 1000);
          SerialBLE_print("s): ");
          SerialBLE_print(m1CurrentMaxInWindow);
          SerialBLE_println(" A");
          m1CurrentLogWindowStartMillis = currentMillis;
          m1CurrentMaxInWindow = 0.0;
        }
      
      if (digitalRead(microswitchClosePin) == LOW) {
        SerialBLE_println("Microswitch closed");
        
        // Only execute feed once per case 5
        if (!case5FeedExecuted) {
        motors.setM2Speed(-400);
          SerialBLE_println("mechanism closed,feed to relieve strain ");
        delay(300); // feed a bit longer
        motors.setM2Speed(0);
          case5FeedExecuted = true; // Mark as executed
        }
        
        // Normal mode: current > 0.5A is acceptable
        if (m1Current > 0.5) {
          SerialBLE_println("Normal mode: Current > 0.5A confirmed");
        motors.setM1Speed(0);
        setFanSpeed(0);  // Stop M3 motor
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

    case 6: {
      // Cut mode: run cut motor after H (time above K), while still heating; then heat for extra CUT_MODE_HEAT_TIME
      if (cutBag && !case6CutMotorRun && (timeAboveSetpointMillis >= (unsigned long)(H * 1000))) {
        SerialBLE_println("Cut bag mode: Running cut motor after H (heater stays on for extra CUT_MODE_HEAT_TIME)");
        Serial.printf("DEBUG: MOTOR_CUT_TIME = %.3f seconds\n", MOTOR_CUT_TIME);
        Serial.printf("DEBUG: delay = %d ms\n", int(MOTOR_CUT_TIME * 1000));
        motors.setM1Speed(400); // Full speed for cutting
        delay(int(MOTOR_CUT_TIME * 1000)); // Run for MOTOR_CUT_TIME seconds
        motors.setM1Speed(0); // Stop cut motor
        SerialBLE_println("Cut motor stopped");
        case6CutMotorRun = true;
        timeAboveCutModeTempMillis = 0;
        lastCutModeTempCheckMillis = millis();
        heaterTargetTemp = CUT_MODE_TEMP;  // Switch heater target for CUT_MODE_HEAT_TIME
        Serial.printf("DEBUG: Heater target raised to CUT_MODE_TEMP = %.1f °C\n", CUT_MODE_TEMP);
      }

      bool timeAboveKReached;
      if (cutBag) {
        timeAboveKReached = (timeAboveSetpointMillis >= (unsigned long)(H * 1000)) && case6CutMotorRun
                            && (timeAboveCutModeTempMillis >= (unsigned long)(CUT_MODE_HEAT_TIME * 1000));
      } else {
        timeAboveKReached = (timeAboveSetpointMillis >= (unsigned long)(totalHeaterTime * 1000));
      }
      bool maxWallTimeExceeded = ((currentMillis - stepStartMillis) >= maxHeaterWallTimeMs);

      if (timeAboveKReached || maxWallTimeExceeded) {
        if (maxWallTimeExceeded && !timeAboveKReached) {
          Serial.println("WARNING: Heater max wall time reached - temperature did not stay above K for required time");
          SerialBLE_println("WARNING: Heater max time - time above K target not reached");
          if (ERROR_CODE == 0) {
            ERROR_CODE = 6;
            LEDErrorCode(ERROR_CODE);
          }
        } else {
          SerialBLE_println("Heater time complete - Begin cooling");
        }

        if (cutBag && case6CutMotorRun) {
          SerialBLE_println("Cut bag mode: Running pre-cooling cut pulse");
          motors.setM1Speed(400); // Recut while bag is hottest, before cooling starts
          delay(int(MOTOR_CUT_TIME * 1000));
          motors.setM1Speed(0);
          SerialBLE_println("Pre-cooling cut pulse complete");
        }

        flushStep++;
        SerialBLE_print("Moving to Case7 from case 6: ");
        SerialBLE_println(millis());

        SerialBLE_println("Heater off");
        heaterOn = false;
        heaterOff();
        heaterTargetTemp = K;  // Restore target for next flush
        stepStartMillis = currentMillis;
        lastCoolingTempLogMillis = 0;  // Reset cooling log timer for immediate first reading in case 7
      }
      break;
    }

    case 7: {
      float coolingTemp = readTemperature();
      if (currentMillis - lastCoolingTempLogMillis >= COOLING_TEMP_LOG_INTERVAL_MS) {
        SerialBLE_print("Cooling temp: ");
        SerialBLE_print(coolingTemp);
        SerialBLE_println(" C");
        lastCoolingTempLogMillis = currentMillis;
      }

      bool coolingThresholdReached = coolingTemp < COOL_OPEN_TEMP_C;
      bool coolingTimeoutReached = (currentMillis - stepStartMillis) >= (MAX_COOL_WAIT_S * 1000UL);

      if (coolingThresholdReached || coolingTimeoutReached) {
        if (coolingThresholdReached) {
          SerialBLE_print("Cooling threshold reached, opening sealer at ");
          SerialBLE_print(coolingTemp);
          SerialBLE_println(" C");
        } else {
          SerialBLE_print("WARNING: Cooling timeout reached at ");
          SerialBLE_print(coolingTemp);
          SerialBLE_println(" C - opening sealer");
        }
        Serial.print(currentMillis);
        SerialBLE_println("  Cooling complete opening sealer");
        mechanismMotorRunning = true;
        motorStartMillis = millis();
        motors.setM1Speed(-400);
        SerialBLE_print("MOTOR START TIME: ");
        Serial.println(motorStartMillis);
        motors.setM2Speed(400);
        flushStep++;
        SerialBLE_print("Moving to Case 8: ");
        SerialBLE_println(millis());
        stepStartMillis = currentMillis;
      }
      break;
    }

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
          motors.setM2Speed(400);  // M2 in reverse for backup
          case10BackupStartTime = currentMillis;
          case10BackupStarted = true;
        }
        
        // Wait for backupTimeAfterReopen before proceeding to fan
        if (case10BackupStarted && (currentMillis - case10BackupStartTime >= backupTimeAfterReopen * 1000)) {
          // Stop backup and start fan (only once)
          if (!case10FanStarted) {
            SerialBLE_println("Backup complete, stopping backup and activating fan");
            motors.setM2Speed(0);  // Stop backup
            setFanSpeed(400);  // M3 at full power
            stepStartMillis = currentMillis;
            case10FanStarted = true;
          }
        }
        
        // Wait for postCoolingFanDuration before starting feed motors (only after backup and fan started)
        if (case10BackupStarted && case10FanStarted && (currentMillis - stepStartMillis >= postCoolingFanDuration * 1000)) {
          SerialBLE_println("Fan delay complete, starting feed motors");
          motors.setM2Speed(-400);
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
        setFanSpeed(0);// Stop M3 motor/fan
        // Start end-hold timer: keep all LEDs on for 3 seconds in case 13.
        stepStartMillis = currentMillis;
        flushStep++;
        SerialBLE_print("Moving to Case13:");
        SerialBLE_println(millis());
      }
      break;

    case 13:
      // Hold full LED bar for 3 seconds before turning everything off.
      if (currentMillis - stepStartMillis < 3000) {
        for (int i = 0; i < totalLeds; i++) {
          mcp_digitalWrite(ledPins[i], HIGH);
        }
      } else {
        Serial.println("All LEDs off");
        for (int i = 0; i < totalLeds; i++) {
          mcp_digitalWrite(ledPins[i], LOW);
        }
        incrementFlushCount();
        isFlushing = false;
        flushStep = 0;
        cutBag = false;  // Reset cutBag for next cycle
        lastActivityMillis = millis();  // Restart inactivity/sleep timer at end of flush
        SerialBLE_println("Moving to Case0 from 13 - end of sequence:");
        Serial.println(millis());
      }
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
  for (int i = 0; i < totalLeds; i++) {
    mcp_digitalWrite(ledPins[i], HIGH);
    delay(duration);
    mcp_digitalWrite(ledPins[i], LOW);
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

// Enable OTA mode - set flags and clear battery display; update characteristic already exists in main service
void enableOTA() {
  if (otaEnabled) {
    Serial.println("OTA already enabled - returning");
    return; // Already enabled
  }
  // Force hardware idle so PREPARE_UPDATE is not blocked (flush/motors/heater off)
  stopEverything();
  // Exit battery display so battery level LEDs don't stay lit; only slow circle will show
  batteryDisplayMode = false;
  batteryMonitoringActive = false;
  // Turn off all LEDs immediately (clear battery level display if it was on)
  for (int i = 0; i < totalLeds; i++) {
    mcp_digitalWrite(ledPins[i], LOW);
  }
  Serial.println("\n=== OTA MODE ENABLED ===");
  Serial.println("OTA window open for 1 minute - awaiting firmware update over USB serial or BLE");
  sendSerialToBLE("=== OTA MODE ENABLED ===");
  sendSerialToBLE("OTA window open for 1 minute");
  // Reset OTA state to ensure clean slate for new update attempt
  resetOTAState();
  updateInProgress = false;
  isFlushing = false;
  otaEnabled = true;
  otaWindowStartTime = millis();
  slowCircleLedIndex = 0;
  slowCircleLastUpdate = millis();
  Serial.println("OTA state reset complete - ready for update");
  sendSerialToBLE("OTA state reset - ready for update");
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
  setOTAState(OTA_IDLE, "ota disabled");
  
  // Turn off all LEDs
  for (int i = 0; i < totalLeds; i++) {
    mcp_digitalWrite(ledPins[i], LOW);
  }
  
  Serial.println("OTA mode disabled");
}

const char* otaStateToString(OTAState state) {
  switch (state) {
    case OTA_IDLE: return "OTA_IDLE";
    case OTA_PREPARING: return "OTA_PREPARING";
    case OTA_RECEIVING: return "OTA_RECEIVING";
    case OTA_VALIDATING: return "OTA_VALIDATING";
    case OTA_FINALIZING: return "OTA_FINALIZING";
    case OTA_ERROR: return "OTA_ERROR";
    case OTA_ROLLBACK: return "OTA_ROLLBACK";
    default: return "OTA_UNKNOWN";
  }
}

bool isValidOTATransition(OTAState from, OTAState to) {
  if (from == to) {
    return true;
  }
  if (to == OTA_ERROR || to == OTA_IDLE) {
    return true;
  }
  switch (from) {
    case OTA_IDLE:
      return to == OTA_PREPARING;
    case OTA_PREPARING:
      return to == OTA_RECEIVING;
    case OTA_RECEIVING:
      return to == OTA_VALIDATING;
    case OTA_VALIDATING:
      return to == OTA_FINALIZING;
    case OTA_ERROR:
      return to == OTA_PREPARING;
    default:
      return false;
  }
}

bool setOTAState(OTAState nextState, const char* reason) {
  if (!isValidOTATransition(otaState, nextState)) {
    Serial.printf("WARN: Illegal OTA transition %s -> %s (%s)\n",
                  otaStateToString(otaState), otaStateToString(nextState), reason);
    return false;
  }
  otaState = nextState;
  otaStateEnteredAt = millis();
  Serial.printf("OTA state: %s (%s)\n", otaStateToString(otaState), reason);
  return true;
}

void publishOTAStatus(const String& status, bool notify) {
  last_update_status = status;
  if (update_characteristic) {
    update_characteristic->setValue(status.c_str());
    if (notify) {
      update_characteristic->notify();
    }
  }
}

void publishOTAErrorStatus(const char* reasonCode) {
  String status = "UPDATE_ERROR:";
  status += reasonCode;
  publishOTAStatus(status);
}

bool isKnownOTACommand(const String& command) {
  return command == "CHECK_VERSION" ||
         command == "PREPARE_UPDATE" ||
         command == "START_UPDATE" ||
         command == "FINALIZE_UPDATE";
}

void handleOTACommand(const String& command) {
  Serial.print("DEBUG: OTA command: '");
  Serial.print(command);
  Serial.println("'");

  if (command == "CHECK_VERSION") {
    String versionInfo = getVersionString();
    version_characteristic->setValue(versionInfo.c_str());
    version_characteristic->notify();
    SerialBLE_println("Version check requested");
    return;
  }

  if (command == "PREPARE_UPDATE") {
    if (otaState == OTA_PREPARING) {
      publishOTAStatus("UPDATE_PREPARED");
      return;
    }
    if (otaState == OTA_RECEIVING || otaState == OTA_VALIDATING || otaState == OTA_FINALIZING) {
      publishOTAStatus("UPDATE_IN_PROGRESS");
      return;
    }
    publishOTAStatus("PREPARING");
    if (prepareForOTA()) {
      publishOTAStatus("UPDATE_PREPARED");
      SerialBLE_println("Device prepared for update");
    } else {
      publishOTAStatus("UPDATE_BLOCKED");
      SerialBLE_println("Update preparation blocked");
    }
    return;
  }

  if (command == "START_UPDATE") {
    if (otaState == OTA_RECEIVING || otaState == OTA_VALIDATING || otaState == OTA_FINALIZING) {
      publishOTAStatus("UPDATE_STARTED");
      return;
    }
    if (otaState != OTA_PREPARING) {
      publishOTAStatus("UPDATE_NOT_PREPARED");
      SerialBLE_println("Update not prepared");
      return;
    }
    if (ota_handle == 0) {
      esp_err_t err = esp_ota_begin(update_partition, OTA_SIZE_UNKNOWN, &ota_handle);
      if (err != ESP_OK) {
        Serial.printf("ERROR: esp_ota_begin failed: %s\n", esp_err_to_name(err));
        setOTAState(OTA_ERROR, "esp_ota_begin failed");
        ota_error_message = "OTA begin failed: " + String(esp_err_to_name(err));
        publishOTAErrorStatus("BEGIN_FAILED");
        return;
      }
    }
    if (!setOTAState(OTA_RECEIVING, "start update command")) {
      publishOTAErrorStatus("INVALID_START_TRANSITION");
      return;
    }
    otaLastChunkMillis = millis();
    publishOTAStatus("UPDATE_STARTED");
    SerialBLE_println("OTA update started");
    notifyUpdateProgress(0);
    return;
  }

  if (command == "FINALIZE_UPDATE") {
    if (last_update_status == "UPDATE_COMPLETE" || otaState == OTA_FINALIZING) {
      publishOTAStatus("UPDATE_COMPLETE");
      return;
    }
    if (otaState != OTA_VALIDATING) {
      publishOTAStatus("UPDATE_NOT_READY");
      SerialBLE_println("Update not ready for finalization");
      return;
    }
    if (!validateFirmware()) {
      SerialBLE_println("Firmware validation failed, rolling back...");
      setOTAState(OTA_ERROR, "firmware validation failed");
      rollback_required = true;
      if (ota_handle != 0) {
        esp_ota_abort(ota_handle);
        ota_handle = 0;
      }
      publishOTAStatus("UPDATE_VALIDATION_FAILED");
      if (rollbackOTAUpdate()) {
        SerialBLE_println("Rollback successful, rebooting...");
        delay(1000);
        esp_restart();
      }
      return;
    }

    if (!setOTAState(OTA_FINALIZING, "finalize command")) {
      publishOTAErrorStatus("INVALID_FINALIZE_TRANSITION");
      return;
    }
    publishOTAStatus("UPDATE_FINALIZING");
    esp_err_t err = esp_ota_end(ota_handle);
    if (err != ESP_OK) {
      Serial.printf("ERROR: esp_ota_end failed: %s\n", esp_err_to_name(err));
      setOTAState(OTA_ERROR, "esp_ota_end failed");
      ota_error_message = "OTA end failed: " + String(esp_err_to_name(err));
      publishOTAErrorStatus("END_FAILED");
      return;
    }
    err = esp_ota_set_boot_partition(update_partition);
    if (err != ESP_OK) {
      Serial.printf("ERROR: esp_ota_set_boot_partition failed: %s\n", esp_err_to_name(err));
      setOTAState(OTA_ERROR, "set boot partition failed");
      ota_error_message = "Set boot partition failed: " + String(esp_err_to_name(err));
      publishOTAErrorStatus("BOOT_PARTITION_FAILED");
      return;
    }
    publishOTAStatus("UPDATE_COMPLETE");
    notifyUpdateProgress(100);
    SerialBLE_println("OTA update completed successfully, rebooting...");
    delay(1000);
    esp_restart();
  }
}

void checkOTATimeouts() {
  unsigned long now = millis();
  if (otaState == OTA_PREPARING && (now - otaStateEnteredAt > OTA_PREPARE_TIMEOUT_MS)) {
    setOTAState(OTA_ERROR, "prepare timeout");
    publishOTAErrorStatus("TIMEOUT_PREPARE");
    resetOTAState();
    publishOTAStatus("UPDATE_TIMEOUT_RECOVERED");
    return;
  }

  if (otaState == OTA_RECEIVING) {
    unsigned long lastActivity = (otaLastChunkMillis > 0) ? otaLastChunkMillis : otaStateEnteredAt;
    if (now - lastActivity > OTA_RECEIVE_INACTIVITY_TIMEOUT_MS) {
      if (ota_handle != 0) {
        esp_ota_abort(ota_handle);
        ota_handle = 0;
      }
      setOTAState(OTA_ERROR, "receive timeout");
      publishOTAErrorStatus("TIMEOUT_RECEIVE");
      resetOTAState();
      publishOTAStatus("UPDATE_TIMEOUT_RECOVERED");
      return;
    }
  }

  if (otaState == OTA_FINALIZING && (now - otaStateEnteredAt > OTA_FINALIZE_TIMEOUT_MS)) {
    if (ota_handle != 0) {
      esp_ota_abort(ota_handle);
      ota_handle = 0;
    }
    setOTAState(OTA_ERROR, "finalize timeout");
    publishOTAErrorStatus("TIMEOUT_FINALIZE");
    resetOTAState();
    publishOTAStatus("UPDATE_TIMEOUT_RECOVERED");
  }
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
  bleIdleStartTime = millis();
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
  // Steinhart-Hart coefficients A,B,C expect R in kΩ: 1/T = A + B*ln(R_kΩ) + C*ln(R_kΩ)^3
  float logR = log(resistance / 1000.0);
  float tempKelvin = 1.0 / (A + B * logR + C * logR * logR * logR);
  return tempKelvin - 273.15;
}

float readMainThermistorResistanceOhms() {
  int analogValue = analogRead(thermistorPin);
  float voltage = analogValue * (3.3 / 4095.0);
  if (voltage <= THERMISTOR_VOLTAGE_GUARD_V) {
    return 0.0;
  }
  if (voltage >= (3.3 - THERMISTOR_VOLTAGE_GUARD_V)) {
    return 1000000000.0;
  }
  return (voltage * knownResistor) / (3.3 - voltage);
}

bool isHardwareLikelyDisconnectedForUserAction() {
  return readMainThermistorResistanceOhms() > HARDWARE_DISCONNECT_RESISTANCE_OHMS;
}

void playHardwareNotConnectedAlert() {
  const int cycles = 3;
  const int onMs = 80;
  const int offMs = 80;

  for (int cycle = 0; cycle < cycles; cycle++) {
    for (int i = 0; i < totalLeds; i++) {
      mcp_digitalWrite(ledPins[i], HIGH);
    }
    digitalWrite(buzzerPin, HIGH);
    delay(onMs);

    for (int i = 0; i < totalLeds; i++) {
      mcp_digitalWrite(ledPins[i], LOW);
    }
    digitalWrite(buzzerPin, LOW);
    delay(offMs);
  }

  // Ensure alert outputs are off after the pattern completes.
  digitalWrite(buzzerPin, LOW);
  for (int i = 0; i < totalLeds; i++) {
    mcp_digitalWrite(ledPins[i], LOW);
  }
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
  
  // Debug output: one BLE string to avoid truncation/interleaving
  Serial.printf("Battery Debug: ADC=%d, VMON=%.3fV, Battery=%.2fV\n", analogValue, voltage, batteryVoltage);
  if (serial_streaming_enabled) {
    sendSerialToBLE("Battery Debug analog: " + String(analogValue) + ", voltage: " + String(voltage, 2) + "V, battery: " + String(batteryVoltage, 2) + "V\n");
  }
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

  // Include motor fault health in battery temp debug so it can be monitored
  // without running a full flush sequence.
  String faultSnapshot = buildMotorFaultStatusSnapshot();
  
  // Debug output to Bluetooth: one string to avoid truncation/interleaving
  if (serial_streaming_enabled) {
    sendSerialToBLE("=== BATTERY TEMP DEBUG ===\nADC: " + String(analogValue) + ", Voltage: " + String(voltage, 2) + "V, Resistance: " + String(thermistorResistance, 2) + " Ohm, Temp: " + String(temperature, 2) + " C, " + faultSnapshot + "\n");
  }
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

  // Main thermistor: resistance and temperature
  int mainAnalog = analogRead(thermistorPin);
  float mainVoltage = mainAnalog * (3.3 / 4095.0);
  float thermistorResistance = (mainVoltage * knownResistor) / (3.3 - mainVoltage);
  float mainTemp = readTemperature();

  // Battery thermistor: resistance and temperature
  int batAnalog = analogRead(batteryTempPin);
  float batVoltage = batAnalog * (3.3 / 4095.0);
  float batteryResistance = 10000.0 * (3.3 - batVoltage) / batVoltage;
  float batteryTemp = readBatteryTemperature();

  // Single BLE string to avoid truncation/interleaving
  String bleLine = "Battery Voltage: " + String(batteryVoltage, 2) + "V, Charge Level: " + String(chargeLevel) + "%, Thermistor R: " + String(thermistorResistance, 2) + " Ohm, Thermistor T: " + String(mainTemp, 2) + " C, Battery R: " + String(batteryResistance, 2) + " Ohm, Battery T: " + String(batteryTemp, 2) + " C\n";
  Serial.print(bleLine);
  if (serial_streaming_enabled) {
    sendSerialToBLE(bleLine);
  }

  Serial.printf("DEBUG: Charge level = %d%%, Battery voltage = %.2fV, Thermistor R = %.0f Ohm, Thermistor T = %.1f C, Battery R = %.0f Ohm, Battery T = %.1f C\n",
                chargeLevel, batteryVoltage, thermistorResistance, mainTemp, batteryResistance, batteryTemp);
  
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
  setFanSpeed(0);  // Stop M3 motor
  heaterOff();
  flushStep = 12;
  isFlushing = false;
}

float clampLedProgress(float value) {
  if (value < 0.0f) return 0.0f;
  if (value > 1.0f) return 1.0f;
  return value;
}

int clampLedIndex(int index) {
  if (index < 0) return 0;
  if (index >= totalLeds) return totalLeds - 1;
  return index;
}

void applyLedFillToIndex(int targetIndex, bool forceImmediate = false) {
  targetIndex = clampLedIndex(targetIndex);
  if (targetIndex <= ledIndex) {
    return;
  }

  for (int i = ledIndex + 1; i <= targetIndex; i++) {
    mcp_digitalWrite(ledPins[i], HIGH);
  }
  ledIndex = targetIndex;
  ledLastUpdateMillis = millis();
}

void allocateLedStepsFromDurations(const unsigned long durationsMs[LED_PHASE_COUNT], int totalSteps) {
  bool isActive[LED_PHASE_COUNT] = {false};
  double fractions[LED_PHASE_COUNT] = {0.0};
  int activeCount = 0;
  unsigned long totalDurationMs = 0;

  for (int i = 0; i < LED_PHASE_COUNT; i++) {
    ledSectionSteps[i] = 0;
    if (durationsMs[i] > 0) {
      isActive[i] = true;
      activeCount++;
      totalDurationMs += durationsMs[i];
    }
  }

  if (totalSteps <= 0 || activeCount == 0) {
    return;
  }

  if (totalSteps <= activeCount) {
    for (int i = 0; i < LED_PHASE_COUNT && totalSteps > 0; i++) {
      if (!isActive[i]) continue;
      ledSectionSteps[i] = 1;
      totalSteps--;
    }
    return;
  }

  int remainingSteps = totalSteps;
  for (int i = 0; i < LED_PHASE_COUNT; i++) {
    if (isActive[i]) {
      ledSectionSteps[i] = 1;  // Ensure each active section has at least one LED transition.
      remainingSteps--;
    }
  }

  int assignedExtra = 0;
  for (int i = 0; i < LED_PHASE_COUNT; i++) {
    if (!isActive[i]) continue;
    double rawShare = 0.0;
    if (totalDurationMs > 0) {
      rawShare = ((double)durationsMs[i] / (double)totalDurationMs) * (double)remainingSteps;
    }
    int whole = (int)rawShare;
    ledSectionSteps[i] += whole;
    assignedExtra += whole;
    fractions[i] = rawShare - (double)whole;
  }

  int leftovers = remainingSteps - assignedExtra;
  while (leftovers > 0) {
    int bestIndex = -1;
    double bestFraction = -1.0;
    for (int i = 0; i < LED_PHASE_COUNT; i++) {
      if (!isActive[i]) continue;
      if (fractions[i] > bestFraction) {
        bestFraction = fractions[i];
        bestIndex = i;
      }
    }
    if (bestIndex < 0) {
      break;
    }
    ledSectionSteps[bestIndex]++;
    fractions[bestIndex] = -1.0;
    leftovers--;
  }
}

void rebuildLedSectionBoundaries() {
  int runningIndex = 0;
  for (int i = 0; i < LED_PHASE_COUNT; i++) {
    ledSectionStartIndex[i] = runningIndex;
    runningIndex += ledSectionSteps[i];
    ledSectionEndIndex[i] = runningIndex;
  }
  ledSectionEndIndex[LED_PHASE_FINAL] = totalLeds - 1;
}

int computeSectionTargetLedIndex(int section, float progress) {
  progress = clampLedProgress(progress);
  int steps = ledSectionSteps[section];
  int start = ledSectionStartIndex[section];
  int end = ledSectionEndIndex[section];
  if (steps <= 0) {
    return start;
  }

  int offset = (int)(progress * (float)steps);
  int target = start + offset;
  if (progress >= 1.0f) {
    target = end;
  }
  if (target > end) {
    target = end;
  }
  return clampLedIndex(target);
}

void determineLedSectionAndProgress(unsigned long currentMillis, float currentTemp, int &section, float &progress) {
  // Final completion guard.
  if (flushStep >= 13) {
    section = LED_PHASE_FINAL;
    progress = 1.0f;
    return;
  }

  // Phase A: initial fixed segment (pre-fan/feed/close/setup)
  if (flushStep <= 5) {
    section = LED_PHASE_INITIAL;
    if (flushStep >= 6) {
      progress = 1.0f;
    } else {
      unsigned long elapsedMs = currentMillis - flushStartMillis;
      unsigned long estimateMs = ledSectionEstimateMs[LED_PHASE_INITIAL];
      if (estimateMs == 0) {
        progress = 1.0f;
      } else {
        progress = clampLedProgress((float)elapsedMs / (float)estimateMs);
      }
    }
    return;
  }

  // Phase B/C: heating while in case 6.
  if (flushStep == 6) {
    bool reachedPrimaryTarget = (timeAboveSetpointMillis > 0) || (currentTemp >= (K - 0.25f));
    if (!reachedPrimaryTarget) {
      section = LED_PHASE_HEAT_UP;
      float denominator = K - ledHeatStartTempC;
      if (denominator > 0.5f) {
        progress = clampLedProgress((currentTemp - ledHeatStartTempC) / denominator);
      } else {
        unsigned long elapsedMs = currentMillis - ledHeatStartMillis;
        unsigned long estimateMs = ledSectionEstimateMs[LED_PHASE_HEAT_UP];
        progress = (estimateMs == 0) ? 1.0f : clampLedProgress((float)elapsedMs / (float)estimateMs);
      }
    } else {
      section = LED_PHASE_HEAT_HOLD;
      unsigned long requiredMsAboveK = (unsigned long)(H * 1000UL);
      unsigned long requiredMsCut = cutBag ? (unsigned long)(CUT_MODE_HEAT_TIME * 1000.0f) : 0UL;
      unsigned long requiredTotalMs = requiredMsAboveK + requiredMsCut;
      if (requiredTotalMs == 0) {
        progress = 1.0f;
      } else {
        unsigned long doneAboveK = timeAboveSetpointMillis;
        if (doneAboveK > requiredMsAboveK) {
          doneAboveK = requiredMsAboveK;
        }
        unsigned long doneCut = 0;
        if (cutBag && case6CutMotorRun) {
          doneCut = timeAboveCutModeTempMillis;
          if (doneCut > requiredMsCut) {
            doneCut = requiredMsCut;
          }
        }
        unsigned long doneTotalMs = doneAboveK + doneCut;
        progress = clampLedProgress((float)doneTotalMs / (float)requiredTotalMs);
      }
    }
    return;
  }

  // Phase D: cooling with real temperature progress.
  if (flushStep == 7) {
    section = LED_PHASE_COOLING;
    if (currentTemp <= COOL_OPEN_TEMP_C) {
      progress = 1.0f;
      return;
    }
    float denominator = ledCoolStartRefTempC - COOL_OPEN_TEMP_C;
    if (denominator > 0.5f) {
      progress = clampLedProgress((ledCoolStartRefTempC - currentTemp) / denominator);
    } else {
      unsigned long elapsedMs = currentMillis - ledCoolingStartMillis;
      unsigned long estimateMs = MAX_COOL_WAIT_S * 1000UL;
      progress = (estimateMs == 0) ? 1.0f : clampLedProgress((float)elapsedMs / (float)estimateMs);
    }

    bool coolingTimeoutReached = (currentMillis - stepStartMillis) >= (MAX_COOL_WAIT_S * 1000UL);
    if (coolingTimeoutReached) {
      progress = 1.0f;
    }
    return;
  }

  // Phase E: final fixed tail.
  section = LED_PHASE_FINAL;
  if (flushStep >= 8 && flushStep <= 12) {
    unsigned long elapsedMs = currentMillis - ledFinalStartMillis;
    unsigned long estimateMs = ledSectionEstimateMs[LED_PHASE_FINAL];
    progress = (estimateMs == 0) ? 1.0f : clampLedProgress((float)elapsedMs / (float)estimateMs);
  } else {
    progress = 1.0f;
  }
}

void calculateSequenceTiming() {
  // Build LED section estimates once at flush start.
  float currentTemp = readTemperature();
  long totalHeaterTime = H + (cutBag ? (long)CUT_MODE_HEAT_TIME : 0L);

  float heatRampToKSeconds = (K - currentTemp) / 1.0f;  // 1 C/s approximation
  if (heatRampToKSeconds < 0.0f) {
    heatRampToKSeconds = 0.0f;
  }

  ledSectionEstimateMs[LED_PHASE_INITIAL] =
      (unsigned long)((preFeedFan + F + typicalOpeningTime) * 1000.0f) + 300UL;
  ledSectionEstimateMs[LED_PHASE_HEAT_UP] = (unsigned long)(heatRampToKSeconds * 1000.0f);
  ledSectionEstimateMs[LED_PHASE_HEAT_HOLD] = (unsigned long)(totalHeaterTime * 1000UL);
  ledSectionEstimateMs[LED_PHASE_COOLING] = (unsigned long)(T * 1000UL);
  ledSectionEstimateMs[LED_PHASE_FINAL] =
      (unsigned long)((backupTime + maxOpeningTime + backupTimeAfterReopen +
                       postCoolingFanDuration + continueFeeder + fanDuration) * 1000.0f);

  // Kept for serial debug continuity.
  totalSequenceTime = (ledSectionEstimateMs[LED_PHASE_INITIAL] +
                       ledSectionEstimateMs[LED_PHASE_HEAT_UP] +
                       ledSectionEstimateMs[LED_PHASE_HEAT_HOLD] +
                       ledSectionEstimateMs[LED_PHASE_COOLING] +
                       ledSectionEstimateMs[LED_PHASE_FINAL]) / 1000UL;
  ledUpdateInterval = LED_FLUSH_VISUAL_INTERVAL_MS;

  int totalLedTransitions = totalLeds - 1;
  allocateLedStepsFromDurations(ledSectionEstimateMs, totalLedTransitions);
  rebuildLedSectionBoundaries();

  // Reset per-cycle LED tracking baselines.
  ledLastSectionSeen = -1;
  ledLastFlushStepSeen = -1;
  ledHeatStartMillis = 0;
  ledCoolingStartMillis = 0;
  ledFinalStartMillis = 0;
  ledHeatStartTempC = currentTemp;
  ledHeatTargetTempC = K;
  ledCoolStartTempC = currentTemp;
  ledCoolStartRefTempC = currentTemp;

  Serial.printf("LED section estimates ms: initial=%lu heatUp=%lu hold=%lu cool=%lu final=%lu\n",
                ledSectionEstimateMs[LED_PHASE_INITIAL],
                ledSectionEstimateMs[LED_PHASE_HEAT_UP],
                ledSectionEstimateMs[LED_PHASE_HEAT_HOLD],
                ledSectionEstimateMs[LED_PHASE_COOLING],
                ledSectionEstimateMs[LED_PHASE_FINAL]);
  Serial.printf("LED section steps: initial=%d heatUp=%d hold=%d cool=%d final=%d (total=%d)\n",
                ledSectionSteps[LED_PHASE_INITIAL], ledSectionSteps[LED_PHASE_HEAT_UP],
                ledSectionSteps[LED_PHASE_HEAT_HOLD], ledSectionSteps[LED_PHASE_COOLING],
                ledSectionSteps[LED_PHASE_FINAL], totalLedTransitions);
}

void updateLEDs() {
  unsigned long currentMillis = millis();
  float currentTemp = readTemperature();

  // Capture baseline data when key flush transitions are first entered.
  if (flushStep != ledLastFlushStepSeen) {
    if (flushStep >= 3 && ledLastFlushStepSeen < 3) {
      ledHeatStartMillis = currentMillis;
      ledHeatStartTempC = currentTemp;
      ledHeatTargetTempC = K;
    }
    if (flushStep >= 7 && ledLastFlushStepSeen < 7) {
      ledCoolingStartMillis = currentMillis;
      ledCoolStartTempC = currentTemp;
      float coolTargetStart = cutBag ? CUT_MODE_TEMP : K;
      ledCoolStartRefTempC = (ledCoolStartTempC > coolTargetStart) ? ledCoolStartTempC : coolTargetStart;
    }
    if (flushStep >= 8 && ledLastFlushStepSeen < 8) {
      ledFinalStartMillis = currentMillis;
    }
    ledLastFlushStepSeen = flushStep;
  }

  int section = LED_PHASE_INITIAL;
  float progress = 0.0f;
  determineLedSectionAndProgress(currentMillis, currentTemp, section, progress);

  bool forceImmediate = false;
  if (section != ledLastSectionSeen) {
    if (ledLastSectionSeen >= 0 && section > ledLastSectionSeen) {
      // Snap to section boundary so late progress from previous section doesn't carry forward.
      int sectionStart = ledSectionStartIndex[section];
      if (ledIndex < sectionStart) {
        applyLedFillToIndex(sectionStart, true);
      }
    }
    ledLastSectionSeen = section;
    forceImmediate = true;
  }

  if (!forceImmediate && currentMillis - ledLastUpdateMillis < LED_FLUSH_VISUAL_INTERVAL_MS) {
    return;
  }

  int targetIndex = computeSectionTargetLedIndex(section, progress);

  // Hard completion guarantee: final LED is on at true cycle end.
  if (flushStep >= 13) {
    targetIndex = totalLeds - 1;
    forceImmediate = true;
  }

  applyLedFillToIndex(targetIndex, forceImmediate);
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
    motors.setM1Speed(400); // Start closing motor
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
    motors.setM1Speed(-400); // Start opening
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
  if (errorCode == EEPROM_INVALID_ERROR_CODE) {
    // EEPROM warning mode: steady indicator while allowing device operation.
    mcp_digitalWrite(ledPins[EEPROM_INVALID_ERROR_CODE], HIGH);
    return;
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

void updateHeaterControl(float currentTemp) {
  unsigned long now = millis();
  float lowerTol = heaterLowerToleranceC;
  float upperTol = heaterUpperToleranceC;

  if ((lowerTol != lowerTol) || lowerTol < 0.0f) {
    lowerTol = 0.0f;
  }
  if (upperTol != upperTol) {
    upperTol = 2.0f;
  }
  if (upperTol <= -lowerTol) {
    upperTol = -lowerTol + 0.1f;
  }

  float onThreshold = heaterTargetTemp;
  float offThreshold = heaterTargetTemp + upperTol;
  bool nextHeaterOutput = heaterOutputOn;

  if (currentTemp <= onThreshold) {
    nextHeaterOutput = true;
  } else if (currentTemp >= offThreshold) {
    nextHeaterOutput = false;
  }

  if (nextHeaterOutput != heaterOutputOn) {
    heaterOutputOn = nextHeaterOutput;
    digitalWrite(heaterPin, heaterOutputOn ? HIGH : LOW);
  }

  if (lastHeaterControlLogMillis == 0 || (now - lastHeaterControlLogMillis >= HEATER_CONTROL_LOG_INTERVAL_MS)) {
    SerialBLE_print("Heater temp: ");
    SerialBLE_print(currentTemp);
    SerialBLE_print(" C, target: ");
    SerialBLE_print(heaterTargetTemp);
    SerialBLE_print(" C, state: ");
    SerialBLE_println(heaterOutputOn ? "ON" : "OFF");
    Serial.printf("Heater Debug: GPIO%d state=%s, target=%.1f, temp=%.1f, on<=%.1f, off>=%.1f\n",
                  heaterPin, heaterOutputOn ? "ON" : "OFF", heaterTargetTemp, currentTemp, onThreshold, offThreshold);
    lastHeaterControlLogMillis = now;
  }
}

void heaterOff() {
  digitalWrite(heaterPin, LOW);
  heaterOutputOn = false;
  Serial.printf("Heater OFF: GPIO%d state=OFF\n", heaterPin);
  heaterOn = false;
}

// Motor 3 control functions (PWM fan)
void setFanSpeed(int speed) {
  if (speed > 0) {
    digitalWrite(sealerFanPwr, HIGH);
    digitalWrite(sealerFanReverse, LOW);   // Forward
    ledcWrite(sealerFanPWM, 255);          // 100% duty
  } else if (speed < 0) {
    digitalWrite(sealerFanPwr, HIGH);
    digitalWrite(sealerFanReverse, HIGH);  // Reverse
    ledcWrite(sealerFanPWM, 255);          // 100% duty
  } else {
    digitalWrite(sealerFanPwr, LOW);
    ledcWrite(sealerFanPWM, 0);
    digitalWrite(sealerFanReverse, LOW);   // Forward
  }
}

bool getM3Fault() {
  return false;  // PWM fan has no fault pin
}

// Current monitoring function
float readM1Current() {
  int analogValue = analogRead(m1CurrentPin);

  // Convert ADC reading to voltage (0-3.3V)
  float voltage = analogValue * (3.3 / 4095.0);

  // Based on INA169 circuit: Current = (Voltage - 0.5) / (0.1 * Rshunt)
  // Rshunt = 200mΩ, so Current = (Voltage - 0.5) / 0.02
  // Assuming 0.5V offset and 0.1V/A sensitivity
  float current = (voltage) / 2.0; //(no offset observed, 80 A at 0.02)
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
void logMotorFaultDebug(const char* context) {
  int m1NfaultRaw = digitalRead(M1NFAULT_PIN);
  int m2NfaultRaw = digitalRead(M2NFAULT_PIN);
  bool m12Fault = motors.getFault();

  String message = "Motor Fault Debug [";
  message += context;
  message += "]: getFault=";
  message += (m12Fault ? "1" : "0");
  message += ", M1NFAULT=";
  message += String(m1NfaultRaw);
  message += ", M2NFAULT=";
  message += String(m2NfaultRaw);

  SerialBLE_println(message);
}

String buildMotorFaultStatusSnapshot() {
  bool m12FaultNow = motors.getFault();
  int m1NfaultRaw = digitalRead(M1NFAULT_PIN);
  int m2NfaultRaw = digitalRead(M2NFAULT_PIN);

  return "M12FaultNow:" + String(m12FaultNow ? 1 : 0) +
         ", M1NFAULT:" + String(m1NfaultRaw) +
         ", M2NFAULT:" + String(m2NfaultRaw) +
         ", IgnoreM12:" + String(ignoreM12Faults ? 1 : 0) +
         ", IgnoredM12Ever:" + String(hasIgnoredM12Fault ? 1 : 0) +
         ", IgnoredM12Count:" + String(ignoredM12FaultCount) +
         ", LastIgnoredMs:" + String(lastIgnoredM12FaultMillis) +
         ", M12RecovOk:" + String(m12RecoverySuccessCount) +
         ", M12RecovFail:" + String(m12RecoveryFailureCount);
}

void checkAllMotorFaults() {
  // Add a small delay to ensure motors are properly initialized
  static unsigned long lastCheck = 0;
  if (millis() - lastCheck < 100) {  // Only check every 100ms
    return;
  }
  lastCheck = millis();
  
  if (motors.getFault() && ERROR_CODE == 0) {
    unsigned long now = millis();
    hasIgnoredM12Fault = true;
    ignoredM12FaultCount++;
    lastIgnoredM12FaultMillis = now;

    bool shouldLogNow = (lastIgnoredM12FaultLogMillis == 0) ||
                        (now - lastIgnoredM12FaultLogMillis >= M12_FAULT_LOG_THROTTLE_MS);
    bool recoveryCooldownElapsed = (lastM12RecoveryCycleMillis == 0) ||
                                   (now - lastM12RecoveryCycleMillis >= M12_FAULT_RECOVERY_COOLDOWN_MS);
    if (!recoveryCooldownElapsed) {
      suppressedM12RecoveryCycles++;
      if (shouldLogNow) {
        String cooldownMsg = "M1/M2 fault present, recovery cooldown active";
        cooldownMsg += " count=" + String(ignoredM12FaultCount);
        cooldownMsg += ", suppressedFaultLogs=" + String(suppressedIgnoredM12FaultLogs);
        cooldownMsg += ", suppressedRecoveryCycles=" + String(suppressedM12RecoveryCycles);
        SerialBLE_println(cooldownMsg);
        lastIgnoredM12FaultLogMillis = now;
        suppressedIgnoredM12FaultLogs = 0;
        suppressedM12RecoveryCycles = 0;
      } else {
        suppressedIgnoredM12FaultLogs++;
      }
      return;
    }

    lastM12RecoveryCycleMillis = now;
    if (shouldLogNow) {
      logMotorFaultDebug("checkAllMotorFaults tripped");
      Serial.println("DEBUG: Motor 1 or 2 fault detected - checking motor state");
      String startMsg = "M1/M2 fault detected, starting auto-recovery";
      startMsg += " count=" + String(ignoredM12FaultCount);
      if (suppressedIgnoredM12FaultLogs > 0 || suppressedM12RecoveryCycles > 0) {
        startMsg += ", suppressedFaultLogs=" + String(suppressedIgnoredM12FaultLogs);
        startMsg += ", suppressedRecoveryCycles=" + String(suppressedM12RecoveryCycles);
      }
      SerialBLE_println(startMsg);
      suppressedIgnoredM12FaultLogs = 0;
      suppressedM12RecoveryCycles = 0;
    } else {
      suppressedIgnoredM12FaultLogs++;
    }

    // Keep mechanism safe while resetting the M1/M2 driver.
    motors.setM1Speed(0);
    motors.setM2Speed(0);

    bool faultRecovered = false;
    for (uint8_t attempt = 1; attempt <= M12_FAULT_RECOVERY_MAX_ATTEMPTS; ++attempt) {
      if (shouldLogNow) {
        String attemptMsg = "M1/M2 auto-recovery attempt " + String(attempt) + "/" + String(M12_FAULT_RECOVERY_MAX_ATTEMPTS);
        SerialBLE_println(attemptMsg);
      }
      motors.disableDrivers();
      delay(M12_FAULT_RECOVERY_DISABLE_MS);
      motors.enableDrivers();
      delay(M12_FAULT_RECOVERY_SETTLE_MS);

      if (!motors.getFault()) {
        faultRecovered = true;
        if (shouldLogNow) {
          String recoveredMsg = "M1/M2 fault recovered on attempt " + String(attempt);
          recoveredMsg += ", totalRecovered=" + String(m12RecoverySuccessCount + 1);
          SerialBLE_println(recoveredMsg);
          lastIgnoredM12FaultLogMillis = millis();
        }
        break;
      }
    }

    if (faultRecovered) {
      m12RecoverySuccessCount++;
      return;
    }

    m12RecoveryFailureCount++;
    if (ignoreM12Faults && !latchM12FaultAfterRecoveryFailure) {
      if (shouldLogNow) {
        String failedMsg = "M1/M2 fault persists after recovery attempts; continuing log-only";
        failedMsg += " failures=" + String(m12RecoveryFailureCount);
        SerialBLE_println(failedMsg);
        lastIgnoredM12FaultLogMillis = millis();
      } else {
        suppressedIgnoredM12FaultLogs++;
      }
      return;
    }

    SerialBLE_println("Motor 1 or 2 Fault Detected! Auto-recovery failed, latching ERROR_CODE=4");
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
  if (bleEnabled && is_device_connected && serial_characteristic) {
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