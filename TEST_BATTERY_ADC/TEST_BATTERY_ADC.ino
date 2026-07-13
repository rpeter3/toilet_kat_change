// Lightweight battery VMON poll - no heater, no BLE, no load assess.
// Compares naive linear ADC math vs factory eFuse calibration + quadratic fit.
// Averages multiple samples to reduce ESP32 ADC noise.
// Detects charging via upward VMON_CAL mV drift over a lookback window.

#include <stdlib.h>

const int batteryVoltagePin = 10;  // GPIO10 (VMON)
const unsigned long POLL_INTERVAL_MS = 5000UL;
const float ADC_REF_V = 3.3f;
const float ADC_MAX = 4095.0f;
const float VMON_SCALE = 7.317f;  // legacy linear divider scale (for comparison)

// Quadratic fit: V_batt from eFuse-calibrated VMON mV (analogReadMilliVolts)
constexpr float kBattMvCalA = 0.000009091f;
constexpr float kBattMvCalB = -0.019148f;
constexpr float kBattMvCalC = 17.397361f;

// ESP32-S3 ADC: single shots are noisy; 32 samples with trim is a good battery-monitor tradeoff
const int ADC_AVG_SAMPLES = 32;
const int ADC_AVG_TRIM = 2;  // drop this many min and max before averaging

// Charging detection: lookback up to inactivity sleep window (5 min).
// High-SOC charge is slow (~1–2.5 mV/min) with ~1 mV sample noise, so use
// smoothed early/late means + slope instead of noisy single endpoints.
const unsigned long INACTIVITY_SLEEP_MS = 5UL * 60UL * 1000UL;
const unsigned long CHARGE_SAMPLE_INTERVAL_MS = POLL_INTERVAL_MS;
const int CHARGE_HISTORY_LEN = (int)(INACTIVITY_SLEEP_MS / CHARGE_SAMPLE_INTERVAL_MS);  // 60
const unsigned long CHARGE_MIN_SPAN_MS = 45000UL;
const int CHARGE_SMOOTH_SAMPLES = 4;          // mean of oldest/newest N samples
const float CHARGE_RISE_MV = 0.5f;            // smoothed rise to assert charging
const float CHARGE_CLEAR_RISE_MV = 0.3f;      // hysteresis clear
const float CHARGE_SLOPE_MV_PER_MIN = 0.1f;   // LS slope to assert charging
const float CHARGE_CLEAR_SLOPE_MV_PER_MIN = 0.05f;
const float CHARGE_DROP_RESET_MV = 5.0f;

struct ChargeSample {
  float mv;
  unsigned long ms;
};

static ChargeSample chargeHistory[CHARGE_HISTORY_LEN];
static int chargeHistoryCount = 0;
static int chargeHistoryHead = 0;  // next write index
static bool hasLastChargeMv = false;
static float lastChargeMv = 0.0f;
bool isCharging = false;
static float lastSmoothedRiseMv = 0.0f;
static float lastSlopeMvPerMin = 0.0f;

int batteryChargeLevelFromVoltage(float batteryVoltage) {
  if (batteryVoltage >= 12.6f) {
    return 100;
  }
  if (batteryVoltage <= 11.0f) {
    return 0;
  }
  return (int)((batteryVoltage - 11.0f) / 1.6f * 100.0f);
}

static int cmpUint32(const void* a, const void* b) {
  uint32_t va = *(const uint32_t*)a;
  uint32_t vb = *(const uint32_t*)b;
  return (va > vb) - (va < vb);
}

// Trimmed mean of N oneshot reads (discards spikes, then averages).
static void readAdcAveraged(int pin, float* outAdc, float* outMvCal) {
  uint32_t adcSamples[ADC_AVG_SAMPLES];
  uint32_t mvSamples[ADC_AVG_SAMPLES];

  for (int i = 0; i < ADC_AVG_SAMPLES; i++) {
    adcSamples[i] = (uint32_t)analogRead(pin);
    mvSamples[i] = analogReadMilliVolts(pin);
  }

  qsort(adcSamples, ADC_AVG_SAMPLES, sizeof(uint32_t), cmpUint32);
  qsort(mvSamples, ADC_AVG_SAMPLES, sizeof(uint32_t), cmpUint32);

  const int start = ADC_AVG_TRIM;
  const int end = ADC_AVG_SAMPLES - ADC_AVG_TRIM;
  const int count = end - start;

  uint32_t adcSum = 0;
  uint32_t mvSum = 0;
  for (int i = start; i < end; i++) {
    adcSum += adcSamples[i];
    mvSum += mvSamples[i];
  }

  *outAdc = (float)adcSum / (float)count;
  *outMvCal = (float)mvSum / (float)count;
}

static float batteryVoltageFromMvCal(float mvCal) {
  return (kBattMvCalA * mvCal * mvCal) + (kBattMvCalB * mvCal) + kBattMvCalC;
}

static void clearChargeHistory() {
  chargeHistoryCount = 0;
  chargeHistoryHead = 0;
  hasLastChargeMv = false;
  isCharging = false;
  lastSmoothedRiseMv = 0.0f;
  lastSlopeMvPerMin = 0.0f;
}

static void pushChargeSample(float mv, unsigned long ms) {
  chargeHistory[chargeHistoryHead].mv = mv;
  chargeHistory[chargeHistoryHead].ms = ms;
  chargeHistoryHead = (chargeHistoryHead + 1) % CHARGE_HISTORY_LEN;
  if (chargeHistoryCount < CHARGE_HISTORY_LEN) {
    chargeHistoryCount++;
  }
}

static int chargeHistoryIndex(int ageFromOldest) {
  int oldestIdx = (chargeHistoryHead - chargeHistoryCount + CHARGE_HISTORY_LEN) % CHARGE_HISTORY_LEN;
  return (oldestIdx + ageFromOldest) % CHARGE_HISTORY_LEN;
}

static ChargeSample oldestChargeSample() {
  return chargeHistory[chargeHistoryIndex(0)];
}

static ChargeSample newestChargeSample() {
  return chargeHistory[chargeHistoryIndex(chargeHistoryCount - 1)];
}

// Mean of the oldest or newest N samples (N capped by history size / 2).
static float meanChargeEdge(bool newestEdge, int n) {
  if (chargeHistoryCount <= 0 || n <= 0) {
    return 0.0f;
  }
  if (n > chargeHistoryCount) {
    n = chargeHistoryCount;
  }
  if (n > chargeHistoryCount / 2) {
    n = chargeHistoryCount / 2;
    if (n < 1) {
      n = 1;
    }
  }

  float sum = 0.0f;
  if (newestEdge) {
    int start = chargeHistoryCount - n;
    for (int i = 0; i < n; i++) {
      sum += chargeHistory[chargeHistoryIndex(start + i)].mv;
    }
  } else {
    for (int i = 0; i < n; i++) {
      sum += chargeHistory[chargeHistoryIndex(i)].mv;
    }
  }
  return sum / (float)n;
}

// Least-squares slope of VMON_CAL mV vs time, in mV/minute.
static float chargeSlopeMvPerMin() {
  if (chargeHistoryCount < 2) {
    return 0.0f;
  }

  ChargeSample oldest = oldestChargeSample();
  double sumT = 0.0;
  double sumV = 0.0;
  double sumTT = 0.0;
  double sumTV = 0.0;
  const int n = chargeHistoryCount;

  for (int i = 0; i < n; i++) {
    ChargeSample s = chargeHistory[chargeHistoryIndex(i)];
    double tMin = (double)(s.ms - oldest.ms) / 60000.0;
    double v = (double)s.mv;
    sumT += tMin;
    sumV += v;
    sumTT += tMin * tMin;
    sumTV += tMin * v;
  }

  double denom = (double)n * sumTT - sumT * sumT;
  if (denom <= 1e-12) {
    return 0.0f;
  }
  return (float)(((double)n * sumTV - sumT * sumV) / denom);
}

// Update isCharging from gradual VMON_CAL drift (noise-tolerant).
static void updateChargingDetection(float mvCal) {
  unsigned long now = millis();

  if (hasLastChargeMv && (lastChargeMv - mvCal) >= CHARGE_DROP_RESET_MV) {
    clearChargeHistory();
  }

  pushChargeSample(mvCal, now);
  lastChargeMv = mvCal;
  hasLastChargeMv = true;
  lastSmoothedRiseMv = 0.0f;
  lastSlopeMvPerMin = 0.0f;

  if (chargeHistoryCount < (CHARGE_SMOOTH_SAMPLES * 2)) {
    return;
  }

  ChargeSample oldest = oldestChargeSample();
  ChargeSample newest = newestChargeSample();
  unsigned long spanMs = newest.ms - oldest.ms;
  if (spanMs < CHARGE_MIN_SPAN_MS) {
    return;
  }

  lastSmoothedRiseMv = meanChargeEdge(true, CHARGE_SMOOTH_SAMPLES) -
                       meanChargeEdge(false, CHARGE_SMOOTH_SAMPLES);
  lastSlopeMvPerMin = chargeSlopeMvPerMin();

  const bool rising =
      (lastSmoothedRiseMv >= CHARGE_RISE_MV) ||
      (lastSlopeMvPerMin >= CHARGE_SLOPE_MV_PER_MIN);
  const bool flat =
      (lastSmoothedRiseMv < CHARGE_CLEAR_RISE_MV) &&
      (lastSlopeMvPerMin < CHARGE_CLEAR_SLOPE_MV_PER_MIN);

  if (rising) {
    isCharging = true;
  } else if (flat) {
    isCharging = false;
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);
  pinMode(batteryVoltagePin, INPUT);
  analogReadResolution(12);
  analogSetPinAttenuation(batteryVoltagePin, ADC_11db);
  clearChargeHistory();
  Serial.printf(
      "TEST_BATTERY_ADC ready - %d-sample trimmed avg every %lus; "
      "charge lookback %d / smoothRise>=%.1fmV or slope>=%.1fmV/min\n",
      ADC_AVG_SAMPLES,
      (unsigned long)(POLL_INTERVAL_MS / 1000UL),
      CHARGE_HISTORY_LEN,
      CHARGE_RISE_MV,
      CHARGE_SLOPE_MV_PER_MIN);
}

void loop() {
  float adc = 0.0f;
  float vmonMvCal = 0.0f;
  readAdcAveraged(batteryVoltagePin, &adc, &vmonMvCal);
  updateChargingDetection(vmonMvCal);

  float vmonLinear = adc * (ADC_REF_V / ADC_MAX);
  float vmonCal = vmonMvCal / 1000.0f;
  float batteryLinear = vmonLinear * VMON_SCALE;
  float batteryCalScale = vmonCal * VMON_SCALE;
  float batteryQuad = batteryVoltageFromMvCal(vmonMvCal);
  int level = batteryChargeLevelFromVoltage(batteryQuad);

  unsigned long spanMs = 0;
  if (chargeHistoryCount >= 2) {
    spanMs = newestChargeSample().ms - oldestChargeSample().ms;
  }

  Serial.print(F("ADC="));
  Serial.print(adc, 1);
  Serial.print(F(" | VMON linear="));
  Serial.print(vmonLinear, 3);
  Serial.print(F("V (Bat "));
  Serial.print(batteryLinear, 2);
  Serial.print(F("V) | VMON cal="));
  Serial.print(vmonCal, 3);
  Serial.print(F("V/"));
  Serial.print(vmonMvCal, 1);
  Serial.print(F("mV (scale "));
  Serial.print(batteryCalScale, 2);
  Serial.print(F("V quad "));
  Serial.print(batteryQuad, 2);
  Serial.print(F("V) Level="));
  Serial.print(level);
  Serial.print(F("% | CHARGING="));
  Serial.print(isCharging ? 1 : 0);
  Serial.print(F(" rise="));
  Serial.print(lastSmoothedRiseMv, 1);
  Serial.print(F("mV slope="));
  Serial.print(lastSlopeMvPerMin, 2);
  Serial.print(F("mV/min span="));
  Serial.print(spanMs / 1000UL);
  Serial.println(F("s"));

  delay(POLL_INTERVAL_MS);
}
