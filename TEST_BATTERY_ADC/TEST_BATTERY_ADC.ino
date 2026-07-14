// Lightweight battery VMON poll - no heater, no BLE, no load assess.
// Mirrors firmware charging detection + batt_full_learn sample math for calibration.
// Every poll prints window stats: min/max/mean/stdev, slope, rise/fall.

#include <math.h>
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
// Charger connect causes a step-up that ripples through the n=60 history and
// can look like a lasting slope; require a real rise (>=5 mV) or >=1 mV/min
// before asserting isCharging (hysteresis clear below that).
const unsigned long INACTIVITY_SLEEP_MS = 5UL * 60UL * 1000UL;
const unsigned long CHARGE_SAMPLE_INTERVAL_MS = POLL_INTERVAL_MS;
const int CHARGE_HISTORY_LEN = (int)(INACTIVITY_SLEEP_MS / CHARGE_SAMPLE_INTERVAL_MS);  // 60
const unsigned long CHARGE_MIN_SPAN_MS = 45000UL;
const int CHARGE_SMOOTH_SAMPLES = 4;          // mean of oldest/newest N samples
const float CHARGE_RISE_MV = 5.0f;            // smoothed rise to assert charging
const float CHARGE_CLEAR_RISE_MV = 0.0f;      // clear when rise <= 0 (flat / falling)
const float CHARGE_SLOPE_MV_PER_MIN = 1.0f;   // LS slope to assert charging
const float CHARGE_CLEAR_SLOPE_MV_PER_MIN = 0.0f;  // and slope <= 0
const float CHARGE_DROP_RESET_MV = 5.0f;

// Full-anchor learn window (matches toilet_kat_change.ino)
constexpr int BATT_FULL_LEARN_WINDOW = 6;
constexpr float LEARN_STDEV_MAX_MV = 1.0f;  // reject jagged windows; candidate is mean
constexpr float LEARN_MIN_DELTA_MV = 0.1f;  // ignore sub-0.1 mV float noise raises
constexpr float kVFullMvSeed = 1550.0f;
constexpr float VFULL_MV_MIN = 1500.0f;
constexpr float kVFullMvMax = 1920.0f;
constexpr float FULL_SLOW_STEP_MV = 3.0f;  // sole upward step rate (charge-gated)

struct ChargeSample {
  float mv;
  unsigned long ms;
};

struct WindowStats {
  int n;
  float minMv;
  float maxMv;
  float meanMv;
  float stdevMv;
  float rangeMv;  // max - min
};

static ChargeSample chargeHistory[CHARGE_HISTORY_LEN];
static int chargeHistoryCount = 0;
static int chargeHistoryHead = 0;  // next write index
static bool hasLastChargeMv = false;
static float lastChargeMv = 0.0f;
bool isCharging = false;
static float lastSmoothedRiseMv = 0.0f;
static float lastSlopeMvPerMin = 0.0f;
static float lastSampleDeltaMv = 0.0f;  // rise(+)/fall(-) vs previous sample
static bool chargeMetricsReady = false;

static float battFullLearnWindow[BATT_FULL_LEARN_WINDOW];
static int battFullLearnCount = 0;
static int battFullLearnHead = 0;
static float vFullMv = kVFullMvSeed;

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

static void clearBattFullLearnWindow() {
  battFullLearnCount = 0;
  battFullLearnHead = 0;
}

static void clearChargeHistory() {
  chargeHistoryCount = 0;
  chargeHistoryHead = 0;
  hasLastChargeMv = false;
  isCharging = false;
  lastSmoothedRiseMv = 0.0f;
  lastSlopeMvPerMin = 0.0f;
  lastSampleDeltaMv = 0.0f;
  chargeMetricsReady = false;
  clearBattFullLearnWindow();
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

static WindowStats statsFromFloats(const float* values, int n) {
  WindowStats s = {};
  s.n = n;
  if (n <= 0) {
    return s;
  }

  float sum = 0.0f;
  s.minMv = values[0];
  s.maxMv = values[0];
  for (int i = 0; i < n; i++) {
    float v = values[i];
    sum += v;
    if (v < s.minMv) {
      s.minMv = v;
    }
    if (v > s.maxMv) {
      s.maxMv = v;
    }
  }
  s.meanMv = sum / (float)n;
  s.rangeMv = s.maxMv - s.minMv;

  float varSum = 0.0f;
  for (int i = 0; i < n; i++) {
    float d = values[i] - s.meanMv;
    varSum += d * d;
  }
  s.stdevMv = sqrtf(varSum / (float)n);
  return s;
}

static WindowStats chargeHistoryStats() {
  float values[CHARGE_HISTORY_LEN];
  for (int i = 0; i < chargeHistoryCount; i++) {
    values[i] = chargeHistory[chargeHistoryIndex(i)].mv;
  }
  return statsFromFloats(values, chargeHistoryCount);
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
// Always recomputes rise/slope for logging when history allows; gate still uses firmware rules.
static void updateChargingDetection(float mvCal) {
  unsigned long now = millis();
  chargeMetricsReady = false;
  lastSampleDeltaMv = 0.0f;

  if (hasLastChargeMv) {
    lastSampleDeltaMv = mvCal - lastChargeMv;
    if ((lastChargeMv - mvCal) >= CHARGE_DROP_RESET_MV) {
      Serial.printf(
          "[charge_reset] drop=%.2fmV (>=%.1f) clearing charge + learn windows\n",
          lastChargeMv - mvCal,
          CHARGE_DROP_RESET_MV);
      clearChargeHistory();
      lastSampleDeltaMv = 0.0f;
    }
  }

  pushChargeSample(mvCal, now);
  lastChargeMv = mvCal;
  hasLastChargeMv = true;
  lastSmoothedRiseMv = 0.0f;
  lastSlopeMvPerMin = 0.0f;

  if (chargeHistoryCount < 2) {
    return;
  }

  // Always compute slope for observation once we have 2+ samples.
  lastSlopeMvPerMin = chargeSlopeMvPerMin();

  if (chargeHistoryCount < (CHARGE_SMOOTH_SAMPLES * 2)) {
    return;
  }

  ChargeSample oldest = oldestChargeSample();
  ChargeSample newest = newestChargeSample();
  unsigned long spanMs = newest.ms - oldest.ms;

  lastSmoothedRiseMv = meanChargeEdge(true, CHARGE_SMOOTH_SAMPLES) -
                       meanChargeEdge(false, CHARGE_SMOOTH_SAMPLES);

  if (spanMs < CHARGE_MIN_SPAN_MS) {
    return;
  }

  chargeMetricsReady = true;

  const bool rising =
      (lastSmoothedRiseMv >= CHARGE_RISE_MV) ||
      (lastSlopeMvPerMin >= CHARGE_SLOPE_MV_PER_MIN);
  const bool flat =
      (lastSmoothedRiseMv <= CHARGE_CLEAR_RISE_MV) &&
      (lastSlopeMvPerMin <= CHARGE_CLEAR_SLOPE_MV_PER_MIN);

  if (rising) {
    if (!isCharging) {
      Serial.printf(
          "CHARGING_DETECTED: mv=%.1f rise=%.2fmV slope=%.2fmV/min span=%lums "
          "(th rise=%.2f slope=%.2f)\n",
          mvCal,
          lastSmoothedRiseMv,
          lastSlopeMvPerMin,
          (unsigned long)spanMs,
          CHARGE_RISE_MV,
          CHARGE_SLOPE_MV_PER_MIN);
    }
    isCharging = true;
  } else if (flat) {
    isCharging = false;
  }
}

static void pushBattFullLearnSample(float mv) {
  battFullLearnWindow[battFullLearnHead] = mv;
  battFullLearnHead = (battFullLearnHead + 1) % BATT_FULL_LEARN_WINDOW;
  if (battFullLearnCount < BATT_FULL_LEARN_WINDOW) {
    battFullLearnCount++;
  }
}

// Copy learn window in chronological order (oldest -> newest).
static int copyBattFullLearnOrdered(float* out, int outMax) {
  int n = battFullLearnCount;
  if (n > outMax) {
    n = outMax;
  }
  int oldest = (battFullLearnHead - battFullLearnCount + BATT_FULL_LEARN_WINDOW) %
               BATT_FULL_LEARN_WINDOW;
  for (int i = 0; i < n; i++) {
    out[i] = battFullLearnWindow[(oldest + i) % BATT_FULL_LEARN_WINDOW];
  }
  return n;
}

// Mirrors onIdleBattMvSample(); returns whether a ratchet occurred.
// outWouldRaise reflects charging + delta (even if stdev gate fails) for calibration.
static bool updateBattFullLearn(float mvCal, WindowStats* outStats, float* outCandidate,
                                bool* outGateOk, bool* outWouldRaise, float* outStep) {
  *outCandidate = 0.0f;
  *outGateOk = false;
  *outWouldRaise = false;
  *outStep = 0.0f;

  pushBattFullLearnSample(mvCal);

  float ordered[BATT_FULL_LEARN_WINDOW];
  int n = copyBattFullLearnOrdered(ordered, BATT_FULL_LEARN_WINDOW);
  *outStats = statsFromFloats(ordered, n);

  const float mean = outStats->meanMv;
  const float stdev = outStats->stdevMv;
  const float delta = mean - vFullMv;
  *outCandidate = mean;
  if (n >= BATT_FULL_LEARN_WINDOW) {
    *outGateOk = (stdev <= LEARN_STDEV_MAX_MV);
  }
  *outWouldRaise = isCharging && (*outGateOk) && (delta > LEARN_MIN_DELTA_MV);

  // Only raise while charging (matches firmware).
  if (!isCharging) {
    return false;
  }

  if (!(*outWouldRaise)) {
    return false;
  }

  float stepCap = FULL_SLOW_STEP_MV;
  float step = delta < stepCap ? delta : stepCap;
  *outStep = step;

  float next = vFullMv + step;
  if (next < VFULL_MV_MIN) {
    next = VFULL_MV_MIN;
  }
  if (next > kVFullMvMax) {
    next = kVFullMvMax;
  }
  if (!(next > vFullMv)) {
    return false;
  }

  float oldV = vFullMv;
  vFullMv = next;
  Serial.printf(
      "[batt_full_learn] vFullMv %.1f->%.1f step=%.1f candidate=%.1f stdev=%.3f\n",
      oldV, vFullMv, step, mean, stdev);
  return true;
}

void setup() {
  Serial.begin(115200);
  delay(500);
  pinMode(batteryVoltagePin, INPUT);
  analogReadResolution(12);
  analogSetPinAttenuation(batteryVoltagePin, ADC_11db);
  clearChargeHistory();
  vFullMv = kVFullMvSeed;

  Serial.println(F("TEST_BATTERY_ADC calibration logger"));
  Serial.printf(
      "Poll=%lus  ADC avg=%d trim=%d  chargeHist=%d  learnWin=%d  vFullSeed=%.1f\n",
      (unsigned long)(POLL_INTERVAL_MS / 1000UL),
      ADC_AVG_SAMPLES,
      ADC_AVG_TRIM,
      CHARGE_HISTORY_LEN,
      BATT_FULL_LEARN_WINDOW,
      kVFullMvSeed);
  Serial.printf(
      "CHARGE th: rise>=%.2f clear<%.2f | slope>=%.2f clear<%.2f | minSpan=%lus | dropReset=%.1f\n",
      CHARGE_RISE_MV,
      CHARGE_CLEAR_RISE_MV,
      CHARGE_SLOPE_MV_PER_MIN,
      CHARGE_CLEAR_SLOPE_MV_PER_MIN,
      (unsigned long)(CHARGE_MIN_SPAN_MS / 1000UL),
      CHARGE_DROP_RESET_MV);
  Serial.printf(
      "LEARN gate: n==%d, stdev<=%.1fmV, candidate=mean, requires charging, "
      "raise if delta>%.1fmV step<=%.1fmV\n",
      BATT_FULL_LEARN_WINDOW,
      LEARN_STDEV_MAX_MV,
      LEARN_MIN_DELTA_MV,
      FULL_SLOW_STEP_MV);
  Serial.println(
      F("Columns: SAMPLE / CHARGE / LEARN  (deltaMv = rise+/fall- vs previous sample)"));
}

void loop() {
  float adc = 0.0f;
  float vmonMvCal = 0.0f;
  readAdcAveraged(batteryVoltagePin, &adc, &vmonMvCal);

  updateChargingDetection(vmonMvCal);

  WindowStats learnStats = {};
  float candidate = 0.0f;
  bool gateOk = false;
  bool wouldRaise = false;
  float step = 0.0f;
  updateBattFullLearn(vmonMvCal, &learnStats, &candidate, &gateOk, &wouldRaise, &step);

  WindowStats chargeStats = chargeHistoryStats();

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

  const char* dLabel = "flat";
  if (lastSampleDeltaMv > 0.0f) {
    dLabel = "rise";
  } else if (lastSampleDeltaMv < 0.0f) {
    dLabel = "fall";
  }

  // Per-sample raw observation
  Serial.printf(
      "[SAMPLE] mv=%.2f V=%.3f quad=%.2fV lvl=%d%% deltaMv=%+.2f (%s) "
      "ADC=%.1f linearBat=%.2fV scaleBat=%.2fV\n",
      vmonMvCal,
      vmonCal,
      batteryQuad,
      level,
      lastSampleDeltaMv,
      dLabel,
      adc,
      batteryLinear,
      batteryCalScale);

  // Charging-detection window (full lookback history)
  Serial.printf(
      "[CHARGE] n=%d span=%lus ready=%d charging=%d | "
      "min=%.2f max=%.2f mean=%.2f stdev=%.3f range=%.2f | "
      "rise=%.3f (th=%.2f/%.2f) slope=%.3f mV/min (th=%.2f/%.2f) | deltaMv=%+.2f\n",
      chargeStats.n,
      spanMs / 1000UL,
      chargeMetricsReady ? 1 : 0,
      isCharging ? 1 : 0,
      chargeStats.minMv,
      chargeStats.maxMv,
      chargeStats.meanMv,
      chargeStats.stdevMv,
      chargeStats.rangeMv,
      lastSmoothedRiseMv,
      CHARGE_RISE_MV,
      CHARGE_CLEAR_RISE_MV,
      lastSlopeMvPerMin,
      CHARGE_SLOPE_MV_PER_MIN,
      CHARGE_CLEAR_SLOPE_MV_PER_MIN,
      lastSampleDeltaMv);

  // batt_full_learn short window + gate math (candidate = mean, charge-gated)
  Serial.printf(
      "[LEARN] n=%d charging=%d | min=%.2f max=%.2f mean=%.2f stdev=%.3f (max=%.1f) range=%.2f | "
      "candidate=%.2f gate=%d wouldRaise=%d step=%.1f vFullMv=%.1f\n",
      learnStats.n,
      isCharging ? 1 : 0,
      learnStats.minMv,
      learnStats.maxMv,
      learnStats.meanMv,
      learnStats.stdevMv,
      LEARN_STDEV_MAX_MV,
      learnStats.rangeMv,
      candidate,
      gateOk ? 1 : 0,
      wouldRaise ? 1 : 0,
      step,
      vFullMv);

  Serial.println();
  delay(POLL_INTERVAL_MS);
}
