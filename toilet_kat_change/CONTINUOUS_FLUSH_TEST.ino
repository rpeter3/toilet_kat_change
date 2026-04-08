// CONTINUOUS_FLUSH_TEST — thermal loop test: heat to 80°C, hold briefly, cool until <60°C, full
// mechanism sequence, then repeat. Starts on first flush button (hold or short press after delay).
//
// Enable by setting CONTINUOUS_FLUSH_TEST to 1 in toilet_kat_change.ino (near the top).
// Default in main sketch is 0 (disabled).

#if CONTINUOUS_FLUSH_TEST

static bool continuousFlushTest_paramsSaved = false;
static float saved_K;
static float saved_COOL_OPEN_TEMP_C;
static long saved_H;
static int saved_F;
static long saved_T;
static long saved_MAX_COOL_WAIT_S;
static float saved_preFeedFan;

static const float CONTINUOUS_TEST_HEAT_TARGET_C = 80.0f;
static const float CONTINUOUS_TEST_COOL_OPEN_C = 60.0f;
static const long CONTINUOUS_TEST_HOLD_SECONDS = 2;
static const int CONTINUOUS_TEST_FEED_F = 2;
static const long CONTINUOUS_TEST_T_COOL_ESTIMATE = 45;
static const float CONTINUOUS_TEST_PREFAN_S = 0.5f;
static const long CONTINUOUS_TEST_MAX_COOL_WAIT_S = 120;

bool continuousFlushTestActive = false;

// Returns true to skip cut-bag LED animation (not used in this mode).
bool continuousFlushTestOnFlushStarting() {
  if (!continuousFlushTest_paramsSaved) {
    saved_K = K;
    saved_COOL_OPEN_TEMP_C = COOL_OPEN_TEMP_C;
    saved_H = H;
    saved_F = F;
    saved_T = T;
    saved_MAX_COOL_WAIT_S = MAX_COOL_WAIT_S;
    saved_preFeedFan = preFeedFan;
    continuousFlushTest_paramsSaved = true;
  }
  K = CONTINUOUS_TEST_HEAT_TARGET_C;
  COOL_OPEN_TEMP_C = CONTINUOUS_TEST_COOL_OPEN_C;
  H = CONTINUOUS_TEST_HOLD_SECONDS;
  F = CONTINUOUS_TEST_FEED_F;
  T = CONTINUOUS_TEST_T_COOL_ESTIMATE;
  MAX_COOL_WAIT_S = CONTINUOUS_TEST_MAX_COOL_WAIT_S;
  preFeedFan = CONTINUOUS_TEST_PREFAN_S;
  cutBag = false;
  continuousFlushTestActive = true;
  Serial.println("CONTINUOUS_FLUSH_TEST: armed — 80°C target, open when <60°C, cycles repeat");
  SerialBLE_println("CONTINUOUS_FLUSH_TEST armed");
  return true;
}

bool continuousFlushTestTryRestartFromCase13() {
  if (!continuousFlushTestActive) {
    return false;
  }
  Serial.println("CONTINUOUS_FLUSH_TEST: restarting cycle");
  SerialBLE_println("CONTINUOUS_FLUSH_TEST: restarting cycle");

  flushStep = 0;
  case1FeedStarted = false;
  case5FeedExecuted = false;
  case6CutMotorRun = false;
  case10FanStarted = false;
  case10BackupStarted = false;
  m3ReverseActive = false;
  m3ReverseCompleted = false;
  m1CloseStartTime = 0;
  mechanismMotorRunning = false;

  flushStartMillis = millis();
  calculateSequenceTiming();
  ledIndex = 0;
  clockwise = true;
  ledLastUpdateMillis = millis();
  for (int i = 0; i < totalLeds; i++) {
    mcp_digitalWrite(ledPins[i], LOW);
  }
  mcp_digitalWrite(ledPins[0], HIGH);
  isFlushing = true;
  return true;
}

#endif
