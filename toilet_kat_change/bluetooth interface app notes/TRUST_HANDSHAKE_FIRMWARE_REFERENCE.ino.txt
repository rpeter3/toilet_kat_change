/*
  TRUST_HANDSHAKE_FIRMWARE_REFERENCE.ino
  Paste-ready reference for integrating simple BLE trust handshake into firmware.

  Contract:
  - TRUST_START -> TRUST_WAITING / TRUST_CONFIRMED
  - TRUST_STATUS -> TRUST_WAITING / TRUST_CONFIRMED / TRUST_TIMEOUT
  - Optional TRUST_CANCEL -> TRUST_CANCEL_ACK
  - Privileged commands while untrusted -> AUTH_REQUIRED

  Physical behavior:
  - TRUST_START: start LED circling pattern.
  - Flush press during waiting: beep twice, stop LEDs, mark connection trusted.
  - Disconnect: clear trust state.
*/

// ---- Trust state ----
enum TrustState {
  TRUST_STATE_UNTRUSTED = 0,
  TRUST_STATE_WAITING = 1,
  TRUST_STATE_TRUSTED = 2,
  TRUST_STATE_TIMEOUT = 3
};

static TrustState g_trustState = TRUST_STATE_UNTRUSTED;
static unsigned long g_trustStartMs = 0;
static const unsigned long TRUST_TIMEOUT_MS = 15000;

// ---- Hooks to map into your existing firmware helpers ----
void startTrustLedCircle() {
  // TODO: call your LED animation start helper.
  // Example: ledMode = LED_MODE_CONNECT_WAIT;
}

void stopTrustLedCircle() {
  // TODO: call your LED animation stop helper.
  // Example: ledMode = LED_MODE_IDLE;
}

void trustDoubleBeep() {
  // TODO: map to your buzzer output.
  // Example timings: 70ms ON, 70ms OFF, 70ms ON.
  tone(BUZZER_PIN, 2400, 70);
  delay(140);
  tone(BUZZER_PIN, 2400, 70);
}

void resetTrustState() {
  g_trustState = TRUST_STATE_UNTRUSTED;
  g_trustStartMs = 0;
  stopTrustLedCircle();
}

void beginTrustWaiting() {
  g_trustState = TRUST_STATE_WAITING;
  g_trustStartMs = millis();
  startTrustLedCircle();
}

void onTrustConfirmedByFlushButton() {
  if (g_trustState != TRUST_STATE_WAITING) {
    return;
  }
  g_trustState = TRUST_STATE_TRUSTED;
  stopTrustLedCircle();
  trustDoubleBeep();
}

void updateTrustTimeout() {
  if (g_trustState != TRUST_STATE_WAITING) {
    return;
  }
  if ((millis() - g_trustStartMs) >= TRUST_TIMEOUT_MS) {
    g_trustState = TRUST_STATE_TIMEOUT;
    stopTrustLedCircle();
  }
}

bool isTrustedConnection() {
  return g_trustState == TRUST_STATE_TRUSTED;
}

// ---- Call this from your existing flush button event path ----
void onFlushButtonPressed() {
  // Existing flush logic remains unchanged.
  onTrustConfirmedByFlushButton();
}

// ---- BLE command handling shim ----
String handleTrustCommand(const String& cmd) {
  updateTrustTimeout();

  if (cmd == "TRUST_START") {
    if (g_trustState == TRUST_STATE_TRUSTED) {
      return "TRUST_CONFIRMED";
    }
    beginTrustWaiting();
    return "TRUST_WAITING";
  }

  if (cmd == "TRUST_STATUS") {
    if (g_trustState == TRUST_STATE_TRUSTED) {
      return "TRUST_CONFIRMED";
    }
    if (g_trustState == TRUST_STATE_TIMEOUT) {
      return "TRUST_TIMEOUT";
    }
    if (g_trustState == TRUST_STATE_WAITING) {
      return "TRUST_WAITING";
    }
    return "TRUST_TIMEOUT";
  }

  if (cmd == "TRUST_CANCEL") {
    resetTrustState();
    return "TRUST_CANCEL_ACK";
  }

  return "";
}

// ---- Example privileged command gate ----
bool requiresTrustedConnection(const String& cmd) {
  // Add/adjust these command tests to match your existing parser.
  if (cmd.startsWith("SET_HW_COMPONENT:")) return true;
  if (cmd.startsWith("HWCFG_APPLY_CHANGE:")) return true;
  if (cmd == "HWCFG_ROLLBACK_LAST_GOOD") return true;

  // Parameter update payload is comma-separated floats in this project.
  // If parser identifies this payload shape, treat as privileged.
  bool hasComma = cmd.indexOf(',') >= 0;
  bool startsWithNumeric = cmd.length() > 0 && (isDigit(cmd[0]) || cmd[0] == '-' || cmd[0] == '.');
  if (hasComma && startsWithNumeric) return true;

  return false;
}

String handleBleCommand(const String& cmd) {
  // 1) Trust command surface.
  String trustResponse = handleTrustCommand(cmd);
  if (trustResponse.length() > 0) {
    return trustResponse;
  }

  // 2) Gate privileged commands until trusted.
  if (requiresTrustedConnection(cmd) && !isTrustedConnection()) {
    return "AUTH_REQUIRED";
  }

  // 3) Existing command handling path.
  // TODO: return your existing command parser result here.
  return "UNHANDLED";
}

// ---- Connection lifecycle hooks ----
void onBleClientConnected() {
  resetTrustState();
}

void onBleClientDisconnected() {
  resetTrustState();
}

