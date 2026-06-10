#include <esp_sleep.h>
#include <driver/rtc_io.h>

// TEST_SLEEP: minimal light-sleep test with no I2C/MCP/motor driver.
// Wake on GPIO2, then return to sleep after a short idle period.

const int controlPanelWake = 2;       // GPIO2 wake/button line
const int buzzerPin = 38;

const unsigned long IDLE_BEFORE_SLEEP_MS = 2000;
unsigned long lastActivityMillis = 0;

void beep(int count) {
  for (int i = 0; i < count; i++) {
    digitalWrite(buzzerPin, HIGH);
    delay(25);
    digitalWrite(buzzerPin, LOW);
    delay(50);
  }
}

bool wakeButtonPressed() {
  return digitalRead(controlPanelWake) == LOW;
}

void enterLightSleep() {
  beep(1);

  rtc_gpio_init((gpio_num_t)controlPanelWake);
  rtc_gpio_pullup_en((gpio_num_t)controlPanelWake);
  rtc_gpio_pulldown_dis((gpio_num_t)controlPanelWake);
  esp_sleep_enable_ext0_wakeup((gpio_num_t)controlPanelWake, 0);

  esp_light_sleep_start();

  rtc_gpio_deinit((gpio_num_t)controlPanelWake);
  pinMode(controlPanelWake, INPUT_PULLUP);

  beep(2);
  lastActivityMillis = millis();
}

void setup() {
  pinMode(controlPanelWake, INPUT_PULLUP);
  pinMode(buzzerPin, OUTPUT);
  digitalWrite(buzzerPin, LOW);

  lastActivityMillis = millis();
  enterLightSleep();
}

void loop() {
  bool buttonPressed = wakeButtonPressed();

  if (buttonPressed) {
    lastActivityMillis = millis();
  }

  if (!buttonPressed &&
      (millis() - lastActivityMillis >= IDLE_BEFORE_SLEEP_MS) &&
      digitalRead(controlPanelWake) == HIGH) {
    enterLightSleep();
  }

  delay(10);
}
