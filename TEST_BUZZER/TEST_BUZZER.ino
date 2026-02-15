// Simple buzzer test - beeps on and off every second
// Buzzer pin (ESP32 GPIO)
const int buzzerPin = 38;

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("Buzzer Test Starting...");
  Serial.println("Buzzer will beep ON and OFF every second");
  
  // Setup buzzer pin
  pinMode(buzzerPin, OUTPUT);
  digitalWrite(buzzerPin, LOW);
  
  Serial.println("Setup complete. Starting beep cycle...");
}

void loop() {
  // Turn buzzer ON
  digitalWrite(buzzerPin, HIGH);
  Serial.println("Buzzer ON");
  delay(1000);  // Wait 1 second
  
  // Turn buzzer OFF
  digitalWrite(buzzerPin, LOW);
  Serial.println("Buzzer OFF");
  delay(1000);  // Wait 1 second
}

