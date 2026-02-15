#include "esp_partition.h"
#include "esp_ota_ops.h"

void setup() {
  Serial.begin(115200);
  while (!Serial); // Wait for USB Serial to connect
  delay(2000);

  Serial.println("\n--- HARDWARE CAPABILITIES ---");
  Serial.printf("Flash Size: %d MB\n", ESP.getFlashChipSize() / (1024 * 1024));
  
  if (psramInit()) {
    Serial.printf("PSRAM Size: %d MB (Detected)\n", ESP.getPsramSize() / (1024 * 1024));
  } else {
    Serial.println("PSRAM: Not detected! Check 'OPI PSRAM' setting.");
  }

  Serial.println("\n--- PARTITION TABLE CHECK ---");
  esp_partition_iterator_t it = esp_partition_find(ESP_PARTITION_TYPE_ANY, ESP_PARTITION_SUBTYPE_ANY, NULL);
  
  while (it != NULL) {
    const esp_partition_t* p = esp_partition_get(it);
    Serial.printf("Name: %-10s | Type: 0x%02x | Sub: 0x%02x | Size: %7d KB\n", 
                  p->label, p->type, p->subtype, p->size / 1024);
    it = esp_partition_next(it);
  }
  esp_partition_iterator_release(it);

  Serial.println("\n--- OTA STATUS ---");
  const esp_partition_t* running = esp_ota_get_running_partition();
  Serial.printf("Currently running from: %s\n", running->label);
}

void loop() {}