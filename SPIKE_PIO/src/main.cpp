// SPIKE_PIO: ROM erase + write good bootloader @0x0 under SARAH.
// Sacrificial hardware only. Verify via esptool MD5, not esp_flash_read.

#include <Arduino.h>
#include <esp_ota_ops.h>
#include <esp_private/cache_utils.h>
#include <esp_rom_spiflash.h>
#include <esp_task_wdt.h>
#include <soc/rtc.h>

#include "esp32-hal.h"
#include "good_bootloader.h"

static const size_t BOOTLOADER_REGION_SIZE = 0x8000;
static const uint32_t SECTOR_SIZE = 4096;
static const char* GOOD_BOOTLOADER_MD5 = "e536c176c97b5905286ed980da47abe8";

RTC_NOINIT_ATTR static uint32_t g_postPatchBoot;

static void logLine(const char* msg) {
  Serial.println(msg);
  Serial.flush();
}

static void logRunningPartition() {
  const esp_partition_t* running = esp_ota_get_running_partition();
  if (!running) {
    logLine("[info] running partition: <unknown>");
    return;
  }
  Serial.printf("[info] running partition: label=%s offset=0x%06X subtype=0x%02X\n",
                running->label, (unsigned)running->address, running->subtype);
  Serial.flush();
}

static void suspendWatchdogs() {
  disableLoopWDT();
  esp_err_t err = esp_task_wdt_deinit();
  Serial.printf("[wdt] esp_task_wdt_deinit err=%d\n", err);
  Serial.flush();
}

IRAM_ATTR static esp_err_t romEraseSector(uint32_t sector, int* romResult) {
  spi_flash_disable_interrupts_caches_and_other_cpu();
  esp_rom_spiflash_result_t r = esp_rom_spiflash_erase_sector(sector);
  spi_flash_enable_interrupts_caches_and_other_cpu();
  if (romResult) {
    *romResult = static_cast<int>(r);
  }
  return (r == ESP_ROM_SPIFLASH_RESULT_OK) ? ESP_OK : ESP_FAIL;
}

IRAM_ATTR static esp_err_t romWriteBulk(uint32_t destAddr, const uint8_t* data, size_t len) {
  if ((len % 4) != 0) {
    return ESP_ERR_INVALID_SIZE;
  }
  spi_flash_disable_interrupts_caches_and_other_cpu();
  esp_rom_spiflash_result_t wr =
      esp_rom_spiflash_write(destAddr, reinterpret_cast<const uint32_t*>(data),
                             static_cast<int32_t>(len));
  spi_flash_enable_interrupts_caches_and_other_cpu();
  return (wr == ESP_ROM_SPIFLASH_RESULT_OK) ? ESP_OK : ESP_FAIL;
}

static void runOverwritePatch() {
  suspendWatchdogs();

  logLine("");
  logLine("=== SPIKE_PIO: ROM bootloader overwrite ===");
  logRunningPartition();
  Serial.printf("[info] embedded good BL len=%u expected_md5=%s\n",
                (unsigned)EMBEDDED_BOOTLOADER_LEN, GOOD_BOOTLOADER_MD5);
  Serial.flush();

  const uint32_t sectorCount = BOOTLOADER_REGION_SIZE / SECTOR_SIZE;
  for (uint32_t sector = 0; sector < sectorCount; sector++) {
    Serial.printf("[erase] sector %lu/%lu begin\n",
                  (unsigned long)(sector + 1), (unsigned long)sectorCount);
    Serial.flush();

    uint32_t t0 = millis();
    int romResult = -1;
    esp_err_t err = romEraseSector(sector, &romResult);

    Serial.printf("[erase] sector %lu end err=%d rom_result=%d dt=%lu ms\n",
                  (unsigned long)(sector + 1), err, romResult, millis() - t0);
    Serial.flush();

    if (err != ESP_OK) {
      Serial.printf("[FAIL] erase stopped at sector %lu\n", (unsigned long)(sector + 1));
      logLine("[done] reflash bootloader via USB before normal use");
      return;
    }
  }

  logLine("[erase] all sectors complete");

  uint32_t t0 = millis();
  esp_err_t err = romWriteBulk(0, EMBEDDED_BOOTLOADER, EMBEDDED_BOOTLOADER_LEN);
  Serial.printf("[write] ROM write @0x0 len=%u err=%d dt=%lu ms\n",
                (unsigned)EMBEDDED_BOOTLOADER_LEN, err, millis() - t0);
  Serial.flush();

  if (err != ESP_OK) {
    logLine("[FAIL] ROM write failed — reflash bootloader via USB");
    return;
  }

  g_postPatchBoot = 1;
  logLine("[PASS] ROM overwrite of 0x0..0x7FFF completed");
  logLine("[step] rebooting in 1s...");
  delay(1000);
  esp_restart();
}

static void runPostPatchBoot() {
  logLine("");
  logLine("=== SPIKE_PIO: post-patch boot ===");
  logRunningPartition();
  logLine("[PASS] booted with new bootloader");
  logLine("[note] verify physical MD5 via esptool read-flash 0 19984");
}

void setup() {
  Serial.begin(115200);
  delay(1500);
  logLine("SPIKE_PIO: setup done");
}

void loop() {
  static bool ran = false;
  if (ran) {
    delay(1000);
    return;
  }
  ran = true;

  if (g_postPatchBoot == 1) {
    runPostPatchBoot();
    return;
  }

  runOverwritePatch();
}
