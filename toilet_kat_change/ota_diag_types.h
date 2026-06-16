#pragma once

#include <stdint.h>

#define OTA_LOG_TAIL_MAX_BYTES 1024
#define OTA_DIAG_MAGIC 0xA5
#define OTA_DIAG_SCHEMA 1
#define OTA_SUBTYPE_UNSET 0xFF
#define OTA_TRIGGER_NONE 0
#define OTA_TRIGGER_AUTO 1
#define OTA_TRIGGER_VALIDATION 2
#define OTA_TRIGGER_MANUAL 3
#define OTA_AUTO_ROLLBACK_MAX_ATTEMPTS 3
#define OTA_GOOD_BOOT_STREAK_REQUIRED 2

struct OtaDiagStore {
  uint8_t magic;
  uint8_t schema;
  uint8_t pending_verify;
  uint8_t ota_target_subtype;
  uint8_t boot_attempts;
  uint8_t good_boot_streak;
  uint8_t log_tail_mirrored;
  uint8_t last_reset_reason;
  uint8_t last_trigger;
  uint16_t log_tail_len;
  uint16_t event_seq;
  uint32_t session_finalize_ms;
  uint32_t session_fw_size;
  uint8_t session_md5_prefix[4];
  uint8_t session_target_subtype;
  uint8_t session_source_subtype;
  char last_reason[24];
  char failed_label[8];
  char good_label[8];
};
