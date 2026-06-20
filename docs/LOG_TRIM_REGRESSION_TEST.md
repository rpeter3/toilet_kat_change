# Log Trim Regression Test

Hardware regression that verifies SPIFFS `/errors.txt` trimming at the three firmware thresholds:

| Trigger | Threshold | Target |
|---------|-----------|--------|
| Write (`logError`) | > 75 KB | 50 KB |
| OTA prep (`PREPARE_UPDATE`) | > 40 KB | 40 KB |
| Deep sleep (inactivity) | > 65 KB | 50 KB |

Constants mirror [`toilet_kat_change.ino`](../toilet_kat_change/toilet_kat_change.ino). **No firmware changes** are required.

## When To Run

Run before release when any of these changed:

- `trimErrorLogToSize()` or log size constants
- `prepareForOTA()` log handling
- Deep-sleep inactivity / trim path

## Hardware Requirements

- ESP32-S3 toilet control board on USB (known COM port)
- BLE adapter
- **Dedicated test bench** recommended — the harness backs up and restores SPIFFS, but rewrites the `storage` partition during each step

## Software Requirements

```powershell
pip install bleak esptool
```

Uses vendored ESP-IDF [`scripts/vendor/spiffsgen.py`](../scripts/vendor/spiffsgen.py) to repack SPIFFS images.

## Quick Start

### Dry-run (no hardware)

```powershell
cd toilet_kat_change
python scripts/run_log_trim_regression.py --dry-run
```

### Full run

```powershell
python scripts/run_log_trim_regression.py `
  --port COM6 `
  --address AA:BB:CC:DD:EE:FF
```

During the **sleep_trim** step (~135 s idle):

- Do **not** press panel buttons
- Do **not** connect the phone app or other BLE clients

The harness disables DEV mode, waits for inactivity deep sleep, then reboots via `esptool run` to read logs.

## Test Steps

| Step | Seed size | Action | Assertion |
|------|-----------|--------|-----------|
| `preflight` | — | Backup SPIFFS @ `0xDA0000`, verify `GET_LOGS` | Backup saved |
| `no_trim` | 30 KB | Flash seed, reboot | Oldest marker kept; no trim |
| `write_trim` | 76.8 KB (`MAX_LOG_SIZE`) | Flash seed, reboot (`ota_boot` append triggers trim) | ≤ 50 KB; oldest dropped, newest kept |
| `ota_trim` | 45 KB | Flash seed, `ENABLE_OTA` + `PREPARE_UPDATE` | ≤ 40 KB; newest kept |
| `sleep_trim` | 67 KB | Flash seed, DEV off, idle ~135 s, reboot | ≤ 50 KB; oldest dropped, newest kept |
| `restore` | — | Flash original SPIFFS backup | Bench restored |

## SPIFFS Seeding

The harness uses **read-modify-write** on the live SPIFFS partition:

1. `read-flash 0xDA0000 0x260000` → backup
2. Extract existing files (`errors.txt`, `hwcfg_*.bin`, …)
3. Replace `errors.txt` with a sized marker file
4. Repack with `spiffsgen.py` and flash back

Marker lines (`TRIM_MARKER_OLD_000001`, `TRIM_MARKER_NEW_999999`) prove line-aligned trimming keeps newest content.

## CLI Options

| Flag | Purpose |
|------|---------|
| `--dry-run` | Orchestration only; no USB/BLE |
| `--skip-restore` | Leave seeded SPIFFS in place (debug) |
| `--steps no_trim write_trim` | Run subset of steps |
| `--sleep-wait 140` | Override idle wait (default ~135 s) |
| `--boot-wait 120` | BLE reconnect timeout after reboot |

## Reports

Written to `test-results/log-trim-regression/`:

- `log_trim_regression_<timestamp>.json`
- `log_trim_regression_<timestamp>.txt`

Artifacts (backups, staged SPIFFS) land in `test-builds/log-trim-regression/`.

## Gate 0 (CI / pre-push)

```powershell
python scripts/run_log_trim_regression.py --dry-run
```

## Related

- OTA partition regression: [`OTA_REGRESSION_TEST.md`](OTA_REGRESSION_TEST.md)
- App release gates: `CompoCloset-S1-app/docs/MAJOR_RELEASE_TEST_PLAN.md`
