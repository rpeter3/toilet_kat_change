# OTA Partition Regression Test

Pre-deployment regression that exercises factory flash, dual-slot OTA updates, rollback, factory return, and a final OTA update.

## When To Run

Run this test **before deploying a new production firmware version** when any of the following changed:

- OTA finalize / boot verification logic
- Partition table or otadata handling
- BLE OTA transport
- Rollback or factory-return behavior

## Hardware Requirements

- ESP32-S3 toilet control board on USB (known COM port, default `COM6`)
- Windows host with Python 3.10+
- BLE adapter (built-in or USB)
- Five distinct OTA test binaries plus one factory binary
- Physical access to the control panel for trust confirmation during rollback steps

## Software Requirements

```powershell
pip install bleak esptool pyserial
```

Arduino ESP32 core and `esptool` are also used by `scripts/flash-factory-app.ps1`.

## Test Binaries

You need **six** `.bin` files:

| Role | Count | Notes |
|------|-------|-------|
| Factory | 1 | Flashed over USB to the `factory` partition |
| OTA1..OTA5 | 5 | Applied over BLE; should differ in `SOFTWARE_VERSION_NUMBER` and/or `SOFTWARE_BUILD_DATE` |

Each OTA binary must be `<= 0x480000` bytes (4.5 MiB).

Recommended workflow:

1. Build six tagged firmware images (or reuse archived test builds).
2. Record expected `SW` and `Build` strings from each image.
3. Fill in `scripts/ota_regression_manifest.example.json` and save as your run manifest.

## Expected Partition Sequence

The harness verifies this exact transition sequence:

```text
factory -> ota_0 -> ota_1 -> ota_0 -> ota_1 -> ota_0 -> factory -> ota_0
```

Step mapping:

| Step | Action | Expected boot partition | Expected version source |
|------|--------|----------------------|-------------------------|
| Flash Factory | USB flash | `factory` | factory binary |
| OTA1 | BLE OTA | `ota_0` | OTA1 binary |
| OTA2 | BLE OTA | `ota_1` | OTA2 binary |
| Rollback | BLE `OTA_ROLLBACK_PREVIOUS` | `ota_0` | OTA1 binary |
| OTA3 | BLE OTA | `ota_1` | OTA3 binary |
| OTA4 | BLE OTA | `ota_0` | OTA4 binary |
| Factory Reset | BLE `OTA_ROLLBACK_FACTORY` | `factory` | factory binary |
| OTA5 | BLE OTA | `ota_0` | OTA5 binary |

## Success Criteria

The run **passes** only if all checks succeed:

- Factory USB flash completes without `esptool` errors.
- Every OTA transfer completes and the device reboots cleanly.
- Observed partition sequence matches the table above.
- Each step reports the expected `SW` / `Build` identity when configured in the manifest.
- OTA pending verification clears (`GET_OTA_DIAG` shows `pending=0`) after each OTA step.
- Manual rollback returns to the previous OTA slot and diagnostics/logs reflect a rollback event.
- Factory rollback returns to `factory` and BLE remains connectable.
- No `UPDATE_ERROR`, MD5 mismatch, panic, watchdog, brownout, or unexpected partition is observed.
- Diagnostic collection steps complete; failures are recorded as test failures.

## Running The Harness

### 1. Preflight only

Checks binaries, serial connectivity, and baseline BLE diagnostics:

```powershell
python scripts/run_ota_regression.py `
  --port COM6 `
  --address AA:BB:CC:DD:EE:FF `
  --manifest scripts/my_ota_manifest.json `
  --preflight-only
```

### 2. Full regression

```powershell
python scripts/run_ota_regression.py `
  --port COM6 `
  --address AA:BB:CC:DD:EE:FF `
  --manifest scripts/my_ota_manifest.json
```

CLI without manifest:

```powershell
python scripts/run_ota_regression.py `
  --port COM6 `
  --address AA:BB:CC:DD:EE:FF `
  --factory-bin path\to\factory.bin `
  --factory-expected-sw 4.0.17.0 `
  --factory-expected-build 2026-06-01 `
  --ota-bin path\to\ota1.bin `
  --ota-bin path\to\ota2.bin `
  --ota-bin path\to\ota3.bin `
  --ota-bin path\to\ota4.bin `
  --ota-bin path\to\ota5.bin `
  --ota-expected-sw 4.0.17.1 `
  --ota-expected-sw 4.0.17.2 `
  --ota-expected-sw 4.0.17.3 `
  --ota-expected-sw 4.0.17.4 `
  --ota-expected-sw 4.0.17.5
```

### 3. Dry-run orchestration check

Exercises step logic without USB flash or BLE I/O:

```powershell
python scripts/run_ota_regression.py `
  --manifest scripts/my_ota_manifest.json `
  --dry-run
```

## Operator Actions During The Run

Rollback steps require a **trusted BLE session**:

1. When prompted, press any control panel button within the trust timeout (default 120 s).
2. Keep phone apps disconnected from the toilet during the run.
3. Allow ~30 s for cold boot before the first BLE connection after power-on.

## Reports

Each run writes artifacts under `test-results/ota-regression/`:

- `ota_regression_<timestamp>.json` — machine-readable full report
- `ota_regression_<timestamp>.txt` — human-readable summary

A checked-in orchestration baseline (dry-run, no hardware) is kept at:

- `test-results/ota-regression/ota_regression_baseline_dryrun.json`
- `test-results/ota-regression/ota_regression_baseline_dryrun.txt`

Re-run `--dry-run` after harness changes and compare step structure to that baseline.

Attach these before approving a deployment:

- JSON report
- Text summary
- Manifest used
- SHA256/MD5 hashes from the report preflight section
- Final `GET_OTA_DIAG` / `GET_BOOT_INFO` snapshot

## Troubleshooting

| Symptom | Likely cause | Action |
|---------|--------------|--------|
| `Serial port preflight failed` | COM port busy or wrong port | Close Serial Monitor / other tools using the port |
| `BLE device not found` | Device still booting or phone app connected | Wait 30 s, disconnect phone app, retry with `--address` |
| `Trust handshake timed out` | No button press during rollback | Re-run rollback step; press panel button when prompted |
| `pending` never clears | Only one post-OTA boot occurred | Harness auto-resets via `esptool run`; increase `--pending-clear-timeout` |
| `UPDATE_BLOCKED` | Flush or busy state | Wait for idle, retry OTA |
| `Partition sequence mismatch` | Wrong binary set or rollback failure | Inspect per-step report and `read_partitions.py --port COM6` |
| `UPDATE_VALIDATION_FAILED` | Corrupt binary or MD5 mismatch | Rebuild binary and verify manifest path |

## Manual Spot Checks

Optional commands while debugging a failed step:

```powershell
python read_partitions.py --port COM6
python get_ota_diag.py --address AA:BB:CC:DD:EE:FF
python ota_update.py path\to\ota.bin --address AA:BB:CC:DD:EE:FF
```

## Pre-Deployment Checklist

- [ ] Six test binaries built and hashed
- [ ] Manifest updated with expected `SW` / `Build`
- [ ] Preflight passed on target hardware
- [ ] Full regression passed (`RESULT: PASS`)
- [ ] Report artifacts archived with release notes
- [ ] Production binary matches or exceeds tested OTA behavior
