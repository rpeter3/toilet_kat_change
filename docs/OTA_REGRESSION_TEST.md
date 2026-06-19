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
- Physical access to the control panel for trust confirmation during rollback steps (not required when DEV mode is on)

## Software Requirements

```powershell
pip install bleak esptool pyserial
```

Arduino ESP32 core and `esptool` are also used by the flash scripts.

## Recommended: Auto-Build (`--auto-build`)

Use `--auto-build` to test **current source code** instead of hand-picked manifest binaries. Each run:

1. Extracts and flashes **SARAH factory** as the baseline partition image
2. Builds two OTA variants from the current tree with distinct version suffixes (`{base}-regA`, `{base}-regB`)
3. Runs the partition sequence and asserts **version evolution** at every hop

`{base}` is parsed from `toilet_kat_change/toilet_kat_change.ino` at build time (e.g. `4.0.17.1` → `4.0.17.1-regA` / `4.0.17.1-regB`).

| Step | Partition | Binary | Expected `SW` |
|------|-----------|--------|---------------|
| Flash Factory | `factory` | SARAH extracted app | SARAH version (read from bin) |
| OTA1 | `ota_0` | regA build | `{base}-regA` |
| OTA2 | `ota_1` | regB build | `{base}-regB` |
| Rollback | `ota_0` | (no flash) | `{base}-regA` |
| OTA3 | `ota_1` | regB | `{base}-regB` |
| OTA4 | `ota_0` | regA | `{base}-regA` |
| Factory Reset | `factory` | (otadata only) | SARAH version |
| OTA5 | `ota_0` | regB (newest) | `{base}-regB` |

Build artifacts land in `test-builds/ota-regression/`:

- `factory_sarah.bin` — SARAH factory app extracted from merged image
- `ota_regA.bin`, `ota_regB.bin` — freshly built test variants
- `manifest.json` — auto manifest consumed by the harness

**Note:** `-regA`/`-regB` builds are test-only. Do not bundle them into the CompoCloset phone app.

### Auto-build fast mode (recommended for iteration)

```powershell
python scripts/run_ota_regression.py `
  --port COM5 `
  --address 58:E6:C5:6C:AF:81 `
  --auto-build `
  --fast
```

### Auto-build full mode (pre-release gate)

```powershell
python scripts/run_ota_regression.py `
  --port COM6 `
  --address AA:BB:CC:DD:EE:FF `
  --auto-build
```

### Auto-build dry-run

Reuses an existing `test-builds/ota-regression/manifest.json` when present; otherwise builds firmware first:

```powershell
python scripts/run_ota_regression.py --auto-build --dry-run
```

## Manual Manifest (special cases)

For archived binaries or non-standard version sets, use a hand-written manifest. See `scripts/ota_regression_manifest.example.json`.

You need **six** `.bin` files:

| Role | Count | Notes |
|------|-------|-------|
| Factory | 1 | Flashed over USB to the `factory` partition |
| OTA1..OTA5 | 5 | Applied over BLE; should differ in `SOFTWARE_VERSION_NUMBER` and/or `SOFTWARE_BUILD_DATE` |

Each OTA binary must be `<= 0x480000` bytes (4.5 MiB).

## Expected Partition Sequence

The harness verifies this exact transition sequence:

```text
factory -> ota_0 -> ota_1 -> ota_0 -> ota_1 -> ota_0 -> factory -> ota_0
```

Step mapping:

| Step | Action | Expected boot partition | Expected version source |
|------|--------|----------------------|-------------------------|
| Flash Factory | USB flash (SARAH when `--auto-build`) | `factory` | factory binary |
| OTA1 | BLE OTA | `ota_0` | OTA1 binary |
| OTA2 | BLE OTA | `ota_1` | OTA2 binary |
| Rollback | BLE `OTA_ROLLBACK_PREVIOUS` / serial sim (`--fast`) | `ota_0` | OTA1 binary |
| OTA3 | BLE OTA / serial sim (`--fast`) | `ota_1` | OTA3 binary |
| OTA4 | BLE OTA / serial sim (`--fast`) | `ota_0` | OTA4 binary |
| Factory Reset | BLE `OTA_ROLLBACK_FACTORY` / serial sim (`--fast`) | `factory` | factory binary |
| OTA5 | BLE OTA / serial sim (`--fast`) | `ota_0` | OTA5 binary |

## Fast Mode (`--fast`)

Use fast mode for day-to-day iteration. It keeps the same partition sequence but only performs **two real BLE OTA transfers** (OTA1 and OTA2). OTA3–OTA5, rollback, and factory reset are simulated by writing the target slot over USB and setting `otadata`.

| Step | Full mode | Fast mode |
|------|-----------|-----------|
| Flash Factory | USB (SARAH with `--auto-build`) | USB |
| OTA1 | BLE | BLE |
| OTA2 | BLE | BLE |
| Rollback | BLE `OTA_ROLLBACK_PREVIOUS` | USB otadata boot to `ota_0` |
| OTA3 | BLE | USB flash `ota_1` + otadata |
| OTA4 | BLE | USB flash `ota_0` + otadata |
| Factory Reset | BLE `OTA_ROLLBACK_FACTORY` | USB otadata erase (factory boot) |
| OTA5 | BLE | USB flash `ota_0` + otadata |

### Fast mode validates

- Partition boot path and version identity after each hop
- BLE connectivity and boot health diagnostics
- Real BLE OTA transport for OTA1 and OTA2 (including `pending=0`)

### Fast mode does not validate

- BLE OTA transport for OTA3–OTA5
- BLE rollback / factory-reset commands
- NVS rollback diagnostics on simulated rollback steps

Use **full mode** (no `--fast`) as the pre-release deployment gate.

## Success Criteria

The run **passes** only if all checks succeed:

- Factory USB flash completes without `esptool` errors.
- Every OTA transfer completes and the device reboots cleanly (full mode: all five; fast mode: OTA1 and OTA2 only).
- Observed partition sequence matches the table above.
- Each step reports the expected `SW` / `Build` identity (from manifest or auto-build manifest).
- Version evolution table in the report shows PASS for every hop.
- OTA pending verification clears (`GET_OTA_DIAG` shows `pending=0`) after each **BLE** OTA step.
- Manual rollback returns to the previous OTA slot and diagnostics/logs reflect a rollback event (**full mode only**).
- Factory rollback returns to `factory` and BLE remains connectable.
- No `UPDATE_ERROR`, MD5 mismatch, panic, watchdog, brownout, or unexpected partition is observed.
- Diagnostic collection steps complete; failures are recorded as test failures.
- Report labels each step `transport` (`ble`, `serial`, or `serial_sim`).

## Running The Harness

### 1. Preflight only

```powershell
python scripts/run_ota_regression.py `
  --port COM6 `
  --address AA:BB:CC:DD:EE:FF `
  --auto-build `
  --preflight-only
```

### 2. Full regression (auto-build)

```powershell
python scripts/run_ota_regression.py `
  --port COM6 `
  --address AA:BB:CC:DD:EE:FF `
  --auto-build
```

### 2b. Fast regression (recommended for iteration)

```powershell
python scripts/run_ota_regression.py `
  --port COM5 `
  --address AA:BB:CC:DD:EE:FF `
  --auto-build `
  --fast
```

No trust-handshake prompts in fast mode (rollback/factory reset are serial-simulated).

### Manual manifest CLI

```powershell
python scripts/run_ota_regression.py `
  --port COM6 `
  --address AA:BB:CC:DD:EE:FF `
  --manifest scripts/my_ota_manifest.json `
  --fast
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

```powershell
python scripts/run_ota_regression.py --auto-build --dry-run
```

## Operator Actions During The Run

**Full mode** rollback steps require a **trusted BLE session** unless DEV mode is on:

1. With **DEV mode on**, firmware auto-grants BLE trust (no button press). The harness enables DEV mode by default (`--ensure-dev-mode`).
2. With DEV mode off, press any control panel button within the trust timeout (default 120 s).
3. Keep phone apps disconnected from the toilet during the run.
4. Allow ~30 s for cold boot before the first BLE connection after power-on.

**Fast mode** skips BLE trust prompts for rollback and factory reset.

## Reports

Each run writes artifacts under `test-results/ota-regression/`:

- `ota_regression_<timestamp>.json` — machine-readable full report
- `ota_regression_<timestamp>.txt` — human-readable summary with **version evolution** table

The text report includes:

```text
Version evolution:
Step                 Partition  Expected SW              Observed SW              Pass
------------------------------------------------------------------------------------------
flash_factory        factory    4.0.16.0                 4.0.16.0                 PASS
ota1                 ota_0      4.0.17.1-regA            4.0.17.1-regA            PASS
...
```

A checked-in orchestration baseline (dry-run, no hardware) is kept at:

- `test-results/ota-regression/ota_regression_baseline_dryrun.json`
- `test-results/ota-regression/ota_regression_baseline_dryrun.txt`

Re-run `--dry-run` after harness changes and compare step structure to that baseline.

Attach these before approving a deployment:

- JSON report
- Text summary (version evolution table)
- Manifest used (or auto-build `test-builds/ota-regression/manifest.json`)
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
| `Expected SW ... observed` | Stale auto-build artifacts | Delete `test-builds/ota-regression/` and re-run `--auto-build` |

## Manual Spot Checks

Optional commands while debugging a failed step:

```powershell
python read_partitions.py --port COM6
python get_ota_diag.py --address AA:BB:CC:DD:EE:FF
python ota_update.py path\to\ota.bin --address AA:BB:CC:DD:EE:FF
```

## Pre-Deployment Checklist

- [ ] Auto-build regression passed (`--auto-build --fast` for iteration, full mode for gate)
- [ ] Version evolution table shows PASS on every hop
- [ ] Preflight passed on target hardware
- [ ] Full regression passed (`RESULT: PASS`)
- [ ] Report artifacts archived with release notes
- [ ] Production binary matches or exceeds tested OTA behavior
