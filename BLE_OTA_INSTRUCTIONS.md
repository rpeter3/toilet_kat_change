# BLE OTA Instructions

To test on your own end, set `dev` on the first line to `true`.

This project includes a BLE-based OTA implementation in `toilet_kat_change/toilet_kat_change.ino` and a client script `ota_update.py` that can push a compiled firmware image to the device over BLE.

## Quick Summary

- The device advertises a single main BLE service (`5636340f-afc7-47b1-b0a8-15bc9d7d29a5`) that includes the update characteristic from startup. When OTA mode is enabled (via `ENABLE_OTA` command over BLE), the update characteristic accepts OTA commands. When disabled, it rejects with `OTA_DISABLED`.
- `ota_update.py` (requires `bleak`) implements the client side: it scans for the device, sends `ENABLE_OTA`, prepares the device for OTA, streams the firmware in chunks, sends an MD5, and finalizes the update.

## Prerequisites

- Windows (PowerShell) or Linux/macOS with Python 3.8+
- `pip install bleak`
- PlatformIO (for building) — use the VS Code PlatformIO extension or the CLI `platformio` command.

## Build Firmware (PlatformIO)

Open PowerShell in the project root and run:

```powershell
# Build the project (PlatformIO CLI)
.pio\penv\Scripts\platformio.exe run

# The firmware binary will be under .pio\build\<env>\firmware.bin
# Typical path for default env (esp32dev):
$firmware = ".pio\build\esp32dev\firmware.bin"
Write-Host "Firmware path: $firmware"
```

If you use the PlatformIO extension, simply click Build; the firmware path is shown in the build output.

## Flash Over BLE Using the Provided Python Script

1. Install dependencies (in PowerShell):

```powershell
python -m pip install --user bleak
```

2. Run the OTA script. The script can scan for the device name `ESP32 Toilet` or accept a BLE address.

Example (scan and update):

```powershell
python ota_update.py ".pio\build\esp32dev\firmware.bin"
```

Example (provide address explicitly):

```powershell
python ota_update.py ".pio\build\esp32dev\firmware.bin" --address "AA:BB:CC:DD:EE:FF"
```

## Tips & Troubleshooting

- **Enter OTA mode:** Send `ENABLE_OTA` to the command characteristic (`c327b077-560f-46a1-8f35-b4ab0332fea0`) over BLE. The response `ENABLE_OTA_ACK` is read from the response characteristic (`c327b077-560f-46a1-8f35-b4ab0332fea4`). The device opens an OTA window (default 1 minute).
- If the script fails to connect, run the scanner-only flow or use the `scanner.py` file in this directory:

```powershell
# Run the scanner in Python REPL or temporary script to confirm device name/address
python - <<'PY'
from bleak import BleakScanner
import asyncio

a = asyncio.get_event_loop().run_until_complete(BleakScanner.discover(timeout=5.0))
for d in a:
    print(d)
PY
```

- **BLE MTU and chunk size:** The script uses 512 bytes per chunk by default; if you encounter write failures, lower this to ~200.
- **Firmware size:** If the firmware is larger than the OTA partition (4.5 MB per slot), the device may reject the update — the script prints firmware size and warns.

## Partition Table

**Use the partition table in `toilet_kat_change/partitions.csv`.** Any other setup will result in error. The code has been tested with this layout on ESP32.

- OTA partitions: `ota_0` and `ota_1` are 0x480000 (4.5 MB) each
- Max firmware size: 4.5 MB

---

# BLE OTA Protocol Specification (for APP)

This section documents the complete BLE OTA protocol so the APP can be implemented to specification.

## BLE UUIDs

| Purpose | UUID |
|---------|------|
| Main service | `5636340f-afc7-47b1-b0a8-15bc9d7d29a5` |
| Command (write ENABLE_OTA) | `c327b077-560f-46a1-8f35-b4ab0332fea0` |
| Response (read ENABLE_OTA_ACK) | `c327b077-560f-46a1-8f35-b4ab0332fea4` |
| Update (write OTA commands, read status) | `c327b077-560f-46a1-8f35-b4ab0332fea3` |
| Version (subscribe for UPDATE_PROGRESS) | `c327b077-560f-46a1-8f35-b4ab0332fea2` |

## Device Name

- Advertised name: `ESP32 Toilet`

## OTA Command Sequence

1. **Enable OTA:** Write `ENABLE_OTA` to command characteristic (fea0). Read `ENABLE_OTA_ACK` from response characteristic (fea4).
2. **Wait:** ~4 seconds for device to reset OTA state.
3. **Prepare:** Write `PREPARE_UPDATE` to update characteristic (fea3). Read until `UPDATE_PREPARED` (or `UPDATE_BLOCKED`).
4. **Start:** Write `START_UPDATE` to fea3. Read until `UPDATE_STARTED`.
5. **Size:** Write `SIZE:<bytes>` to fea3 (e.g. `SIZE:524288`).
6. **Stream:** Send raw firmware chunks (512 bytes each) to fea3.
7. **MD5:** Write `MD5:<32-hex-chars>` to fea3 (e.g. `MD5:a1b2c3d4e5f6...`).
8. **Progress:** Subscribe to version characteristic (fea2) for `UPDATE_PROGRESS:N` notifications.
9. **Finalize:** Write `FINALIZE_UPDATE` to fea3. Read until `UPDATE_COMPLETE` (device reboots).

## Metadata Formats

| Type | Format | Example |
|------|--------|---------|
| Size | `SIZE:<decimal>` | `SIZE:524288` |
| MD5 | `MD5:<32 lowercase hex chars>` | `MD5:a1b2c3d4e5f6789012345678abcdef01` |

## Status and Error Codes

| Code | Meaning |
|------|---------|
| `UPDATE_PREPARED` | Device ready for firmware stream |
| `UPDATE_BLOCKED` | Preparation failed (e.g. flush in progress) |
| `UPDATE_STARTED` | Ready to receive chunks |
| `UPDATE_NOT_PREPARED` | START_UPDATE sent before PREPARE_UPDATE |
| `UPDATE_VALIDATING` | MD5 received, validating |
| `UPDATE_NOT_READY` | FINALIZE_UPDATE sent before validation complete |
| `UPDATE_VALIDATION_FAILED` | MD5 mismatch |
| `UPDATE_FINALIZING` | Finalizing and rebooting |
| `UPDATE_COMPLETE` | Success; device rebooting |
| `UPDATE_ERROR:<code>` | Error (e.g. `TIMEOUT_RECEIVE`, `WRITE_FAILED`) |
| `OTA_DISABLED` | OTA commands rejected; enable OTA first |
