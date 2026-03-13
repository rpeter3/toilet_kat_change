---
name: ""
overview: ""
todos: []
isProject: false
---

# BLE Spec Conformance Plan (Firmware + Python)

## Overview

Refactor both the firmware ([toilet_kat_change.ino](toilet_kat_change/toilet_kat_change.ino)) and Python client ([toilet_bluetooth_interface.py](toilet_bluetooth_interface.py)) to conform to [BLE_APP_MIGRATION_SPEC.md](toilet_kat_change/bluetooth interface app notes/BLE_APP_MIGRATION_SPEC.md). The spec splits the mixed fea0 characteristic into separate channels: command (fea0), response (fea4), param read (fea5), param write (fea6).

---

## Part A: Firmware Updates (toilet_kat_change.ino)

### A1. Add UUID Constants

Add after line 54:

```cpp
#define RESPONSE_CHARACTERISTIC_UUID "c327b077-560f-46a1-8f35-b4ab0332fea4"
#define PARAM_READ_CHARACTERISTIC_UUID "c327b077-560f-46a1-8f35-b4ab0332fea5"
#define PARAM_WRITE_CHARACTERISTIC_UUID "c327b077-560f-46a1-8f35-b4ab0332fea6"
```

Add global pointers:

```cpp
BLECharacteristic * response_characteristic;
BLECharacteristic * param_read_characteristic;
BLECharacteristic * param_write_characteristic;
```

### A2. Create New Characteristics

In BLE setup (after serial_characteristic, ~line 1089), create three new characteristics:

- **Response (fea4)**: READ only. Firmware writes command responses here; client reads.
- **Param read (fea5)**: READ only. Holds 22-float CSV snapshot; updated on param change.
- **Param write (fea6)**: WRITE only. Client writes 22-float CSV; onWrite callback applies params and writes ack to response characteristic.

### A3. Refactor Command Channel (fea0)

Change `blue_characteristic` (fea0) to WRITE-only for commands. Remove READ property and onRead callback.

- **onWrite**: When client writes a command string, dispatch to existing handlers (HWCFG, ENABLE_OTA, GET_DEV_MODE, GET_FLUSH_COUNT, SET_DEV_MODE, GET_HW_*, SET_HW_*, etc.).
- **Response routing**: Instead of `pCharacteristic->setValue(...)` (which wrote back to fea0), call `response_characteristic->setValue(...)` so client reads from fea4.

### A4. Param Read Characteristic (fea5)

- Create with READ only.
- Implement onRead callback (or keep value updated): return 22-float CSV.
- Alternatively: no onRead; keep `setValue()` updated whenever params change so client read returns current snapshot.

### A5. Param Write Characteristic (fea6)

- Create with WRITE only.
- onWrite callback: parse 22-float CSV. If invalid (wrong count, parse error), write `PARAM_WRITE_ERR:BAD_FORMAT` to response characteristic.
- If flush in progress: write `PARAM_UPDATE_BLOCKED_FLUSH` to response characteristic.
- If trust required and not trusted: write `AUTH_REQUIRED` to response characteristic (if trust is implemented).
- On success: apply params, save EEPROM, write `PARAM_WRITE_ACK` to response characteristic.

### A6. Response Characteristic (fea4)

- Create with READ only.
- All command handlers and param-write handler write their response string to this characteristic via `response_characteristic->setValue(...)`.
- Client reads from fea4 after each command write to fea0 or param write to fea6.

### A7. GET_LOGS (Optional)

If firmware supports persistent error logs, add GET_LOGS/GET_LOGS: handler. Write `LOGS:<offset>:<length>:<data>` or `LOGS_END` to response characteristic. Chunk size up to 450 bytes.

### A8. Trust Handshake (Optional)

If trust is required for privileged ops, add TRUST_START, TRUST_STATUS, TRUST_CANCEL handlers. Write TRUST_WAITING, TRUST_CONFIRMED, TRUST_TIMEOUT, TRUST_CANCEL_ACK to response characteristic.

---

## Part B: Python Updates (toilet_bluetooth_interface.py)

### B1. Add UUID Constants

```python
COMMAND_CHARACTERISTIC_UUID = "c327b077-560f-46a1-8f35-b4ab0332fea0"
RESPONSE_CHARACTERISTIC_UUID = "c327b077-560f-46a1-8f35-b4ab0332fea4"
PARAM_READ_CHARACTERISTIC_UUID = "c327b077-560f-46a1-8f35-b4ab0332fea5"
PARAM_WRITE_CHARACTERISTIC_UUID = "c327b077-560f-46a1-8f35-b4ab0332fea6"
```

Rename `CHARACTERISTIC_UUID` to `COMMAND_CHARACTERISTIC_UUID` where used for commands.

### B2. Command/Response Helper

Replace `_send_command_and_read_response`:

- Write UTF-8 command to `COMMAND_CHARACTERISTIC_UUID` (fea0)
- Read response from `RESPONSE_CHARACTERISTIC_UUID` (fea4)
- Validate non-empty; return `Optional[str]`

Update all callers: get_dev_mode_status, set_dev_mode, get_flush_count, hwcfg_*, get_hw_component, set_hw_component.

### B3. Parameter Read

Refactor `read_current_params`:

- Read from `PARAM_READ_CHARACTERISTIC_UUID` (fea5) only
- Strict parser: exactly 22 fields, each parses to float; on invalid return `{}` and report failure (no silent defaults)

### B4. Parameter Write

Refactor `update_params`:

- Write 22-value CSV to `PARAM_WRITE_CHARACTERISTIC_UUID` (fea6)
- Read response from `RESPONSE_CHARACTERISTIC_UUID` (fea4)
- Handle: PARAM_WRITE_ACK, AUTH_REQUIRED, PARAM_UPDATE_BLOCKED_FLUSH, PARAM_WRITE_ERR:BAD_FORMAT

### B5. GET_LOGS

Add `get_logs()`: send GET_LOGS/GET_LOGS:, read from fea4, loop until LOGS_END.

### B6. Trust Handshake (Optional)

Add trust_start, trust_status, trust_cancel; guard privileged ops; handle AUTH_REQUIRED.

### B7. Compatibility Fallback

Add `BLE_USE_LEGACY_PROTOCOL=1` env var to fall back to fea0-only behavior when connecting to pre-refactor firmware.

---

## Implementation Order

1. **Firmware first**: Add fea4, fea5, fea6; refactor fea0 to write-only; route responses to fea4; split param read/write.
2. **Python second**: Add UUIDs; refactor command/response helper; param read (fea5); param write (fea6); GET_LOGS; trust (optional); legacy fallback.

---

## Files to Modify

- [toilet_kat_change/toilet_kat_change.ino](toilet_kat_change/toilet_kat_change.ino)
- [toilet_bluetooth_interface.py](toilet_bluetooth_interface.py)

