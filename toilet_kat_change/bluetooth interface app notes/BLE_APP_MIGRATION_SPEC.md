# BLE App Migration Spec (Post-Refactor)

This document describes the Bluetooth interface changes that the application must adopt to remain compatible with firmware.

## Scope

- Firmware source: `toilet_kat_change/toilet_kat_change.ino`
- Python reference client: `toilet_bluetooth_interface.py`

## Why This Changed

The BLE protocol was split into separate channels to prevent command/status responses from being mistaken for parameter payloads.

Before:
- One characteristic mixed command writes, command responses, and parameter reads.

Now:
- Command traffic and parameter traffic are separated.

---

## BLE UUIDs (Current Contract)

- Service UUID: `5636340f-afc7-47b1-b0a8-15bc9d7d29a5`
- Command characteristic (write commands): `c327b077-560f-46a1-8f35-b4ab0332fea0`
- Serial stream characteristic: `c327b077-560f-46a1-8f35-b4ab0332fea1`
- Response characteristic (read command responses): `c327b077-560f-46a1-8f35-b4ab0332fea4`
- Parameter read characteristic (read 30-float CSV): `c327b077-560f-46a1-8f35-b4ab0332fea5`
- Parameter write characteristic (write 30-float CSV): `c327b077-560f-46a1-8f35-b4ab0332fea6`

---

## Payload Format (No Framing Required)

This spec does **not** require framed payloads (e.g. `0x7E` + length + payload). All channels use plain UTF-8 text:

- **Commands**: Plain strings written to `...fea0`
- **Responses**: Plain strings read from `...fea4`
- **Parameters**: Plain 30-value CSV on `...fea5` / `...fea6`
- **Serial stream** (`...fea1`): Firmware sends raw text notifications; client writes `START_SERIAL` / `STOP_SERIAL` as plain strings (or optionally framed for compatibility with alternate implementations)

Client implementations may use payload chunking to respect BLE MTU limits when writing large payloads, but the protocol itself is plain text. See Design_Decision_Rationale.md for why the Python client chunks writes.

---

## Channel Responsibilities

1. **Command channel (`...fea0`)**
   - Client writes command strings only.
   - Examples: `TRUST_START`, `TRUST_STATUS`, `TRUST_CANCEL`, `GET_DEV_MODE`, `GET_FLUSH_COUNT`, `GET_BATTERY`, `SET_DEV_MODE:<0|1>`, `ENABLE_OTA`, `OTA_ROLLBACK_PREVIOUS`, `OTA_ROLLBACK_FACTORY`, `GET_LOGS`, `GET_LOGS:<offset>`, `GET_OTA_DIAG`

2. **Response channel (`...fea4`)**
   - Client reads command responses only.
   - Examples: `TRUST_WAITING`, `TRUST_CONFIRMED`, `TRUST_TIMEOUT`, `TRUST_CANCEL_ACK`, `AUTH_REQUIRED`, `DEV_MODE:0`, `FLUSH_COUNT:<n>`, `BATTERY:<n>`, `SET_DEV_MODE_ACK:1`, `OTA_ROLLBACK_ACK:REBOOTING`, `OTA_ROLLBACK_FACTORY_ACK:REBOOTING`, `LOGS:<offset>:<length>:<data>`, `LOGS_END`, etc.

3. **Parameter read channel (`...fea5`)**
   - Client reads current parameter snapshot only.
   - Payload format: exactly 30 comma-separated numeric values.

4. **Parameter write channel (`...fea6`)**
   - Client writes 30-value CSV parameter payload only.
   - Write result is read from response channel (`...fea4`), expected `PARAM_WRITE_ACK` on success.

---

## Required App Behavior Changes

## 1) Command/response helper

Implement one helper that:
- writes UTF-8 command to command characteristic (`...fea0`)
- reads response from response characteristic (`...fea4`)
- validates non-empty response

Do not read command responses from the command characteristic anymore.

## 2) Parameter reads

Read parameters directly from parameter read characteristic (`...fea5`).

Validation rules:
- split by comma
- must be exactly 30 fields
- every field must parse to float
- if invalid, treat as read failure (do not silently substitute defaults)

## 3) Parameter writes

Write 30-value CSV to parameter write characteristic (`...fea6`), then read response from response characteristic (`...fea4`).

Expected responses:
- `PARAM_WRITE_ACK` -> success
- `AUTH_REQUIRED` -> trust handshake not complete
- `PARAM_UPDATE_BLOCKED_FLUSH` -> firmware blocked write during flush
- `PARAM_WRITE_ERR:BAD_FORMAT` -> malformed payload
- any other response -> treat as failure and report raw response

---

## Error Log Retrieval (GET_LOGS)

Command `GET_LOGS` retrieves persistent error logs for diagnostics. No trust handshake required (works even when trust flow cannot be completed, e.g. broken flush button).

### Protocol

- **Command**: `GET_LOGS` or `GET_LOGS:<offset>`
- **Response** (chunked): `LOGS:<offset>:<length>:<data>` or `LOGS_END`
- **Flow**: Client sends `GET_LOGS` (or `GET_LOGS:0`), reads response from response channel (`...fea4`). If response starts with `LOGS:`, parse offset and length, append data to buffer, then send `GET_LOGS:<next_offset>` where next_offset = offset + length. Repeat until response is `LOGS_END`.
- **Chunk size**: Up to 450 bytes per chunk. Response format `LOGS:offset:length:data` stays under 512 bytes total.

### Log format (per line)

Each line is CSV: `type,timestamp,code,msg[,context]`

- **type**: `reset` | `runtime` | `eeprom` | `ota` | `ota_boot` | `ota_boot_fail` | `ota_rollback` | `ota_boot_ok` | `ota_spiffs_capture` | `mcp` | `boot_status` | `reset_forensics`
- **timestamp**: millis at log time
- **code**: numeric (0–7 for runtime; 0 for others)
- **msg**: short string (e.g. `panic`, `motor_timeout`, `low_battery`, `MCP_UNAVAILABLE`)
- **context** (optional): `,step=N,cut=N,feed=N,bat=X,temp=X,m1A=X,heaterA=X,fan=off|forward|reverse`

### Log types

| type    | When logged                                      |
|---------|---------------------------------------------------|
| reset   | Unexpected boot (panic, WDT, brownout, SDIO)     |
| runtime | Motor timeout (1), low battery (2), heater overheat (3), motor fault (4), heater current fail (5), heater max wall (6) |
| eeprom  | EEPROM invalid (7) with reason string            |
| ota     | OTA update failure with reason                   |
| ota_boot | Every boot while running last OTA target partition |
| ota_boot_fail | Abnormal reset while on OTA target partition  |
| ota_rollback | Rollback triggered (auto, validation, manual) |
| ota_boot_ok | OTA verification completed (2 consecutive good boots) |
| ota_spiffs_capture | NVS log tail merged back into SPIFFS after rollback |
| mcp     | MCP23017 I2C expander init failed                |

### App use case

Use for "Export logs" / "Get support" flows: retrieve full log, concatenate chunks, then share as text (email, ticket, etc.). Also send `GET_OTA_DIAG` before `GET_LOGS` to include the NVS OTA rollback snapshot (see below).

---

## OTA Boot Diagnostics (GET_OTA_DIAG)

Command `GET_OTA_DIAG` returns a compact NVS snapshot of the last OTA boot/rollback event. No trust handshake required.

### Protocol

- **Command**: `GET_OTA_DIAG`
- **Response**: single line on response channel (`...fea4`), e.g.
  `OTA_DIAG:V1|pending=0|attempts=0|streak=0|target=255|last_trigger=auto|from=ota_1|to=ota_0|reset=task_wdt|md5=abcd1234|size=1234567|reason=boot_attempts_exceeded|seq=12|tail_len=512|log_tail=...`
- **Empty**: `OTA_DIAG:EMPTY` when no OTA diagnostic record exists
- **Flow**: Client writes `GET_OTA_DIAG` to command characteristic (`...fea0`), reads one response from response channel

### App use case

Prepend `GET_OTA_DIAG` output to support log exports alongside chunked `GET_LOGS` data.

---

## Battery Level (GET_BATTERY)

Command `GET_BATTERY` retrieves the current **usable** battery charge level as a percentage. No trust handshake required.

### Protocol

- **Command**: `GET_BATTERY`
- **Response**: `BATTERY:<n>,<v>V` where `<n>` is 0–100 (integer **usable** percentage from load-based assessment) and `<v>` is idle VMON battery voltage for diagnostics (e.g. `BATTERY:85,12.34V`). Legacy firmware may respond with `BATTERY:<n>` only or return idle-based percentages.
- **Assessment cache**: firmware refreshes the load-based assessment when stale (~2 min) and the device is idle; dual-button battery display and flush preflight always force a fresh assessment.
- **During OTA transfer**: firmware returns the last cached usable reading (or idle voltage with level 0 if no cache); clients should not poll battery during OTA.
- **Flow**: Client writes `GET_BATTERY` to command characteristic (`...fea0`), reads response from response channel (`...fea4`).
- **Parsing**: Extract percentage and optional voltage from `BATTERY:NN,VVV` (e.g. `BATTERY:85,12.34V` → 85%, 12.34 V). Regex: `^BATTERY[:_]?\s*(\d+)\s*%?(?:[,;\s]+([\d.]+)\s*V?)?$/i` or equivalent.

### App use case

Use for battery status indicator in the UI. Poll periodically (e.g. on connect and every 30–60 seconds) to display charge level or low-battery warning. Values ≤ `batteryThreshold` (default 7%) indicate flush is blocked on-device.

### Dual-button battery assess report (serial stream)

When the user presses both control-panel buttons while idle, firmware runs a load-based assessment and prints a multi-line report to USB Serial. If BLE serial streaming is enabled (`START_SERIAL` / trust path), the same lines are sent on the serial notify characteristic.

Format (one line per row, MTU-safe):

```
=== BATTERY ASSESS ===
tag=display vIdle=11.49V capDuty=100 steps=2
step duty=64 vLoad=11.46V sag=0.03V adc=149 I=0.06A
step duty=100 vLoad=10.83V sag=0.65V adc=175 I=0.07A
summary vLoadWorst=10.83V sagWorst=0.65V usable=0% passed=0 flushAllowed=0
limits vLoadMin=11.20V sagMax=0.40V threshold=7%
=== END BATTERY ASSESS ===
```

Followed by a one-line operator summary: `Battery: idle=… usable=… vLoadWorst=… flushOK=…`.

---

## Trust Handshake (Unchanged Commands, New Response Path)

Use existing trust commands on command channel:
- `TRUST_START`
- `TRUST_STATUS`
- `TRUST_CANCEL`

Read trust responses on response channel:
- `TRUST_WAITING`
- `TRUST_CONFIRMED`
- `TRUST_TIMEOUT`
- `TRUST_CANCEL_ACK`

Physical confirmation: firmware reads ESP32 GPIO2 (SW1 wake line), not MCP pins. On v5 only flush pulls GPIO2; on v6 either control-panel button may confirm via diode-OR.

Recommended timeout remains 60s.

---

## Hardware Matrix Interface

Commands for querying and updating hardware component metadata (versions, install dates). Write commands to `...fea0`, read responses from `...fea4`.

### Commands

| Command | Description |
|---------|-------------|
| `GET_HW_MATRIX` | Returns `HW_COMPONENTS:` + comma-separated component names |
| `GET_HW_COMPONENT:<name>` | Returns `HW_COMPONENT:` + name + version + description + install_date (pipe-separated) |
| `SET_HW_COMPONENT:<name>:<version>:<install_date>:<description>` | Update component (trust required). `SOFTWARE_VERSION_NUMBER` is read-only; returns `SET_HW_COMPONENT_ERR:READ_ONLY` |

### Component list

CONTROL_PANEL, HEATING_ELEMENT, MAIN_CIRCUIT_BOARD, VACUUM_FAN, FEED_MOTOR, MECHANISM_MOTOR, THERMISTOR, BATTERY, FACTORY_SOFTWARE_DATE, SOFTWARE_VERSION_NUMBER

### CHECK_VERSION

`CHECK_VERSION` is handled by the OTA/update command path in current firmware, not the normal command/response path. It updates/notifies the version characteristic `...fea2` with `HW:<hw_ver>|SW:<sw_ver>|Build:<date>|Desc:<desc>`.

### Breaking change

Component `FACTORY_SOFTWARE_VERSION_NUMBER` was renamed to `SOFTWARE_VERSION_NUMBER`. Apps using the old name will receive `HW_COMPONENT_ERR:UNKNOWN_COMPONENT`.

---

## Manual OTA Rollback

Command `OTA_ROLLBACK_PREVIOUS` intentionally switches the ESP32 boot partition back to the previous OTA firmware image and reboots. This is a support/admin recovery action, not a normal app flow.

This is different from `HWCFG_ROLLBACK_LAST_GOOD`:

- `OTA_ROLLBACK_PREVIOUS` rolls back the firmware image to the previous OTA partition.
- `HWCFG_ROLLBACK_LAST_GOOD` rolls back hardware/profile configuration only.

### Protocol

- **Command**: `OTA_ROLLBACK_PREVIOUS`
- **Channel**: write command to `...fea0`, read response from `...fea4`
- **Trust**: required
- **When allowed**: device must be idle, not flushing, not homing/running mechanism motors, and not in an OTA transfer/finalize state

### Responses

- `OTA_ROLLBACK_ACK:REBOOTING` -> rollback boot partition was selected; device will reboot shortly
- `OTA_ROLLBACK_ERR:AUTH_REQUIRED` -> trust handshake is not complete
- `OTA_ROLLBACK_ERR:BUSY_OTA` -> OTA transfer/finalize is active
- `OTA_ROLLBACK_ERR:BUSY_FLUSH` -> flush or critical motor action is active
- `OTA_ROLLBACK_ERR:NO_ROLLBACK_INFO` -> firmware has no recorded previous partition
- `OTA_ROLLBACK_ERR:NO_ROLLBACK_PARTITION` -> recorded previous partition could not be found
- `OTA_ROLLBACK_ERR:SET_BOOT_FAILED` -> ESP-IDF failed to set the rollback boot partition
- `OTA_ROLLBACK_ERR:NVS_OPEN_FAILED` -> rollback metadata namespace could not be opened

Apps should show an explicit confirmation before sending this command, for example a typed confirmation in support tools. Do not automatically send this command after transient BLE failures.

---

## Manual OTA Factory Rollback

Command `OTA_ROLLBACK_FACTORY` switches the ESP32 boot partition to the **factory** app partition and reboots. This is a support/admin recovery action for field units that need to return to the factory-flashed firmware image, not the NVS-recorded previous OTA slot.

This is different from `OTA_ROLLBACK_PREVIOUS`:

- `OTA_ROLLBACK_PREVIOUS` rolls back to the partition recorded in NVS at OTA `PREPARE_UPDATE` (usually the last running OTA slot before the update).
- `OTA_ROLLBACK_FACTORY` always targets the `factory` partition from `partitions.csv` (offset `0x10000`).

### Protocol

- **Command**: `OTA_ROLLBACK_FACTORY`
- **Channel**: write command to `...fea0`, read response from `...fea4`
- **Trust**: required
- **When allowed**: device must be idle, not flushing, not homing/running mechanism motors, and not in an OTA transfer/finalize state

### Responses

- `OTA_ROLLBACK_FACTORY_ACK:REBOOTING` -> factory boot partition was selected; device will reboot shortly
- `OTA_ROLLBACK_FACTORY_ERR:AUTH_REQUIRED` -> trust handshake is not complete
- `OTA_ROLLBACK_FACTORY_ERR:BUSY_OTA` -> OTA transfer/finalize is active
- `OTA_ROLLBACK_FACTORY_ERR:BUSY_FLUSH` -> flush or critical motor action is active
- `OTA_ROLLBACK_FACTORY_ERR:NO_FACTORY_PARTITION` -> factory partition missing from partition table
- `OTA_ROLLBACK_FACTORY_ERR:ALREADY_ON_FACTORY` -> device is already running from factory
- `OTA_ROLLBACK_FACTORY_ERR:INVALID_FACTORY_IMAGE` -> factory slot is empty or corrupt (image validation failed)
- `OTA_ROLLBACK_FACTORY_ERR:SET_BOOT_FAILED` -> ESP-IDF failed to set the factory boot partition

`GET_OTA_DIAG` should record the rollback with `to=factory` and `reason=factory_request` (BLE) or `hardware_factory_hold` (hardware path).

Apps should show an explicit confirmation before sending this command. There is no customer-app UI for this command in the current phase; support tools may expose it for lab use.

---

## HWCFG Profile Interface

HWCFG commands manage transactional hardware/profile changes. Write commands to `...fea0`, read responses from `...fea4`.

### Storage

Current firmware stores HWCFG active and last-known-good records in SPIFFS files, not NVS, so production units can receive this change by OTA without changing partition sizes. On first boot after update, firmware may read valid legacy NVS HWCFG records and copy them into SPIFFS. Future HWCFG writes go to SPIFFS only.

### Commands

| Command | Description |
|---------|-------------|
| `HWCFG_GET_CAPS` | Returns supported HWCFG capabilities |
| `HWCFG_GET_ACTIVE_CONFIG` | Returns active component/profile state |
| `HWCFG_GET_LAST_GOOD_CONFIG` | Returns last-known-good component/profile state |
| `HWCFG_PROFILE_LIST` | Returns stored profile identifiers |
| `HWCFG_PROFILE_GET:<component>|version=<ver>` | Returns profile details for a component/version |
| `HWCFG_PROFILE_PUT:<profile_id>|component=<name>|version=<ver>|<params>` | Stores or updates a profile; trust required |
| `HWCFG_VALIDATE_CHANGE:<component>|new_version=<ver>` | Validates whether a component can change to a version |
| `HWCFG_APPLY_CHANGE:<component>|new_version=<ver>|install_date=<YYYY-MM-DD>|desc=<text>` | Transactionally applies a validated change; trust required |
| `HWCFG_ROLLBACK_LAST_GOOD` | Restores last-known-good HWCFG state; trust required |

### Response examples

- `HWCFG_CAPS:V1|PROFILE_STORE|TXN_APPLY|ROLLBACK`
- `HWCFG_ACTIVE:<component>=<ver>;...|profile_id=<id>|validated=1`
- `HWCFG_LAST_GOOD:<component>=<ver>;...|profile_id=<id>`
- `HWCFG_PROFILE:<profile_id>|component=<name>|version=<ver>|<params>`
- `HWCFG_VALIDATE_OK:<component>|version=<ver>|profile_id=<id>`
- `HWCFG_VALIDATE_ERR:<reason>`
- `HWCFG_APPLY_ACK:<component>|version=<ver>`
- `HWCFG_APPLY_ERR:<reason>`
- `HWCFG_ROLLBACK_ACK`
- `HWCFG_ROLLBACK_ERR:<reason>`

Persistence-related failures such as `HWCFG_VALIDATE_ERR:PERSIST_FAIL`, `HWCFG_APPLY_ERR:PERSIST_FAIL`, or `HWCFG_ROLLBACK_ERR:PERSIST_FAIL` indicate firmware could not write the SPIFFS-backed HWCFG store. Apps should surface the raw response and suggest exporting logs.

---

## Error Handling Requirements

- Never assume a response belongs to params unless it came from parameter read characteristic.
- Never default to material presets when parsing fails.
- Surface the raw firmware response in UI/logs for unknown responses.
- Guard privileged actions behind trust state and handle `AUTH_REQUIRED` by resetting local trust state.

---

## Migration Checklist for App Agent

1. Add UUID constants for response, param-read, param-write channels.
2. Refactor all command APIs to write `...fea0` and read `...fea4`.
3. Refactor parameter read API to use `...fea5` only.
4. Refactor parameter write API to write `...fea6` + read ack from `...fea4`.
5. Add strict parser for 30-float parameter payload.
6. Update reconnect flow to read current params from `...fea5` after trust confirmation.
7. Add GET_LOGS flow for "Export logs" / "Get support": chunked retrieval, concatenate, present/share as text.
8. Update tests for:
   - trust flow
   - parameter write/read persistence across reconnect
   - auth-required path
   - malformed/unknown response handling
   - GET_LOGS chunked retrieval and LOGS_END handling

---

## Reference in Repo

Use these as implementation references:
- `toilet_bluetooth_interface.py` (updated client behavior)
- `toilet_kat_change/toilet_kat_change.ino` (current firmware protocol behavior)

