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
- Parameter read characteristic (read 22-float CSV): `c327b077-560f-46a1-8f35-b4ab0332fea5`
- Parameter write characteristic (write 22-float CSV): `c327b077-560f-46a1-8f35-b4ab0332fea6`

---

## Payload Format (No Framing Required)

This spec does **not** require framed payloads (e.g. `0x7E` + length + payload). All channels use plain UTF-8 text:

- **Commands**: Plain strings written to `...fea0`
- **Responses**: Plain strings read from `...fea4`
- **Parameters**: Plain 22-value CSV on `...fea5` / `...fea6`
- **Serial stream** (`...fea1`): Firmware sends raw text notifications; client writes `START_SERIAL` / `STOP_SERIAL` as plain strings (or optionally framed for compatibility with alternate implementations)

Client implementations may use payload chunking to respect BLE MTU limits when writing large payloads, but the protocol itself is plain text. See Design_Decision_Rationale.md for why the Python client chunks writes.

---

## Channel Responsibilities

1. **Command channel (`...fea0`)**
   - Client writes command strings only.
   - Examples: `TRUST_START`, `TRUST_STATUS`, `TRUST_CANCEL`, `GET_DEV_MODE`, `GET_FLUSH_COUNT`, `GET_BATTERY`, `SET_DEV_MODE:<0|1>`, `ENABLE_OTA`, `GET_LOGS`, `GET_LOGS:<offset>`

2. **Response channel (`...fea4`)**
   - Client reads command responses only.
   - Examples: `TRUST_WAITING`, `TRUST_CONFIRMED`, `TRUST_TIMEOUT`, `TRUST_CANCEL_ACK`, `AUTH_REQUIRED`, `DEV_MODE:0`, `FLUSH_COUNT:<n>`, `BATTERY:<n>`, `SET_DEV_MODE_ACK:1`, `LOGS:<offset>:<length>:<data>`, `LOGS_END`, etc.

3. **Parameter read channel (`...fea5`)**
   - Client reads current parameter snapshot only.
   - Payload format: exactly 22 comma-separated numeric values.

4. **Parameter write channel (`...fea6`)**
   - Client writes 22-value CSV parameter payload only.
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
- must be exactly 22 fields
- every field must parse to float
- if invalid, treat as read failure (do not silently substitute defaults)

## 3) Parameter writes

Write 22-value CSV to parameter write characteristic (`...fea6`), then read response from response characteristic (`...fea4`).

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

- **type**: `reset` | `runtime` | `eeprom` | `ota` | `mcp`
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
| mcp     | MCP23017 I2C expander init failed                |

### App use case

Use for "Export logs" / "Get support" flows: retrieve full log, concatenate chunks, then share as text (email, ticket, etc.).

---

## Battery Level (GET_BATTERY)

Command `GET_BATTERY` retrieves the current battery charge level as a percentage. No trust handshake required.

### Protocol

- **Command**: `GET_BATTERY`
- **Response**: `BATTERY:<n>` where `<n>` is 0–100 (integer percentage)
- **Flow**: Client writes `GET_BATTERY` to command characteristic (`...fea0`), reads response from response channel (`...fea4`).
- **Parsing**: Extract the integer from `BATTERY:NN` (e.g. `BATTERY:85` → 85). Regex: `^BATTERY[:_]?\s*(\d+)\s*%?$/i` or equivalent.

### App use case

Use for battery status indicator in the UI. Poll periodically (e.g. on connect and every 30–60 seconds) to display charge level or low-battery warning.

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

Write `CHECK_VERSION` to command channel; read response from response channel. Returns `HW:<hw_ver>|SW:<sw_ver>|Build:<date>|Desc:<desc>` (also exposed on version characteristic `...fea2`).

### Breaking change

Component `FACTORY_SOFTWARE_VERSION_NUMBER` was renamed to `SOFTWARE_VERSION_NUMBER`. Apps using the old name will receive `HW_COMPONENT_ERR:UNKNOWN_COMPONENT`.

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
5. Add strict parser for 22-float parameter payload.
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

