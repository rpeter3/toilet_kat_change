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

## Channel Responsibilities

1. **Command channel (`...fea0`)**
   - Client writes command strings only.
   - Examples: `TRUST_START`, `TRUST_STATUS`, `TRUST_CANCEL`, `GET_DEV_MODE`, `GET_FLUSH_COUNT`, `SET_DEV_MODE:<0|1>`, `ENABLE_OTA`

2. **Response channel (`...fea4`)**
   - Client reads command responses only.
   - Examples: `TRUST_WAITING`, `TRUST_CONFIRMED`, `TRUST_TIMEOUT`, `TRUST_CANCEL_ACK`, `AUTH_REQUIRED`, `DEV_MODE:0`, `FLUSH_COUNT:<n>`, `SET_DEV_MODE_ACK:1`, etc.

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

Recommended timeout remains 60s.

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
7. Update tests for:
   - trust flow
   - parameter write/read persistence across reconnect
   - auth-required path
   - malformed/unknown response handling

---

## Reference in Repo

Use these as implementation references:
- `toilet_bluetooth_interface.py` (updated client behavior)
- `toilet_kat_change/toilet_kat_change.ino` (current firmware protocol behavior)

