# BLE Trust Handshake Interface Specification

## Purpose

This interface enforces a simple physical trust gate for each BLE connection:

- when a client starts trust flow, the toilet LEDs circle,
- the toilet blocks privileged commands from that connection,
- user presses a control panel button (GPIO2 wake line / SW1) to answer,
- toilet beeps two times, stops LED circling, and marks connection trusted.

## Location

- Firmware command handling: `toilet_kat_change/toilet_kat_change.ino`
- Python BLE tool: `BluetoothNotes/toilet_bluetooth_interface.py`
- Mobile app BLE client: `app/index.tsx`

## Transport and Encoding

- Service UUID: `5636340f-afc7-47b1-b0a8-15bc9d7d29a5`
- Command characteristic UUID: `c327b077-560f-46a1-8f35-b4ab0332fea0`
- Commands are UTF-8 strings over the command characteristic.
- Client writes a command then reads back the response.

## Trust State Model

Per BLE connection:

- `UNTRUSTED` - default after connect
- `WAITING_FOR_BUTTON` - trust flow active, LEDs circling
- `TRUSTED` - GPIO2 wake line accepted, double beep emitted, LEDs stopped
- `TIMEOUT` - trust flow expired, still untrusted

Rules:

- Trust is per connection and is cleared on disconnect.
- Only one wait cycle is active per connection.
- Firmware blocks privileged commands while not trusted.

## Command Contract

### Start trust flow

- Command: `TRUST_START`

Success responses:

- `TRUST_WAITING` (entered waiting state)
- `TRUST_CONFIRMED` (already trusted for current connection)

Error responses:

- `TRUST_START_ERR:BUSY`
- `TRUST_START_ERR:INTERNAL`

Firmware behavior on transition to `TRUST_WAITING`:

- Start LED circling animation.
- Start trust timeout timer.
- Keep command gating active.

### Poll trust status

- Command: `TRUST_STATUS`

Responses:

- `TRUST_WAITING`
- `TRUST_CONFIRMED`
- `TRUST_TIMEOUT`
- `TRUST_STATUS_ERR:INTERNAL`

### Optional cancel

- Command: `TRUST_CANCEL`

Responses:

- `TRUST_CANCEL_ACK`
- `TRUST_CANCEL_ERR:INTERNAL`

Behavior on cancel:

- Stop LED circling.
- Return to untrusted state.

## Physical confirmation via GPIO2 (SW1)

When GPIO2 (`controlPanelWake`) is pulled LOW during `TRUST_WAITING`:

1. Mark connection trusted.
2. Play two beeps.
3. Stop LED circling animation.
4. Return `TRUST_CONFIRMED` on subsequent `TRUST_STATUS`.

Notes:

- Firmware reads ESP32 GPIO2 only (not MCP expander pins). On **CONTROL_PANEL v5**, GPIO2 is the flush switch. On **v6**, either panel button pulls SW1 (GPIO2) low via diode-OR, so either button confirms trust.
- Presses before `TRUST_START` do not count.
- Firmware should debounce button input.

## Command Gating

Privileged commands must return:

- `AUTH_REQUIRED`

until the connection reaches `TRUSTED`.

Suggested privileged commands to gate:

- parameter writes (`update_params` path)
- hardware mutation (`SET_HW_COMPONENT` path)
- hardware apply/rollback commands (`HWCFG_APPLY_CHANGE`, `HWCFG_ROLLBACK_LAST_GOOD`)

## Timing

Recommended defaults:

- trust wait timeout: 15000 ms
- client poll interval: 250-300 ms
- retry attempts per connection: 3

On timeout:

- stop LED circling,
- keep connection untrusted,
- return `TRUST_TIMEOUT` until a new `TRUST_START`.

## Recommended Client Workflow

1. Connect and discover services.
2. Send `TRUST_START`.
3. Show prompt: "Press a control panel button to confirm connection." (On v5 this is flush only; on v6 either button pulls GPIO2.)
4. Poll `TRUST_STATUS` until `TRUST_CONFIRMED` or timeout.
5. Enable write controls only after confirmation.
6. On disconnect, clear local trusted state.

## DEV Mode Trust Bypass

When persisted **DEV mode** is enabled (`GET_DEV_MODE` returns `DEV_MODE:1`):

- New BLE connections are trusted immediately on connect.
- `TRUST_START` and `TRUST_STATUS` return `TRUST_CONFIRMED` without a control-panel button press.
- Privileged commands (parameter writes, OTA rollback, HWCFG apply, etc.) proceed without physical confirmation.

DEV mode is intended for field debugging and automated test harnesses. Production units should run with DEV mode off so the physical trust handshake remains required.

## Error Codes

Standard reason codes used in `*_ERR:<reason>` responses:

- `BUSY`
- `INTERNAL`

Command gating response:

- `AUTH_REQUIRED`


