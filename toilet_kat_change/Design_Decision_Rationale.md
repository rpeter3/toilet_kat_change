# Design Decision Rationale

## About this document

This document captures the reasoning behind the functionality in this project so we can understand why each decision was made and evaluate future changes against that rationale.

## EEPROM initialization strategy: detect virgin magic at runtime

When the firmware sees a virgin EEPROM header (`0xFFFF`), it now writes the in-firmware default parameters on first boot instead of requiring those parameters to be preflashed into EEPROM during manufacturing.

Rationale:

- Simpler production flow: only firmware flashing is required; no separate EEPROM data image generation, versioning, and flashing step.
- Better maintainability: default parameter changes live in one place (firmware source), reducing mismatch risk between code and factory images.
- Safer updates: if storage is erased during reflash or board replacement, the device can self-initialize to valid defaults automatically.
- Clear fault separation: virgin/uninitialized storage is treated as expected first-boot state, while non-virgin invalid magic is treated as genuine corruption and flagged as an error.
- Lower operational risk at scale: fewer manufacturing steps and fewer opportunities for process drift or stale parameter images.

## EEPROM error handling for a safety-critical toilet workflow

When EEPROM data is corrupt (non-virgin invalid magic), firmware rewrites default parameters so the toilet can continue operating, but keeps a latched EEPROM warning state (`ERROR_CODE = 7`) until parameters are successfully updated over Bluetooth.

Rationale:

- Product continuity first: for this device, degraded operation is preferable to full lockout so users can still use the toilet.
- Visible persistent alert: while the device is awake, the EEPROM warning LED remains illuminated to clearly signal maintenance is required.
- Controlled latch clearing: the EEPROM warning is only cleared after a verified BLE parameter write, preventing silent recovery without operator action.
- Wake-time awareness: after each wake from sleep, a 10-second alert pause keeps the warning visible before normal control resumes.
- Balanced behavior: users retain functionality while still being prompted to correct parameters for optimal heat-seal performance across supported bag materials.

## EEPROM error latch persistence over power cycles

The EEPROM error latch (`eepromErrorState`) is currently runtime-only (RAM) and is not persisted independently in non-volatile storage.

Implications:

- On each boot, firmware re-evaluates EEPROM health from the stored magic/header.
- If EEPROM magic is valid, the system boots without EEPROM error.
- If EEPROM is invalid/non-virgin, firmware rewrites defaults and sets `ERROR_CODE = 7` for that runtime session.
- A successful BLE parameter update clears the latched runtime EEPROM warning.
- Because the latch itself is not persisted, power cycling does not preserve the warning state on its own; warning reappearance depends on EEPROM validity at next boot.

## Heater over-temperature shutdown threshold

The firmware enforces a hard safety shutdown when measured heater temperature reaches 20% above the active target temperature.

Behavior:

- Safety threshold is computed as `active_target * 1.2`.
- In normal heating, `active_target` is the current PID setpoint (`K`).
- During cut-bag flushing, overheat protection uses the higher heating target (`max(K, CUT_MODE_TEMP)`) to avoid false trips while still preserving the 20% safety margin.
- If exceeded, firmware raises heater over-temp error (`ERROR_CODE = 3`), stops all relevant activity, and signals the fault via BLE/LED error handling.

Rationale:

- Protect hardware and surrounding components from runaway heating.
- Keep overheat logic proportional to the intended process temperature, rather than using a fixed absolute threshold.
- Maintain process continuity in cut mode by preventing premature shutdown at temperatures that are expected for `CUT_MODE_TEMP`, while preserving a clear safety boundary.

## BLE timeout behavior when clients stay connected

BLE auto-shutdown now uses an idle timer that respects serial streaming state:

- If serial streaming is active, BLE remains enabled and the idle timer is continuously refreshed.
- If serial streaming is not active, BLE can auto-shutdown after 10 minutes even if a client remains connected (to close accidental idle connections).
- When streaming stops or the client disconnects, BLE timeout behavior resumes from the idle timer and is re-evaluated continuously in the main loop.

Rationale:

- Preserve active diagnostics sessions: live serial monitoring should not be interrupted by a background BLE timeout.
- Avoid accidental battery drain: a forgotten/stale BLE connection without streaming should not keep radio power on indefinitely.
- Match operator intent: starting serial stream is treated as explicit "keep BLE alive" activity.
- Keep behavior predictable after session end: once streaming/disconnect transitions the link to idle, the same 10-minute idle shutdown policy applies.
- Improve shutdown safety: BLE send paths are guarded by `bleEnabled`, and streaming/connection state is cleared before BLE deinit to avoid stale-notify behavior during shutdown.

## Heater tolerance gap enforcement policy (2C minimum)

The system now enforces a minimum 2C separation between `heaterLowerToleranceC` and `heaterUpperToleranceC` using a split policy:

- Bluetooth interface (`toilet_bluetooth_interface.py`) performs strict validation and rejects updates when `heaterUpperToleranceC - heaterLowerToleranceC < 2.0`.
- Firmware (`toilet_kat_change.ino`) acts as a safety backstop for non-compliant clients by auto-correcting invalid pairs to `heaterLowerToleranceC = heaterUpperToleranceC - 2.0` and continuing operation.

Rationale:

- Operator-facing tools should fail fast and clearly when a parameter set is invalid.
- Firmware must remain resilient even if a third-party or stale client bypasses interface validation.
- The chosen correction direction prioritizes keeping the requested upper bound while lowering the ON threshold, which preserves process continuity and biases behavior toward safer/lower heating when payloads are malformed.
- Applying the same normalization at BLE ingest and EEPROM load prevents legacy or corrupted persisted values from violating the minimum gap rule after reboot.

## Error log persistence and BLE retrieval

The firmware persists error codes, crashes, brownouts, and runtime faults to a file on SPIFFS, retrievable over BLE via chunked `GET_LOGS` so users can share logs with support when troubleshooting.

**What is logged**:

- **Reset/crash/brownout**: `esp_reset_reason()` at boot for unexpected resets (panic, WDT, brownout, SDIO). Normal power-on and software resets are skipped.
- **Runtime errors**: When `LEDErrorCode()` is called (ERROR_CODE 1–7): motor timeout, low battery, heater overheat, motor fault, heater current fail, heater max wall time, EEPROM invalid.
- **EEPROM invalid**: Full reason from `enterEEPROMInvalidErrorState()`.
- **OTA failures**: When `setOTAState(OTA_ERROR)` is invoked.
- **MCP unavailable**: When I2C expander init fails after retries.

**Context for runtime errors**: Each runtime error log includes flush/feed state (`step`, `cut`, `feed`), sensor values (`bat`, `temp`, `m1A`, `heaterA`), and fan status (`off`, `forward`, `reverse`) to aid diagnostics.

**Storage**: SPIFFS file `/logs/errors.txt`. Max 8 KB; when full, oldest entries are dropped. Line length capped at 60 chars for messages.

**Retrieval**: BLE command `GET_LOGS` or `GET_LOGS:<offset>`. Response `LOGS:<offset>:<length>:<data>` (chunked, ~450 bytes per chunk) or `LOGS_END`. No trust handshake required so diagnostics work even when the user cannot complete trust (e.g. broken flush button).

**Rationale**:

- Support needs visibility into device failures; on-device logs survive power cycles and are accessible without a debugger.
- Bounded size prevents unbounded flash wear and memory use.
- Chunked retrieval fits BLE MTU limits (~512 bytes).
- No auth for GET_LOGS keeps diagnostics available when trust flow cannot be completed.
- Rich context (flush step, sensors, fan) helps support correlate failures with process state.

## Hardware matrix history and rollback metadata (on-device, offline)

The hardware matrix stores both current and previous values per component directly on-device (no cloud dependency) so service users can inspect what was installed before an upgrade and support rollback workflows.

Per component, the persisted record should include:

- `current_version`
- `current_description`
- `install_date` (ISO 8601 date, `YYYY-MM-DD`)
- `previous_version`
- `previous_description`
- `previous_install_date` (ISO 8601 date, `YYYY-MM-DD`)

Update logic (single component update over BLE):

1. Validate component name and payload bounds.
2. Read existing component record.
3. Copy current fields to previous fields:
   - `previous_version = current_version`
   - `previous_description = current_description`
   - `previous_install_date = install_date`
4. Write new current fields from request:
   - `current_version = new_version`
   - `current_description = new_description`
   - `install_date = new_install_date` (ISO 8601)
5. Persist and verify write success before ACK.

Rationale:

- Enables offline serviceability: technicians can see pre-upgrade component metadata without network access.
- Supports practical rollback decisions by preserving immediate historical context on-device.
- Keeps storage bounded and deterministic by retaining one previous snapshot per component (instead of unbounded history).
- Provides consistent parsing/sorting across firmware and Python tools by standardizing date format to ISO 8601.

## BLE payload chunking in the Python client

The Python BLE client (`toilet_bluetooth_interface.py`) splits large writes into chunks when sending to GATT characteristics.

**Why**: BLE GATT writes are limited by the ATT MTU. Each `write_gatt_char` call can send at most `MTU - 3` bytes (3 bytes reserved for ATT protocol overhead). The default MTU is 23 bytes, so only ~20 bytes can be sent per write. Even with negotiated MTU (e.g. 185), payloads larger than `MTU - 3` must be split.

**What gets chunked**: Parameter writes (22-float CSV), long commands, and any framed payloads that exceed the per-write limit. The client uses `_write_ble_payload_chunked` to split payloads and send them sequentially.

**Rationale**:

- BLE_APP_MIGRATION_SPEC does not require framed payloads; the protocol is plain UTF-8. Chunking is a transport-layer necessity, not a protocol requirement.
- Chunk size is derived from `client.mtu_size` (or 23 if unknown) minus 3.
- Optional `BLE_CHUNK_PACING_S` allows inserting delays between chunks to avoid overwhelming slower stacks (e.g. some Android BLE implementations).
