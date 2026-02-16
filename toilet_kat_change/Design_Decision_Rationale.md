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
