# HWCFG Interface Specification

## Purpose

The `HWCFG` interface is the Bluetooth command surface for safe hardware-configuration updates.

It is designed so the app can:

- preload a parameter profile for a specific hardware component/version,
- validate compatibility and safety before hardware activation,
- apply hardware changes with rollback-aware behavior,
- inspect active and last-known-good configuration state on-device.

This supports field upgrades where hardware can change over time without requiring cloud access.

## Location

- Firmware command handling: `toilet_kat_change/toilet_kat_change.ino`
- Bluetooth client/tooling: `toilet_bluetooth_interface.py`

## Data Model Summary

Firmware stores an `HWCFG` profile store in NVS:

- namespace: `hwcfg`
- active key: `active`
- last-known-good key: `lkg`

Store-level protections:

- magic (`HWCFG_MAGIC`)
- schema version (`HWCFG_SCHEMA_VERSION`)
- CRC32 over serialized store data

Profile limits (current implementation):

- max profiles: `24`
- profile ID max length: `24`
- profile parameter blob max length: `320`

## Command Contract (Implemented)

### Capability and state read

- `HWCFG_GET_CAPS`
  - Response: `HWCFG_CAPS:V1|PROFILE_STORE|TXN_APPLY|ROLLBACK`

- `HWCFG_GET_ACTIVE_CONFIG`
  - Response prefix: `HWCFG_ACTIVE:`
  - Format: `<component>=<version>;...|profile_id=<id>|validated=<0|1>`

- `HWCFG_GET_LAST_GOOD_CONFIG`
  - Response prefix: `HWCFG_LAST_GOOD:`
  - Format: `<component>=<version>;...|profile_id=<id>`

### Profile management

- `HWCFG_PROFILE_LIST`
  - Response prefix: `HWCFG_PROFILE_LIST:`
  - Format: `<profile_id>@<component>:<version>,...`

- `HWCFG_PROFILE_GET:<component>|version=<ver>`
  - Success response prefix: `HWCFG_PROFILE:`
  - Format: `<profile_id>|component=<name>|version=<ver>|params=<k=v;...>`
  - Error response prefix: `HWCFG_VALIDATE_ERR:`

- `HWCFG_PROFILE_PUT:<profile_id>|component=<name>|version=<ver>|k1=v1;k2=v2;...`
  - Success response prefix: `HWCFG_VALIDATE_OK:`
  - Error response prefix: `HWCFG_VALIDATE_ERR:`

### Validation and apply

- `HWCFG_VALIDATE_CHANGE:<component>|new_version=<ver>`
  - Success response prefix: `HWCFG_VALIDATE_OK:`
  - Format: `<component>|version=<ver>|profile_id=<id>`
  - Error response prefix: `HWCFG_VALIDATE_ERR:`

- `HWCFG_APPLY_CHANGE:<component>|new_version=<ver>|install_date=<YYYY-MM-DD>|desc=<text>`
  - Success response prefix: `HWCFG_APPLY_ACK:`
  - Format: `<component>|version=<ver>`
  - Error response prefix: `HWCFG_APPLY_ERR:`

### Rollback command

- `HWCFG_ROLLBACK_LAST_GOOD`
  - Success response: `HWCFG_ROLLBACK_ACK`
  - Error response prefix: `HWCFG_ROLLBACK_ERR:`

## Validation Rules

The firmware validates before accepting profile changes and hardware apply:

- component name must be recognized,
- profile must exist for `(component, version)` during validate/apply,
- parameter keys must be known firmware parameters,
- parameter values must parse as numeric values,
- safety ranges are checked on critical parameters,
- additional compatibility checks are enforced for some components (for example, heater-related required keys).

Date format for apply:

- install date must be ISO 8601 date: `YYYY-MM-DD`.

## Transactional Behavior

High-level apply flow:

1. Receive `HWCFG_APPLY_CHANGE`.
2. Resolve and validate candidate profile for component/version.
3. Apply candidate parameter blob to runtime + persist parameters.
4. Apply hardware matrix component change metadata.
5. Update HWCFG active/last-good profile pointers and persist.
6. Return `HWCFG_APPLY_ACK` on success or `HWCFG_APPLY_ERR:<reason>` on failure.

### Rollback/Recovery Intent

- On boot, firmware attempts active store recovery first, then last-known-good.
- If both are invalid, firmware enters a safe-fault posture for HWCFG operations.
- `HWCFG_ROLLBACK_LAST_GOOD` restores state from the last-known-good store when available.

## Common Error Codes

Observed response reason codes include:

- `BAD_FORMAT`
- `NO_PROFILE`
- `UNKNOWN_COMPONENT`
- `UNKNOWN_PARAM`
- `OUT_OF_RANGE`
- `INCOMPATIBLE`
- `TOO_LONG`
- `PERSIST_FAIL`
- `BAD_DATE`
- `BAD_DESC`
- `SAFE_FAULT`
- `INIT_FAIL`
- `MATRIX_INIT_FAIL`

## Recommended App Workflow

For a safe field hardware upgrade:

1. `HWCFG_GET_CAPS`
2. `HWCFG_PROFILE_PUT` (preload/update profile)
3. `HWCFG_VALIDATE_CHANGE` (must pass)
4. `HWCFG_APPLY_CHANGE` (commit hardware + profile)
5. `HWCFG_GET_ACTIVE_CONFIG` (verify active)
6. If needed, `HWCFG_ROLLBACK_LAST_GOOD`

## Notes

- The HWCFG interface is local BLE and does not require cloud connectivity.
- Previous hardware metadata remains stored on-device for service visibility.
- Existing non-HWCFG parameter flows remain in place for backward compatibility.
