# QC Test Procedure

Quality control test sequence for the toilet device. Firmware: `QC_TEST.ino`. Device advertises as "QC_Test".

---

## Sequence Overview

```
LOCATE → LOCATE_CONFIRM → THERMISTOR → HEATER → MOTOR → FEED_WAIT → FAN_WAIT → FLUSH_CLOSE → FLUSH_EXTRA_1S → FLUSH_HEAT → FLUSH_COOL → FLUSH_OPEN → DONE
```

---

## Test Steps

| # | Step | Description | Pass Criteria | Timeout | Halt Reason on Fail |
|---|------|--------------|---------------|---------|---------------------|
| 1 | **LOCATE** | Locate motor to known open position | Mechanism moves to open; microswitch open triggered | 15 s per phase | `LOCATE_CLOSE_TIMEOUT`, `LOCATE_OPEN_TIMEOUT` |
| 2 | **LOCATE_CONFIRM** | Operator visual inspection after locate | Operator confirms mechanism position via BLE | 15 s | `LOCATE_CONFIRM_TIMEOUT` |
| 3 | **THERMISTOR** | Thermistor connectivity check | Resistance < 100 kΩ | — | `THERMISTOR_DISCONNECTED` |
| 4 | **HEATER** | Heater current detection | ADC > 200 within 500 ms | — | `HEATER_CURRENT_FAIL` |
| 5 | **MOTOR** | Motor fault check | No M1/M2 fault after enabling drivers | — | `MOTOR_FAULT` |
| 6 | **FEED_WAIT** | Feed motor (M2) test | Operator confirms feed motor running | 15 s | `FEED_CONFIRM_TIMEOUT` |
| 7 | **FAN_WAIT** | Fan forward test | Operator confirms fan running forward | 15 s | `FAN_CONFIRM_TIMEOUT` |
| 8 | **FLUSH_CLOSE** | Mechanism close | Microswitch closed + M1 current > 0.5 A | 15 s | `FLUSH_CLOSE_TIMEOUT` |
| 9 | **FLUSH_EXTRA_1S** | Extra close time | M1 runs 1 s after close | 1 s | — |
| 10 | **FLUSH_HEAT** | Heating phase | Thermistor reaches 80 °C | — | — |
| 11 | **FLUSH_COOL** | Cooling phase | Thermistor cools to 60 °C | — | — |
| 12 | **FLUSH_OPEN** | 10× (open→1s→close→0.5s) then final open | Open switch triggered at end | 15 s per phase | `FLUSH_OPEN_TIMEOUT` |
| 13 | **DONE** | Test complete | All steps passed | — | — |

---

## Constants

| Constant | Value | Purpose |
|----------|-------|---------|
| `TIMEOUT_MS` | 15 000 | Timeout for locate, locate confirm, feed confirm, fan confirm, flush close, flush open |
| `M1_STALL_CURRENT_A` | 0.5 | Minimum M1 current (A) for close detection |
| `QC_HEAT_TARGET_C` | 80 | Target temperature (°C) for heating phase |
| `QC_COOL_TARGET_C` | 60 | Target temperature (°C) for cooling phase |
| `HEATER_ADC_THRESHOLD` | 200 | ADC threshold for heater current detection |
| `FLUSH_OPEN_CYCLES` | 10 | Number of open/close cycles before final open |
| `BATTERY_SLEEP_THRESHOLD` | 10 | Battery % below which device enters deep sleep until power cycle |

---

## Battery and Low-Battery Sleep

- **Charge calculation**: 11.0 V = 0%, 12.6 V = 100% (linear).
- **Low-battery sleep**: If battery falls below 10%, device stops motors/fan/heater and enters deep sleep. Recovery requires power cycle (hard reboot).
- **BAT in interface**: `QC_STATUS` and `QC_READY` include `,BAT:<0-100>` for operator display.

---

## LOCATE_CONFIRM Phase Detail

After LOCATE completes, the mechanism is at the open position. The device pauses for operator visual inspection (e.g. verify mechanism alignment, no obstructions). Within 15 s, send `QC_CONFIRM_LOCATE` to proceed to THERMISTOR.

---

## FLUSH_OPEN Phase Detail

1. Repeat 10 times:
   - **Open** until close switch releases (mechanism has left closed position)
   - **Continue opening** for 1 s
   - **Close** until close switch triggered (M1 current > 0.5 A)
   - **Continue closing** for 0.5 s
2. **Final open**: Open until open switch triggered to confirm fully open.

---

## BLE Commands (App → Device)

| Command | When to Send | Response |
|---------|--------------|----------|
| `START_TEST` | After BLE connect; begins QC test sequence from LOCATE | `QC_READY` or `QC_STATUS:LOCATE` (device proceeds through steps) |
| `QC_CONFIRM_LOCATE` | During LOCATE_CONFIRM (after visual inspection of mechanism at open) | `QC_CONFIRM_LOCATE_ACK` or `QC_ERR:NOT_IN_LOCATE_STEP` |
| `QC_CONFIRM_FEED` | During FEED_WAIT (after observing feed motor) | `QC_CONFIRM_FEED_ACK` or `QC_ERR:NOT_IN_FEED_STEP` |
| `QC_CONFIRM_FAN` | During FAN_WAIT (after observing fan forward) | `QC_CONFIRM_FAN_ACK` or `QC_ERR:NOT_IN_FAN_STEP` |
| `QC_STATUS` | Anytime | `QC_STATUS:<step>,BAT:<%>` or `QC_STATUS:<step>,TEMP:<°C>,BAT:<%>` (FLUSH_HEAT/FLUSH_COOL) or `QC_STATUS:<step>,HALTED:<reason>,BAT:<%>` |
| `RESET` | After DONE or HALTED; resets to idle for next test | `QC_READY,BAT:<%>` |
| `M1_CLOSE_1S` | When idle only; runs main mechanism motor (M1) for 1 s to partially close (repair aid). Blocked if close switch already triggered. | — or `QC_ERR:TEST_IN_PROGRESS` or `QC_ERR:ALREADY_CLOSED` |
| `START_SERIAL` | Enable serial streaming | `SERIAL_ON` |
| `STOP_SERIAL` | Disable serial streaming | `SERIAL_OFF` |

---

## BLE Responses (Device → App)

| Response | Meaning |
|----------|---------|
| `QC_READY,BAT:<%>` | Device ready at startup or after RESET; includes battery charge 0–100 |
| `QC_CONFIRM_LOCATE_ACK` | Locate step confirmed; thermistor check started |
| `QC_CONFIRM_FEED_ACK` | Feed step confirmed; fan forward started |
| `QC_CONFIRM_FAN_ACK` | Fan step confirmed; flush close started |
| `QC_DONE` | All tests passed |
| `QC_HALTED:<reason>` | Test halted due to failure |
| `QC_ERR:NOT_IN_LOCATE_STEP` | QC_CONFIRM_LOCATE sent at wrong step |
| `QC_ERR:NOT_IN_FEED_STEP` | QC_CONFIRM_FEED sent at wrong step |
| `QC_ERR:NOT_IN_FAN_STEP` | QC_CONFIRM_FAN sent at wrong step |
| `QC_ERR:TEST_IN_PROGRESS` | M1_CLOSE_1S sent while test running (only allowed when idle) |
| `QC_ERR:ALREADY_CLOSED` | M1_CLOSE_1S blocked; close switch indicates mechanism already fully closed |

---

## QC_STATUS Response (App Spec)

When the app sends `QC_STATUS`, the device responds on the response characteristic. Parse the string as comma-separated key-value pairs.

| Field | Format | When Present | App Usage |
|-------|--------|--------------|-----------|
| `QC_STATUS` | `QC_STATUS:<step>` | Always | Current step name (e.g. `LOCATE`, `FLUSH_HEAT`, `DONE`) |
| `TEMP` | `TEMP:<value>` | Only during `FLUSH_HEAT` or `FLUSH_COOL` | Integer °C. Display during heating (target 80 °C) and cooling (target 60 °C). Poll `QC_STATUS` periodically to update UI. |
| `BAT` | `BAT:<value>` | Always | Integer 0–100. Battery charge %. Display to operator. |
| `HALTED` | `HALTED:<reason>` | When test has failed | Halt reason code; show error to operator |

**Examples:**
- `QC_STATUS:FLUSH_HEAT,TEMP:75,BAT:72` — Heating phase, 75 °C (target 80), 72% battery
- `QC_STATUS:FLUSH_COOL,TEMP:62,BAT:70` — Cooling phase, 62 °C (target 60), 70% battery
- `QC_STATUS:FEED_WAIT,BAT:68` — No temp; waiting for operator confirm, 68% battery
- `QC_STATUS:HALTED,HALTED:FLUSH_CLOSE_TIMEOUT,BAT:45` — Test failed; show halt reason, 45% battery

---

## Halt Reasons

| Code | Cause |
|------|-------|
| `LOCATE_CLOSE_TIMEOUT` | Mechanism did not leave open switch within 15 s |
| `LOCATE_OPEN_TIMEOUT` | Mechanism did not reach open position within 15 s |
| `LOCATE_CONFIRM_TIMEOUT` | QC_CONFIRM_LOCATE not received within 15 s |
| `THERMISTOR_DISCONNECTED` | Thermistor resistance > 100 kΩ |
| `HEATER_CURRENT_FAIL` | Heater current ADC did not exceed 200 within 500 ms |
| `MOTOR_FAULT` | M1 or M2 fault pin active after enabling drivers |
| `FEED_CONFIRM_TIMEOUT` | QC_CONFIRM_FEED not received within 15 s |
| `FAN_CONFIRM_TIMEOUT` | QC_CONFIRM_FAN not received within 15 s |
| `FLUSH_CLOSE_TIMEOUT` | Mechanism did not close within 15 s |
| `FLUSH_OPEN_TIMEOUT` | Mechanism did not open within 15 s |

---

## Operator Procedure

1. Flash QC_TEST firmware to device. Power on. (If battery &lt; 10%, device sleeps immediately; power cycle after charging.)
2. Connect BLE app to "QC_Test".
3. Press **START TEST** in app to begin. Device runs LOCATE.
4. **LOCATE_CONFIRM**: Visually inspect mechanism at open position. Within 15 s, send `QC_CONFIRM_LOCATE`.
5. Steps 3–5 (THERMISTOR, HEATER, MOTOR) run automatically.
6. **FEED_WAIT**: Observe feed motor (M2) running. Within 15 s, send `QC_CONFIRM_FEED`.
7. **FAN_WAIT**: Observe fan running forward. Within 15 s, send `QC_CONFIRM_FAN`.
8. Steps 8–12 (FLUSH_CLOSE through FLUSH_OPEN) run automatically.
9. On `QC_DONE`, all tests passed. On `QC_HALTED:<reason>`, note reason and troubleshoot.
10. To run another test, send `RESET` then `START_TEST`.
11. If battery falls below 10% during test, device enters deep sleep. Power cycle after charging to recover.

---

## BLE UUIDs

- Service: `5636340f-afc7-47b1-b0a8-15bc9d7d29a5`
- Command (write): `c327b077-560f-46a1-8f35-b4ab0332fea0`
- Response (read/notify): `c327b077-560f-46a1-8f35-b4ab0332fea4`
- Serial (read/write/notify): `c327b077-560f-46a1-8f35-b4ab0332fea1`

---

## App–Firmware Interface (for Firmware Implementation)

This section documents the exact protocol the app uses. Implement these in firmware to ensure compatibility.

### Command Format

- **Transport**: App writes ASCII string to Command characteristic (UUID above).
- **Encoding**: UTF-8, null-terminated or newline-terminated as per BLE characteristic.
- **Case**: Commands are case-sensitive; use exact strings below.

### Commands (App → Device)

| Command | Exact String | When App Sends | Firmware Action |
|---------|--------------|----------------|-----------------|
| Start test | `START_TEST` | After BLE connect; user presses START TEST | Begin QC sequence from LOCATE |
| Reset | `RESET` | After DONE or HALTED; user presses STOP TEST (or RESTART TEST on success) | Clear state; return to idle (QC_READY) |
| Confirm locate | `QC_CONFIRM_LOCATE` | During LOCATE_CONFIRM; after operator confirms equal distance | Proceed to THERMISTOR |
| Confirm feed | `QC_CONFIRM_FEED` | During FEED_WAIT; after operator confirms both motor 1 and motor 2 | Proceed to FAN_WAIT |
| Confirm fan | `QC_CONFIRM_FAN` | During FAN_WAIT; after operator confirms fan | Proceed to FLUSH_CLOSE |
| Status poll | `QC_STATUS` | Every 1.5 s while test running | Reply with current status |
| Partial close (repair) | `M1_CLOSE_1S` | When idle; user presses PARTIAL CLOSE (1s) | Run M1 for 1 s to partially close mechanism |

### Step Names (QC_STATUS)

Firmware must emit `QC_STATUS:<step>` with these exact step names. App uses them for UI and flow control.

| Step | Exact String | Notes |
|------|--------------|-------|
| Locate | `LOCATE` | Mechanism moving to open |
| Locate confirm | `LOCATE_CONFIRM` or `LOCATE_POS` | App treats `LOCATE_POS` as alias for `LOCATE_CONFIRM` |
| Thermistor | `THERMISTOR` | |
| Heater | `HEATER` | |
| Motor | `MOTOR` | |
| Feed wait | `FEED_WAIT` | Awaiting `QC_CONFIRM_FEED` |
| Fan wait | `FAN_WAIT` | Awaiting `QC_CONFIRM_FAN` |
| Flush close | `FLUSH_CLOSE` | |
| Flush extra 1s | `FLUSH_EXTRA_1S` | |
| Flush heat | `FLUSH_HEAT` | Include `TEMP:<°C>` in response when applicable |
| Flush cool | `FLUSH_COOL` | Include `TEMP:<°C>` in response when applicable |
| Flush open | `FLUSH_OPEN` | |
| Done | `DONE` | |

### Response Format (Device → App)

- **Transport**: Device writes to Response characteristic (notify or read).
- **Format**: ASCII, one message per write. Comma-separated key-value pairs for `QC_STATUS`.

**Simple responses** (exact strings):

- `QC_READY,BAT:<%>` — Device idle, ready for `START_TEST` or `M1_CLOSE_1S`; includes battery charge %
- `QC_DONE` — Test passed
- `QC_CONFIRM_LOCATE_ACK` — Locate confirmed
- `QC_CONFIRM_FEED_ACK` — Feed confirmed
- `QC_CONFIRM_FAN_ACK` — Fan confirmed

**QC_STATUS** (comma-separated, always includes `BAT:<%>`):

- `QC_STATUS:<step>,BAT:<%>` — e.g. `QC_STATUS:FEED_WAIT,BAT:68`
- `QC_STATUS:<step>,TEMP:<value>,BAT:<%>` — e.g. `QC_STATUS:FLUSH_HEAT,TEMP:75,BAT:72`
- `QC_STATUS:<step>,HALTED:<reason>,BAT:<%>` — e.g. `QC_STATUS:FEED_WAIT,HALTED:FEED_CONFIRM_TIMEOUT,BAT:45`
- Or `QC_HALTED:<reason>` — e.g. `QC_HALTED:FLUSH_CLOSE_TIMEOUT`

**Error responses**:

- `QC_ERR:NOT_IN_LOCATE_STEP` — `QC_CONFIRM_LOCATE` sent at wrong step
- `QC_ERR:NOT_IN_FEED_STEP` — `QC_CONFIRM_FEED` sent at wrong step
- `QC_ERR:NOT_IN_FAN_STEP` — `QC_CONFIRM_FAN` sent at wrong step
- `QC_ERR:TEST_IN_PROGRESS` — `M1_CLOSE_1S` sent while test running (only allowed when idle)
- `QC_ERR:ALREADY_CLOSED` — `M1_CLOSE_1S` blocked; close switch indicates mechanism already fully closed

### Halt Reasons (HALTED:<reason>)

App uses these for fail modal and troubleshooting images. Use exact strings.

| Reason | When to Emit |
|--------|--------------|
| `LOCATE_CLOSE_TIMEOUT` | Mechanism did not leave open switch in 15 s |
| `LOCATE_OPEN_TIMEOUT` | Mechanism did not reach open in 15 s |
| `LOCATE_CONFIRM_TIMEOUT` | `QC_CONFIRM_LOCATE` not received in 15 s |
| `THERMISTOR_DISCONNECTED` | Thermistor resistance > 100 kΩ |
| `HEATER_CURRENT_FAIL` | Heater ADC did not exceed 200 in 500 ms |
| `MOTOR_FAULT` | M1 or M2 fault active |
| `FEED_CONFIRM_TIMEOUT` | `QC_CONFIRM_FEED` not received in 15 s |
| `FAN_CONFIRM_TIMEOUT` | `QC_CONFIRM_FAN` not received in 15 s |
| `FLUSH_CLOSE_TIMEOUT` | Mechanism did not close in 15 s |
| `FLUSH_OPEN_TIMEOUT` | Mechanism did not open in 15 s |
| `FLUSH_HEAT_FAIL` or `FLUSH_HEAT_TIMEOUT` or `FLUSH_HEAT_*` | Heating phase failure (if implemented) |

### M1_CLOSE_1S (Repair Command)

- **When**: Only when device is in idle state (after `RESET` or before `START_TEST`).
- **Action**: Run main mechanism motor (M1) in close direction for 1 second.
- **Purpose**: Aid repair by partially closing mechanism (e.g. after connection fix).
- **Safety**: Do not run if close switch (switch 2) is already triggered (mechanism fully closed).
- **Response**: No response on success; device remains idle. `QC_ERR:TEST_IN_PROGRESS` if test running; `QC_ERR:ALREADY_CLOSED` if close switch already triggered.

### RESET Behavior

- App sends `RESET` when user presses STOP TEST (fail or success).
- Firmware must clear test state and return to idle (`QC_READY,BAT:<%>`).
- Device must accept `START_TEST` or `M1_CLOSE_1S` after `RESET`.
