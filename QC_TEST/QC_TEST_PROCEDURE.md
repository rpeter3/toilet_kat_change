# QC Test Procedure

Quality control test sequence for the toilet device. Firmware: `QC_TEST.ino`. Device advertises as "QC_Test".

---

## Sequence Overview

```
LOCATE → THERMISTOR → HEATER → MOTOR → FEED_WAIT → FAN_WAIT → FLUSH_CLOSE → FLUSH_EXTRA_1S → FLUSH_HEAT → FLUSH_COOL → FLUSH_OPEN → DONE
```

---

## Test Steps

| # | Step | Description | Pass Criteria | Timeout | Halt Reason on Fail |
|---|------|--------------|---------------|---------|---------------------|
| 1 | **LOCATE** | Locate motor to known open position | Mechanism moves to open; microswitch open triggered | 15 s per phase | `LOCATE_CLOSE_TIMEOUT`, `LOCATE_OPEN_TIMEOUT` |
| 2 | **THERMISTOR** | Thermistor connectivity check | Resistance < 100 kΩ | — | `THERMISTOR_DISCONNECTED` |
| 3 | **HEATER** | Heater current detection | ADC > 200 within 500 ms | — | `HEATER_CURRENT_FAIL` |
| 4 | **MOTOR** | Motor fault check | No M1/M2 fault after enabling drivers | — | `MOTOR_FAULT` |
| 5 | **FEED_WAIT** | Feed motor (M2) test | Operator confirms feed motor running | 15 s | `FEED_CONFIRM_TIMEOUT` |
| 6 | **FAN_WAIT** | Fan reverse test | Operator confirms fan running in reverse | 15 s | `FAN_CONFIRM_TIMEOUT` |
| 7 | **FLUSH_CLOSE** | Mechanism close | Microswitch closed + M1 current > 0.5 A | 15 s | `FLUSH_CLOSE_TIMEOUT` |
| 8 | **FLUSH_EXTRA_1S** | Extra close time | M1 runs 1 s after close | 1 s | — |
| 9 | **FLUSH_HEAT** | Heating phase | Thermistor reaches 80 °C | — | — |
| 10 | **FLUSH_COOL** | Cooling phase | Thermistor cools to 60 °C | — | — |
| 11 | **FLUSH_OPEN** | Mechanism open | Microswitch open triggered | 15 s | `FLUSH_OPEN_TIMEOUT` |
| 12 | **DONE** | Test complete | All steps passed | — | — |

---

## Constants

| Constant | Value | Purpose |
|----------|-------|---------|
| `TIMEOUT_MS` | 15 000 | Timeout for locate, feed confirm, fan confirm, flush close, flush open |
| `M1_STALL_CURRENT_A` | 0.5 | Minimum M1 current (A) for close detection |
| `QC_HEAT_TARGET_C` | 80 | Target temperature (°C) for heating phase |
| `QC_COOL_TARGET_C` | 60 | Target temperature (°C) for cooling phase |
| `HEATER_ADC_THRESHOLD` | 200 | ADC threshold for heater current detection |

---

## BLE Commands (App → Device)

| Command | When to Send | Response |
|---------|--------------|----------|
| `START_TEST` | After BLE connect; begins QC test sequence from LOCATE | `QC_READY` or `QC_STATUS:LOCATE` (device proceeds through steps) |
| `QC_CONFIRM_FEED` | During FEED_WAIT (after observing feed motor) | `QC_CONFIRM_FEED_ACK` or `QC_ERR:NOT_IN_FEED_STEP` |
| `QC_CONFIRM_FAN` | During FAN_WAIT (after observing fan reverse) | `QC_CONFIRM_FAN_ACK` or `QC_ERR:NOT_IN_FAN_STEP` |
| `QC_STATUS` | Anytime | `QC_STATUS:<step>` or `QC_STATUS:<step>,HALTED:<reason>` |
| `START_SERIAL` | Enable serial streaming | `SERIAL_ON` |
| `STOP_SERIAL` | Disable serial streaming | `SERIAL_OFF` |

---

## BLE Responses (Device → App)

| Response | Meaning |
|----------|---------|
| `QC_READY` | Device ready at startup |
| `QC_CONFIRM_FEED_ACK` | Feed step confirmed; fan reverse started |
| `QC_CONFIRM_FAN_ACK` | Fan step confirmed; flush close started |
| `QC_DONE` | All tests passed |
| `QC_HALTED:<reason>` | Test halted due to failure |
| `QC_ERR:NOT_IN_FEED_STEP` | QC_CONFIRM_FEED sent at wrong step |
| `QC_ERR:NOT_IN_FAN_STEP` | QC_CONFIRM_FAN sent at wrong step |

---

## Halt Reasons

| Code | Cause |
|------|-------|
| `LOCATE_CLOSE_TIMEOUT` | Mechanism did not leave open switch within 15 s |
| `LOCATE_OPEN_TIMEOUT` | Mechanism did not reach open position within 15 s |
| `THERMISTOR_DISCONNECTED` | Thermistor resistance > 100 kΩ |
| `HEATER_CURRENT_FAIL` | Heater current ADC did not exceed 200 within 500 ms |
| `MOTOR_FAULT` | M1 or M2 fault pin active after enabling drivers |
| `FEED_CONFIRM_TIMEOUT` | QC_CONFIRM_FEED not received within 15 s |
| `FAN_CONFIRM_TIMEOUT` | QC_CONFIRM_FAN not received within 15 s |
| `FLUSH_CLOSE_TIMEOUT` | Mechanism did not close within 15 s |
| `FLUSH_OPEN_TIMEOUT` | Mechanism did not open within 15 s |

---

## Operator Procedure

1. Flash QC_TEST firmware to device. Power on.
2. Connect BLE app to "QC_Test".
3. Press **START TEST** in app to begin. Device runs LOCATE, then proceeds through steps. Wait for completion or halt.
4. Steps 2–4 (THERMISTOR, HEATER, MOTOR) run automatically.
5. **FEED_WAIT**: Observe feed motor (M2) running. Within 15 s, send `QC_CONFIRM_FEED`.
6. **FAN_WAIT**: Observe fan running in reverse. Within 15 s, send `QC_CONFIRM_FAN`.
7. Steps 7–11 (FLUSH_CLOSE through FLUSH_OPEN) run automatically.
8. On `QC_DONE`, all tests passed. On `QC_HALTED:<reason>`, note reason and troubleshoot.

---

## BLE UUIDs

- Service: `5636340f-afc7-47b1-b0a8-15bc9d7d29a5`
- Command (write): `c327b077-560f-46a1-8f35-b4ab0332fea0`
- Response (read/notify): `c327b077-560f-46a1-8f35-b4ab0332fea4`
- Serial (read/write/notify): `c327b077-560f-46a1-8f35-b4ab0332fea1`
