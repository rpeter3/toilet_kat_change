# ESP32 Toilet System Bluetooth Interface

A Python program to communicate with the ESP32 toilet system via Bluetooth Low Energy (BLE) to read and update system parameters.

## Features

- **BLE Communication**: Connects to ESP32 via Bluetooth Low Energy
- **Parameter Management**: Read current parameters and update them wirelessly
- **File Operations**: Save/load parameter configurations to/from JSON files
- **Interactive Interface**: User-friendly command-line interface
- **Parameter Validation**: Ensures proper data types and ranges

## Installation

### Prerequisites
- Python 3.7 or higher
- Bluetooth adapter on your laptop
- ESP32 toilet system with BLE enabled

### Install Dependencies
```bash
pip install -r requirements.txt
```

Or install manually:
```bash
pip install bleak
```

## Usage

### 1. Run the Interface
```bash
python toilet_bluetooth_interface.py
```

### 2. Menu Options

1. **Read current parameters** - Retrieves and displays current system parameters
2. **Update parameters** - Allows you to modify individual parameters
3. **Save parameters to file** - Exports current parameters to JSON file
4. **Load parameters from file** - Imports parameters from JSON file
5. **Display parameter definitions** - Shows all available parameters with descriptions
6. **Exit** - Disconnects and exits the program

## Parameters

The system supports 30 configurable parameters over BLE. Default values are set for 1.5mil High Barrier Plastic material (see `material_parameters.csv`):

| Parameter | Description | Units | Default |
|-----------|-------------|-------|---------|
| batteryThreshold | Minimum usable battery percent before flush | % | 7 |
| K | Temperature setpoint for PID control | °C | 150.0 |
| F | How long to feed the bag at the START of a flush | seconds | 8 |
| T | Cooling Time | seconds | 60 |
| backupTime | How long to back up the bag when re-opening | seconds | 1.0 |
| fanDuration | How long to run the fan after feeding at the end of a flush | seconds | 5 |
| H | Heater On time | seconds | 30 |
| continueFeeder | How long to feed the bag at the END of a flush | seconds | 6.0 |
| maxOpeningTime | Max opening time | seconds | 12 |
| typicalOpeningTime | Typical opening time | seconds | 10 |
| MOTOR_CUT_TIME | Motor cut duration | seconds | 0.5 |
| CUT_MODE_HEAT_TIME | Additional heater time in cut mode | seconds | 15.0 |
| postCoolingFanDuration | Fan duration before feed motors start in case 10 | seconds | 5.0 |
| preFeedFan | Fan duration before feed motor starts in case 1 and button 2 | seconds | 2.0 |
| fanReverseTime | Duration M3 runs in reverse after starting | seconds | 12.0 |
| fanReverseStartTime | Delay before M3 reverse starts (% of typicalOpeningTime) | % | 0.0 |
| backupTimeAfterReopen | Feed bag backup duration after mechanism motor finishes opening | seconds | 1.7 |
| CUT_MODE_TEMP | Temperature to maintain for CUT_MODE_HEAT_TIME after cut motor | °C | 150.0 |
| heaterLowerToleranceC | Heater ON threshold below target | °C | 0.0 |
| heaterUpperToleranceC | Heater OFF threshold above target | °C | 2.0 |
| COOL_OPEN_TEMP_C | Open sealer when thermistor cools below this temperature | °C | 80.0 |
| MAX_COOL_WAIT_S | Safety timeout for cooling stage before forcing open | seconds | 180 |
| minLoadedBatteryV | Minimum loaded battery voltage during flush preflight heater test | V | 11.2 |
| maxBatterySagV | Maximum allowed battery sag during flush preflight heater test | V | 0.85 |
| minIdleBatteryVFloor | Minimum idle battery voltage floor before flush preflight | V | 11.3 |
| usableVFull | Loaded battery voltage mapped to 100% usable | V | 12.4 |
| batteryAssessSettleMs | Heater pulse settle time during battery assessment | ms | 50 |
| heaterCapV255 | Idle battery voltage for maximum heater PWM cap during assessment | V | 11.23 |
| heaterCapV170 | Idle battery voltage for 170 heater PWM cap during assessment | V | 11.22 |
| heaterCapV100 | Idle battery voltage for 100 heater PWM cap during assessment | V | 11.21 |

**Note**: Material-specific parameters can be found in `material_parameters.csv` for different bag materials (1mil/1.5mil High Barrier Plastic, Compostable 1.5mil).

## Example Usage

### Reading Parameters
```
Options:
1. Read current parameters
2. Update parameters
...
Enter your choice (1-6): 1

Reading current parameters...
Received message: Welcome to the Toilet Server!
```

### Updating Parameters
```
Enter your choice (1-6): 2

Updating parameters...
Enter new values (press Enter to keep current value):
batteryThreshold (Minimum usable battery percent before flush) [%] [current: 7]: 8
K (Temperature setpoint for PID control) [°C] [current: 120.0]: 125.5
F (How long to feed the bag at the START of a flush) [seconds] [current: 6]: 
...
```

### Saving Configuration
```
Enter your choice (1-6): 3
Enter filename (default: toilet_params.json): my_config.json
Parameters saved to my_config.json
```

## File Format

Parameters are saved in JSON format with all 30 parameters:
```json
{
  "batteryThreshold": 7,
  "K": 120.0,
  "F": 6,
  "T": 60,
  "thermistorResistance": 10000.0,
  "r2": 2,
  "backupTime": 1.7,
  "r4": 2,
  "fanDuration": 5,
  "H": 40,
  "continueFeeder": 7.0,
  "maxOpeningTime": 12,
  "typicalOpeningTime": 10,
  "MOTOR_CUT_TIME": 0.5,
  "CUT_MODE_HEAT_TIME": 25.0,
  "postCoolingFanDuration": 5.0,
  "preFeedFan": 3.0,
  "fanReverseTime": 3.0,
  "fanReverseStartTime": 0.0,
  "backupTimeAfterReopen": 1.7
}
```

## Troubleshooting

### Connection Issues
- Ensure ESP32 is powered on and BLE is enabled
- Check that Bluetooth is enabled on your laptop
- Make sure no other devices are connected to the ESP32
- Try restarting the ESP32 if connection fails

### Parameter Update Issues
- Ensure all parameters are provided in the correct order
- Check that parameter values are within valid ranges
- Verify ESP32 is not in the middle of a flush sequence

### Bluetooth Issues
- On Windows: Ensure Bluetooth is enabled and drivers are installed
- On Linux: May need to install `bluez` package
- On macOS: Should work out of the box

## Technical Details

- **BLE Service UUID**: `5636340f-afc7-47b1-b0a8-15bc9d7d29a5`
- **BLE Characteristic UUID**: `c327b077-560f-46a1-8f35-b4ab0332fea0`
- **Serial Characteristic UUID**: `c327b077-560f-46a1-8f35-b4ab0332fea1`
- **Version Characteristic UUID**: `c327b077-560f-46a1-8f35-b4ab0332fea2`
- **Device Name**: `ESP32 Toilet`
- **Communication**: Comma-separated values over BLE characteristic
- **Data Format**: UTF-8 encoded strings
- **Parameter Persistence**: Parameters are automatically saved to EEPROM when updated
- **BLE Timeout**: BLE automatically shuts down after 10 minutes to save power (can be re-enabled by restarting device)
- **OTA Support**: Over-the-air firmware updates available via BLE (requires special activation sequence)

## BLE Framing (Android-safe streaming)

Serial stream transport now supports a framed mode that is safe across BLE packet fragmentation and callback timing differences:

- Frame format: `[0x7E][len_lo][len_hi][payload...]`
- `len` is a little-endian `uint16` payload size (0..65535)
- Payload can be UTF-8 text or binary bytes

Receiver behavior:

- Incoming notification bytes are appended to a persistent buffer
- Parser resynchronizes by scanning for `0x7E`
- A frame is emitted only when all `3 + len` bytes are present
- Partial trailing data is preserved for the next callback

Sender behavior:

- Outgoing framed messages are chunked at `max(1, mtu - 3)` bytes per write
- Optional pacing delay can be enabled to stress-test callback and flow-control timing

### Compatibility / migration

- Default mode is `BLE_SERIAL_TRANSPORT_MODE=auto`:
  - Tries framed serial commands first
  - Falls back to legacy unframed text if framed write path fails
- Legacy-only mode is available with `BLE_SERIAL_TRANSPORT_MODE=legacy`
- Force framed mode with `BLE_SERIAL_TRANSPORT_MODE=framed`

### Environment flags

- `BLE_SERIAL_TRANSPORT_MODE=auto|framed|legacy`
- `BLE_PREFERRED_MTU=185` (best-effort MTU request when backend/platform supports it)
- `BLE_CHUNK_PACING_S=0.0` (seconds delay between chunks)
- `BLE_FRAME_DEBUG=1` (prints frame length, chunk count/sizes, parser counters)

## License

This software is provided as-is for interfacing with the ESP32 toilet system.