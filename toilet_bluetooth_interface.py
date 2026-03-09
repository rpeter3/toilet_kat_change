#!/usr/bin/env python3
"""
ESP32 Toilet System Bluetooth Interface
Connects to ESP32 via BLE to read and update system parameters
"""

import asyncio
import struct
import time
import json
import platform
import os
from datetime import datetime
from typing import Dict, Any, Optional, List

try:
    from bleak import BleakClient, BleakScanner
except ImportError:
    BleakClient = Any  # type: ignore[assignment]
    BleakScanner = None  # type: ignore[assignment]

# ESP32 BLE Configuration (from toilet_kat_change.ino)
SERVICE_UUID = "5636340f-afc7-47b1-b0a8-15bc9d7d29a5"
CHARACTERISTIC_UUID = "c327b077-560f-46a1-8f35-b4ab0332fea0"
SERIAL_CHARACTERISTIC_UUID = "c327b077-560f-46a1-8f35-b4ab0332fea1"
DEVICE_NAME = "ESP32 Toilet"
FRAME_START_BYTE = 0x7E
FRAME_HEADER_SIZE = 3
MAX_FRAME_PAYLOAD = 0xFFFF


def make_frame(payload: bytes) -> bytes:
    """Encode payload into [start][len_lo][len_hi][payload]."""
    if len(payload) > MAX_FRAME_PAYLOAD:
        raise ValueError(f"Payload too large for BLE frame: {len(payload)} bytes")
    return bytes([FRAME_START_BYTE]) + struct.pack("<H", len(payload)) + payload


class BleFrameParser:
    """Incremental parser for framed BLE stream payloads."""

    def __init__(self, start_byte: int = FRAME_START_BYTE, max_payload: int = MAX_FRAME_PAYLOAD):
        self.start_byte = start_byte
        self.max_payload = max_payload
        self.buffer = bytearray()
        self.frames_parsed = 0
        self.bytes_dropped_resync = 0
        self.malformed_frame_count = 0

    def reset(self):
        self.buffer.clear()
        self.frames_parsed = 0
        self.bytes_dropped_resync = 0
        self.malformed_frame_count = 0

    def append(self, chunk: bytes) -> List[bytes]:
        self.buffer.extend(chunk)
        frames: List[bytes] = []

        while True:
            if len(self.buffer) < FRAME_HEADER_SIZE:
                break

            if self.buffer[0] != self.start_byte:
                start_index = self.buffer.find(bytes([self.start_byte]))
                if start_index == -1:
                    self.bytes_dropped_resync += len(self.buffer)
                    self.buffer.clear()
                    break
                self.bytes_dropped_resync += start_index
                del self.buffer[:start_index]
                if len(self.buffer) < FRAME_HEADER_SIZE:
                    break

            payload_len = self.buffer[1] | (self.buffer[2] << 8)
            if payload_len > self.max_payload:
                self.malformed_frame_count += 1
                self.bytes_dropped_resync += 1
                del self.buffer[0]
                continue

            frame_len = FRAME_HEADER_SIZE + payload_len
            if len(self.buffer) < frame_len:
                break

            payload = bytes(self.buffer[FRAME_HEADER_SIZE:frame_len])
            del self.buffer[:frame_len]
            self.frames_parsed += 1
            frames.append(payload)

        return frames

class ToiletSystemInterface:
    def __init__(self):
        self.client: Optional[BleakClient] = None
        self.connected = False
        self.current_params = {}
        self.serial_streaming = False
        self.preferred_mtu = int(os.getenv("BLE_PREFERRED_MTU", "185"))
        self.chunk_pacing_s = float(os.getenv("BLE_CHUNK_PACING_S", "0"))
        self.frame_debug = os.getenv("BLE_FRAME_DEBUG", "0") == "1"
        # Modes: "auto" (try framed then fallback), "framed", "legacy"
        self.serial_transport_mode = os.getenv("BLE_SERIAL_TRANSPORT_MODE", "auto").strip().lower()
        if self.serial_transport_mode not in ("auto", "framed", "legacy"):
            self.serial_transport_mode = "auto"
        self.serial_framed_active = False
        self.serial_parser = BleFrameParser()
        self.serial_frames_received = 0
        
        # Parameter definitions with descriptions and units (defaults = 1.5mil High Barrier Plastic from material_parameters.csv)
        self.param_definitions = {
            "batteryThreshold": {"description": "Battery voltage threshold", "units": "ADC", "default": 5.0},
            "K": {"description": "Temperature setpoint", "units": "°C", "default": 150.0},
            "F": {"description": "How long to feed the bag at the START of a flush", "units": "sec", "default": 6.0},
            "T": {"description": "Cooling Time", "units": "sec", "default": 60.0},
            "backupTime": {"description": "How long to back up the bag when re-opening", "units": "sec", "default": 1.0},
            "fanDuration": {"description": "How long to run the fan after feeding at the end of a flush", "units": "sec", "default": 5.0},
            "H": {"description": "Heater On time", "units": "sec", "default": 30.0},
            "continueFeeder": {"description": "How long to feed the bag at the END of a flush", "units": "sec", "default": 6.0},
            "maxOpeningTime": {"description": "Max opening time", "units": "sec", "default": 12.0},
            "typicalOpeningTime": {"description": "Typical opening time", "units": "sec", "default": 10.0},
            "MOTOR_CUT_TIME": {"description": "Motor cut duration", "units": "sec", "default": 0.5},
            "CUT_MODE_HEAT_TIME": {"description": "Additional heater time in cut mode", "units": "sec", "default": 15.0},
            "postCoolingFanDuration": {"description": "Fan duration before feed motors start in case 10", "units": "sec", "default": 5.0},
            "preFeedFan": {"description": "Fan duration before feed motor starts in case 1 and button 2", "units": "sec", "default": 2.0},
            "fanReverseTime": {"description": "Duration M3 runs in reverse after starting", "units": "sec", "default": 12.0},
            "fanReverseStartTime": {"description": "Delay before M3 reverse starts as percentage of typicalOpeningTime after M1 begins closing", "units": "%", "default": 0.0},
            "backupTimeAfterReopen": {"description": "Feed bag backup duration after mechanism motor finishes opening", "units": "sec", "default": 1.7},
            "CUT_MODE_TEMP": {"description": "Temperature to maintain for CUT_MODE_HEAT_TIME after cut motor", "units": "°C", "default": 150.0},
            "heaterLowerToleranceC": {"description": "Heater ON threshold below target (temp <= target - lower)", "units": "°C", "default": 0.0},
            "heaterUpperToleranceC": {"description": "Heater OFF threshold above target (temp >= target + upper can be negative)", "units": "°C", "default": 2.0},
            "COOL_OPEN_TEMP_C": {"description": "Open sealer when thermistor cools below this temperature", "units": "°C", "default": 80.0},
            "MAX_COOL_WAIT_S": {"description": "Safety timeout for cooling stage before forcing open", "units": "sec", "default": 180.0}
        }
        
        # Predefined parameter sets for different materials (match material_parameters.csv)
        self.parameter_sets = {
            "1.5mil High Barrier Plastic": {
                "batteryThreshold": 5.0,
                "K": 150.0,
                "F": 6.0,
                "T": 60.0,
                "backupTime": 1.0,
                "fanDuration": 5.0,
                "H": 30.0,
                "continueFeeder": 6.0,
                "maxOpeningTime": 12.0,
                "typicalOpeningTime": 10.0,
                "MOTOR_CUT_TIME": 0.5,
                "CUT_MODE_HEAT_TIME": 15.0,
                "postCoolingFanDuration": 5.0,
                "preFeedFan": 2.0,
                "fanReverseTime": 12.0,
                "fanReverseStartTime": 0.0,
                "backupTimeAfterReopen": 1.7,
                "CUT_MODE_TEMP": 150.0,
                "heaterLowerToleranceC": 0.0,
                "heaterUpperToleranceC": 2.0,
                "COOL_OPEN_TEMP_C": 80.0,
                "MAX_COOL_WAIT_S": 180.0
            },
            "Compostable 1.5mil": {
                "batteryThreshold": 5.0,
                "K": 100.0,
                "F": 6.0,
                "T": 40.0,
                "backupTime": 1.0,
                "fanDuration": 5.0,
                "H": 20.0,
                "continueFeeder": 6.0,
                "maxOpeningTime": 12.0,
                "typicalOpeningTime": 10.0,
                "MOTOR_CUT_TIME": 0.5,
                "CUT_MODE_HEAT_TIME": 10.0,
                "postCoolingFanDuration": 1.0,
                "preFeedFan": 1.5,
                "fanReverseTime": 9.0,
                "fanReverseStartTime": 0.0,
                "backupTimeAfterReopen": 1.7,
                "CUT_MODE_TEMP": 100.0,
                "heaterLowerToleranceC": 0.0,
                "heaterUpperToleranceC": 2.0,
                "COOL_OPEN_TEMP_C": 80.0,
                "MAX_COOL_WAIT_S": 180.0
            }
        }
        
        # Parameter order (22 values, as expected by ESP32 BLE)
        self.param_order = [
            "batteryThreshold", "K", "F", "T", "backupTime",
            "fanDuration", "H", "continueFeeder", "maxOpeningTime", "typicalOpeningTime",
            "MOTOR_CUT_TIME", "CUT_MODE_HEAT_TIME", "postCoolingFanDuration", "preFeedFan",
            "fanReverseTime", "fanReverseStartTime", "backupTimeAfterReopen", "CUT_MODE_TEMP",
            "heaterLowerToleranceC", "heaterUpperToleranceC", "COOL_OPEN_TEMP_C", "MAX_COOL_WAIT_S"
        ]
        self.min_heater_tolerance_gap_c = 2.0
        self.hardware_components: List[str] = [
            "CONTROL_PANEL",
            "HEATING_ELEMENT",
            "MAIN_CIRCUIT_BOARD",
            "VACUUM_FAN",
            "FEED_MOTOR",
            "MECHANISM_MOTOR",
            "THERMISTOR",
            "BATTERY",
            "FACTORY_SOFTWARE_DATE",
            "FACTORY_SOFTWARE_VERSION_NUMBER",
        ]

    async def _request_preferred_mtu(self):
        """Best-effort MTU request. Safe no-op when backend doesn't support it."""
        if not self.client:
            return

        if platform.system().lower() != "android":
            return

        request_candidates = [
            getattr(self.client, "request_mtu", None),
            getattr(getattr(self.client, "_backend", None), "request_mtu", None),
            getattr(getattr(self.client, "_backend", None), "set_mtu", None),
        ]

        for request_mtu in request_candidates:
            if callable(request_mtu):
                try:
                    negotiated = await request_mtu(self.preferred_mtu)
                    if negotiated:
                        print(f"MTU negotiated: requested {self.preferred_mtu}, got {negotiated}")
                    else:
                        print(f"MTU request issued for {self.preferred_mtu}")
                    return
                except Exception as mtu_error:
                    print(f"MTU request failed, continuing with default MTU: {mtu_error}")
                    return

        print("MTU request API not available on this backend; using negotiated default")

    def _resolve_chunk_size(self) -> int:
        mtu = 23
        if self.client:
            mtu = getattr(self.client, "mtu_size", 23) or 23
        return max(1, mtu - 3)

    async def _write_ble_payload_chunked(self, characteristic_uuid: str, payload: bytes, response: bool = False):
        if not self.client:
            raise RuntimeError("Cannot write BLE payload while disconnected")

        chunk_size = self._resolve_chunk_size()
        chunks = [payload[i:i + chunk_size] for i in range(0, len(payload), chunk_size)] or [b""]

        if self.frame_debug:
            chunk_sizes = [len(chunk) for chunk in chunks]
            print(
                "BLE chunked write: "
                f"payload={len(payload)} bytes, chunks={len(chunks)}, chunk_sizes={chunk_sizes}"
            )

        for chunk in chunks:
            await self.client.write_gatt_char(characteristic_uuid, chunk, response=response)
            if self.chunk_pacing_s > 0:
                await asyncio.sleep(self.chunk_pacing_s)

    async def send_framed_payload(self, characteristic_uuid: str, payload: bytes, response: bool = False):
        frame = make_frame(payload)
        if self.frame_debug:
            print(f"Sending BLE frame: payload={len(payload)} bytes frame={len(frame)} bytes")
        await self._write_ble_payload_chunked(characteristic_uuid, frame, response=response)

    async def _send_serial_command(self, command: str, prefer_framed: bool, allow_legacy_fallback: bool = True):
        command_bytes = command.encode("utf-8")
        if prefer_framed:
            try:
                await self.send_framed_payload(SERIAL_CHARACTERISTIC_UUID, command_bytes)
                self.serial_framed_active = True
                return
            except Exception as framed_error:
                if not allow_legacy_fallback:
                    raise
                print(f"Framed serial command failed, falling back to legacy write: {framed_error}")

        await self.client.write_gatt_char(SERIAL_CHARACTERISTIC_UUID, command_bytes)
        self.serial_framed_active = False

    async def scan_for_device(self) -> Optional[str]:
        """Scan for ESP32 Toilet device"""
        if BleakScanner is None:
            print("Bleak is not installed; BLE scanning unavailable.")
            return None
        print("Scanning for ESP32 Toilet device...")
        devices = await BleakScanner.discover(timeout=10.0)
        
        for device in devices:
            if device.name == DEVICE_NAME:
                print(f"Found device: {device.name} at {device.address}")
                return device.address
        
        print("ESP32 Toilet device not found!")
        return None

    async def connect(self, address: str) -> bool:
        """Connect to ESP32 device"""
        try:
            self.client = BleakClient(address)
            await self.client.connect()
            await self._request_preferred_mtu()
            self.connected = True
            print(f"Connected to {DEVICE_NAME} at {address}")
            print(
                "BLE transport config: "
                f"mode={self.serial_transport_mode}, preferred_mtu={self.preferred_mtu}, "
                f"chunk_pacing_s={self.chunk_pacing_s}, frame_debug={self.frame_debug}"
            )
            
            # Wait a moment for ESP32 to fully initialize
            await asyncio.sleep(1.0)
            print("Connection stabilized, ready to read parameters")
            return True
        except Exception as e:
            print(f"Failed to connect: {e}")
            return False

    async def disconnect(self):
        """Disconnect from ESP32 device"""
        if self.client and self.connected:
            # Stop serial streaming before disconnecting
            if self.serial_streaming:
                await self.stop_serial_streaming()
            await self.client.disconnect()
            self.connected = False
            print("Disconnected from ESP32")

    async def read_current_params(self) -> Dict[str, Any]:
        """Read current parameter values from ESP32"""
        if not self.connected:
            print("Not connected to device")
            return {}
        
        try:
            # Read the characteristic value
            data = await self.client.read_gatt_char(CHARACTERISTIC_UUID)
            message = data.decode('utf-8')
            print(f"Received message: {message}")
            
            # Parse comma-separated values
            values = message.split(',')
            params = {}
            
            for i, param_name in enumerate(self.param_order):
                if i < len(values):
                    try:
                        params[param_name] = float(values[i])
                    except ValueError:
                        print(f"Invalid value for {param_name}: {values[i]}")
                        params[param_name] = float(self.param_definitions[param_name]["default"])
                else:
                    params[param_name] = float(self.param_definitions[param_name]["default"])
            
            self.current_params = params
            return params
            
        except Exception as e:
            print(f"Failed to read parameters: {e}")
            return {}

    def _build_effective_params(self, new_params: Dict[str, Any]) -> Dict[str, float]:
        """Merge updates with current/default values so cross-field validation is deterministic."""
        effective_params: Dict[str, float] = {}
        for param_name in self.param_order:
            if param_name in new_params:
                candidate_value = new_params[param_name]
            else:
                candidate_value = self.current_params.get(
                    param_name,
                    self.param_definitions[param_name]["default"],
                )
            effective_params[param_name] = float(candidate_value)
        return effective_params

    def _validate_heater_tolerance_gap(self, new_params: Dict[str, Any]) -> bool:
        """Require heaterUpperToleranceC - heaterLowerToleranceC >= configured minimum gap."""
        try:
            effective_params = self._build_effective_params(new_params)
            lower_tol = effective_params["heaterLowerToleranceC"]
            upper_tol = effective_params["heaterUpperToleranceC"]
            gap = upper_tol - lower_tol
            if gap < self.min_heater_tolerance_gap_c:
                print(
                    "Parameter update rejected: heater tolerance gap must be at least "
                    f"{self.min_heater_tolerance_gap_c}°C "
                    f"(lower={lower_tol}, upper={upper_tol}, gap={gap})."
                )
                return False
            return True
        except (TypeError, ValueError, KeyError) as validation_error:
            print(f"Parameter update rejected: invalid tolerance values ({validation_error}).")
            return False

    async def update_params(self, new_params: Dict[str, Any]) -> bool:
        """Update parameters on ESP32"""
        if not self.connected:
            print("Not connected to device")
            return False
        if not self._validate_heater_tolerance_gap(new_params):
            return False
        
        try:
            # Create comma-separated message (all values as float strings for BLE)
            message_parts = []
            for param_name in self.param_order:
                if param_name in new_params:
                    val = new_params[param_name]
                else:
                    val = self.current_params.get(param_name, self.param_definitions[param_name]["default"])
                message_parts.append(str(float(val)))
            
            message = ",".join(message_parts)
            print(f"Sending message: {message}")
            
            # Write to characteristic
            await self.client.write_gatt_char(CHARACTERISTIC_UUID, message.encode('utf-8'))
            # Read firmware feedback after write so flush-blocked updates are reported to user.
            await asyncio.sleep(0.1)
            response = ""
            try:
                data = await self.client.read_gatt_char(CHARACTERISTIC_UUID)
                response = data.decode('utf-8', errors='ignore').strip()
            except Exception as read_error:
                print(f"Warning: unable to read firmware response after update: {read_error}")

            if response == "PARAM_UPDATE_BLOCKED_FLUSH":
                print("Parameter update rejected: flush in progress (firmware blocked this write).")
                return False

            print("Parameters updated successfully")
            return True
            
        except Exception as e:
            print(f"Failed to update parameters: {e}")
            return False

    async def update_single_param(self, param_name: str, new_value: Any) -> bool:
        """Update a single parameter on ESP32, leaving others unchanged"""
        if not self.connected:
            print("Not connected to device")
            return False
        
        if param_name not in self.param_order:
            print(f"Invalid parameter name: {param_name}")
            return False
        
        # Create a dict with only the single parameter to update
        single_param_dict = {param_name: new_value}
        return await self.update_params(single_param_dict)

    def display_params_table(self, params: Dict[str, Any]):
        """Display parameters in a formatted table"""
        print("\n" + "="*95)
        print("ESP32 TOILET SYSTEM PARAMETERS")
        print("="*95)
        print(f"{'Parameter':<16} {'Description':<30} {'Units':<8} {'Current':<12} {'Default':<12}")
        print("-"*95)
        
        for param_name in self.param_order:
            desc = self.param_definitions[param_name]["description"]
            units = self.param_definitions[param_name]["units"]
            current = params.get(param_name, "N/A")
            default = self.param_definitions[param_name]["default"]
            
            print(f"{param_name:<16} {desc:<30} {units:<8} {str(current):<12} {str(default):<12}")
        
        print("="*95)

    def display_params_numbered_table(self, params: Dict[str, Any]):
        """Display parameters in a numbered table for selection"""
        print("\n" + "="*100)
        print("ESP32 TOILET SYSTEM PARAMETERS - SELECT PARAMETER TO UPDATE")
        print("="*100)
        print(f"{'#':<4} {'Parameter':<16} {'Description':<30} {'Units':<8} {'Current':<12}")
        print("-"*100)
        
        for i, param_name in enumerate(self.param_order, 1):
            desc = self.param_definitions[param_name]["description"]
            units = self.param_definitions[param_name]["units"]
            current = params.get(param_name, "N/A")
            
            print(f"{i:<4} {param_name:<16} {desc:<30} {units:<8} {str(current):<12}")
        
        print("="*100)

    def save_params_to_file(self, params: Dict[str, Any], filename: str = "toilet_params.json"):
        """Save parameters to JSON file"""
        try:
            with open(filename, 'w') as f:
                json.dump(params, f, indent=2)
            print(f"Parameters saved to {filename}")
        except Exception as e:
            print(f"Failed to save parameters: {e}")

    def load_params_from_file(self, filename: str = "toilet_params.json") -> Dict[str, Any]:
        """Load parameters from JSON file"""
        try:
            with open(filename, 'r') as f:
                params = json.load(f)
            print(f"Parameters loaded from {filename}")
            return params
        except Exception as e:
            print(f"Failed to load parameters: {e}")
            return {}

    def load_parameter_set(self, set_name: str) -> Dict[str, Any]:
        """Load a predefined parameter set"""
        if set_name in self.parameter_sets:
            print(f"Loaded parameter set: {set_name}")
            return self.parameter_sets[set_name].copy()
        else:
            print(f"Parameter set '{set_name}' not found!")
            return {}

    def display_parameter_sets(self):
        """Display available parameter sets"""
        print("\n" + "="*60)
        print("AVAILABLE PARAMETER SETS")
        print("="*60)
        
        for i, set_name in enumerate(self.parameter_sets.keys(), 1):
            print(f"{i}. {set_name}")
        
        print("="*60)

    async def start_serial_streaming(self) -> bool:
        """Start serial streaming from ESP32"""
        if not self.connected:
            print("Not connected to device")
            return False
        
        try:
            print("DEBUG: Sending START_SERIAL command to ESP32")
            self.serial_parser.reset()
            self.serial_frames_received = 0

            prefer_framed = self.serial_transport_mode in ("auto", "framed")
            allow_fallback = self.serial_transport_mode == "auto"
            await self._send_serial_command(
                "START_SERIAL",
                prefer_framed=prefer_framed,
                allow_legacy_fallback=allow_fallback,
            )
            self.serial_streaming = True
            transport = "framed" if self.serial_framed_active else "legacy"
            print(f"Serial streaming started ({transport} transport)")
            return True
        except Exception as e:
            print(f"Failed to start serial streaming: {e}")
            return False

    async def stop_serial_streaming(self) -> bool:
        """Stop serial streaming from ESP32"""
        if not self.connected:
            return False
        
        try:
            await self._send_serial_command(
                "STOP_SERIAL",
                prefer_framed=self.serial_framed_active,
                allow_legacy_fallback=True,
            )
            self.serial_streaming = False
            print("Serial streaming stopped")
            return True
        except Exception as e:
            print(f"Failed to stop serial streaming: {e}")
            return False

    async def monitor_serial_stream(self):
        """Monitor serial stream from ESP32 (automatically starts streaming)"""
        if not self.connected:
            print("Not connected to device")
            return
        
        print("Starting serial stream monitoring...")
        print("Press Ctrl+C to stop monitoring and return to menu")
        print("-" * 50)
        
        try:
            # Start serial streaming on ESP32
            await self.start_serial_streaming()
            
            # Enable notifications for serial characteristic
            await self.client.start_notify(SERIAL_CHARACTERISTIC_UUID, self.serial_notification_handler)
            
            # Read and display current parameters
            print("\nReading current parameters from ESP32...")
            params = await self.read_current_params()
            if params:
                self.display_params_table(params)
            else:
                print("Failed to read parameters")
            
            print("\nSerial monitoring active. Press Ctrl+C to stop...")
            
            # Simple infinite loop - Ctrl+C will interrupt
            while self.serial_streaming:
                await asyncio.sleep(0.1)
                
        except KeyboardInterrupt:
            print("\nStopping serial monitoring...")
        except asyncio.CancelledError:
            print("\nSerial monitoring cancelled")
        except Exception as e:
            print(f"Error monitoring serial stream: {e}")
        finally:
            # Always stop streaming and notifications
            try:
                await self.client.stop_notify(SERIAL_CHARACTERISTIC_UUID)
                await self.stop_serial_streaming()
                print("Serial monitoring stopped")
            except:
                pass

    def serial_notification_handler(self, sender, data):
        """Handle serial data notifications from ESP32"""
        try:
            if not data:
                return

            payloads: List[bytes] = []
            should_attempt_framed_parse = self.serial_transport_mode != "legacy"
            if should_attempt_framed_parse:
                payloads = self.serial_parser.append(bytes(data))

            if payloads:
                self.serial_framed_active = True
                self.serial_frames_received += len(payloads)
                for payload in payloads:
                    message = payload.decode("utf-8", errors="replace")
                    print(f"{message}", end="", flush=True)
                if self.frame_debug:
                    print(
                        "\n[FRAME_DEBUG] "
                        f"frames_received={self.serial_frames_received} "
                        f"frames_parsed={self.serial_parser.frames_parsed} "
                        f"resync_bytes_dropped={self.serial_parser.bytes_dropped_resync} "
                        f"malformed_frames={self.serial_parser.malformed_frame_count} "
                        f"buffered_partial={len(self.serial_parser.buffer)}"
                    )
                return

            # Legacy path (for older firmware that does not frame serial notifications).
            if not self.serial_framed_active:
                message = data.decode("utf-8", errors="replace")
                print(f"{message}", end="", flush=True)
        except Exception as e:
            print(f"Error decoding serial data: {e}")
            print(f"Raw data: {data}")

    async def get_dev_mode_status(self) -> Optional[int]:
        """Read current DEV mode from firmware (returns 0/1 or None on failure)."""
        if not self.connected:
            print("Not connected to device")
            return None

        try:
            for _ in range(3):
                await self.client.write_gatt_char(CHARACTERISTIC_UUID, b"GET_DEV_MODE")
                await asyncio.sleep(0.15)
                data = await self.client.read_gatt_char(CHARACTERISTIC_UUID)
                response = data.decode("utf-8").strip()
                if response.startswith("DEV_MODE:"):
                    mode_str = response.split(":", 1)[1].strip()
                    if mode_str in ("0", "1"):
                        return int(mode_str)
                await asyncio.sleep(0.1)
            print("Failed to read DEV mode status from firmware")
            return None
        except Exception as e:
            print(f"Failed to read DEV mode status: {e}")
            return None

    async def get_flush_count(self) -> Optional[int]:
        """Read lifetime flush count from firmware (returns int or None on failure)."""
        response = await self._send_command_and_read_response("GET_FLUSH_COUNT")
        if not response:
            print("No response from firmware for GET_FLUSH_COUNT")
            return None
        if not response.startswith("FLUSH_COUNT:"):
            print(f"Unexpected GET_FLUSH_COUNT response: {response}")
            return None

        count_str = response.split(":", 1)[1].strip()
        try:
            flush_count = int(count_str)
        except ValueError:
            print(f"Malformed flush count payload: {response}")
            return None

        if flush_count < 0:
            print(f"Invalid negative flush count payload: {response}")
            return None
        return flush_count

    async def set_dev_mode(self, new_mode: int) -> bool:
        """Set firmware DEV mode to 0 or 1."""
        if not self.connected:
            print("Not connected to device")
            return False
        if new_mode not in (0, 1):
            print("Invalid DEV mode value. Use 0 or 1.")
            return False

        try:
            command = f"SET_DEV_MODE:{new_mode}"
            await self.client.write_gatt_char(CHARACTERISTIC_UUID, command.encode("utf-8"))
            await asyncio.sleep(0.15)

            data = await self.client.read_gatt_char(CHARACTERISTIC_UUID)
            response = data.decode("utf-8").strip()

            if response.startswith("SET_DEV_MODE_ACK:"):
                ack_value = response.split(":", 1)[1].strip()
                return ack_value == str(new_mode)

            if response.startswith("SET_DEV_MODE_ERR:"):
                print(f"Firmware rejected DEV mode update: {response}")
                return False

            # If response is not an ACK/ERR, verify by reading back.
            verify_mode = await self.get_dev_mode_status()
            return verify_mode == new_mode
        except Exception as e:
            print(f"Failed to set DEV mode: {e}")
            return False

    async def _send_command_and_read_response(self, command: str, retries: int = 3, response_delay_s: float = 0.15) -> Optional[str]:
        if not self.connected:
            print("Not connected to device")
            return None
        try:
            for _ in range(retries):
                await self.client.write_gatt_char(CHARACTERISTIC_UUID, command.encode("utf-8"))
                await asyncio.sleep(response_delay_s)
                data = await self.client.read_gatt_char(CHARACTERISTIC_UUID)
                response = data.decode("utf-8").strip()
                if response:
                    return response
                await asyncio.sleep(0.1)
            return None
        except Exception as e:
            print(f"Command failed ({command}): {e}")
            return None

    # HWCFG command reference (exact command strings):
    #   HWCFG_GET_CAPS
    #   HWCFG_GET_ACTIVE_CONFIG
    #   HWCFG_GET_LAST_GOOD_CONFIG
    #   HWCFG_PROFILE_PUT:<profile_id>|component=<name>|version=<ver>|k1=v1;k2=v2;...
    #   HWCFG_PROFILE_GET:<component>|version=<ver>
    #   HWCFG_PROFILE_LIST
    #   HWCFG_VALIDATE_CHANGE:<component>|new_version=<ver>
    #   HWCFG_APPLY_CHANGE:<component>|new_version=<ver>|install_date=<YYYY-MM-DD>|desc=<text>
    #   HWCFG_ROLLBACK_LAST_GOOD
    #
    # Response patterns:
    #   HWCFG_CAPS:V1|PROFILE_STORE|TXN_APPLY|ROLLBACK
    #   HWCFG_ACTIVE:<component>=<ver>;...|profile_id=<id>|validated=1
    #   HWCFG_LAST_GOOD:<component>=<ver>;...|profile_id=<id>
    #   HWCFG_VALIDATE_OK:<component>|version=<ver>|profile_id=<id>
    #   HWCFG_VALIDATE_ERR:<reason_code>
    #   HWCFG_APPLY_ACK:<component>|version=<ver>
    #   HWCFG_APPLY_ERR:<reason_code>
    #   HWCFG_ROLLBACK_ACK
    #   HWCFG_ROLLBACK_ERR:<reason_code>
    def _is_valid_iso_date(self, value: str) -> bool:
        try:
            datetime.strptime(value, "%Y-%m-%d")
            return True
        except ValueError:
            return False

    def _parse_component_versions_blob(self, value: str) -> Dict[str, str]:
        parsed: Dict[str, str] = {}
        for token in value.split(";"):
            token = token.strip()
            if not token or "=" not in token:
                continue
            key, version = token.split("=", 1)
            parsed[key.strip()] = version.strip()
        return parsed

    async def hwcfg_get_caps(self) -> Optional[str]:
        response = await self._send_command_and_read_response("HWCFG_GET_CAPS")
        if not response or not response.startswith("HWCFG_CAPS:"):
            print(f"Unexpected HWCFG caps response: {response}")
            return None
        return response

    async def hwcfg_get_active_config(self) -> Optional[Dict[str, Any]]:
        response = await self._send_command_and_read_response("HWCFG_GET_ACTIVE_CONFIG")
        if not response or not response.startswith("HWCFG_ACTIVE:"):
            print(f"Unexpected active config response: {response}")
            return None
        payload = response.split(":", 1)[1]
        pieces = payload.split("|")
        component_map = self._parse_component_versions_blob(pieces[0] if pieces else "")
        profile_id = ""
        validated = None
        for piece in pieces[1:]:
            if piece.startswith("profile_id="):
                profile_id = piece.split("=", 1)[1]
            elif piece.startswith("validated="):
                val = piece.split("=", 1)[1].strip()
                validated = (val == "1")
        return {"components": component_map, "profile_id": profile_id, "validated": validated}

    async def hwcfg_get_last_good_config(self) -> Optional[Dict[str, Any]]:
        response = await self._send_command_and_read_response("HWCFG_GET_LAST_GOOD_CONFIG")
        if not response or not response.startswith("HWCFG_LAST_GOOD:"):
            print(f"Unexpected last-good config response: {response}")
            return None
        payload = response.split(":", 1)[1]
        pieces = payload.split("|")
        component_map = self._parse_component_versions_blob(pieces[0] if pieces else "")
        profile_id = ""
        for piece in pieces[1:]:
            if piece.startswith("profile_id="):
                profile_id = piece.split("=", 1)[1]
        return {"components": component_map, "profile_id": profile_id}

    async def hwcfg_profile_list(self) -> Optional[List[str]]:
        response = await self._send_command_and_read_response("HWCFG_PROFILE_LIST")
        if not response or not response.startswith("HWCFG_PROFILE_LIST:"):
            print(f"Unexpected profile list response: {response}")
            return None
        payload = response.split(":", 1)[1].strip()
        if not payload:
            return []
        return [item.strip() for item in payload.split(",") if item.strip()]

    async def hwcfg_profile_get(self, component: str, version: str) -> Optional[str]:
        command = f"HWCFG_PROFILE_GET:{component}|version={version}"
        response = await self._send_command_and_read_response(command)
        if not response:
            print("No response from firmware for HWCFG_PROFILE_GET")
            return None
        if response.startswith("HWCFG_VALIDATE_ERR:"):
            print(f"Firmware rejected profile get: {response}")
            return None
        if not response.startswith("HWCFG_PROFILE:"):
            print(f"Unexpected profile get response: {response}")
            return None
        return response

    async def hwcfg_profile_put(self, profile_id: str, component: str, version: str, param_pairs: Dict[str, float]) -> bool:
        if component not in self.hardware_components:
            print("Invalid component")
            return False
        if not profile_id.strip() or not version.strip():
            print("profile_id and version are required")
            return False
        if not param_pairs:
            print("At least one parameter pair is required")
            return False
        param_blob = ";".join([f"{k}={v}" for k, v in param_pairs.items()])
        command = (
            f"HWCFG_PROFILE_PUT:{profile_id.strip()}|component={component.strip()}|"
            f"version={version.strip()}|{param_blob}"
        )
        response = await self._send_command_and_read_response(command)
        if not response:
            print("No response from firmware for HWCFG_PROFILE_PUT")
            return False
        if response.startswith("HWCFG_VALIDATE_OK:"):
            return True
        print(f"Profile put failed: {response}")
        return False

    async def hwcfg_validate_change(self, component: str, new_version: str) -> Optional[str]:
        command = f"HWCFG_VALIDATE_CHANGE:{component}|new_version={new_version}"
        response = await self._send_command_and_read_response(command)
        if not response:
            print("No response from firmware for HWCFG_VALIDATE_CHANGE")
            return None
        if response.startswith("HWCFG_VALIDATE_OK:"):
            return response
        print(f"Validation failed: {response}")
        return None

    async def hwcfg_apply_change(self, component: str, new_version: str, install_date: str, desc: str) -> bool:
        if not self._is_valid_iso_date(install_date):
            print("install_date must use YYYY-MM-DD")
            return False
        command = (
            f"HWCFG_APPLY_CHANGE:{component}|new_version={new_version}|"
            f"install_date={install_date}|desc={desc}"
        )
        response = await self._send_command_and_read_response(command)
        if not response:
            print("No response from firmware for HWCFG_APPLY_CHANGE")
            return False
        if response.startswith("HWCFG_APPLY_ACK:"):
            return True
        print(f"Apply failed: {response}")
        return False

    async def hwcfg_rollback_last_good(self) -> bool:
        response = await self._send_command_and_read_response("HWCFG_ROLLBACK_LAST_GOOD")
        if not response:
            print("No response from firmware for HWCFG_ROLLBACK_LAST_GOOD")
            return False
        if response == "HWCFG_ROLLBACK_ACK":
            return True
        print(f"Rollback failed: {response}")
        return False

    async def get_hw_component(self, component: str) -> Optional[Dict[str, str]]:
        component = component.strip().upper()
        if component not in self.hardware_components:
            print(f"Unsupported component: {component}")
            return None

        response = await self._send_command_and_read_response(f"GET_HW_COMPONENT:{component}")
        if not response:
            print("No response from firmware for GET_HW_COMPONENT")
            return None
        if response.startswith("HW_COMPONENT_ERR:"):
            print(f"Firmware rejected GET_HW_COMPONENT: {response}")
            return None
        if not response.startswith("HW_COMPONENT:"):
            print(f"Unexpected GET_HW_COMPONENT response: {response}")
            return None

        payload = response.split(":", 1)[1]
        parts = payload.split("|")
        if len(parts) != 7:
            print(f"Malformed HW_COMPONENT payload: {response}")
            return None
        return {
            "component": parts[0],
            "current_version": parts[1],
            "current_description": parts[2],
            "install_date": parts[3],
            "previous_version": parts[4],
            "previous_description": parts[5],
            "previous_install_date": parts[6],
        }

    async def get_hw_matrix(self) -> Optional[Dict[str, Dict[str, str]]]:
        matrix: Dict[str, Dict[str, str]] = {}
        for component in self.hardware_components:
            row = await self.get_hw_component(component)
            if row is None:
                print(f"Failed to read component record: {component}")
                return None
            matrix[component] = row
        return matrix

    async def set_hw_component(self, component: str, version: str, install_date: str, description: str) -> bool:
        component = component.strip().upper()
        version = version.strip()
        install_date = install_date.strip()
        description = description.strip()

        if component not in self.hardware_components:
            print(f"Unsupported component: {component}")
            return False
        if not version:
            print("Version cannot be empty")
            return False
        if not description:
            print("Description cannot be empty")
            return False
        if ":" in version or ":" in description or "|" in version or "|" in description:
            print("Version/description cannot contain ':' or '|'")
            return False
        if not self._is_valid_iso_date(install_date):
            print("install_date must use ISO format YYYY-MM-DD")
            return False

        command = f"SET_HW_COMPONENT:{component}:{version}:{install_date}:{description}"
        response = await self._send_command_and_read_response(command)
        if not response:
            print("No response from firmware for SET_HW_COMPONENT")
            return False
        if response.startswith("SET_HW_COMPONENT_ACK:"):
            return response.split(":", 1)[1].strip().upper() == component
        if response.startswith("SET_HW_COMPONENT_ERR:"):
            print(f"Firmware rejected SET_HW_COMPONENT: {response}")
            return False

        # Fallback: verify by reading back.
        readback = await self.get_hw_component(component)
        if readback is None:
            return False
        return (
            readback["current_version"] == version
            and readback["install_date"] == install_date
            and readback["current_description"] == description
        )

    def display_hw_matrix_table(self, matrix: Dict[str, Dict[str, str]]) -> None:
        print("\nHardware Matrix")
        print("=" * 150)
        print(
            f"{'Component':<32} {'CurrentVersion':<16} {'InstallDate':<12} "
            f"{'PreviousVersion':<16} {'PreviousDate':<12}"
        )
        print("-" * 150)
        for component in self.hardware_components:
            row = matrix.get(component, {})
            print(
                f"{component:<32} "
                f"{row.get('current_version', ''):<16} "
                f"{row.get('install_date', ''):<12} "
                f"{row.get('previous_version', ''):<16} "
                f"{row.get('previous_install_date', ''):<12}"
            )
            current_desc = row.get("current_description", "")
            prev_desc = row.get("previous_description", "")
            print(f"  current_desc: {current_desc}")
            if prev_desc:
                print(f"  previous_desc: {prev_desc}")
        print("=" * 150)

async def main():
    """Main program loop"""
    interface = ToiletSystemInterface()
    
    print("ESP32 Toilet System Bluetooth Interface")
    print("="*50)
    
    # Scan for device
    address = await interface.scan_for_device()
    if not address:
        return
    
    # Connect to device
    if not await interface.connect(address):
        return
    
    try:
        while True:
            print("\nOptions:")
            print("1. Read current parameters")
            print("2. Update parameters")
            print("3. Save parameters to file")
            print("4. Load parameters from file")
            print("5. Load predefined parameter set")
            print("6. Display parameter definitions")
            print("7. Monitor serial stream")
            print("8. Exit")
            print("9. Update single parameter")
            print("10. Toggle DEV mode")
            print("11. Read hardware matrix")
            print("12. Update hardware component")
            print("13. HWCFG capabilities")
            print("14. HWCFG active/last-good config")
            print("15. HWCFG profile list/get")
            print("16. HWCFG profile put")
            print("17. HWCFG validate + apply change")
            print("18. HWCFG rollback last good")
            print("19. Read flush count")
            
            choice = input("\nEnter your choice (1-19): ").strip()
            
            if choice == "1":
                print("\nReading current parameters...")
                params = await interface.read_current_params()
                if params:
                    interface.display_params_table(params)
                else:
                    print("Failed to read parameters")
            
            elif choice == "2":
                print("\nUpdating parameters...")
                print("Enter new values (press Enter to keep current value):")
                
                new_params = {}
                for param_name in interface.param_order:
                    current = interface.current_params.get(param_name, interface.param_definitions[param_name]["default"])
                    desc = interface.param_definitions[param_name]["description"]
                    units = interface.param_definitions[param_name]["units"]
                    
                    user_input = input(f"{param_name} ({desc}) [{units}] [current: {current}]: ").strip()
                    
                    if user_input:
                        try:
                            new_params[param_name] = float(user_input)
                        except ValueError:
                            print(f"Invalid value for {param_name}, keeping current value")
                    else:
                        new_params[param_name] = float(current)
                
                if await interface.update_params(new_params):
                    interface.current_params.update(new_params)
                    print("Parameters updated successfully!")
                else:
                    print("Failed to update parameters")
            
            elif choice == "3":
                if interface.current_params:
                    filename = input("Enter filename (default: toilet_params.json): ").strip()
                    if not filename:
                        filename = "toilet_params.json"
                    interface.save_params_to_file(interface.current_params, filename)
                else:
                    print("No parameters to save. Read parameters first.")
            
            elif choice == "4":
                filename = input("Enter filename (default: toilet_params.json): ").strip()
                if not filename:
                    filename = "toilet_params.json"
                params = interface.load_params_from_file(filename)
                if params:
                    interface.current_params = params
                    interface.display_params_table(params)
            
            elif choice == "5":
                print("\nLoading predefined parameter set...")
                interface.display_parameter_sets()
                
                set_names = list(interface.parameter_sets.keys())
                try:
                    set_choice = int(input(f"\nSelect parameter set (1-{len(set_names)}): ").strip())
                    if 1 <= set_choice <= len(set_names):
                        selected_set = set_names[set_choice - 1]
                        params = interface.load_parameter_set(selected_set)
                        if params:
                            interface.display_params_table(params)
                            
                            # Automatically apply parameter set to ESP32
                            print("\nApplying parameter set to ESP32...")
                            if await interface.update_params(params):
                                interface.current_params = params
                                print("Parameter set applied successfully!")
                            else:
                                print("Failed to apply parameter set")
                    else:
                        print("Invalid selection")
                except ValueError:
                    print("Invalid input. Please enter a number.")
            
            elif choice == "6":
                interface.display_params_table(interface.param_definitions)
            
            elif choice == "7":
                print("\nStarting serial stream monitoring...")
                await interface.monitor_serial_stream()
            
            elif choice == "8":
                break
            
            elif choice == "9":
                print("\nUpdating single parameter...")
                
                # Ensure we have current parameters
                if not interface.current_params:
                    print("Reading current parameters first...")
                    params = await interface.read_current_params()
                    if not params:
                        print("Failed to read parameters. Cannot proceed.")
                        continue
                
                # Display numbered parameter table
                interface.display_params_numbered_table(interface.current_params)
                
                # Get parameter selection
                try:
                    param_num = int(input(f"\nSelect parameter number (1-{len(interface.param_order)}): ").strip())
                    if 1 <= param_num <= len(interface.param_order):
                        selected_param = interface.param_order[param_num - 1]
                        desc = interface.param_definitions[selected_param]["description"]
                        units = interface.param_definitions[selected_param]["units"]
                        current = interface.current_params.get(selected_param, interface.param_definitions[selected_param]["default"])
                        
                        # Get new value
                        user_input = input(f"\n{selected_param} ({desc}) [{units}] [current: {current}]: ").strip()
                        
                        if user_input:
                            try:
                                new_value = float(user_input)
                                if await interface.update_single_param(selected_param, new_value):
                                    interface.current_params[selected_param] = new_value
                                    print(f"\nParameter '{selected_param}' updated successfully to {new_value}!")
                                else:
                                    print(f"\nFailed to update parameter '{selected_param}'")
                            except ValueError:
                                print(f"Invalid value. Please enter a valid number.")
                        else:
                            print("No value entered. Parameter not updated.")
                    else:
                        print(f"Invalid selection. Please enter a number between 1 and {len(interface.param_order)}.")
                except ValueError:
                    print("Invalid input. Please enter a number.")

            elif choice == "10":
                print("\nChecking firmware DEV mode...")
                current_mode = await interface.get_dev_mode_status()
                if current_mode is None:
                    print("Unable to read current DEV mode from firmware.")
                    continue

                current_label = "ON (1)" if current_mode == 1 else "OFF (0)"
                print(f"Current DEV mode: {current_label}")
                new_mode_input = input("Enter new DEV mode value (1=ON, 0=OFF, Enter to cancel): ").strip()
                if not new_mode_input:
                    print("DEV mode change cancelled.")
                    continue
                if new_mode_input not in ("0", "1"):
                    print("Invalid input. Enter 1 or 0.")
                    continue

                requested_mode = int(new_mode_input)
                if requested_mode == current_mode:
                    print("Requested value is already set. No change made.")
                    continue

                confirm = input(f"Change DEV mode to {requested_mode}? (y/N): ").strip().lower()
                if confirm != "y":
                    print("DEV mode change cancelled.")
                    continue

                if await interface.set_dev_mode(requested_mode):
                    updated_mode = await interface.get_dev_mode_status()
                    if updated_mode is None:
                        print("DEV mode update sent, but readback failed.")
                    else:
                        updated_label = "ON (1)" if updated_mode == 1 else "OFF (0)"
                        print(f"DEV mode updated successfully. Current firmware DEV mode: {updated_label}")
                else:
                    print("Failed to update DEV mode")

            elif choice == "11":
                print("\nReading hardware matrix...")
                matrix = await interface.get_hw_matrix()
                if matrix is None:
                    print("Failed to read hardware matrix")
                else:
                    interface.display_hw_matrix_table(matrix)

            elif choice == "12":
                print("\nUpdate hardware component")
                print("Supported components:")
                for idx, component in enumerate(interface.hardware_components, start=1):
                    print(f"{idx}. {component}")

                selected_component = input("Enter component name (or number): ").strip()
                component_name = selected_component.upper()
                if selected_component.isdigit():
                    index = int(selected_component) - 1
                    if 0 <= index < len(interface.hardware_components):
                        component_name = interface.hardware_components[index]
                    else:
                        print("Invalid component index")
                        continue

                if component_name not in interface.hardware_components:
                    print("Invalid component name")
                    continue

                current_row = await interface.get_hw_component(component_name)
                if current_row:
                    print(
                        f"Current value: version={current_row['current_version']}, "
                        f"date={current_row['install_date']}, "
                        f"description={current_row['current_description']}"
                    )

                version = input("New version value: ").strip()
                install_date = input("Install date (YYYY-MM-DD): ").strip()
                description = input("Description: ").strip()

                if not version or not install_date or not description:
                    print("All fields are required.")
                    continue

                confirm = input(
                    f"Apply update to {component_name}? (y/N): "
                ).strip().lower()
                if confirm != "y":
                    print("Hardware update cancelled.")
                    continue

                if await interface.set_hw_component(component_name, version, install_date, description):
                    updated = await interface.get_hw_component(component_name)
                    print(f"{component_name} updated successfully.")
                    if updated:
                        print(
                            f"New current: version={updated['current_version']}, "
                            f"date={updated['install_date']}"
                        )
                        if updated["previous_version"] or updated["previous_install_date"]:
                            print(
                                f"Previous: version={updated['previous_version']}, "
                                f"date={updated['previous_install_date']}"
                            )
                else:
                    print(f"Failed to update {component_name}")

            elif choice == "13":
                caps = await interface.hwcfg_get_caps()
                if caps:
                    print(f"\n{caps}")
                else:
                    print("Failed to read HWCFG capabilities")

            elif choice == "14":
                active = await interface.hwcfg_get_active_config()
                last_good = await interface.hwcfg_get_last_good_config()
                if active is None:
                    print("Failed to read active config")
                else:
                    print("\nActive config:")
                    print(active)
                if last_good is None:
                    print("Failed to read last-good config")
                else:
                    print("\nLast-good config:")
                    print(last_good)

            elif choice == "15":
                profiles = await interface.hwcfg_profile_list()
                if profiles is None:
                    print("Failed to read profile list")
                else:
                    print("\nProfiles:")
                    if profiles:
                        for profile in profiles:
                            print(f"- {profile}")
                    else:
                        print("(none)")
                inspect = input("Fetch profile details? (y/N): ").strip().lower()
                if inspect == "y":
                    component = input("Component: ").strip().upper()
                    version = input("Version: ").strip()
                    profile = await interface.hwcfg_profile_get(component, version)
                    if profile:
                        print(profile)

            elif choice == "16":
                component = input("Component: ").strip().upper()
                version = input("Version: ").strip()
                profile_id = input("Profile ID: ").strip()
                print("Enter parameter pairs as key=value;key=value")
                params_input = input("Params: ").strip()
                pair_dict: Dict[str, float] = {}
                try:
                    for token in params_input.split(";"):
                        token = token.strip()
                        if not token:
                            continue
                        if "=" not in token:
                            raise ValueError(f"Missing '=' in token: {token}")
                        key, value = token.split("=", 1)
                        pair_dict[key.strip()] = float(value.strip())
                except ValueError as parse_error:
                    print(f"Invalid params input: {parse_error}")
                    continue
                ok = await interface.hwcfg_profile_put(profile_id, component, version, pair_dict)
                print("Profile stored." if ok else "Profile store failed.")

            elif choice == "17":
                component = input("Component: ").strip().upper()
                version = input("New version: ").strip()
                validation_response = await interface.hwcfg_validate_change(component, version)
                if not validation_response:
                    print("Validation failed. Not applying.")
                    continue
                print(f"Validation OK: {validation_response}")
                install_date = input("Install date (YYYY-MM-DD): ").strip()
                description = input("Description: ").strip()
                confirm = input("Apply hardware change now? (y/N): ").strip().lower()
                if confirm != "y":
                    print("Apply cancelled.")
                    continue
                ok = await interface.hwcfg_apply_change(component, version, install_date, description)
                print("Apply succeeded." if ok else "Apply failed.")

            elif choice == "18":
                confirm = input("Rollback to last-good HWCFG state? (y/N): ").strip().lower()
                if confirm != "y":
                    print("Rollback cancelled.")
                    continue
                ok = await interface.hwcfg_rollback_last_good()
                print("Rollback succeeded." if ok else "Rollback failed.")

            elif choice == "19":
                print("\nReading flush count...")
                flush_count = await interface.get_flush_count()
                if flush_count is None:
                    print("Failed to read flush count")
                else:
                    print(f"Lifetime flush count: {flush_count}")
            
            else:
                print("Invalid choice. Please enter 1-19.")
    
    except KeyboardInterrupt:
        print("\nProgram interrupted by user")
    
    finally:
        await interface.disconnect()

if __name__ == "__main__":
    # Check if bleak is installed
    try:
        import bleak
    except ImportError:
        print("Error: bleak library not installed.")
        print("Install it with: pip install bleak")
        exit(1)
    
    # Run the main program
    asyncio.run(main())