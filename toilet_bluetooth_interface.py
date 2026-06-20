#!/usr/bin/env python3
"""
ESP32 Toilet System Bluetooth Interface
Connects to ESP32 via BLE to read and update system parameters
"""

import asyncio
import json
import os
import platform
import re
import struct
import time
from datetime import datetime
from typing import Dict, Any, Optional, List, Union

try:
    from bleak import BleakClient, BleakScanner
except ImportError:
    BleakClient = Any  # type: ignore[assignment]
    BleakScanner = None  # type: ignore[assignment]

# ESP32 BLE Configuration (from BLE_APP_MIGRATION_SPEC.md)
SERVICE_UUID = "5636340f-afc7-47b1-b0a8-15bc9d7d29a5"
COMMAND_CHARACTERISTIC_UUID = "c327b077-560f-46a1-8f35-b4ab0332fea0"
RESPONSE_CHARACTERISTIC_UUID = "c327b077-560f-46a1-8f35-b4ab0332fea4"
PARAM_READ_CHARACTERISTIC_UUID = "c327b077-560f-46a1-8f35-b4ab0332fea5"
PARAM_WRITE_CHARACTERISTIC_UUID = "c327b077-560f-46a1-8f35-b4ab0332fea6"
SERIAL_CHARACTERISTIC_UUID = "c327b077-560f-46a1-8f35-b4ab0332fea1"
UPDATE_CHARACTERISTIC_UUID = "c327b077-560f-46a1-8f35-b4ab0332fea3"
OTA_ENABLE_WAIT_S = 4.0
OTA_ROLLBACK_RESPONSE_TIMEOUT_S = 5.0
OTA_ROLLBACK_POLL_INTERVAL_S = 0.15
# Legacy: pre-refactor firmware used fea0 for commands, responses, and params
CHARACTERISTIC_UUID = COMMAND_CHARACTERISTIC_UUID
DEVICE_NAME = "ESP32 Toilet"
FRAME_START_BYTE = 0x7E
FRAME_HEADER_SIZE = 3
MAX_FRAME_PAYLOAD = 0xFFFF
# Stale fea4 payloads left by prior command-channel traffic (e.g. TRUST_STATUS after handshake).
_STALE_PARAM_WRITE_RESPONSE_RE = re.compile(
    r"^(FLUSH_COUNT|BATTERY|TRUST_|DEV_MODE|LOGS:|HW_|HWCFG_|SET_DEV_MODE_|TASK_WDT:|DISABLE_TASK_WDT|ENABLE_TASK_WDT|UNKNOWN_COMMAND:|READY)"
)


def _normalize_uuid(uuid_val: Any) -> str:
    """Normalize UUID for comparison (lowercase, standard format)."""
    s = str(uuid_val).lower().strip()
    return s.replace("-", "") if s else ""


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
        self.trusted = False  # Trust handshake state; cleared on disconnect per BLE_APP_MIGRATION_SPEC
        self.trust_timeout_s = 60.0  # Per spec recommended timeout
        self.trust_poll_interval_s = 0.25  # Per BLE_HANDSHAKE_INTERFACE_SPEC
        self.ble_address: Optional[str] = None

        # Parameter definitions with descriptions and units (defaults = 1.5mil High Barrier Plastic from material_parameters.csv)
        self.param_definitions = {
            "batteryThreshold": {"description": "Minimum usable battery percent before flush", "units": "%", "default": 7.0},
            "K": {"description": "Temperature setpoint", "units": "°C", "default": 140.0},
            "F": {"description": "How long to feed the bag at the START of a flush", "units": "sec", "default": 8.0},
            "T": {"description": "Cooling Time", "units": "sec", "default": 60.0},
            "backupTime": {"description": "How long to back up the bag when re-opening", "units": "sec", "default": 1.0},
            "fanDuration": {"description": "How long to run the fan after feeding at the end of a flush", "units": "sec", "default": 5.0},
            "H": {"description": "Heater On time", "units": "sec", "default": 30.0},
            "continueFeeder": {"description": "How long to feed the bag at the END of a flush", "units": "sec", "default": 6.0},
            "maxOpeningTime": {"description": "Max opening time", "units": "sec", "default": 16.5},
            "typicalOpeningTime": {"description": "Typical opening time", "units": "sec", "default": 10.0},
            "MOTOR_CUT_TIME": {"description": "Motor cut duration", "units": "sec", "default": 0.5},
            "CUT_MODE_HEAT_TIME": {"description": "Additional heater time in cut mode", "units": "sec", "default": 15.0},
            "postCoolingFanDuration": {"description": "Fan duration before feed motors start in case 10", "units": "sec", "default": 5.0},
            "preFeedFan": {"description": "Fan duration before feed motor starts in case 1 and button 2", "units": "sec", "default": 2.0},
            "fanReverseTime": {"description": "Duration M3 runs in reverse after starting", "units": "sec", "default": 12.0},
            "fanReverseStartTime": {"description": "Delay before M3 reverse starts as percentage of typicalOpeningTime after M1 begins closing", "units": "%", "default": 0.0},
            "backupTimeAfterReopen": {"description": "Feed bag backup duration after mechanism motor finishes opening", "units": "sec", "default": 1.7},
            "CUT_MODE_TEMP": {"description": "Temperature to maintain for CUT_MODE_HEAT_TIME after cut motor", "units": "°C", "default": 140.0},
            "heaterLowerToleranceC": {"description": "Heater ON threshold below target (temp <= target - lower)", "units": "°C", "default": 0.0},
            "heaterUpperToleranceC": {"description": "Heater OFF threshold above target (temp >= target + upper can be negative)", "units": "°C", "default": 2.0},
            "COOL_OPEN_TEMP_C": {"description": "Open sealer when thermistor cools below this temperature", "units": "°C", "default": 80.0},
            "MAX_COOL_WAIT_S": {"description": "Safety timeout for cooling stage before forcing open", "units": "sec", "default": 180.0},
            "minLoadedBatteryV": {"description": "Minimum loaded battery voltage during flush preflight heater test", "units": "V", "default": 11.2},
            "maxBatterySagV": {"description": "Maximum allowed battery sag during flush preflight heater test", "units": "V", "default": 0.85},
            "minIdleBatteryVFloor": {"description": "Minimum idle battery voltage floor before flush preflight", "units": "V", "default": 11.3},
            "usableVFull": {"description": "Loaded battery voltage mapped to 100 percent usable", "units": "V", "default": 12.4},
            "batteryAssessSettleMs": {"description": "Heater pulse settle time during battery assessment", "units": "ms", "default": 50.0},
            "heaterCapV255": {"description": "Idle battery voltage for maximum heater PWM cap during assessment", "units": "V", "default": 11.23},
            "heaterCapV170": {"description": "Idle battery voltage for 170 heater PWM cap during assessment", "units": "V", "default": 11.22},
            "heaterCapV100": {"description": "Idle battery voltage for 100 heater PWM cap during assessment", "units": "V", "default": 11.21},
        }
        
        # Predefined parameter sets for different materials (match material_parameters.csv)
        self.parameter_sets = {
            "1.5mil High Barrier Plastic": {
                "batteryThreshold": 7.0,
                "K": 140.0,
                "F": 8.0,
                "T": 60.0,
                "backupTime": 1.0,
                "fanDuration": 5.0,
                "H": 30.0,
                "continueFeeder": 6.0,
                "maxOpeningTime": 16.5,
                "typicalOpeningTime": 10.0,
                "MOTOR_CUT_TIME": 0.5,
                "CUT_MODE_HEAT_TIME": 15.0,
                "postCoolingFanDuration": 5.0,
                "preFeedFan": 2.0,
                "fanReverseTime": 12.0,
                "fanReverseStartTime": 0.0,
                "backupTimeAfterReopen": 1.7,
                "CUT_MODE_TEMP": 140.0,
                "heaterLowerToleranceC": 0.0,
                "heaterUpperToleranceC": 2.0,
                "COOL_OPEN_TEMP_C": 80.0,
                "MAX_COOL_WAIT_S": 180.0,
                "minLoadedBatteryV": 11.2,
                "maxBatterySagV": 0.85,
                "minIdleBatteryVFloor": 11.3,
                "usableVFull": 12.4,
                "batteryAssessSettleMs": 50.0,
                "heaterCapV255": 11.23,
                "heaterCapV170": 11.22,
                "heaterCapV100": 11.21,
            },
            "Compostable 1.5mil": {
                "batteryThreshold": 7.0,
                "K": 90.0,
                "F": 8.0,
                "T": 40.0,
                "backupTime": 1.0,
                "fanDuration": 5.0,
                "H": 20.0,
                "continueFeeder": 6.0,
                "maxOpeningTime": 16.5,
                "typicalOpeningTime": 10.0,
                "MOTOR_CUT_TIME": 0.5,
                "CUT_MODE_HEAT_TIME": 10.0,
                "postCoolingFanDuration": 1.0,
                "preFeedFan": 1.5,
                "fanReverseTime": 9.0,
                "fanReverseStartTime": 0.0,
                "backupTimeAfterReopen": 1.7,
                "CUT_MODE_TEMP": 90.0,
                "heaterLowerToleranceC": 0.0,
                "heaterUpperToleranceC": 2.0,
                "COOL_OPEN_TEMP_C": 80.0,
                "MAX_COOL_WAIT_S": 180.0,
                "minLoadedBatteryV": 11.2,
                "maxBatterySagV": 0.85,
                "minIdleBatteryVFloor": 11.3,
                "usableVFull": 12.4,
                "batteryAssessSettleMs": 50.0,
                "heaterCapV255": 11.23,
                "heaterCapV170": 11.22,
                "heaterCapV100": 11.21,
            }
        }
        
        # Parameter order (30 values, as expected by ESP32 BLE)
        self.param_order = [
            "batteryThreshold", "K", "F", "T", "backupTime",
            "fanDuration", "H", "continueFeeder", "maxOpeningTime", "typicalOpeningTime",
            "MOTOR_CUT_TIME", "CUT_MODE_HEAT_TIME", "postCoolingFanDuration", "preFeedFan",
            "fanReverseTime", "fanReverseStartTime", "backupTimeAfterReopen", "CUT_MODE_TEMP",
            "heaterLowerToleranceC", "heaterUpperToleranceC", "COOL_OPEN_TEMP_C", "MAX_COOL_WAIT_S",
            "minLoadedBatteryV", "maxBatterySagV", "minIdleBatteryVFloor", "usableVFull",
            "batteryAssessSettleMs", "heaterCapV255", "heaterCapV170", "heaterCapV100",
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
            "SOFTWARE_VERSION_NUMBER",
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

    async def _get_gatt_service_collection(self):
        """Return GATT services across bleak versions (get_services removed in bleak 3.x)."""
        get_services = getattr(self.client, "get_services", None)
        if get_services is not None:
            return await get_services()
        services = getattr(self.client, "services", None)
        if services is None:
            raise RuntimeError("Could not obtain GATT services from BleakClient")
        return services

    async def _detect_new_ble_protocol(self) -> bool:
        """Return True if device exposes fea4/fea5/fea6 split-channel protocol."""
        new_uuids = {
            _normalize_uuid(RESPONSE_CHARACTERISTIC_UUID),
            _normalize_uuid(PARAM_READ_CHARACTERISTIC_UUID),
            _normalize_uuid(PARAM_WRITE_CHARACTERISTIC_UUID),
        }
        try:
            services = await self._get_gatt_service_collection()
            for char in services.characteristics.values():
                if char.uuid and _normalize_uuid(char.uuid) in new_uuids:
                    return True
        except Exception as discovery_error:
            if self.frame_debug:
                print(f"Protocol discovery via services failed: {discovery_error}")

        try:
            data = await self.client.read_gatt_char(RESPONSE_CHARACTERISTIC_UUID)
            msg = data.decode("utf-8", errors="replace").strip()
            if msg:
                print(f"Protocol probe: fea4 response channel read succeeded ({msg[:40]})")
                return True
        except Exception:
            pass

        try:
            data = await self.client.read_gatt_char(PARAM_READ_CHARACTERISTIC_UUID)
            msg = data.decode("utf-8", errors="replace").strip()
            if self._parse_param_payload(msg) is not None or msg == "AUTH_REQUIRED":
                print("Protocol probe: fea5 param read channel present")
                return True
        except Exception:
            pass

        return False

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

    async def connect(self, address: str, *, require_trust: bool = True) -> bool:
        """Connect to ESP32 device. Set require_trust=False for diagnostic-only commands."""
        try:
            self.ble_address = address
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
            await asyncio.sleep(1.0)
            if not await self._detect_new_ble_protocol():
                print("New protocol (fea4/fea5/fea6) not found. Device may need firmware update.")
                await self.disconnect()
                return False
            if not require_trust:
                print("Diagnostic connection (trust handshake skipped).")
                return True
            # Per BLE_HANDSHAKE_INTERFACE_SPEC: trust handshake required before allowing session
            print("Trust handshake required before connection is fully established...")
            if not await self.trust_handshake():
                print("Connection rejected: trust handshake failed or timed out.")
                await self.disconnect()
                return False
            print("Connection established and trusted.")
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
            self.trusted = False  # Per spec: clear trust state on disconnect
            print("Disconnected from ESP32")

    def _parse_param_payload(self, message: str) -> Optional[Dict[str, Any]]:
        """Strict parser: exactly 30 comma-separated floats. Returns None on invalid."""
        values = [v.strip() for v in message.split(",")]
        if len(values) != 30:
            return None
        try:
            floats = [float(v) for v in values]
        except ValueError:
            return None
        return dict(zip(self.param_order, floats))

    async def read_current_params(self) -> Dict[str, Any]:
        """Read current parameter values from ESP32"""
        if not self.connected:
            print("Not connected to device")
            return {}
        
        try:
            data = await self.client.read_gatt_char(PARAM_READ_CHARACTERISTIC_UUID)
            message = data.decode("utf-8", errors="replace")
            print(f"Received message: {message[:80]}...")
            if message.strip() == "AUTH_REQUIRED":
                self.trusted = False
                print("Parameter read blocked: trust handshake required.")
                return {}
            params = self._parse_param_payload(message)
            if params is None:
                print("Parameter read failed: invalid payload (expected 30 floats)")
                return {}
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

    def _is_stale_param_write_response(self, response: str) -> bool:
        """True when fea4 still holds a prior command-channel response, not param-write ack."""
        return not response or bool(_STALE_PARAM_WRITE_RESPONSE_RE.match(response))

    async def _read_param_write_response(self, *, retries: int = 5, delay_s: float = 0.1) -> str:
        """Read fea4 after param write, retrying past stale command-channel payloads."""
        response = ""
        for attempt in range(retries):
            if attempt > 0:
                await asyncio.sleep(delay_s)
            data = await self.client.read_gatt_char(RESPONSE_CHARACTERISTIC_UUID)
            response = data.decode("utf-8", errors="replace").strip()
            if not self._is_stale_param_write_response(response):
                return response
        return response

    async def update_params(self, new_params: Dict[str, Any]) -> bool:
        """Update parameters on ESP32"""
        if not self.connected:
            print("Not connected to device")
            return False
        if not self._validate_heater_tolerance_gap(new_params):
            return False
        
        try:
            message_parts = []
            for param_name in self.param_order:
                if param_name in new_params:
                    val = new_params[param_name]
                else:
                    val = self.current_params.get(param_name, self.param_definitions[param_name]["default"])
                message_parts.append(str(float(val)))
            message = ",".join(message_parts)
            print(f"Sending message: {message[:60]}...")
            
            # Guard behind trust; write to fea6, read ack from fea4
            if not self.trusted:
                print("Trust handshake required before parameter update. Starting trust flow...")
                if not await self.trust_handshake():
                    print("Parameter update aborted: trust handshake failed or timed out.")
                    return False
            await self.client.write_gatt_char(
                PARAM_WRITE_CHARACTERISTIC_UUID,
                message.encode("utf-8"),
                response=True,
            )
            await asyncio.sleep(0.2)
            response = await self._read_param_write_response()
            if response == "PARAM_WRITE_ACK":
                print("Parameters updated successfully")
                return True
            if response == "PARAM_UPDATE_BLOCKED_FLUSH":
                print("Parameter update rejected: flush in progress (firmware blocked this write).")
                return False
            if response == "AUTH_REQUIRED":
                self.trusted = False  # Per spec: reset local trust state
                print("Parameter update rejected: trust handshake required. Run trust handshake and retry.")
                return False
            if response.startswith("PARAM_WRITE_ERR:"):
                print(f"Parameter update rejected: {response}")
                return False
            print(f"Parameter update failed: unexpected response: {response}")
            return False
            
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

            # Firmware sends raw (unframed) serial stream; always fall through to legacy when framed parse yields nothing.
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
                response = await self._send_command_and_read_response("GET_DEV_MODE")
                if response and response.startswith("DEV_MODE:"):
                    mode_str = response.split(":", 1)[1].strip()
                    if mode_str in ("0", "1"):
                        return int(mode_str)
                await asyncio.sleep(0.1)
            print("Failed to read DEV mode status from firmware")
            return None
        except Exception as e:
            print(f"Failed to read DEV mode status: {e}")
            return None

    async def get_task_wdt_status(self) -> Optional[str]:
        """Read task watchdog status (enabled, disabled, or pending_flush)."""
        if not self.connected:
            print("Not connected to device")
            return None
        try:
            response = await self._send_command_and_read_response("GET_TASK_WDT")
            if response and response.startswith("TASK_WDT:"):
                status = response.split(":", 1)[1].strip()
                if status in ("enabled", "disabled", "pending_flush"):
                    return status
            print(f"Failed to read task watchdog status: {response or '(empty)'}")
            return None
        except Exception as e:
            print(f"Failed to read task watchdog status: {e}")
            return None

    async def disable_task_wdt(self) -> bool:
        """Disable the task watchdog (requires trusted connection)."""
        if not self.connected:
            print("Not connected to device")
            return False
        try:
            response = await self._send_command_and_read_response("DISABLE_TASK_WDT")
            if response == "DISABLE_TASK_WDT_ACK":
                return True
            if response == "AUTH_REQUIRED":
                print("DISABLE_TASK_WDT requires trust handshake")
            elif response == "DISABLE_TASK_WDT_ERR:FLUSH_ACTIVE":
                print("DISABLE_TASK_WDT rejected: flush active")
            else:
                print(f"DISABLE_TASK_WDT failed: {response or '(empty)'}")
            return False
        except Exception as e:
            print(f"DISABLE_TASK_WDT failed: {e}")
            return False

    async def enable_task_wdt(self) -> bool:
        """Arm task watchdog re-init at next flush case 0 (requires trusted connection)."""
        if not self.connected:
            print("Not connected to device")
            return False
        try:
            response = await self._send_command_and_read_response("ENABLE_TASK_WDT")
            if response == "ENABLE_TASK_WDT_ACK:PENDING_FLUSH":
                return True
            if response == "AUTH_REQUIRED":
                print("ENABLE_TASK_WDT requires trust handshake")
            else:
                print(f"ENABLE_TASK_WDT failed: {response or '(empty)'}")
            return False
        except Exception as e:
            print(f"ENABLE_TASK_WDT failed: {e}")
            return False

    async def get_logs(self) -> Optional[str]:
        """Retrieve persistent error logs from firmware (GET_LOGS). Returns None on failure."""
        if not self.connected:
            print("Not connected to device")
            return None
        try:
            chunks: List[str] = []
            offset = 0
            while True:
                cmd = f"GET_LOGS:{offset}" if offset > 0 else "GET_LOGS"
                response = await self._send_command_and_read_response(cmd)
                if not response:
                    break
                if response == "LOGS_END":
                    break
                if response.startswith("LOGS_ERR:"):
                    print(f"Error log unavailable: {response}")
                    return None
                if response.startswith("LOGS:"):
                    parts = response.split(":", 3)
                    if len(parts) >= 4:
                        offset = int(parts[1])
                        length = int(parts[2])
                        data = parts[3]
                        chunks.append(data)
                        offset += length
                    else:
                        break
                else:
                    print(f"Unexpected GET_LOGS response: {response[:80]}")
                    break
            return "".join(chunks) if chunks else ""
        except Exception as e:
            print(f"Failed to get logs: {e}")
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

    async def get_active_partition(self) -> Optional[dict]:
        """Read the app partition the firmware is currently running from."""
        response = await self._send_command_and_read_response("GET_ACTIVE_PARTITION")
        if not response:
            print("No response from firmware for GET_ACTIVE_PARTITION")
            return None
        if response.startswith("ACTIVE_PARTITION_ERR:"):
            print(f"Firmware error for GET_ACTIVE_PARTITION: {response}")
            return None
        if not response.startswith("ACTIVE_PARTITION:"):
            print(f"Unexpected GET_ACTIVE_PARTITION response: {response}")
            return None

        payload = response.split(":", 1)[1]
        result: dict = {}
        for part in payload.split("|"):
            if "=" not in part:
                continue
            key, value = part.split("=", 1)
            result[key.strip()] = value.strip()

        if "label" not in result:
            print(f"Malformed GET_ACTIVE_PARTITION payload: {response}")
            return None

        if "subtype" in result:
            try:
                result["subtype"] = int(result["subtype"])
            except ValueError:
                pass
        if "offset" in result:
            try:
                result["offset"] = int(result["offset"], 0)
            except ValueError:
                pass
        return result

    async def get_battery(self) -> Optional[int]:
        """Read battery charge level (0-100%) from firmware. Returns int or None on failure."""
        response = await self._send_command_and_read_response("GET_BATTERY")
        if not response:
            print("No response from firmware for GET_BATTERY")
            return None
        m = re.match(r"^BATTERY[:_]?\s*(\d+)\s*%?$", response, re.IGNORECASE)
        if not m:
            print(f"Unexpected GET_BATTERY response: {response}")
            return None
        try:
            level = int(m.group(1))
        except ValueError:
            return None
        if level < 0 or level > 100:
            print(f"Invalid battery level out of range 0-100: {response}")
            return None
        return level

    async def set_dev_mode(self, new_mode: int) -> bool:
        """Set firmware DEV mode to 0 or 1."""
        if not self.connected:
            print("Not connected to device")
            return False
        if new_mode not in (0, 1):
            print("Invalid DEV mode value. Use 0 or 1.")
            return False
        try:
            response = await self._send_command_and_read_response(f"SET_DEV_MODE:{new_mode}")
            if not response:
                return False
            if response.startswith("SET_DEV_MODE_ACK:"):
                ack_value = response.split(":", 1)[1].strip()
                return ack_value == str(new_mode)
            if response.startswith("SET_DEV_MODE_ERR:"):
                print(f"Firmware rejected DEV mode update: {response}")
                return False
            verify_mode = await self.get_dev_mode_status()
            return verify_mode == new_mode
        except Exception as e:
            print(f"Failed to set DEV mode: {e}")
            return False

    async def _send_command_and_read_response(self, command: str, retries: int = 3, response_delay_s: float = 0.15) -> Optional[str]:
        """Write command to fea0, read response from fea4."""
        if not self.connected:
            return None
        write_uuid = COMMAND_CHARACTERISTIC_UUID
        read_uuid = RESPONSE_CHARACTERISTIC_UUID
        try:
            for _ in range(retries):
                await self.client.write_gatt_char(write_uuid, command.encode("utf-8"))
                await asyncio.sleep(response_delay_s)
                data = await self.client.read_gatt_char(read_uuid)
                response = data.decode("utf-8", errors="replace").strip()
                if response:
                    if response == "AUTH_REQUIRED":
                        self.trusted = False  # Per spec: reset local trust state
                    return response
                await asyncio.sleep(0.1)
            return None
        except Exception as e:
            err_str = str(e).lower()
            if "not connected" in err_str or "disconnected" in err_str:
                # Throttle connection-error spam (e.g. during trust poll when link drops)
                if not hasattr(self, "_last_conn_err_log"):
                    self._last_conn_err_log = 0.0
                now = time.monotonic()
                if now - self._last_conn_err_log >= 1.0:
                    self._last_conn_err_log = now
                    print(f"Command failed ({command}): {e}")
            else:
                print(f"Command failed ({command}): {e}")
            return None

    async def _read_response_channel(self) -> Optional[str]:
        """Read fea4 response characteristic without sending a command."""
        if not self.connected or not self.client:
            return None
        try:
            data = await self.client.read_gatt_char(RESPONSE_CHARACTERISTIC_UUID)
            return data.decode("utf-8", errors="replace").strip()
        except Exception as e:
            err_str = str(e).lower()
            if "not connected" in err_str or "disconnected" in err_str:
                return None
            print(f"Response read failed: {e}")
            return None

    async def _send_command_and_poll_response(
        self,
        command: str,
        *,
        success_responses: tuple[str, ...],
        error_prefixes: tuple[str, ...],
        timeout_s: float = OTA_ROLLBACK_RESPONSE_TIMEOUT_S,
        poll_interval_s: float = OTA_ROLLBACK_POLL_INTERVAL_S,
    ) -> Optional[str]:
        """
        Write command and poll fea4 until an expected ACK/ERR or timeout.
        Ignores stale unrelated values left on fea4 by prior commands.
        """
        if not self.connected or not self.client:
            return None
        try:
            await self.client.write_gatt_char(COMMAND_CHARACTERISTIC_UUID, command.encode("utf-8"))
        except Exception as e:
            print(f"Command failed ({command}): {e}")
            return None

        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            await asyncio.sleep(poll_interval_s)
            response = await self._read_response_channel()
            if not response:
                continue
            if response == "AUTH_REQUIRED":
                self.trusted = False
                return response
            if response in success_responses:
                return response
            for prefix in error_prefixes:
                if response.startswith(prefix):
                    return response
        return None

    # --- Trust handshake (BLE_APP_MIGRATION_SPEC, BLE_HANDSHAKE_INTERFACE_SPEC) ---
    async def trust_start(self) -> Optional[str]:
        """Send TRUST_START, read response from fea4. Returns TRUST_WAITING, TRUST_CONFIRMED, or error."""
        return await self._send_command_and_read_response("TRUST_START")

    async def trust_status(self) -> Optional[str]:
        """Send TRUST_STATUS, read response from fea4. Returns TRUST_WAITING, TRUST_CONFIRMED, TRUST_TIMEOUT."""
        return await self._send_command_and_read_response("TRUST_STATUS")

    async def trust_cancel(self) -> Optional[str]:
        """Send TRUST_CANCEL, read response from fea4. Returns TRUST_CANCEL_ACK."""
        return await self._send_command_and_read_response("TRUST_CANCEL")

    async def trust_handshake(self, timeout_s: Optional[float] = None) -> bool:
        """
        Run full trust flow: TRUST_START, poll TRUST_STATUS until TRUST_CONFIRMED or timeout.
        Per spec: user presses a control panel button (GPIO2 wake line) during TRUST_WAITING to confirm.
        When firmware DEV mode is on, trust is auto-granted without a button press.
        Returns True if trusted, False on timeout/cancel/error.
        """
        timeout = timeout_s if timeout_s is not None else self.trust_timeout_s
        start_resp = await self.trust_start()
        if not start_resp:
            print("Trust handshake failed: no response to TRUST_START")
            return False
        if start_resp == "TRUST_CONFIRMED":
            self.trusted = True
            dev_mode = await self.get_dev_mode_status()
            if dev_mode == 1:
                print("Trust granted (DEV mode bypass).")
            else:
                print("Already trusted for this connection.")
            return True
        if start_resp != "TRUST_WAITING":
            print(f"Trust handshake failed: unexpected TRUST_START response: {start_resp}")
            return False
        print("Press a control panel button on device to confirm connection (GPIO2 wake line)...")
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            status = await self.trust_status()
            if status == "TRUST_CONFIRMED":
                self.trusted = True
                print("Trust confirmed.")
                return True
            if status == "TRUST_TIMEOUT":
                print("Trust handshake timed out.")
                self.trusted = False
                return False
            await asyncio.sleep(self.trust_poll_interval_s)
        print("Trust handshake timed out (client).")
        self.trusted = False
        return False

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

    def _print_hwcfg_failure(self, action: str, response: str) -> None:
        print(f"{action} failed: {response}")
        if response.endswith(":PERSIST_FAIL") or response.endswith(":SAFE_FAULT"):
            print("HWCFG persistence failed in firmware. Export logs and check SPIFFS/HWCFG storage.")

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
        self._print_hwcfg_failure("Profile put", response)
        return False

    async def hwcfg_validate_change(self, component: str, new_version: str) -> Optional[str]:
        command = f"HWCFG_VALIDATE_CHANGE:{component}|new_version={new_version}"
        response = await self._send_command_and_read_response(command)
        if not response:
            print("No response from firmware for HWCFG_VALIDATE_CHANGE")
            return None
        if response.startswith("HWCFG_VALIDATE_OK:"):
            return response
        self._print_hwcfg_failure("Validation", response)
        return None

    async def hwcfg_apply_change(self, component: str, new_version: str, install_date: str, desc: str) -> bool:
        if not self._is_valid_iso_date(install_date):
            print("install_date must use YYYY-MM-DD")
            return False
        if not self.trusted:
            if not await self.trust_handshake():
                print("HWCFG apply aborted: trust handshake required.")
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
        if response == "AUTH_REQUIRED":
            print("Firmware rejected: trust handshake required. Run trust handshake and retry.")
            return False
        self._print_hwcfg_failure("Apply", response)
        return False

    async def hwcfg_rollback_last_good(self) -> bool:
        if not self.trusted:
            if not await self.trust_handshake():
                print("HWCFG rollback aborted: trust handshake required.")
                return False
        response = await self._send_command_and_read_response("HWCFG_ROLLBACK_LAST_GOOD")
        if not response:
            print("No response from firmware for HWCFG_ROLLBACK_LAST_GOOD")
            return False
        if response == "HWCFG_ROLLBACK_ACK":
            return True
        if response == "AUTH_REQUIRED":
            print("Firmware rejected: trust handshake required. Run trust handshake and retry.")
            return False
        self._print_hwcfg_failure("Rollback", response)
        return False

    async def prepare_ota_for_update(self) -> bool:
        """ENABLE_OTA + PREPARE_UPDATE (no firmware transfer). For OTA updates or explicit NVS seed."""
        if not self.trusted:
            if not await self.trust_handshake():
                print("PREPARE_UPDATE aborted: trust handshake required.")
                return False
        if not await self.enable_ota_mode():
            return False
        return await self.prepare_ota_update()

    async def ota_rollback_previous(self) -> bool:
        """Request firmware rollback to the previous OTA partition. Device reboots on success."""
        if not self.trusted:
            if not await self.trust_handshake():
                print("OTA rollback aborted: trust handshake required.")
                return False
        response = await self._send_command_and_poll_response(
            "OTA_ROLLBACK_PREVIOUS",
            success_responses=("OTA_ROLLBACK_ACK:REBOOTING",),
            error_prefixes=("OTA_ROLLBACK_ERR:",),
        )
        if not response:
            print("No rollback ACK from firmware for OTA_ROLLBACK_PREVIOUS (timed out waiting on fea4)")
            return False
        if response == "OTA_ROLLBACK_ACK:REBOOTING":
            print("Firmware accepted OTA rollback. Device is rebooting to previous partition.")
            self.connected = False
            self.trusted = False
            return True
        if response.startswith("OTA_ROLLBACK_ERR:"):
            print(f"Firmware rejected OTA rollback: {response}")
            return False
        if response == "AUTH_REQUIRED":
            print("Firmware rejected OTA rollback: trust handshake required.")
            return False
        print(f"Unexpected OTA rollback response: {response}")
        return False

    async def enable_ota_mode(self) -> bool:
        response = await self._send_command_and_read_response("ENABLE_OTA")
        if not response or "ENABLE_OTA_ACK" not in response:
            print(f"ENABLE_OTA failed: {response or '(no response)'}")
            return False
        await asyncio.sleep(OTA_ENABLE_WAIT_S)
        return True

    async def prepare_ota_update(self) -> bool:
        """PREPARE_UPDATE only — records current partition in rollback NVS; does not transfer firmware."""
        if not self.client or not self.connected:
            print("Not connected to device")
            return False
        try:
            await self.client.write_gatt_char(UPDATE_CHARACTERISTIC_UUID, b"PREPARE_UPDATE")
            await asyncio.sleep(1.5)
            response = await self.client.read_gatt_char(UPDATE_CHARACTERISTIC_UUID)
            response_str = response.decode("utf-8", errors="replace")
            if "UPDATE_PREPARED" in response_str:
                print("PREPARE_UPDATE OK")
                return True
            print(f"PREPARE_UPDATE failed: {response_str}")
            return False
        except Exception as exc:
            print(f"PREPARE_UPDATE failed: {exc}")
            return False

    async def ota_rollback_factory(self) -> bool:
        """Request firmware rollback to the factory partition. Device reboots on success."""
        if not self.trusted:
            if not await self.trust_handshake():
                print("OTA factory rollback aborted: trust handshake required.")
                return False
        response = await self._send_command_and_poll_response(
            "OTA_ROLLBACK_FACTORY",
            success_responses=("OTA_ROLLBACK_FACTORY_ACK:REBOOTING",),
            error_prefixes=("OTA_ROLLBACK_FACTORY_ERR:",),
        )
        if not response:
            print("No rollback ACK from firmware for OTA_ROLLBACK_FACTORY (timed out waiting on fea4)")
            return False
        if response == "OTA_ROLLBACK_FACTORY_ACK:REBOOTING":
            print("Firmware accepted factory rollback. Device is rebooting to factory partition.")
            self.connected = False
            self.trusted = False
            return True
        if response.startswith("OTA_ROLLBACK_FACTORY_ERR:"):
            print(f"Firmware rejected OTA factory rollback: {response}")
            return False
        if response == "AUTH_REQUIRED":
            print("Firmware rejected OTA factory rollback: trust handshake required.")
            return False
        print(f"Unexpected OTA factory rollback response: {response}")
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

        if not self.trusted:
            if not await self.trust_handshake():
                print("Hardware component update aborted: trust handshake required.")
                return False

        command = f"SET_HW_COMPONENT:{component}:{version}:{install_date}:{description}"
        response = await self._send_command_and_read_response(command)
        if not response:
            print("No response from firmware for SET_HW_COMPONENT")
            return False
        if response.startswith("SET_HW_COMPONENT_ACK:"):
            return response.split(":", 1)[1].strip().upper() == component
        if response == "AUTH_REQUIRED":
            print("Firmware rejected: trust handshake required. Run trust handshake and retry.")
            return False
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
            print("20. Read error logs (GET_LOGS)")
            print("21. Trust handshake (press control panel button / GPIO2 wake line to confirm)")
            print("22. Manual OTA rollback to previous firmware")
            print("23. Manual OTA rollback to factory firmware")
            print("24. Prepare OTA update (ENABLE_OTA + PREPARE_UPDATE, no transfer)")
            print("25. Read active OTA/factory partition (GET_ACTIVE_PARTITION)")
            print("26. Read task watchdog status (GET_TASK_WDT)")
            print("27. Disable task watchdog (DISABLE_TASK_WDT, requires trust)")
            print("28. Enable task watchdog (ENABLE_TASK_WDT, re-init at next flush)")
            
            choice = input("\nEnter your choice (1-28): ").strip()
            
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

            elif choice == "20":
                print("\nReading error logs...")
                logs = await interface.get_logs()
                if logs is None:
                    print("Failed to read logs")
                elif logs:
                    print(logs)
                else:
                    print("(no logs)")

            elif choice == "21":
                print("\nTrust handshake...")
                if await interface.trust_handshake():
                    print("Trust confirmed. Parameter updates and privileged commands are now allowed.")
                    params = await interface.read_current_params()
                    if params:
                        interface.current_params = params
                        interface.display_params_table(params)
                else:
                    print("Trust handshake failed or timed out.")

            elif choice == "22":
                print("\nManual OTA rollback")
                print("This will reboot the device to the previous OTA partition.")
                print("Firmware auto-seeds rollback NVS from the other OTA slot if missing.")
                confirm = input("Type ROLLBACK to continue: ").strip()
                if confirm != "ROLLBACK":
                    print("OTA rollback cancelled.")
                    continue
                ok = await interface.ota_rollback_previous()
                print("OTA rollback command accepted." if ok else "OTA rollback failed.")
                if ok:
                    break

            elif choice == "23":
                print("\nManual OTA factory rollback")
                print("This will reboot the device to the factory firmware partition.")
                confirm = input("Type FACTORY_ROLLBACK to continue: ").strip()
                if confirm != "FACTORY_ROLLBACK":
                    print("OTA factory rollback cancelled.")
                    continue
                ok = await interface.ota_rollback_factory()
                print("OTA factory rollback command accepted." if ok else "OTA factory rollback failed.")
                if ok:
                    break

            elif choice == "24":
                print("\nPrepare OTA update (PREPARE_UPDATE)")
                print("Records current partition for rollback NVS before a BLE OTA transfer.")
                ok = await interface.prepare_ota_for_update()
                print("PREPARE_UPDATE succeeded." if ok else "PREPARE_UPDATE failed.")

            elif choice == "25":
                print("\nReading active partition...")
                partition = await interface.get_active_partition()
                if partition is None:
                    print("Failed to read active partition")
                else:
                    print(
                        f"Active partition: {partition.get('label')} "
                        f"(subtype={partition.get('subtype')}, "
                        f"offset={partition.get('offset', 'n/a')}, "
                        f"version={partition.get('version', 'unknown')})"
                    )

            elif choice == "26":
                print("\nReading task watchdog status...")
                status = await interface.get_task_wdt_status()
                if status is None:
                    print("Failed to read task watchdog status")
                else:
                    print(f"Task watchdog status: {status}")

            elif choice == "27":
                print("\nDisable task watchdog (requires trusted connection)...")
                if not interface.trusted:
                    print("Trust handshake not completed. Use option 21 first.")
                    continue
                confirm = input("Disable task watchdog? (y/N): ").strip().lower()
                if confirm != "y":
                    print("Cancelled.")
                    continue
                if await interface.disable_task_wdt():
                    status = await interface.get_task_wdt_status()
                    print(f"Task watchdog disabled. Status: {status or 'unknown'}")
                else:
                    print("Failed to disable task watchdog")

            elif choice == "28":
                print("\nEnable task watchdog (re-init at next flush case 0)...")
                if not interface.trusted:
                    print("Trust handshake not completed. Use option 21 first.")
                    continue
                confirm = input("Arm watchdog re-init at next flush? (y/N): ").strip().lower()
                if confirm != "y":
                    print("Cancelled.")
                    continue
                if await interface.enable_task_wdt():
                    status = await interface.get_task_wdt_status()
                    print(f"Task watchdog re-init armed. Status: {status or 'unknown'}")
                else:
                    print("Failed to arm task watchdog re-init")

            else:
                print("Invalid choice. Please enter 1-28.")
    
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