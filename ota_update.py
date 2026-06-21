#!/usr/bin/env python3
"""
ESP32 Toilet System OTA Update Script
Updates firmware over Bluetooth Low Energy (BLE)

Usage:
    python ota_update.py <firmware.bin>                    # Update firmware (auto-scan for device)
    python ota_update.py <firmware.bin> --address XX:XX:XX:XX:XX:XX  # Update with specific address
    python ota_update.py <firmware.bin> --check-version  # Only check version, don't update

Requirements:
    pip install bleak

Example:
    python ota_update.py build/toilet_kat_change.bin
"""

import asyncio
import hashlib
import sys
import argparse
import time
from bleak import BleakClient, BleakScanner
from typing import Callable, Optional, TYPE_CHECKING

if TYPE_CHECKING:
    from ota_regression_diagnostics import BleConnectionTracker, DiagnosticsHeartbeat

LogFn = Callable[[str], None]
CONNECT_TIMEOUT_S = 20.0

# ESP32 BLE Configuration (from toilet_kat_change.ino)
UPDATE_SERVICE_UUID = "5636340f-afc7-47b1-b0a8-15bcb9d7d29a6"
UPDATE_CHARACTERISTIC_UUID = "c327b077-560f-46a1-8f35-b4ab0332fea3"
VERSION_CHARACTERISTIC_UUID = "c327b077-560f-46a1-8f35-b4ab0332fea2"
DEVICE_NAME = "ESP32 Toilet"
COMMAND_CHARACTERISTIC_UUID = "c327b077-560f-46a1-8f35-b4ab0332fea0"
RESPONSE_CHARACTERISTIC_UUID = "c327b077-560f-46a1-8f35-b4ab0332fea4"
CHARACTERISTIC_UUID = COMMAND_CHARACTERISTIC_UUID
SERIAL_CHARACTERISTIC_UUID = "c327b077-560f-46a1-8f35-b4ab0332fea1"
OTA_ENABLE_WAIT_S = 4.0

# BLE MTU is typically 20-512 bytes, we'll use 400 bytes per chunk for safety
CHUNK_SIZE = 512

class OTAUpdater:
    def __init__(
        self,
        log: Optional[LogFn] = None,
        tracker: Optional["BleConnectionTracker"] = None,
    ):
        self.client: Optional[BleakClient] = None
        self.connected = False
        self.update_progress = 0
        self.device_progress = 0
        self._log = log or (lambda msg: print(msg, flush=True))
        self.tracker = tracker
        self.address: Optional[str] = None

    def log(self, message: str) -> None:
        self._log(message)

    def _set_state(self, state: str, detail: str = "") -> None:
        if self.tracker:
            self.tracker.set(state, detail)

    def _fail(self, operation: str, reason: str) -> None:
        self.connected = False
        if self.tracker:
            self.tracker.fail(operation, reason)
        else:
            self.log(f"[ble] FAILED {operation} | {reason}")

    def _require_connected(self, operation: str) -> bool:
        if not self.client or not self.connected:
            self._fail(operation, "not connected")
            return False
        try:
            is_up = getattr(self.client, "is_connected", True)
            if callable(is_up):
                is_up = is_up()
            if is_up is False:
                self._fail(operation, "client.is_connected=False")
                return False
        except Exception as exc:
            self._fail(operation, f"connection check error: {exc}")
            return False
        return True
        
    def notification_handler(self, sender, data):
        """Handle notifications from device"""
        try:
            message = data.decode('utf-8', errors='replace')
            if "UPDATE_PROGRESS:" in message:
                # Extract progress percentage
                progress_str = message.split(":")[1]
                self.device_progress = int(progress_str)
                self.log(f"Device progress: {self.device_progress}%")
            elif "UPDATE_" in message:
                self.log(f"Device status: {message}")
        except Exception as e:
            self.log(f"Error handling notification: {e}")
        
    async def scan_for_device(self) -> Optional[str]:
        """Scan for ESP32 Toilet device"""
        self.log("Scanning for ESP32 Toilet device...")
        devices = await BleakScanner.discover(timeout=20.0)
        
        for device in devices:
            if device.name == DEVICE_NAME:
                self.log(f"Found device: {device.name} at {device.address}")
                return device.address
        
        self.log("ESP32 Toilet device not found!")
        return None
    
    async def connect(self, address: str, max_retries: int = 5) -> bool:
        """Connect to ESP32 device (Windows BLE often needs retries)."""
        last_error = None
        for attempt in range(1, max_retries + 1):
            self._set_state("connecting", f"attempt={attempt}/{max_retries} addr={address}")
            try:
                if attempt > 1:
                    self.log(f"[ble] retry {attempt}/{max_retries} after prior failure: {last_error}")
                    await asyncio.sleep(2.0)
                self.client = BleakClient(address)
                self.log(f"[ble] GATT connect starting (timeout {CONNECT_TIMEOUT_S:.0f}s)...")
                await asyncio.wait_for(self.client.connect(), timeout=CONNECT_TIMEOUT_S)
                self.address = address
                self.connected = True
                self._set_state("connected", address)

                try:
                    await self.client.start_notify(VERSION_CHARACTERISTIC_UUID, self.notification_handler)
                    self.log("[ble] notifications enabled on version characteristic")
                except Exception as e:
                    self.log(f"[ble] WARN: notifications disabled ({e}); OTA may still work")

                await asyncio.sleep(1.0)
                if not self._require_connected("post-connect"):
                    last_error = "connection dropped immediately after connect"
                    continue
                return True
            except asyncio.TimeoutError:
                last_error = f"connect timeout after {CONNECT_TIMEOUT_S:.0f}s"
                self.connected = False
                self.log(f"[ble] {last_error}")
            except Exception as e:
                last_error = str(e)
                self.connected = False
                self.log(f"[ble] connect error: {e}")

        self._fail("connect", f"exhausted {max_retries} attempts: {last_error}")
        return False
    
    async def disconnect(self):
        """Disconnect from ESP32 device"""
        if self.client and self.connected:
            try:
                await self.client.stop_notify(VERSION_CHARACTERISTIC_UUID)
            except Exception:
                pass
            try:
                await self.client.disconnect()
            except Exception as exc:
                self.log(f"[ble] disconnect error: {exc}")
            self.connected = False
            if self.tracker:
                self.tracker.disconnected("ota session ended")
            else:
                self.log("[ble] disconnected")

    async def log_services(self):
        """Log discovered services and characteristics for debugging"""
        if not self.client or not self.connected:
            return
        # Support multiple bleak versions: try get_services(), else use .services
        try:
            services = await self.client.get_services()
        except AttributeError:
            services = getattr(self.client, 'services', None)
            if services is None:
                try:
                    services = await self.client.get_services()
                except Exception:
                    self.log("Could not obtain services from BleakClient")
                    return
        self.log("Discovered services:")
        for service in services:
            self.log(f"  Service: {service.uuid}")
            for char in service.characteristics:
                props = []
                if "read" in char.properties:
                    props.append("read")
                if "write" in char.properties:
                    props.append("write")
                if "write-without-response" in char.properties:
                    props.append("write-without-response")
                if "notify" in char.properties:
                    props.append("notify")
                self.log(f"    Char: {char.uuid}  props: {props}")
    
    async def check_version(self) -> Optional[str]:
        """Check current firmware version"""
        if not self.connected:
            self.log("Not connected to device")
            return None

        try:
            await self.client.write_gatt_char(UPDATE_CHARACTERISTIC_UUID, b"CHECK_VERSION")
            version_str = "v1.0"
            for attempt in range(1, 8):
                await asyncio.sleep(0.75 if attempt > 1 else 1.0)
                version_data = await self.client.read_gatt_char(VERSION_CHARACTERISTIC_UUID)
                try:
                    version_str = version_data.decode("utf-8")
                except UnicodeDecodeError:
                    version_str = version_data.decode("utf-8", errors="replace")
                version_str = version_str.strip("\x00").strip()
                if "SW:" in version_str or (version_str and version_str != "v1.0"):
                    break
            self.log(f"Current firmware version: {version_str}")
            return version_str
        except Exception as e:
            self.log(f"Failed to check version: {e}")
            # Version check is informational; don't fail the update if it errors
            return "UNKNOWN"
    
    async def prepare_update(self, max_retries: int = 3) -> bool:
        """Prepare device for OTA update with retry logic"""
        if not self._require_connected("PREPARE_UPDATE"):
            return False
        self._set_state("ota_prepare")

        for attempt in range(1, max_retries + 1):
            try:
                if attempt > 1:
                    self.log(f"[ota] PREPARE_UPDATE retry {attempt}/{max_retries}...")
                else:
                    self.log("[ota] PREPARE_UPDATE -> device")
                
                await self.client.write_gatt_char(UPDATE_CHARACTERISTIC_UUID, b"PREPARE_UPDATE")
                await asyncio.sleep(1.5)  # Increased wait time for device to process
                
                # Read response
                response = await self.client.read_gatt_char(UPDATE_CHARACTERISTIC_UUID)
                response_str = response.decode('utf-8')
                
                if "UPDATE_PREPARED" in response_str:
                    self.log("[ota] PREPARE_UPDATE ack: UPDATE_PREPARED")
                    return True
                elif "UPDATE_BLOCKED" in response_str:
                    if attempt < max_retries:
                        self.log(f"Update preparation blocked (attempt {attempt}/{max_retries})")
                        self.log("Device may be busy - waiting 3 seconds before retry...")
                        await asyncio.sleep(3.0)  # Wait before retry
                    else:
                        self.log("ERROR: Update preparation blocked (device may be busy)")
                        self.log("Troubleshooting: stop flush, wait, reboot device, check serial logs")
                        return False
                else:
                    self.log(f"Unexpected response: {response_str}")
                    if attempt < max_retries:
                        self.log("Retrying in 2 seconds...")
                        await asyncio.sleep(2.0)
                    else:
                        return False
            except Exception as e:
                self.log(f"Failed to prepare update: {e}")
                if attempt < max_retries:
                    self.log("Retrying in 2 seconds...")
                    await asyncio.sleep(2.0)
                else:
                    return False
        
        return False
    
    async def start_update(self) -> bool:
        """Start OTA update process"""
        if not self._require_connected("START_UPDATE"):
            return False
        self._set_state("ota_start")

        max_retries = 3
        for attempt in range(1, max_retries + 1):
            try:
                if attempt > 1:
                    self.log(f"[ota] START_UPDATE retry {attempt}/{max_retries}...")
                    await asyncio.sleep(2.0)
                self.log("[ota] START_UPDATE -> beginning firmware transfer")
                await self.client.write_gatt_char(UPDATE_CHARACTERISTIC_UUID, b"START_UPDATE")
                await asyncio.sleep(1.5)

                response = await self.client.read_gatt_char(UPDATE_CHARACTERISTIC_UUID)
                response_str = response.decode("utf-8")

                if "UPDATE_STARTED" in response_str:
                    self.log("[ota] START_UPDATE ack: UPDATE_STARTED — transfer open")
                    return True
                if response_str in {"START_UPDATE", "PREPARING"} and attempt < max_retries:
                    self.log(f"Unexpected response: {response_str}")
                    continue
                self.log(f"ERROR: Failed to start update: {response_str}")
                return False
            except Exception as e:
                self.log(f"Failed to start update: {e}")
                if attempt >= max_retries:
                    return False
        return False
    
    async def send_firmware_size(self, size: int) -> bool:
        """Send firmware size metadata"""
        if not self._require_connected("SIZE"):
            return False

        try:
            self.log(f"[ota] SIZE:{size} -> update characteristic")
            size_msg = f"SIZE:{size}".encode('utf-8')
            await self.client.write_gatt_char(UPDATE_CHARACTERISTIC_UUID, size_msg)
            await asyncio.sleep(0.2)
            self.log(f"Sent firmware size: {size} bytes")
            return True
        except Exception as e:
            self.log(f"Failed to send firmware size: {e}")
            return False
    
    async def send_firmware_chunks(
        self, firmware_data: bytes, md5_hash: str, heartbeat=None
    ) -> bool:
        """Send firmware in chunks"""
        if not self._require_connected("firmware_transfer"):
            return False

        total_size = len(firmware_data)
        chunks_sent = 0
        total_chunks = (total_size + CHUNK_SIZE - 1) // CHUNK_SIZE
        self._set_state("ota_transfer", f"0/{total_chunks} chunks")

        self.log(f"[ota] TRANSFER begin: {total_chunks} chunks, {total_size} bytes")

        try:
            for i in range(0, total_size, CHUNK_SIZE):
                if not self._require_connected("firmware_chunk"):
                    return False
                chunk = firmware_data[i:i + CHUNK_SIZE]
                await self.client.write_gatt_char(UPDATE_CHARACTERISTIC_UUID, chunk)
                chunks_sent += 1

                progress = (chunks_sent * 100) // total_chunks
                if progress != self.update_progress:
                    self.update_progress = progress
                    self._set_state("ota_transfer", f"{chunks_sent}/{total_chunks} ({progress}%)")
                    self.log(f"[ota] TRANSFER {progress}% ({chunks_sent}/{total_chunks} chunks)")

                if heartbeat is not None and chunks_sent % 100 == 0:
                    await heartbeat.maybe_emit_async()

                await asyncio.sleep(0.05)

            if not self._require_connected("firmware_md5"):
                return False
            self.log("[ota] TRANSFER sending MD5...")
            md5_msg = f"MD5:{md5_hash}".encode('utf-8')
            await self.client.write_gatt_char(UPDATE_CHARACTERISTIC_UUID, md5_msg)
            await asyncio.sleep(0.5)
            
            self.log("[ota] TRANSFER complete; all chunks sent")
            return True
        except Exception as e:
            self._fail("firmware_transfer", str(e))
            return False
    
    async def finalize_update(self) -> bool:
        """Finalize OTA update"""
        if not self._require_connected("FINALIZE_UPDATE"):
            return False
        self._set_state("ota_finalize")
        
        try:
            self.log("[ota] FINALIZE_UPDATE -> device")
            await self.client.write_gatt_char(UPDATE_CHARACTERISTIC_UUID, b"FINALIZE_UPDATE")
            await asyncio.sleep(1.0)
            
            # Read response
            response = await self.client.read_gatt_char(UPDATE_CHARACTERISTIC_UUID)
            response_str = response.decode('utf-8')
            
            if "UPDATE_COMPLETE" in response_str:
                self.log("Update finalized successfully! Device will reboot...")
                return True
            elif "UPDATE_VALIDATION_FAILED" in response_str:
                self.log("ERROR: Firmware validation failed (MD5 mismatch please verify the calculations manually)")
                return True
            elif "UPDATE_ERROR" in response_str:
                self.log(f"ERROR: Update failed: {response_str}")
                return False
            else:
                self.log(f"Unexpected response: {response_str}")
                return False
        except Exception as e:
            self.log(f"Failed to finalize update: {e}")
            return False
    
    def calculate_md5(self, data: bytes) -> str:
        """Calculate MD5 hash of firmware"""
        md5_hash = hashlib.md5(data).hexdigest()
        return md5_hash

    @staticmethod
    def _services_have_update_char(services) -> bool:
        if not services:
            return False
        for svc in services:
            for char in svc.characteristics:
                if str(char.uuid).lower() == UPDATE_CHARACTERISTIC_UUID.lower():
                    return True
        return False

    async def _wait_for_device(self, addr: str, timeout: float = 20.0) -> bool:
        deadline = asyncio.get_event_loop().time() + timeout
        while asyncio.get_event_loop().time() < deadline:
            self.log("Scanning for device...")
            try:
                devices = await BleakScanner.discover(timeout=3.0)
            except Exception as e:
                self.log(f"Scan failed: {e}")
                devices = []
            for device in devices:
                if device.address.lower() == addr.lower():
                    self.log(f"Device {addr} found")
                    return True
            await asyncio.sleep(0.5)
        return False

    async def _write_command_read_response(self, command: str, retries: int = 3) -> Optional[str]:
        """Write command to fea0, read ack from fea4 (same as toilet_bluetooth_interface)."""
        if not self.connected:
            return None
        payload = command.encode("utf-8")
        for _ in range(retries):
            await self.client.write_gatt_char(COMMAND_CHARACTERISTIC_UUID, payload)
            await asyncio.sleep(0.15)
            data = await self.client.read_gatt_char(RESPONSE_CHARACTERISTIC_UUID)
            response = data.decode("utf-8", errors="replace").strip()
            if response:
                return response
            await asyncio.sleep(0.1)
        return None

    async def enable_ota_mode(self, heartbeat=None) -> bool:
        """Send ENABLE_OTA on fea0, confirm ENABLE_OTA_ACK on fea4, keep connection."""
        if not self._require_connected("ENABLE_OTA"):
            return False

        self._set_state("ota_enable", "sending ENABLE_OTA")
        self.log("[ota] ENABLE_OTA -> command channel")
        try:
            await self.client.write_gatt_char(COMMAND_CHARACTERISTIC_UUID, b"ENABLE_OTA")
        except Exception as exc:
            self._fail("ENABLE_OTA write", str(exc))
            return False

        deadline = time.monotonic() + 15.0
        wait_start = time.monotonic()
        last_status = wait_start
        response = None
        read_attempt = 0
        while time.monotonic() < deadline:
            if not self._require_connected("ENABLE_OTA wait"):
                return False
            await asyncio.sleep(0.25)
            read_attempt += 1
            now = time.monotonic()
            if now - last_status >= 2.0:
                self.log(
                    f"[ota] ENABLE_OTA waiting for ACK... "
                    f"{now - wait_start:.0f}s elapsed, {read_attempt} reads"
                )
                last_status = now
            try:
                data = await self.client.read_gatt_char(RESPONSE_CHARACTERISTIC_UUID)
            except Exception as exc:
                self.log(f"[ota] ENABLE_OTA read error (attempt {read_attempt}): {exc}")
                if not self._require_connected("ENABLE_OTA read"):
                    return False
                continue
            response = data.decode("utf-8", errors="replace").strip()
            if response:
                self.log(f"[ota] ENABLE_OTA response: {response}")
            if "ENABLE_OTA_ACK" in (response or ""):
                self._set_state("ota_enable", "ENABLE_OTA_ACK received")
                break

        if not response or "ENABLE_OTA_ACK" not in response:
            self._fail("ENABLE_OTA", f"no ACK within 15s (last={response!r})")
            return False

        self._set_state("ota_enable_wait", f"{OTA_ENABLE_WAIT_S:.0f}s OTA window")
        self.log(f"[ota] ENABLE_OTA_ACK ok; waiting {OTA_ENABLE_WAIT_S:.0f}s for OTA service...")
        if heartbeat is not None:
            await heartbeat.sleep(OTA_ENABLE_WAIT_S, phase="ota_enable_window")
        else:
            await asyncio.sleep(OTA_ENABLE_WAIT_S)

        if not self._require_connected("post ENABLE_OTA wait"):
            return False

        try:
            services = getattr(self.client, "services", None)
            if services is None:
                services = await self.client.get_services()
        except Exception as exc:
            self._fail("service discovery", str(exc))
            return False
        if not self._services_have_update_char(services):
            self._fail("ENABLE_OTA", "update characteristic fea3 not found after ACK")
            return False
        self.log("[ota] OTA service ready on existing connection")
        return True
    
    async def update_firmware(self, firmware_path: str, heartbeat=None) -> bool:
        """Complete OTA update process"""

        # Read firmware file first
        try:
            self.log(f"Reading firmware file: {firmware_path}")
            with open(firmware_path, 'rb') as f:
                firmware_data = f.read()
        except FileNotFoundError:
            self.log(f"ERROR: Firmware file not found: {firmware_path}")
            return False
        except Exception as e:
            self.log(f"ERROR: Could not read firmware file: {e}")
            return False

        firmware_size = len(firmware_data)
        self.log(f"Firmware size: {firmware_size} bytes")
        MAX_OTA_PARTITION_SIZE = 0x480000
        if firmware_size > MAX_OTA_PARTITION_SIZE:
            self.log(f"WARNING: Firmware size ({firmware_size} bytes) exceeds OTA partition size ({MAX_OTA_PARTITION_SIZE} bytes)")
        else:
            self.log(f"Firmware size OK: {firmware_size / 1024 / 1024:.2f} MB (max: {MAX_OTA_PARTITION_SIZE / 1024 / 1024:.2f} MB)")

        self.log("Calculating MD5 hash...")
        md5_hash = self.calculate_md5(firmware_data)
        self.log(f"MD5 hash: {md5_hash}")

        if heartbeat is not None:
            heartbeat.set_phase("ota_enable_ota")
        if not await self.enable_ota_mode(heartbeat=heartbeat):
            self._fail("update_firmware", "ENABLE_OTA phase failed")
            return False

        await self.check_version()

        if not await self.prepare_update():
            self._fail("update_firmware", "PREPARE_UPDATE failed")
            return False
        if not await self.start_update():
            self._fail("update_firmware", "START_UPDATE failed")
            return False
        if not await self.send_firmware_size(firmware_size):
            self._fail("update_firmware", "SIZE send failed")
            return False
        if heartbeat is not None:
            heartbeat.set_phase("ota_transfer")
        if not await self.send_firmware_chunks(firmware_data, md5_hash, heartbeat=heartbeat):
            self._fail("update_firmware", "firmware transfer failed")
            return False
        if heartbeat is not None:
            heartbeat.set_phase("ota_finalize")
        if not await self.finalize_update():
            self._fail("update_firmware", "FINALIZE_UPDATE failed")
            return False

        self._set_state("connected", "OTA complete, reboot pending")
        self.log("[ota] UPDATE COMPLETE — device rebooting")
        return True


async def main():
    parser = argparse.ArgumentParser(description='OTA Update for ESP32 Toilet System')
    parser.add_argument('firmware', type=str, help='Path to firmware binary file (.bin)')
    parser.add_argument('--address', type=str, help='BLE device address (optional, will scan if not provided)')
    parser.add_argument('--check-version', action='store_true', help='Only check firmware version, do not update')
    
    args = parser.parse_args()
    
    updater = OTAUpdater()
    
    print("="*60)
    print("ESP32 Toilet System OTA Update")
    print("="*60)
    
    # Scan for device or use provided address
    if args.address:
        address = args.address
        print(f"Using provided address: {address}")
    else:
        address = await updater.scan_for_device()
        if not address:
            print("ERROR: Could not find device")
            return 1
    
    # Connect to device
    if not await updater.connect(address):
        print("ERROR: Failed to connect to device")
        return 1
    
    try:
        if args.check_version:
            # Only check version
            await updater.check_version()
        else:
            # Perform full update
            success = await updater.update_firmware(args.firmware)
            if not success:
                print("\n" + "="*60)
                print("OTA UPDATE FAILED!")
                print("="*60)
                print("The device should automatically rollback to the previous firmware.")
                return 1
    finally:
        await updater.disconnect()
    
    return 0


if __name__ == "__main__":
    # Check if bleak is installed
    try:
        import bleak
    except ImportError:
        print("ERROR: bleak library not installed.")
        print("Install it with: pip install bleak")
        sys.exit(1)
    
    # Run the update
    exit_code = asyncio.run(main())
    sys.exit(exit_code)
