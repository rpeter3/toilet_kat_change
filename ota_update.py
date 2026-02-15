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
from bleak import BleakClient, BleakScanner
from typing import Optional

# ESP32 BLE Configuration (from toilet_kat_change.ino)
UPDATE_SERVICE_UUID = "5636340f-afc7-47b1-b0a8-15bcb9d7d29a6"
UPDATE_CHARACTERISTIC_UUID = "c327b077-560f-46a1-8f35-b4ab0332fea3"
VERSION_CHARACTERISTIC_UUID = "c327b077-560f-46a1-8f35-b4ab0332fea2"
DEVICE_NAME = "ESP32 Toilet"
CHARACTERISTIC_UUID = "c327b077-560f-46a1-8f35-b4ab0332fea0"
SERIAL_CHARACTERISTIC_UUID = "c327b077-560f-46a1-8f35-b4ab0332fea1"

# BLE MTU is typically 20-512 bytes, we'll use 400 bytes per chunk for safety
CHUNK_SIZE = 512

class OTAUpdater:
    def __init__(self):
        self.client: Optional[BleakClient] = None
        self.connected = False
        self.update_progress = 0
        self.device_progress = 0
        
    def notification_handler(self, sender, data):
        """Handle notifications from device"""
        try:
            message = data.decode('utf-8', errors='replace')
            if "UPDATE_PROGRESS:" in message:
                # Extract progress percentage
                progress_str = message.split(":")[1]
                self.device_progress = int(progress_str)
                print(f"Device progress: {self.device_progress}%")
            elif "UPDATE_" in message:
                print(f"Device status: {message}")
        except Exception as e:
            print(f"Error handling notification: {e}")
        
    async def scan_for_device(self) -> Optional[str]:
        """Scan for ESP32 Toilet device"""
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
            # store address for potential reconnects
            self.address = address
            self.connected = True
            print(f"Connected to {DEVICE_NAME} at {address}")
            
            # Enable notifications for version characteristic to receive progress updates
            try:
                await self.client.start_notify(VERSION_CHARACTERISTIC_UUID, self.notification_handler)
            except Exception as e:
                print(f"Warning: Could not enable notifications: {e}")
            
            # Wait for connection to stabilize
            await asyncio.sleep(1.0)
            return True
        except Exception as e:
            print(f"Failed to connect: {e}")
            return False
    
    async def disconnect(self):
        """Disconnect from ESP32 device"""
        if self.client and self.connected:
            try:
                # Stop notifications
                await self.client.stop_notify(VERSION_CHARACTERISTIC_UUID)
            except:
                pass
            await self.client.disconnect()
            self.connected = False
            print("Disconnected from ESP32")

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
                    print("Could not obtain services from BleakClient")
                    return
        print("Discovered services:")
        for service in services:
            print(f"  Service: {service.uuid}")
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
                print(f"    Char: {char.uuid}  props: {props}")
    
    async def check_version(self) -> Optional[str]:
        """Check current firmware version"""
        if not self.connected:
            print("Not connected to device")
            return None
        
        try:
            # Send CHECK_VERSION command
            await self.client.write_gatt_char(UPDATE_CHARACTERISTIC_UUID, b"CHECK_VERSION")
            await asyncio.sleep(0.5)
            
            # Read version from version characteristic
            version_data = await self.client.read_gatt_char(VERSION_CHARACTERISTIC_UUID)
            try:
                version_str = version_data.decode('utf-8')
            except UnicodeDecodeError:
                # If version contains non-UTF-8 data, try with errors='replace'
                version_str = version_data.decode('utf-8', errors='replace')
            print(f"Current firmware version: {version_str}")
            return version_str
        except Exception as e:
            print(f"Failed to check version: {e}")
            # Version check is informational; don't fail the update if it errors
            return "UNKNOWN"
    
    async def prepare_update(self, max_retries: int = 3) -> bool:
        """Prepare device for OTA update with retry logic"""
        if not self.connected:
            print("Not connected to device")
            return False
        
        for attempt in range(1, max_retries + 1):
            try:
                if attempt > 1:
                    print(f"Retry attempt {attempt}/{max_retries}...")
                else:
                    print("Preparing device for update...")
                
                await self.client.write_gatt_char(UPDATE_CHARACTERISTIC_UUID, b"PREPARE_UPDATE")
                await asyncio.sleep(1.5)  # Increased wait time for device to process
                
                # Read response
                response = await self.client.read_gatt_char(UPDATE_CHARACTERISTIC_UUID)
                response_str = response.decode('utf-8')
                
                if "UPDATE_PREPARED" in response_str:
                    print("Device prepared for update")
                    return True
                elif "UPDATE_BLOCKED" in response_str:
                    if attempt < max_retries:
                        print(f"Update preparation blocked (attempt {attempt}/{max_retries})")
                        print(f"Device may be busy - waiting 3 seconds before retry...")
                        await asyncio.sleep(3.0)  # Wait before retry
                    else:
                        print("ERROR: Update preparation blocked (device may be busy)")
                        print("\nTroubleshooting suggestions:")
                        print("  1. Ensure no flush operation is running on the device")
                        print("  2. Wait a few seconds and try again")
                        print("  3. Restart the ESP32 device and retry")
                        print("  4. Check device serial output for detailed error messages")
                        return False
                else:
                    print(f"Unexpected response: {response_str}")
                    if attempt < max_retries:
                        print(f"Retrying in 2 seconds...")
                        await asyncio.sleep(2.0)
                    else:
                        return False
            except Exception as e:
                print(f"Failed to prepare update: {e}")
                if attempt < max_retries:
                    print(f"Retrying in 2 seconds...")
                    await asyncio.sleep(2.0)
                else:
                    return False
        
        return False
    
    async def start_update(self) -> bool:
        """Start OTA update process"""
        if not self.connected:
            print("Not connected to device")
            return False
        
        try:
            print("Starting OTA update...")
            await self.client.write_gatt_char(UPDATE_CHARACTERISTIC_UUID, b"START_UPDATE")
            await asyncio.sleep(0.5)
            
            # Read response
            response = await self.client.read_gatt_char(UPDATE_CHARACTERISTIC_UUID)
            response_str = response.decode('utf-8')
            
            if "UPDATE_STARTED" in response_str:
                print("OTA update started, ready to receive firmware")
                return True
            else:
                print(f"ERROR: Failed to start update: {response_str}")
                return False
        except Exception as e:
            print(f"Failed to start update: {e}")
            return False
    
    async def send_firmware_size(self, size: int) -> bool:
        """Send firmware size metadata"""
        if not self.connected:
            return False
        
        try:
            size_msg = f"SIZE:{size}".encode('utf-8')
            await self.client.write_gatt_char(UPDATE_CHARACTERISTIC_UUID, size_msg)
            await asyncio.sleep(0.2)
            print(f"Sent firmware size: {size} bytes")
            return True
        except Exception as e:
            print(f"Failed to send firmware size: {e}")
            return False
    
    async def send_firmware_chunks(self, firmware_data: bytes, md5_hash: str) -> bool:
        """Send firmware in chunks"""
        if not self.connected:
            return False
        
        total_size = len(firmware_data)
        chunks_sent = 0
        total_chunks = (total_size + CHUNK_SIZE - 1) // CHUNK_SIZE
        
        print(f"Sending firmware in {total_chunks} chunks...")
        
        try:
            # Send firmware chunks
            for i in range(0, total_size, CHUNK_SIZE):
                chunk = firmware_data[i:i + CHUNK_SIZE]
                await self.client.write_gatt_char(UPDATE_CHARACTERISTIC_UUID, chunk)
                chunks_sent += 1
                
                # Calculate and display progress
                progress = (chunks_sent * 100) // total_chunks
                if progress != self.update_progress:
                    self.update_progress = progress
                    print(f"Progress: {progress}% ({chunks_sent}/{total_chunks} chunks)")
                
                # Small delay to prevent overwhelming BLE
                await asyncio.sleep(0.05)
            
            # Send MD5 hash after all chunks
            print("Sending MD5 hash...")
            md5_msg = f"MD5:{md5_hash}".encode('utf-8')
            await self.client.write_gatt_char(UPDATE_CHARACTERISTIC_UUID, md5_msg)
            await asyncio.sleep(0.5)
            
            print("All firmware chunks sent successfully")
            return True
        except Exception as e:
            print(f"Failed to send firmware chunks: {e}")
            return False
    
    async def finalize_update(self) -> bool:
        """Finalize OTA update"""
        if not self.connected:
            return False
        
        try:
            print("Finalizing update...")
            await self.client.write_gatt_char(UPDATE_CHARACTERISTIC_UUID, b"FINALIZE_UPDATE")
            await asyncio.sleep(1.0)
            
            # Read response
            response = await self.client.read_gatt_char(UPDATE_CHARACTERISTIC_UUID)
            response_str = response.decode('utf-8')
            
            if "UPDATE_COMPLETE" in response_str:
                print("Update finalized successfully! Device will reboot...")
                return True
            elif "UPDATE_VALIDATION_FAILED" in response_str:
                print("ERROR: Firmware validation failed (MD5 mismatch please verify the calculations manually)")
                return True
            elif "UPDATE_ERROR" in response_str:
                print(f"ERROR: Update failed: {response_str}")
                return False
            else:
                print(f"Unexpected response: {response_str}")
                return False
        except Exception as e:
            print(f"Failed to finalize update: {e}")
            return False
    
    def calculate_md5(self, data: bytes) -> str:
        """Calculate MD5 hash of firmware"""
        md5_hash = hashlib.md5(data).hexdigest()
        return md5_hash
    
    async def update_firmware(self, firmware_path: str) -> bool:
        """Complete OTA update process"""

        # Read firmware file first
        try:
            print(f"Reading firmware file: {firmware_path}")
            with open(firmware_path, 'rb') as f:
                firmware_data = f.read()
        except FileNotFoundError:
            print(f"ERROR: Firmware file not found: {firmware_path}")
            return False
        except Exception as e:
            print(f"ERROR: Could not read firmware file: {e}")
            return False

        firmware_size = len(firmware_data)
        print(f"Firmware size: {firmware_size} bytes")
        MAX_OTA_PARTITION_SIZE = 0x480000
        if firmware_size > MAX_OTA_PARTITION_SIZE:
            print(f"WARNING: Firmware size ({firmware_size} bytes) exceeds OTA partition size ({MAX_OTA_PARTITION_SIZE} bytes)")
        else:
            print(f"Firmware size OK: {firmware_size / 1024 / 1024:.2f} MB (max: {MAX_OTA_PARTITION_SIZE / 1024 / 1024:.2f} MB)")

        print("Calculating MD5 hash...")
        md5_hash = self.calculate_md5(firmware_data)
        print(f"MD5 hash: {md5_hash}")

        # Request device to enter OTA mode
        print("Requesting device to enter OTA mode via main characteristic...")
        wrote_enable = False
        # Try different write modes
        try:
            await self.log_services()
        except Exception:
            pass

        try:
            print("Attempting ENABLE_OTA (with response)")
            await self.client.write_gatt_char(CHARACTERISTIC_UUID, b"ENABLE_OTA", response=True)
            wrote_enable = True
        except Exception:
            try:
                print("Attempting ENABLE_OTA (without response)")
                await self.client.write_gatt_char(CHARACTERISTIC_UUID, b"ENABLE_OTA", response=False)
                wrote_enable = True
            except Exception:
                try:
                    print("Attempting ENABLE_OTA on serial characteristic as fallback")
                    await self.client.write_gatt_char(SERIAL_CHARACTERISTIC_UUID, b"ENABLE_OTA", response=False)
                    wrote_enable = True
                except Exception as e:
                    print(f"Fallback write failed: {e}")

        if not wrote_enable:
            print("Warning: could not request OTA enable via BLE characteristic")
            return False

        # Give device time to process and reset OTA state
        print("Waiting for device to reset OTA state...")
        await asyncio.sleep(4.0)  # Increased from 1.5s to 4.0s to allow full state reset

        # Check if update characteristic is visible on current connection
        try:
            try:
                services = await self.client.get_services()
            except AttributeError:
                services = getattr(self.client, 'services', None)
        except Exception:
            services = None

        def find_update_char(svcs):
            if not svcs:
                return False
            for svc in svcs:
                for char in svc.characteristics:
                    if str(char.uuid).lower() == UPDATE_CHARACTERISTIC_UUID.lower():
                        return True
            return False

        if find_update_char(services):
            print("Update characteristic available on current connection; proceeding")
        else:
            # Try scan/reconnect flow
            print("Update characteristic not visible on current connection; waiting for device to re-advertise")

            async def wait_for_device(addr: str, timeout: float = 15.0) -> bool:
                deadline = asyncio.get_event_loop().time() + timeout
                while asyncio.get_event_loop().time() < deadline:
                    print("Scanning for device to reappear...")
                    try:
                        devices = await BleakScanner.discover(timeout=3.0)
                    except Exception as e:
                        print(f"Scan failed: {e}")
                        devices = []
                    for d in devices:
                        if d.address.lower() == addr.lower():
                            print(f"Device {addr} found again")
                            return True
                    await asyncio.sleep(0.5)
                return False

            # Disconnect current client and wait for device
            try:
                await self.disconnect()
            except Exception:
                pass

            found = await wait_for_device(self.address, timeout=20.0)
            if not found:
                print("ERROR: Device did not reappear after requesting OTA mode")
                return False

            if not await self.connect(self.address):
                print("ERROR: Failed to connect to device after it reappeared")
                return False

            # Poll for update characteristic
            ota_found = False
            deadline = asyncio.get_event_loop().time() + 15.0
            while asyncio.get_event_loop().time() < deadline:
                try:
                    try:
                        services = await self.client.get_services()
                    except AttributeError:
                        services = getattr(self.client, 'services', None)
                except Exception:
                    services = None

                if find_update_char(services):
                    ota_found = True
                    break

                await asyncio.sleep(0.8)

            if not ota_found:
                try:
                    val = await self.client.read_gatt_char(CHARACTERISTIC_UUID)
                    valstr = val.decode('utf-8', errors='ignore')
                    print(f"Main characteristic value after enable attempt: {valstr}")
                except Exception as e:
                    print(f"Could not read main characteristic: {e}")
                print("ERROR: Update characteristic did not appear on device after enable request")
                return False

        # Check version (informational only; continue even if it fails)
        v = await self.check_version()

        # Prepare, start, send and finalize update
        if not await self.prepare_update():
            return False
        if not await self.start_update():
            return False
        if not await self.send_firmware_size(firmware_size):
            return False
        if not await self.send_firmware_chunks(firmware_data, md5_hash):
            return False
        if not await self.finalize_update():
            return False

        print("\n" + "="*60)
        print("OTA UPDATE COMPLETED SUCCESSFULLY!")
        print("="*60)
        print("The device will reboot with the new firmware.")
        print("You may need to reconnect after the device reboots.")
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
