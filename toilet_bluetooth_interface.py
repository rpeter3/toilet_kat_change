#!/usr/bin/env python3
"""
ESP32 Toilet System Bluetooth Interface
Connects to ESP32 via BLE to read and update system parameters
"""

import asyncio
import struct
from bleak import BleakClient, BleakScanner
import time
import json
from typing import Dict, Any, Optional

# ESP32 BLE Configuration (from toilet_kat_change.ino)
SERVICE_UUID = "5636340f-afc7-47b1-b0a8-15bc9d7d29a5"
CHARACTERISTIC_UUID = "c327b077-560f-46a1-8f35-b4ab0332fea0"
SERIAL_CHARACTERISTIC_UUID = "c327b077-560f-46a1-8f35-b4ab0332fea1"
DEVICE_NAME = "ESP32 Toilet"

class ToiletSystemInterface:
    def __init__(self):
        self.client: Optional[BleakClient] = None
        self.connected = False
        self.current_params = {}
        self.serial_streaming = False
        
        # Parameter definitions with descriptions and units (defaults = 1.5mil High Barrier Plastic from material_parameters.csv)
        self.param_definitions = {
            "batteryThreshold": {"description": "Battery voltage threshold", "units": "ADC", "default": 5.0},
            "K": {"description": "Temperature setpoint", "units": "°C", "default": 150.0},
            "F": {"description": "How long to feed the bag at the START of a flush", "units": "sec", "default": 6.0},
            "T": {"description": "Cooling Time", "units": "sec", "default": 40.0},
            "backupTime": {"description": "How long to back up the bag when re-opening", "units": "sec", "default": 1.0},
            "fanDuration": {"description": "How long to run the fan after feeding at the end of a flush", "units": "sec", "default": 5.0},
            "H": {"description": "Heater On time", "units": "sec", "default": 20.0},
            "continueFeeder": {"description": "Feeder continue time", "units": "sec", "default": 6.0},
            "maxOpeningTime": {"description": "Max opening time", "units": "sec", "default": 12.0},
            "typicalOpeningTime": {"description": "Typical opening time", "units": "sec", "default": 10.0},
            "MOTOR_CUT_TIME": {"description": "Motor cut duration", "units": "sec", "default": 0.5},
            "CUT_MODE_HEAT_TIME": {"description": "Additional heater time in cut mode", "units": "sec", "default": 10.0},
            "postCoolingBagDuration": {"description": "Fan duration before feed motors start in case 10", "units": "sec", "default": 5.0},
            "preFeedFan": {"description": "Fan duration before feed motor starts in case 1 and button 2", "units": "sec", "default": 2.0},
            "fanReverseTime": {"description": "Duration M3 runs in reverse after starting", "units": "sec", "default": 10.0},
            "fanReverseStartTime": {"description": "Delay before M3 reverse starts as percentage of typicalOpeningTime after M1 begins closing", "units": "%", "default": 0.0},
            "backupTimeAfterReopen": {"description": "Feed bag backup duration after mechanism motor finishes opening", "units": "sec", "default": 1.7},
            "CUT_MODE_TEMP": {"description": "Temperature to maintain for CUT_MODE_HEAT_TIME after cut motor", "units": "°C", "default": 175.0}
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
                "postCoolingBagDuration": 5.0,
                "preFeedFan": 2.0,
                "fanReverseTime": 10.0,
                "fanReverseStartTime": 0.0,
                "backupTimeAfterReopen": 1.7,
                "CUT_MODE_TEMP": 175.0
            },
            "Compostable 1.5mil": {
                "batteryThreshold": 5.0,
                "K": 90.0,
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
                "postCoolingBagDuration": 5.0,
                "preFeedFan": 2.0,
                "fanReverseTime": 10.0,
                "fanReverseStartTime": 0.0,
                "backupTimeAfterReopen": 1.7,
                "CUT_MODE_TEMP": 105.0
            }
        }
        
        # Parameter order (18 values, as expected by ESP32 BLE)
        self.param_order = [
            "batteryThreshold", "K", "F", "T", "backupTime",
            "fanDuration", "H", "continueFeeder", "maxOpeningTime", "typicalOpeningTime",
            "MOTOR_CUT_TIME", "CUT_MODE_HEAT_TIME", "postCoolingBagDuration", "preFeedFan",
            "fanReverseTime", "fanReverseStartTime", "backupTimeAfterReopen", "CUT_MODE_TEMP"
        ]

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
            self.connected = True
            print(f"Connected to {DEVICE_NAME} at {address}")
            
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

    async def update_params(self, new_params: Dict[str, Any]) -> bool:
        """Update parameters on ESP32"""
        if not self.connected:
            print("Not connected to device")
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
            # Send start command to ESP32
            await self.client.write_gatt_char(SERIAL_CHARACTERISTIC_UUID, b"START_SERIAL")
            self.serial_streaming = True
            print("Serial streaming started")
            return True
        except Exception as e:
            print(f"Failed to start serial streaming: {e}")
            return False

    async def stop_serial_streaming(self) -> bool:
        """Stop serial streaming from ESP32"""
        if not self.connected:
            return False
        
        try:
            # Send stop command to ESP32
            await self.client.write_gatt_char(SERIAL_CHARACTERISTIC_UUID, b"STOP_SERIAL")
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
            message = data.decode('utf-8')
            print(f"{message}", end='', flush=True)  # Use end='' to handle newlines from ESP32
        except Exception as e:
            print(f"Error decoding serial data: {e}")
            print(f"Raw data: {data}")

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
            
            choice = input("\nEnter your choice (1-9): ").strip()
            
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
            
            else:
                print("Invalid choice. Please enter 1-9.")
    
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