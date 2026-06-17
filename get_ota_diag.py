#!/usr/bin/env python3
"""One-shot diagnostic query: GET_OTA_DIAG, GET_LOGS, GET_BOOT_INFO (no trust required)."""

import argparse
import asyncio
import sys

from bleak import BleakScanner

from toilet_bluetooth_interface import DEVICE_NAME, ToiletSystemInterface


async def find_device(timeout_s: float) -> str | None:
    from toilet_bluetooth_interface import SERVICE_UUID

    print(f"Scanning for BLE devices ({timeout_s:.0f}s)...")
    print("Power on the toilet, wait for boot (~30s on 4.0.2), disconnect phone app.")
    devices = await BleakScanner.discover(timeout=timeout_s, service_uuids=[SERVICE_UUID])
    if not devices:
        print("No devices advertising toilet service UUID; retrying broad scan...")
        devices = await BleakScanner.discover(timeout=timeout_s)
    if not devices:
        print("No BLE devices seen.")
        return None

    print("BLE devices:")
    target = None
    for device in devices:
        name = device.name or "(no name)"
        is_target = device.name == DEVICE_NAME
        if is_target:
            target = device.address
        print(f"  {name} @ {device.address}{'  <-- ESP32 Toilet' if is_target else ''}")

    if target:
        return target

    # If only one device matched service UUID filter, use it even without the name.
    if len(devices) == 1:
        only = devices[0]
        print(f"\nUsing sole service-UUID match: {only.name or '(no name)'} @ {only.address}")
        return only.address

    return None


async def main() -> int:
    parser = argparse.ArgumentParser(description="Query OTA/boot diagnostics over BLE")
    parser.add_argument(
        "--address",
        help="BLE MAC address (skip scan if provided, e.g. AA:BB:CC:DD:EE:FF)",
    )
    parser.add_argument(
        "--scan-seconds",
        type=float,
        default=20.0,
        help="BLE scan duration when address not provided (default: 20)",
    )
    args = parser.parse_args()

    interface = ToiletSystemInterface()
    address = args.address
    if not address:
        address = await find_device(args.scan_seconds)
    if not address:
        print("\nDevice not found. Retry with --address if you know the MAC from a prior session.")
        return 1

    print(f"\nConnecting to {address} (no trust handshake)...")
    if not await interface.connect(address, require_trust=False):
        print("Connection failed.")
        return 1

    try:
        for command in ("GET_OTA_DIAG", "GET_BOOT_INFO", "GET_LOGS"):
            print(f"\n--- {command} ---")
            if command == "GET_LOGS":
                logs = await interface.get_logs()
                if logs is None:
                    print("(unavailable)")
                elif logs == "":
                    print("(empty)")
                else:
                    print(logs[:4000])
                    if len(logs) > 4000:
                        print(f"... truncated, total {len(logs)} chars")
            else:
                resp = await interface._send_command_and_read_response(command)
                print(resp if resp else "(no response)")
    finally:
        await interface.disconnect()
        print("\nDone.")
    return 0


if __name__ == "__main__":
    sys.exit(asyncio.run(main()))
