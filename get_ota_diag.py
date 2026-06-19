#!/usr/bin/env python3
"""One-shot diagnostic query: GET_OTA_DIAG, GET_LOGS, GET_BOOT_INFO (no trust required)."""

import argparse
import asyncio
import sys

from bleak import BleakScanner

from toilet_bluetooth_interface import DEVICE_NAME, ToiletSystemInterface


def _normalize_address(address: str) -> str:
    return address.strip().lower()


def _format_device_line(device, *, expected_address: str | None = None) -> str:
    name = device.name or "(no name)"
    markers: list[str] = []
    if device.name == DEVICE_NAME:
        markers.append("ESP32 Toilet")
    if expected_address and _normalize_address(device.address) == _normalize_address(expected_address):
        markers.append("expected address")
    suffix = f"  <-- {', '.join(markers)}" if markers else ""
    return f"  {name} @ {device.address}{suffix}"


async def report_visible_ble_scan(
    timeout_s: float,
    *,
    expected_address: str | None = None,
    reason: str = "BLE target not found",
) -> list:
    """Scan, print all visible BLE devices, and state whether the toilet is advertising."""
    from toilet_bluetooth_interface import SERVICE_UUID

    print(f"\n=== BLE scan ({reason}) ===")
    print(f"Scanning for BLE devices ({timeout_s:.0f}s)...")
    print("Power on the toilet, wait for boot (~30s on 4.0.2), disconnect phone app.")

    filtered = await BleakScanner.discover(timeout=timeout_s, service_uuids=[SERVICE_UUID])
    broad = await BleakScanner.discover(timeout=timeout_s) if not filtered else filtered

    seen: dict[str, object] = {}
    for device in filtered:
        seen[_normalize_address(device.address)] = device
    for device in broad:
        seen.setdefault(_normalize_address(device.address), device)
    devices = list(seen.values())

    target_by_name = None
    target_by_address = None
    for device in devices:
        if device.name == DEVICE_NAME:
            target_by_name = device.address
        if expected_address and _normalize_address(device.address) == _normalize_address(expected_address):
            target_by_address = device.address

    if not filtered:
        print("No devices advertising toilet service UUID in filtered scan.")

    if not devices:
        print("No BLE devices visible.")
        if expected_address:
            print(f"Expected address {expected_address} is NOT advertising.")
        else:
            print(f"{DEVICE_NAME} is NOT advertising.")
        return []

    print(f"Visible BLE devices ({len(devices)}):")
    for device in sorted(devices, key=lambda d: (d.name or "", d.address)):
        print(_format_device_line(device, expected_address=expected_address))

    if target_by_name:
        print(f"\n{DEVICE_NAME} is advertising at {target_by_name}.")
    elif target_by_address:
        matched = next(
            d for d in devices if _normalize_address(d.address) == _normalize_address(expected_address)
        )
        print(
            f"\nExpected address {expected_address} is advertising "
            f"(name: {matched.name or '(no name)'})."
        )
    else:
        if expected_address:
            print(f"\nExpected address {expected_address} is NOT advertising.")
        print(f"{DEVICE_NAME} is NOT advertising.")
    print("=== End BLE scan ===\n")
    return devices


async def find_device(timeout_s: float) -> str | None:
    from toilet_bluetooth_interface import SERVICE_UUID

    print(f"Scanning for BLE devices ({timeout_s:.0f}s)...")
    print("Power on the toilet, wait for boot (~30s on 4.0.2), disconnect phone app.")
    devices = await BleakScanner.discover(timeout=timeout_s, service_uuids=[SERVICE_UUID])
    if not devices:
        print("No devices advertising toilet service UUID; retrying broad scan...")
        devices = await BleakScanner.discover(timeout=timeout_s)
    if not devices:
        await report_visible_ble_scan(timeout_s, reason="initial discovery found no devices")
        return None

    print("BLE devices:")
    target = None
    for device in devices:
        print(_format_device_line(device))
        if device.name == DEVICE_NAME:
            target = device.address

    if target:
        return target

    # If only one device matched service UUID filter, use it even without the name.
    if len(devices) == 1:
        only = devices[0]
        print(f"\nUsing sole service-UUID match: {only.name or '(no name)'} @ {only.address}")
        return only.address

    await report_visible_ble_scan(
        timeout_s,
        reason=f"{DEVICE_NAME} not found by name in discovery results",
    )
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
        for command in ("GET_ACTIVE_PARTITION", "GET_OTA_DIAG", "GET_BOOT_INFO", "GET_LOGS"):
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
