#!/usr/bin/env python3
"""Connect to the toilet over BLE, complete trust handshake, set maxOpeningTime."""

import argparse
import asyncio
import sys
from pathlib import Path

# Repo root on sys.path so toilet_bluetooth_interface / get_ota_diag resolve when run from scripts/
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from get_ota_diag import find_device
from toilet_bluetooth_interface import ToiletSystemInterface

PARAM_NAME = "maxOpeningTime"
DEFAULT_VALUE = 16.5


async def main() -> int:
    parser = argparse.ArgumentParser(
        description="BLE connect + trust handshake, then set maxOpeningTime (seconds)."
    )
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
    parser.add_argument(
        "--trust-timeout",
        type=float,
        default=60.0,
        help="Trust handshake timeout in seconds (default: 60)",
    )
    parser.add_argument(
        "--value",
        type=float,
        default=DEFAULT_VALUE,
        help=f"maxOpeningTime value in seconds (default: {DEFAULT_VALUE})",
    )
    args = parser.parse_args()

    address = args.address
    if not address:
        address = await find_device(args.scan_seconds)
    if not address:
        print("\nDevice not found. Retry with --address if you know the MAC.")
        return 1

    interface = ToiletSystemInterface()
    interface.trust_timeout_s = args.trust_timeout

    print(f"\nConnecting to {address} (trust handshake required)...")
    print("Press a control panel button on the toilet when LEDs circle to confirm trust.")
    if not await interface.connect(address, require_trust=True):
        print("Connection or trust handshake failed.")
        return 1

    try:
        params = await interface.read_current_params()
        if not params:
            print("Failed to read current parameters.")
            return 1

        before = params.get(PARAM_NAME)
        print(f"\nCurrent {PARAM_NAME}: {before}")

        if not await interface.update_single_param(PARAM_NAME, args.value):
            print(f"Failed to update {PARAM_NAME}.")
            return 1

        params = await interface.read_current_params()
        after = params.get(PARAM_NAME)
        print(f"Updated {PARAM_NAME}: {after} (requested {args.value})")

        print("\nDone.")
        return 0
    finally:
        await interface.disconnect()


if __name__ == "__main__":
    raise SystemExit(asyncio.run(main()))
