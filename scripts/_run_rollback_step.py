#!/usr/bin/env python3
"""One-shot BLE OTA_ROLLBACK_PREVIOUS test with DEV mode auto-trust."""
from __future__ import annotations

import argparse
import asyncio
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from toilet_bluetooth_interface import ToiletSystemInterface  # noqa: E402


async def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--address", required=True)
    parser.add_argument("--boot-wait", type=float, default=45.0)
    args = parser.parse_args()

    print(f"Waiting {args.boot_wait:.0f}s for post-OTA boot...")
    await asyncio.sleep(args.boot_wait)

    interface = ToiletSystemInterface()
    if not await interface.connect(args.address, require_trust=False):
        print("FAIL: could not connect for diagnostics")
        return 1

    try:
        partition = await interface.get_active_partition()
        version_resp = await interface._send_command_and_read_response(
            "GET_HW_COMPONENT:SOFTWARE_VERSION_NUMBER"
        )
        ota_diag = await interface._send_command_and_read_response("GET_OTA_DIAG")
        print(f"Pre-rollback partition: {partition}")
        print(f"Pre-rollback version: {version_resp}")
        print(f"Pre-rollback OTA diag: {ota_diag}")

        dev_mode = await interface.get_dev_mode_status()
        if dev_mode != 1:
            print("Enabling DEV mode for automated trust bypass...")
            if not await interface.set_dev_mode(1):
                print("FAIL: SET_DEV_MODE:1 rejected")
                return 1
    finally:
        await interface.disconnect()

    if not await interface.connect(args.address, require_trust=True):
        print("FAIL: trusted connect failed (press panel button if DEV mode off)")
        return 1

    try:
        ok = await interface.ota_rollback_previous()
        if not ok:
            print("FAIL: OTA_ROLLBACK_PREVIOUS rejected")
            return 1
        print("PASS: OTA_ROLLBACK_PREVIOUS accepted; device rebooting")
    finally:
        await interface.disconnect()

    await asyncio.sleep(args.boot_wait)

    if not await interface.connect(args.address, require_trust=False):
        print("FAIL: could not reconnect after rollback reboot")
        return 1
    try:
        partition = await interface.get_active_partition()
        version_resp = await interface._send_command_and_read_response(
            "GET_HW_COMPONENT:SOFTWARE_VERSION_NUMBER"
        )
        ota_diag = await interface._send_command_and_read_response("GET_OTA_DIAG")
        print(f"Post-rollback partition: {partition}")
        print(f"Post-rollback version: {version_resp}")
        print(f"Post-rollback OTA diag: {ota_diag}")

        label = (partition or {}).get("label", "")
        if "ota_0" not in label:
            print(f"FAIL: expected ota_0 after rollback, got {label}")
            return 1
        if version_resp and "4.1.8-regA" not in version_resp:
            print(f"FAIL: expected 4.1.8-regA after rollback, got {version_resp}")
            return 1
        print("PASS: rollback landed on ota_0 / 4.1.8-regA")
        return 0
    finally:
        await interface.disconnect()


if __name__ == "__main__":
    raise SystemExit(asyncio.run(main()))
