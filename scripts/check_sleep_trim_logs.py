#!/usr/bin/env python3
"""One-shot check of current device logs for sleep_trim regression evidence."""

import asyncio
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from scripts import log_trim_assertions as assertions
from toilet_bluetooth_interface import ToiletSystemInterface


async def main() -> int:
    address = sys.argv[1] if len(sys.argv) > 1 else "58:E6:C5:6C:AF:81"
    iface = ToiletSystemInterface()
    if not await iface.connect(address, require_trust=False):
        print("BLE connect failed")
        return 1
    try:
        dev_mode = await iface.get_dev_mode_status()
        logs = await iface.get_logs()
    finally:
        await iface.disconnect()

    if logs is None:
        print("GET_LOGS failed")
        return 1

    size = assertions.log_byte_length(logs)
    lines = logs.splitlines()
    print(f"DEV mode: {dev_mode}")
    print(f"Log bytes: {size}")
    print(f"Line count: {len(lines)}")
    print(f"TRIM_MARKER_OLD_000001 present: {assertions.MARKER_OLD_FIRST in logs}")
    print(f"TRIM_MARKER_NEW_999999 present: {assertions.MARKER_NEW_LAST in logs}")
    print(f"trim_test line count: {sum(1 for ln in lines if ln.startswith('trim_test,'))}")
    print(f"At/below sleep target ({assertions.LOG_TRIM_TARGET}+slack): {size <= assertions.LOG_TRIM_TARGET + assertions.LINE_SLACK}")
    print(f"Above sleep trigger ({assertions.LOG_SLEEP_TRIM_TRIGGER}): {size > assertions.LOG_SLEEP_TRIM_TRIGGER}")

    if size <= assertions.LOG_TRIM_TARGET + assertions.LINE_SLACK:
        if assertions.MARKER_NEW_LAST in logs and assertions.MARKER_OLD_FIRST not in logs:
            print("\nSleep trim verdict: LIKELY PASS (trimmed to ~50KB, newest kept, oldest dropped)")
        elif assertions.MARKER_NEW_LAST in logs:
            print("\nSleep trim verdict: PARTIAL (size OK, newest marker present, oldest may remain)")
        else:
            print("\nSleep trim verdict: INCONCLUSIVE (size OK but no trim_test markers — may be restored pre-test SPIFFS)")
    elif size > assertions.LOG_SLEEP_TRIM_TRIGGER:
        print("\nSleep trim verdict: LIKELY FAIL or NOT TRIMMED (still above 65KB trigger)")
    else:
        print("\nSleep trim verdict: INCONCLUSIVE (between 50KB and 65KB)")

    print("\n--- first 3 lines ---")
    for line in lines[:3]:
        print(line[:120])
    print("--- last 3 lines ---")
    for line in lines[-3:]:
        print(line[:120])
    return 0


if __name__ == "__main__":
    raise SystemExit(asyncio.run(main()))
