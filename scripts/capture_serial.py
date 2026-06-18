#!/usr/bin/env python3
"""Capture serial output for a fixed duration and optionally check for markers."""

import sys
import time

import serial


def main() -> int:
    if len(sys.argv) < 3:
        print("usage: capture_serial.py PORT LOG_PATH [seconds]", file=sys.stderr)
        return 1

    port = sys.argv[1]
    log_path = sys.argv[2]
    duration = float(sys.argv[3]) if len(sys.argv) > 3 else 15.0

    ser = serial.Serial(port, 115200, timeout=0.2)
    lines: list[str] = []
    deadline = time.time() + duration
    try:
        while time.time() < deadline:
            chunk = ser.read(4096)
            if chunk:
                text = chunk.decode("utf-8", errors="replace")
                print(text, end="", flush=True)
                lines.append(text)
    finally:
        ser.close()

    body = "".join(lines)
    with open(log_path, "w", encoding="utf-8") as f:
        f.write(body)

    if "[PASS] Phase 0 ROM round-trip" in body:
        print("\n=== RESULT: PASS ===")
        return 0
    if "Phase 0 ROM round-trip: erase+write+verify succeeded" in body:
        print("\n=== RESULT: PASS ===")
        return 0
    if "[PASS] in-app ROM erase of 0x0..0x8000 completed" in body:
        print("\n=== RESULT: PASS ===")
        return 0
    if "[FAIL]" in body or "Phase 0: read-back MD5 mismatch" in body:
        print("\n=== RESULT: FAIL ===")
        return 2
    print("\n=== RESULT: INCONCLUSIVE (no PASS/FAIL marker) ===")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
