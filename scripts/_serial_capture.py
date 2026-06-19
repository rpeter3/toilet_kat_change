"""Capture serial output from COM port for a fixed duration."""
import argparse
import sys
import time

import serial


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="COM5")
    parser.add_argument("--seconds", type=float, default=15.0)
    args = parser.parse_args()

    ser = serial.Serial(args.port, 115200, timeout=0.5)
    end = time.time() + args.seconds
    while time.time() < end:
        chunk = ser.read(4096)
        if chunk:
            sys.stdout.write(chunk.decode("utf-8", errors="replace"))
            sys.stdout.flush()
    ser.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
