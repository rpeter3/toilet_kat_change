#!/usr/bin/env python3
"""Extract bootloader bytes from a merged flash image."""

from __future__ import annotations

import argparse
import hashlib
from pathlib import Path

BOOTLOADER_LEN = 19984


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("merged_image", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--length", type=int, default=BOOTLOADER_LEN)
    args = parser.parse_args()

    data = args.merged_image.read_bytes()[: args.length]
    if len(data) < args.length:
        raise SystemExit(f"input too small: {len(data)} < {args.length}")

    args.output.write_bytes(data)
    print(f"Wrote {len(data)} bytes -> {args.output}")
    print(f"MD5: {hashlib.md5(data).hexdigest()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
