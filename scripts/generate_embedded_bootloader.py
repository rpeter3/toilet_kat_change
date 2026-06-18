#!/usr/bin/env python3
"""Generate C header with embedded bootloader bytes."""

from __future__ import annotations

import argparse
import hashlib
from pathlib import Path

BOOTLOADER_LEN = 19984


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("bootloader_bin", type=Path)
    parser.add_argument("output_header", type=Path)
    args = parser.parse_args()

    data = args.bootloader_bin.read_bytes()
    if len(data) != BOOTLOADER_LEN:
        raise SystemExit(f"expected {BOOTLOADER_LEN} bytes, got {len(data)}")

    md5 = hashlib.md5(data).hexdigest()
    lines = [
        "#pragma once",
        "#include <stddef.h>",
        "#include <stdint.h>",
        "",
        f"static const size_t EMBEDDED_BOOTLOADER_LEN = {len(data)};",
        f'static const char EMBEDDED_BOOTLOADER_MD5[] = "{md5}";',
        "static const uint8_t EMBEDDED_BOOTLOADER[] = {",
    ]

    for i in range(0, len(data), 16):
        chunk = data[i : i + 16]
        hexes = ", ".join(f"0x{b:02X}" for b in chunk)
        lines.append(f"  {hexes},")

    lines.append("};")
    lines.append("")

    args.output_header.write_text("\n".join(lines), encoding="utf-8")
    print(f"Wrote {args.output_header} ({len(data)} bytes, md5={md5})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
