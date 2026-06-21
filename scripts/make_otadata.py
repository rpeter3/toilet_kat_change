#!/usr/bin/env python3
"""Build otadata image that boots from the given OTA slot label."""

from __future__ import annotations

import argparse
import struct
import zlib
from pathlib import Path

# ESP-IDF: bootloader_common_ota_select_crc() CRCs only ota_seq (4 bytes)
# using esp_rom_crc32_le(UINT32_MAX, ...).


def ota_select_crc(seq: int) -> int:
    payload = struct.pack("<I", seq)
    return zlib.crc32(payload, 0xFFFFFFFF) & 0xFFFFFFFF


def make_entry(seq: int, label: str, ota_state: int = 0xFFFFFFFF) -> bytes:
    label_bytes = label.encode("ascii")[:20]
    label_bytes = label_bytes.ljust(20, b"\x00")
    crc = ota_select_crc(seq)
    return struct.pack("<I", seq) + label_bytes + struct.pack("<I", ota_state) + struct.pack("<I", crc)


def make_otadata(active_label: str, active_seq: int | None = None) -> bytes:
    if active_seq is None:
        # odd seq -> ota_0, even seq -> ota_1 (ESP-IDF bootloader convention)
        active_seq = 1 if active_label == "ota_0" else 2

    entry = make_entry(active_seq, active_label)
    sector0 = bytearray(0x1000)
    sector1 = bytearray(0x1000)
    sector0[0:32] = entry
    sector1[0:32] = entry  # duplicate copy required by bootloader
    return bytes(sector0 + sector1)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("label", choices=["ota_0", "ota_1", "factory"])
    parser.add_argument("out", type=Path)
    parser.add_argument("--seq", type=int, default=None)
    args = parser.parse_args()

    if args.label == "factory":
        data = bytes([0xFF] * 0x2000)
    else:
        data = make_otadata(args.label, args.seq)

    args.out.write_bytes(data)
    seq = struct.unpack_from("<I", data, 0)[0]
    crc = struct.unpack_from("<I", data, 28)[0]
    expected = ota_select_crc(seq)
    print(f"Wrote {len(data)} bytes for boot label {args.label} -> {args.out}")
    print(f"seq={seq} crc=0x{crc:08X} expected_crc=0x{expected:08X} ok={crc == expected}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
