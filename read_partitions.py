#!/usr/bin/env python3
"""Read and display ESP32 flash partition table via esptool (USB serial)."""

from __future__ import annotations

import argparse
import struct
import subprocess
import sys
import tempfile
from pathlib import Path

PARTITION_TABLE_OFFSET = 0x8000
PARTITION_TABLE_MAX_SIZE = 0xC00
OTADATA_OFFSET = 0xE000
OTADATA_SIZE = 0x2000

APP_SUBTYPES = {
    0x00: "factory",
    0x10: "ota_0",
    0x11: "ota_1",
    0x12: "ota_2",
}

DATA_SUBTYPES = {
    0x00: "ota",
    0x01: "phy",
    0x02: "nvs",
    0x03: "coredump",
    0x82: "spiffs",
}


def subtype_name(ptype: int, subtype: int) -> str:
    if ptype == 0x00:
        return APP_SUBTYPES.get(subtype, f"app_{subtype}")
    if ptype == 0x01:
        return DATA_SUBTYPES.get(subtype, f"data_{subtype}")
    return f"type{ptype}_sub{subtype}"


PARTITION_ENTRY_MAGIC = 0x50AA
PARTITION_ENTRY_SIZE = 32


def parse_partition_entries(data: bytes) -> list[dict]:
    entries: list[dict] = []
    i = 0
    while i + PARTITION_ENTRY_SIZE <= len(data):
        chunk = data[i : i + PARTITION_ENTRY_SIZE]
        magic = struct.unpack_from("<H", chunk, 0)[0]
        if magic != PARTITION_ENTRY_MAGIC:
            break
        ptype = chunk[2]
        subtype = chunk[3]
        offset = struct.unpack_from("<I", chunk, 4)[0]
        size = struct.unpack_from("<I", chunk, 8)[0]
        name = chunk[12:28].split(b"\x00")[0].decode("ascii", errors="replace")
        flags = struct.unpack_from("<I", chunk, 28)[0]
        if not name:
            break
        entries.append(
            {
                "name": name,
                "type": ptype,
                "subtype": subtype,
                "subtype_name": subtype_name(ptype, subtype),
                "offset": offset,
                "size": size,
                "flags": flags,
            }
        )
        i += PARTITION_ENTRY_SIZE
    return entries


def parse_otadata(data: bytes) -> dict:
    """Parse otadata (two OTA selection records, 32 bytes each)."""
    records = []
    for slot, offset in enumerate((0, 0x1000)):
        if offset + 32 > len(data):
            break
        rec = data[offset : offset + 32]
        if len(rec) < 32:
            break
        seq = struct.unpack_from("<I", rec, 0)[0]
        label_raw = rec[4:24]
        if all(b == 0xFF for b in label_raw):
            label = "(unspecified)"
        else:
            label = label_raw.split(b"\x00")[0].decode("ascii", errors="replace")
        ota_state = struct.unpack_from("<I", rec, 24)[0]
        crc = struct.unpack_from("<I", rec, 28)[0]
        records.append(
            {
                "slot": slot,
                "seq": seq,
                "label": label,
                "ota_state": ota_state,
                "crc": crc,
            }
        )

    active = None
    valid = [r for r in records if r["seq"] not in (0xFFFFFFFF, 0)]
    if valid:
        active = max(valid, key=lambda r: r["seq"])
    return {"records": records, "active": active}


def read_flash(port: str, chip: str, address: int, size: int) -> bytes:
    with tempfile.NamedTemporaryFile(delete=False, suffix=".bin") as tmp:
        out_path = tmp.name
    try:
        cmd = [
            sys.executable,
            "-m",
            "esptool",
            "--chip",
            chip,
            "--port",
            port,
            "--baud",
            "460800",
            "read-flash",
            str(address),
            str(size),
            out_path,
        ]
        result = subprocess.run(cmd, capture_output=True, text=True)
        if result.returncode != 0:
            print(result.stdout, file=sys.stderr)
            print(result.stderr, file=sys.stderr)
            raise RuntimeError(
                f"esptool read-flash failed (exit {result.returncode}). "
                f"If COM6 is busy, close Serial Monitor, the BLE Python tool, or any app using the port."
            )
        return Path(out_path).read_bytes()
    finally:
        Path(out_path).unlink(missing_ok=True)


def format_size(size: int) -> str:
    if size % 1024 == 0:
        return f"{size // 1024} KiB"
    return f"{size} bytes"


def main() -> int:
    parser = argparse.ArgumentParser(description="Read ESP32 partition table over serial")
    parser.add_argument("--port", "-p", default="COM6", help="Serial port (default: COM6)")
    parser.add_argument("--chip", "-c", default="esp32s3", help="Chip type (default: esp32s3)")
    args = parser.parse_args()

    print(f"Reading partition table from {args.port} ({args.chip})...")
    print(f"  flash offset 0x{PARTITION_TABLE_OFFSET:X}, size 0x{PARTITION_TABLE_MAX_SIZE:X}")
    pt_data = read_flash(args.port, args.chip, PARTITION_TABLE_OFFSET, PARTITION_TABLE_MAX_SIZE)
    entries = parse_partition_entries(pt_data)
    if not entries:
        print("ERROR: No partition entries found (empty or unreadable table).")
        return 1

    print(f"\nPartition table ({len(entries)} entries):\n")
    print(f"{'Name':<12} {'Type':<12} {'Offset':>10} {'Size':>10}  {'End':>10}  Flags")
    print("-" * 72)
    for e in entries:
        end = e["offset"] + e["size"]
        name = e["name"].encode("ascii", errors="backslashreplace").decode("ascii")
        print(
            f"{name:<12} {e['subtype_name']:<12} "
            f"0x{e['offset']:08X} 0x{e['size']:08X}  0x{end:08X}  0x{e['flags']:08X}"
        )
        print(f"             ({format_size(e['size'])})")

    print(f"\nReading otadata from 0x{OTADATA_OFFSET:X}...")
    ota_data = read_flash(args.port, args.chip, OTADATA_OFFSET, OTADATA_SIZE)
    ota = parse_otadata(ota_data)
    print("\nOTA boot selection (otadata):")
    for r in ota["records"]:
        print(
            f"  slot {r['slot']}: seq={r['seq']}, label={r['label']!r}, "
            f"ota_state=0x{r['ota_state']:08X}, crc=0x{r['crc']:08X}"
        )
    if ota["active"]:
        a = ota["active"]
        print(f"\nActive OTA boot record (highest valid seq): seq={a['seq']}, label={a['label']}")
        match = None
        if a["label"] not in ("(unspecified)", ""):
            match = next((e for e in entries if e["name"] == a["label"]), None)
        if match:
            print(
                f"  -> boot partition: {match['name']} @ 0x{match['offset']:08X} "
                f"({match['subtype_name']})"
            )
        elif a["seq"] >= 1:
            print("  -> label unspecified; device has likely booted from ota_0 or ota_1 (not factory)")
    else:
        print("\nNo valid OTA seq in otadata (bootloader may use factory app).")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
