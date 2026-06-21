#!/usr/bin/env python3
"""Shared ESP32 flash I/O and OTA partition parsing (esptool + otadata)."""

from __future__ import annotations

import re
import struct
import subprocess
import sys
import tempfile
import zlib
from pathlib import Path

_HW_MATRIX_DEFAULT_DATES = frozenset({"2026-02-28"})

PARTITION_TABLE_OFFSET = 0x8000
PARTITION_TABLE_MAX_SIZE = 0xC00
OTADATA_OFFSET = 0xE000
OTADATA_SIZE = 0x2000
OTADATA_RECORD_SIZE = 32
OTADATA_SECTOR_OFFSETS = (0, 0x1000)

ESP_IMAGE_MAGIC = 0xE9
ESP_APP_DESC_MAGIC = 0xABCD5432
ESP_APP_DESC_VERSION_OFFSET = 16
ESP_APP_DESC_VERSION_LEN = 32

PARTITION_ENTRY_MAGIC = 0x50AA
PARTITION_ENTRY_SIZE = 32

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

OTA_STATE_NAMES = {
    0: "NEW",
    1: "PENDING_VERIFY",
    2: "VALID",
    3: "INVALID",
    4: "ABORTED",
    0xFFFFFFFF: "UNDEFINED",
}


def subtype_name(ptype: int, subtype: int) -> str:
    if ptype == 0x00:
        return APP_SUBTYPES.get(subtype, f"app_{subtype}")
    if ptype == 0x01:
        return DATA_SUBTYPES.get(subtype, f"data_{subtype}")
    return f"type{ptype}_sub{subtype}"


def ota_select_crc(seq: int) -> int:
    """ESP-IDF bootloader_common_ota_select_crc(): CRC32-LE over ota_seq only."""
    payload = struct.pack("<I", seq)
    return zlib.crc32(payload, 0xFFFFFFFF) & 0xFFFFFFFF


def ota_state_name(state: int) -> str:
    if state in OTA_STATE_NAMES:
        return OTA_STATE_NAMES[state]
    return f"UNKNOWN(0x{state:08X})"


def seq_to_boot_slot(seq: int) -> str | None:
    """Map otadata ota_seq to ESP-IDF OTA slot (odd -> ota_0, even -> ota_1)."""
    if seq in (0, 0xFFFFFFFF):
        return None
    return "ota_0" if seq % 2 == 1 else "ota_1"


def seq_parity_label(seq: int) -> str:
    if seq % 2 == 1:
        return "odd seq"
    return "even seq"


def parse_otadata_record(rec: bytes, *, sector: int, offset: int) -> dict:
    seq = struct.unpack_from("<I", rec, 0)[0]
    label_raw = rec[4:24]
    if all(b == 0xFF for b in label_raw):
        label = "(unspecified)"
    else:
        label = label_raw.split(b"\x00")[0].decode("ascii", errors="replace") or "(unspecified)"
    ota_state = struct.unpack_from("<I", rec, 24)[0]
    crc = struct.unpack_from("<I", rec, 28)[0]
    expected_crc = ota_select_crc(seq)
    crc_ok = crc == expected_crc
    boot_slot = seq_to_boot_slot(seq)
    return {
        "sector": sector,
        "offset": offset,
        "seq": seq,
        "label": label,
        "ota_state": ota_state,
        "ota_state_name": ota_state_name(ota_state),
        "crc": crc,
        "expected_crc": expected_crc,
        "crc_ok": crc_ok,
        "boot_slot": boot_slot,
        "raw": rec,
    }


def parse_otadata(data: bytes) -> dict:
    """Parse otadata partition (two esp_ota_select_entry_t copies, 32 bytes each)."""
    records: list[dict] = []
    for sector, sector_offset in enumerate(OTADATA_SECTOR_OFFSETS):
        if sector_offset + OTADATA_RECORD_SIZE > len(data):
            break
        rec = data[sector_offset : sector_offset + OTADATA_RECORD_SIZE]
        records.append(parse_otadata_record(rec, sector=sector, offset=sector_offset))

    valid = [
        r
        for r in records
        if r["seq"] not in (0xFFFFFFFF, 0) and r["crc_ok"]
    ]
    active = max(valid, key=lambda r: r["seq"]) if valid else None
    return {"records": records, "active": active, "raw": data}


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


def parse_app_build_date(
    partition_bytes: bytes,
    *,
    expected_sw: str | None = None,
) -> str | None:
    """Extract SOFTWARE_BUILD_DATE (YYYY-MM-DD) embedded in an app partition image."""
    if not partition_bytes:
        return None
    if partition_bytes[0] != ESP_IMAGE_MAGIC:
        if len(partition_bytes) >= 256 and all(b == 0xFF for b in partition_bytes[:256]):
            return None
        return None

    text = partition_bytes.decode("latin-1", errors="ignore")

    if expected_sw:
        idx = text.find(expected_sw)
        if idx >= 0:
            after = text[idx + len(expected_sw) : idx + len(expected_sw) + 48]
            after_dates = re.findall(r"(\d{4}-\d{2}-\d{2})", after)
            if after_dates:
                return after_dates[0]
            window = text[max(0, idx - 96) : idx + len(expected_sw) + 96]
            window_dates = re.findall(r"(\d{4}-\d{2}-\d{2})", window)
            if window_dates:
                return window_dates[-1]

    dates = [
        date
        for date in set(re.findall(r"(\d{4}-\d{2}-\d{2})", text))
        if date not in _HW_MATRIX_DEFAULT_DATES
    ]
    if not dates:
        return None
    return max(dates)


def build_date_from_firmware_path(path: Path, *, expected_sw: str | None = None) -> str | None:
    """Read build date from a firmware .bin on disk."""
    if not path.is_file():
        return None
    return parse_app_build_date(path.read_bytes(), expected_sw=expected_sw)


def parse_app_version(partition_bytes: bytes) -> str:
    """Extract SOFTWARE_VERSION from esp_app_desc embedded in an app partition image."""
    if not partition_bytes:
        return "(empty)"
    if partition_bytes[0] != ESP_IMAGE_MAGIC:
        if all(b == 0xFF for b in partition_bytes[:256]):
            return "(erased)"
        return "(invalid image)"

    magic_offset = partition_bytes.find(struct.pack("<I", ESP_APP_DESC_MAGIC))
    if magic_offset < 0:
        return "(no app_desc)"

    version_start = magic_offset + ESP_APP_DESC_VERSION_OFFSET
    version_end = version_start + ESP_APP_DESC_VERSION_LEN
    if version_end > len(partition_bytes):
        return "(truncated app_desc)"

    version = (
        partition_bytes[version_start:version_end]
        .split(b"\x00")[0]
        .decode("ascii", errors="replace")
        .strip()
    )
    return version or "(blank version)"


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
                "Close Serial Monitor, BLE tools, or any app using the port."
            )
        return Path(out_path).read_bytes()
    finally:
        Path(out_path).unlink(missing_ok=True)


def format_size(size: int) -> str:
    if size % 1024 == 0:
        return f"{size // 1024} KiB"
    return f"{size} bytes"


def format_hex_line(offset: int, chunk: bytes, *, annotate: str | None = None) -> str:
    hex_part = " ".join(f"{b:02X}" for b in chunk)
    line = f"  0x{offset:04X}: {hex_part}"
    if annotate:
        line += f"  # {annotate}"
    return line


def format_otadata_record_hex(record: dict) -> list[str]:
    lines: list[str] = []
    rec = record["raw"]
    lines.append(
        f"Sector {record['sector']} @ 0x{record['offset']:04X} "
        f"(seq={record['seq']}, boot={record['boot_slot'] or 'factory/none'}, "
        f"state={record['ota_state_name']}, crc_ok={record['crc_ok']})"
    )
    lines.append(format_hex_line(record["offset"], rec[0:4], annotate="ota_seq (uint32 LE)"))
    lines.append(format_hex_line(record["offset"] + 4, rec[4:24], annotate="seq_label[20]"))
    lines.append(format_hex_line(record["offset"] + 24, rec[24:28], annotate="ota_state (uint32 LE)"))
    lines.append(
        format_hex_line(
            record["offset"] + 28,
            rec[28:32],
            annotate=f"crc (expected 0x{record['expected_crc']:08X})",
        )
    )
    return lines


def format_otadata_hex(data: bytes) -> list[str]:
    parsed = parse_otadata(data)
    lines = [f"otadata raw ({len(data)} bytes):"]
    for record in parsed["records"]:
        lines.extend(format_otadata_record_hex(record))
        lines.append("")
    return lines
