#!/usr/bin/env python3
"""USB OTA partition diagnostic report (partition table, slot versions, otadata)."""

from __future__ import annotations

import argparse
import re
import sys
from datetime import datetime, timezone
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR))

from esp_flash_tools import (  # noqa: E402
    APP_SUBTYPES,
    OTADATA_OFFSET,
    OTADATA_SIZE,
    PARTITION_TABLE_MAX_SIZE,
    PARTITION_TABLE_OFFSET,
    format_otadata_hex,
    format_size,
    parse_app_version,
    parse_otadata,
    parse_partition_entries,
    read_flash,
    seq_parity_label,
    seq_to_boot_slot,
)

APP_SLOT_NAMES = ("factory", "ota_0", "ota_1")
APP_PROBE_SIZE = 0x10000
SEMVER_RE = re.compile(rb"\d+\.\d+\.\d+(?:\.\d+)?(?:-[A-Za-z0-9]+)?")


def detect_software_version(partition_bytes: bytes) -> str:
    """Prefer esp_app_desc version; fall back to semver literal in image."""
    desc_version = parse_app_version(partition_bytes)
    if desc_version not in ("(empty)", "(erased)", "(invalid image)", "(no app_desc)", "(truncated app_desc)", "(blank version)"):
        if re.fullmatch(r"\d+\.\d+\.\d+", desc_version):
            return desc_version

    window = partition_bytes[: min(len(partition_bytes), 0x80000)]
    candidates = SEMVER_RE.findall(window)
    if not candidates:
        return desc_version

    decoded = [c.decode("ascii", errors="replace") for c in candidates]
    for candidate in decoded:
        if re.fullmatch(r"\d+\.\d+\.\d+", candidate):
            return candidate
    return decoded[0]


def image_label_for_slot(name: str, version: str) -> str:
    if version in ("(erased)", "(empty)", "(invalid image)"):
        return "(empty)"
    if name == "factory" and version not in ("(no app_desc)", "(blank version)"):
        return "SARAH" if version == "3.0.0" or "64767cc" in version else f"factory ({version})"
    if version.startswith("("):
        return "(unknown)"
    return f"firmware-{version}.bin"


def build_report(port: str, chip: str) -> str:
    lines: list[str] = []
    now = datetime.now(timezone.utc).strftime("%Y-%m-%d %H:%M:%S UTC")

    lines.append("ESP32 OTA Partition Report")
    lines.append(f"Port: {port}  Chip: {chip}  Generated: {now}")
    lines.append("")

    pt_data = read_flash(port, chip, PARTITION_TABLE_OFFSET, PARTITION_TABLE_MAX_SIZE)
    entries = parse_partition_entries(pt_data)
    if not entries:
        raise RuntimeError("No partition entries found in flash.")

    app_entries = {e["name"]: e for e in entries if e["name"] in APP_SLOT_NAMES}

    lines.append("=== 1. Current flash layout ===")
    lines.append(f"{'Partition':<12} {'Offset':>12} {'Image':<28} {'SOFTWARE_VERSION':<16}")
    lines.append("-" * 72)

    slot_versions: dict[str, str] = {}
    for name in APP_SLOT_NAMES:
        entry = app_entries.get(name)
        if not entry:
            lines.append(f"{name:<12} {'(missing)':>12} {'':<28} {'':<16}")
            continue
        probe = read_flash(port, chip, entry["offset"], APP_PROBE_SIZE)
        version = detect_software_version(probe)
        slot_versions[name] = version
        image = image_label_for_slot(name, version)
        lines.append(
            f"{name:<12} 0x{entry['offset']:08X} {image:<28} {version:<16}"
        )

    lines.append("")
    lines.append("=== 2. otadata (boot selection) ===")
    ota_data = read_flash(port, chip, OTADATA_OFFSET, OTADATA_SIZE)
    ota = parse_otadata(ota_data)
    active = ota["active"]

    if not active:
        lines.append("Active seq: (none valid)")
        lines.append("ESP-IDF boots: factory (no valid otadata seq)")
        lines.append("Label in otadata: (n/a)")
    else:
        boot_slot = active["boot_slot"] or "factory"
        parity = seq_parity_label(active["seq"])
        lines.append(f"Active seq: {active['seq']}")
        lines.append(f"ESP-IDF boots: {boot_slot} ({parity})")
        lines.append(f"Label in otadata: {active['label']} (informational only)")
        lines.append(
            f"ota_state: {active['ota_state_name']} (0x{active['ota_state']:08X})  "
            f"crc_ok: {active['crc_ok']}"
        )

    lines.append("")
    lines.append("=== 3. otadata records (decoded) ===")
    for record in ota["records"]:
        lines.append(
            f"Sector {record['sector']} @ 0x{record['offset']:04X}: "
            f"seq={record['seq']}, label={record['label']!r}, "
            f"state={record['ota_state_name']}, "
            f"crc=0x{record['crc']:08X} (ok={record['crc_ok']}), "
            f"boot={record['boot_slot'] or 'factory/none'}"
        )

    lines.append("")
    lines.append("=== 4. Raw otadata binary (boot selection fields) ===")
    lines.extend(format_otadata_hex(ota_data))

    lines.append("=== 5. Seq reference (ESP-IDF) ===")
    lines.append("odd seq -> ota_0, even seq -> ota_1 (seq 0 / 0xFFFFFFFF -> factory)")
    if active and active["seq"] >= 1:
        active_seq = active["seq"]
        prev_seq = active_seq - 1
        lines.append(
            f"seq {active_seq} (now) -> {seq_to_boot_slot(active_seq)}  "
            f"version in slot: {slot_versions.get(seq_to_boot_slot(active_seq) or '', 'n/a')}"
        )
        if prev_seq >= 1:
            prev_slot = seq_to_boot_slot(prev_seq)
            lines.append(
                f"seq {prev_seq} (prev) -> {prev_slot}  "
                f"version in slot: {slot_versions.get(prev_slot or '', 'n/a')}"
            )
    else:
        lines.append("(no active OTA seq)")

    lines.append("")
    lines.append("=== 6. Full partition table ===")
    lines.append(f"{'Name':<12} {'Type':<12} {'Offset':>10} {'Size':>10}  Flags")
    lines.append("-" * 60)
    for e in entries:
        lines.append(
            f"{e['name']:<12} {e['subtype_name']:<12} "
            f"0x{e['offset']:08X} 0x{e['size']:08X} ({format_size(e['size'])})  "
            f"0x{e['flags']:08X}"
        )

    return "\n".join(lines)


def main() -> int:
    parser = argparse.ArgumentParser(description="USB OTA partition state report")
    parser.add_argument("--port", "-p", default="COM6", help="Serial port (default: COM6)")
    parser.add_argument("--chip", "-c", default="esp32s3", help="Chip type (default: esp32s3)")
    parser.add_argument("--output", "-o", type=Path, help="Write report to file")
    args = parser.parse_args()

    try:
        report = build_report(args.port, args.chip)
    except RuntimeError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1

    print(report)
    if args.output:
        args.output.write_text(report, encoding="utf-8")
        print(f"\nWrote report -> {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
