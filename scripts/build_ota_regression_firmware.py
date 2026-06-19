#!/usr/bin/env python3
"""Build OTA regression firmware variants and emit an auto manifest."""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import shutil
import subprocess
import sys
from datetime import date
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
SKETCH_DIR = REPO_ROOT / "toilet_kat_change"
INO_PATH = SKETCH_DIR / "toilet_kat_change.ino"
VERSION_HEADER = SKETCH_DIR / "software_version_build.h"
BUILD_SCRIPT = REPO_ROOT / "scripts" / "build-firmware.ps1"
EXTRACT_SCRIPT = REPO_ROOT / "scripts" / "extract_factory_app.py"
SARAH_MERGED = REPO_ROOT / "FACTORY_BINARIES" / "SARAH_toilet_kat_change.ino.merged.bin"
OUT_DIR = REPO_ROOT / "test-builds" / "ota-regression"
BUILD_OUTPUT = SKETCH_DIR / "build" / "esp32.esp32.esp32s3" / "toilet_kat_change.ino.bin"

OTA_STEP_VARIANTS = ["regA", "regB", "regB", "regA", "regB"]


def parse_base_version(ino_text: str) -> str:
    match = re.search(r'const char\* SOFTWARE_VERSION_NUMBER = "([^"]+)"', ino_text)
    if not match:
        raise ValueError(f"Could not parse SOFTWARE_VERSION_NUMBER from {INO_PATH}")
    return match.group(1)


def write_version_header(version: str, build_date: str) -> None:
    VERSION_HEADER.write_text(
        "\n".join(
            [
                "#pragma once",
                f'const char* SOFTWARE_VERSION_NUMBER = "{version}";',
                f'const char* FACTORY_SOFTWARE_DATE = "{build_date}";',
                f'const char* SOFTWARE_BUILD_DATE = "{build_date}";',
                "",
            ]
        ),
        encoding="utf-8",
    )


def remove_version_header() -> None:
    if VERSION_HEADER.exists():
        VERSION_HEADER.unlink()


def md5_file(path: Path) -> str:
    digest = hashlib.md5()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def run_build() -> None:
    cmd = [
        "powershell",
        "-NoProfile",
        "-ExecutionPolicy",
        "Bypass",
        "-File",
        str(BUILD_SCRIPT),
    ]
    result = subprocess.run(cmd, cwd=REPO_ROOT)
    if result.returncode != 0:
        raise RuntimeError(f"build-firmware.ps1 failed with exit code {result.returncode}")
    if not BUILD_OUTPUT.exists():
        raise RuntimeError(f"Build output missing: {BUILD_OUTPUT}")


def extract_sarah_artifacts() -> dict[str, str | None]:
    if not SARAH_MERGED.exists():
        raise FileNotFoundError(f"SARAH merged image not found: {SARAH_MERGED}")

    OUT_DIR.mkdir(parents=True, exist_ok=True)
    cmd = [
        sys.executable,
        str(EXTRACT_SCRIPT),
        str(SARAH_MERGED),
        "--out-dir",
        str(OUT_DIR),
        "--label",
        "sarah",
    ]
    result = subprocess.run(cmd, cwd=REPO_ROOT, capture_output=True, text=True)
    if result.stdout:
        print(result.stdout)
    if result.stderr:
        print(result.stderr, file=sys.stderr)
    if result.returncode != 0:
        raise RuntimeError(f"extract_factory_app.py failed with exit code {result.returncode}")

    detected = None
    for line in (result.stdout or "").splitlines():
        if line.startswith("Detected version string:"):
            detected = line.split(":", 1)[1].strip()
    factory_app = OUT_DIR / "sarah_factory_app.bin"
    factory_copy = OUT_DIR / "factory_sarah.bin"
    shutil.copy2(factory_app, factory_copy)
    return {"factory_version": detected, "factory_path": str(factory_copy)}


def build_variant(label: str, version: str, build_date: str) -> Path:
    print(f"\n=== Building {label} ({version}) ===")
    write_version_header(version, build_date)
    try:
        run_build()
    finally:
        remove_version_header()

    dest = OUT_DIR / f"ota_{label}.bin"
    shutil.copy2(BUILD_OUTPUT, dest)
    print(f"Copied {BUILD_OUTPUT} -> {dest}")
    return dest


def emit_manifest(
    base_version: str,
    build_date: str,
    factory_info: dict[str, str | None],
    variants: dict[str, dict[str, str]],
) -> Path:
    factory_path = Path(factory_info["factory_path"])
    factory_sw = factory_info.get("factory_version") or "unknown"
    manifest = {
        "auto_build": True,
        "base_version": base_version,
        "build_date": build_date,
        "sarah_merged": str(SARAH_MERGED),
        "factory": {
            "path": str(factory_path.relative_to(REPO_ROOT)).replace("\\", "/"),
            "expected_sw": factory_sw,
            "expected_build": build_date,
            "md5": md5_file(factory_path),
            "size": factory_path.stat().st_size,
        },
        "variants": {
            label: {
                "path": str(Path(info["path"]).relative_to(REPO_ROOT)).replace("\\", "/"),
                "expected_sw": info["expected_sw"],
                "expected_build": build_date,
                "md5": info["md5"],
                "size": info["size"],
            }
            for label, info in variants.items()
        },
        "ota": [],
    }

    for index, variant_label in enumerate(OTA_STEP_VARIANTS, start=1):
        variant = variants[variant_label]
        manifest["ota"].append(
            {
                "label": f"ota{index}",
                "variant": variant_label,
                "path": variant["path"],
                "expected_sw": variant["expected_sw"],
                "expected_build": build_date,
            }
        )

    manifest_path = OUT_DIR / "manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    print(f"\nWrote manifest -> {manifest_path}")
    return manifest_path


def main() -> int:
    parser = argparse.ArgumentParser(description="Build OTA regression firmware set")
    parser.add_argument(
        "--skip-firmware-build",
        action="store_true",
        help="Only extract SARAH and write manifest using existing ota_regA/B bins",
    )
    args = parser.parse_args()

    ino_text = INO_PATH.read_text(encoding="utf-8")
    base_version = parse_base_version(ino_text)
    build_date = date.today().isoformat()
    reg_a_version = f"{base_version}-regA"
    reg_b_version = f"{base_version}-regB"

    print(f"Base version: {base_version}")
    print(f"Build date: {build_date}")
    print(f"OTA variants: {reg_a_version}, {reg_b_version}")

    factory_info = extract_sarah_artifacts()
    factory_info["factory_path"] = str(OUT_DIR / "factory_sarah.bin")

    variants: dict[str, dict[str, str]] = {}
    if args.skip_firmware_build:
        for label, version in (("regA", reg_a_version), ("regB", reg_b_version)):
            path = OUT_DIR / f"ota_{label}.bin"
            if not path.exists():
                raise FileNotFoundError(f"Missing {path}; run without --skip-firmware-build first")
            variants[label] = {
                "path": str(path),
                "expected_sw": version,
                "md5": md5_file(path),
                "size": str(path.stat().st_size),
            }
    else:
        for label, version in (("regA", reg_a_version), ("regB", reg_b_version)):
            built = build_variant(label, version, build_date)
            variants[label] = {
                "path": str(built),
                "expected_sw": version,
                "md5": md5_file(built),
                "size": str(built.stat().st_size),
            }

    emit_manifest(base_version, build_date, factory_info, variants)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
