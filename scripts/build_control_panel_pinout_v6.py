#!/usr/bin/env python3
"""Build firmware with CONTROL_PANEL_PINOUT_OVERRIDE=6 (force v6 MCP pinout)."""

from __future__ import annotations

import json
import os
import shutil
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SKETCH_DIR = ROOT / "toilet_kat_change"
BUILD_DIR = SKETCH_DIR / "build" / "esp32.esp32.esp32s3"
TOOLCHAIN = json.loads((ROOT / "scripts" / "firmware-toolchain.json").read_text())
PINOUT_DEFINE = "-DCONTROL_PANEL_PINOUT_OVERRIDE=6"


def find_arduino_cli() -> Path:
    env = os.environ.get("ARDUINO_CLI")
    if env:
        return Path(env)
    for candidate in (
        Path(r"C:\Program Files\Arduino CLI\arduino-cli.exe"),
        Path("/usr/local/bin/arduino-cli"),
        Path("/opt/homebrew/bin/arduino-cli"),
    ):
        if candidate.is_file():
            return candidate
    found = shutil.which("arduino-cli")
    if found:
        return Path(found)
    raise SystemExit("arduino-cli not found (set ARDUINO_CLI or install Arduino CLI)")


def main() -> int:
    cli = find_arduino_cli()
    sketch_build = SKETCH_DIR / "build"
    if sketch_build.exists():
        shutil.rmtree(sketch_build)

    cmd = [
        str(cli),
        "compile",
        "--clean",
        "--build-path",
        str(BUILD_DIR),
        "--fqbn",
        TOOLCHAIN["fqbn"],
        str(SKETCH_DIR),
        "--build-property",
        f"build.extra_flags={PINOUT_DEFINE}",
        "--build-property",
        "build.partitions_file=partitions.csv",
        "--build-property",
        "build.sdkconfig.defaults=sdkconfig.defaults",
    ]
    print("Building with", PINOUT_DEFINE)
    subprocess.run(cmd, check=True)

    out_bin = BUILD_DIR / "toilet_kat_change.ino.bin"
    if not out_bin.is_file():
        raise SystemExit(f"Build finished but binary missing: {out_bin}")

    print(f"Firmware: {out_bin}")
    print(f"Size: {out_bin.stat().st_size} bytes")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except subprocess.CalledProcessError as exc:
        print(f"arduino-cli failed (exit {exc.returncode})", file=sys.stderr)
        raise SystemExit(exc.returncode)
