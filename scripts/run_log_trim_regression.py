#!/usr/bin/env python3
"""
Automated SPIFFS error-log trim regression harness.

Exercises write-path, OTA prep, and deep-sleep trim thresholds on hardware
using SPIFFS backup/inject/flash and BLE GET_LOGS verification.
"""

from __future__ import annotations

import argparse
import asyncio
import json
import sys
import time
from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Optional

REPO_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_BUILD_DIR = REPO_ROOT / "test-builds" / "log-trim-regression"
DEFAULT_RESULTS_DIR = REPO_ROOT / "test-results" / "log-trim-regression"

if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from bleak import BleakScanner  # noqa: E402

from get_ota_diag import find_device, report_visible_ble_scan  # noqa: E402
from scripts import log_trim_assertions as assertions  # noqa: E402
from scripts import spiffs_tools  # noqa: E402
from toilet_bluetooth_interface import ToiletSystemInterface  # noqa: E402

STEP_ORDER = [
    "preflight",
    "no_trim",
    "write_trim",
    "ota_trim",
    "sleep_trim",
    "restore",
]

# Seed sizes (bytes) chosen relative to firmware thresholds in toilet_kat_change.ino
SEED_NO_TRIM = 30_000
SEED_WRITE_TRIM = assertions.MAX_LOG_SIZE  # 76800; ota_boot append on reboot pushes over threshold
SEED_OTA_TRIM = 45_000    # above OTA_PREP_LOG_TRIM_TARGET (40960)
SEED_SLEEP_TRIM = 67_000  # above LOG_SLEEP_TRIM_TRIGGER (66560)

INACTIVITY_SLEEP_MS = 120_000
SLEEP_SETTLE_MS = 5_000
DEFAULT_SLEEP_WAIT_S = (INACTIVITY_SLEEP_MS + SLEEP_SETTLE_MS) / 1000.0 + 10.0  # 135s


@dataclass
class StepResult:
    name: str
    passed: bool
    seeded_bytes: Optional[int] = None
    observed_log_bytes: Optional[int] = None
    errors: list[str] = field(default_factory=list)
    logs_excerpt: Optional[str] = None
    started_at: str = ""
    ended_at: str = ""
    duration_s: float = 0.0


@dataclass
class RegressionReport:
    run_id: str
    started_at: str
    ended_at: str
    passed: bool
    dry_run: bool
    port: str
    chip: str
    address: Optional[str]
    backup_path: Optional[str] = None
    steps: list[StepResult] = field(default_factory=list)
    failures: list[str] = field(default_factory=list)


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def logs_excerpt(logs: str, limit: int = 400) -> str:
    if len(logs) <= limit:
        return logs
    return logs[: limit // 2] + "\n...(truncated)...\n" + logs[-limit // 2 :]


class LogTrimRegressionRunner:
    def __init__(self, args: argparse.Namespace):
        self.args = args
        self.port = args.port
        self.chip = args.chip
        self.address = args.address
        self.dry_run = args.dry_run
        self.skip_restore = args.skip_restore
        self.boot_wait_s = args.boot_wait
        self.scan_timeout_s = args.scan_timeout
        self.sleep_wait_s = args.sleep_wait
        self.build_dir = Path(args.build_dir)
        self.results_dir = Path(args.results_dir)
        self.run_id = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        self.steps: list[StepResult] = []
        self.failures: list[str] = []
        self.backup_path: Optional[Path] = None
        self.backup_image: Optional[bytes] = None
        self.selected_steps = set(args.steps or STEP_ORDER)
        if getattr(args, "backup", None):
            self.load_backup(Path(args.backup))

    def load_backup(self, path: Path) -> None:
        if not path.is_file():
            raise RuntimeError(f"SPIFFS backup not found: {path}")
        self.backup_path = path.resolve()
        self.backup_image = self.backup_path.read_bytes()
        if len(self.backup_image) != spiffs_tools.SPIFFS_SIZE:
            raise RuntimeError(
                f"SPIFFS backup size {len(self.backup_image)} != expected {spiffs_tools.SPIFFS_SIZE}"
            )
        self.log(f"Loaded SPIFFS backup: {self.backup_path} ({len(self.backup_image)} bytes)")

    def log(self, message: str) -> None:
        print(message, flush=True)

    def should_run(self, step: str) -> bool:
        return step in self.selected_steps

    async def ensure_address(self) -> str:
        if self.address:
            return self.address
        address = await find_device(self.scan_timeout_s)
        if not address:
            raise RuntimeError(
                "BLE device not found; provide --address or power on the toilet"
            )
        self.address = address
        return address

    async def wait_for_ble(self, timeout_s: Optional[float] = None) -> str:
        timeout = timeout_s if timeout_s is not None else self.boot_wait_s
        self.log(f"Waiting up to {timeout:.0f}s for BLE device...")
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.address:
                devices = await BleakScanner.discover(timeout=3.0)
                for device in devices:
                    if device.address.lower() == self.address.lower():
                        self.log(f"Device visible at {device.address}")
                        return device.address
            else:
                found = await find_device(3.0)
                if found:
                    self.address = found
                    return found
            await asyncio.sleep(2.0)
        await report_visible_ble_scan(
            min(self.scan_timeout_s, 15.0),
            expected_address=self.address,
            reason="device not visible before timeout",
        )
        target = self.address or "ESP32 Toilet"
        raise TimeoutError(f"BLE device not visible within {timeout:.0f}s ({target})")

    async def fetch_logs(self, *, require_trust: bool = False, retries: int = 3) -> str:
        last_error: Optional[Exception] = None
        for attempt in range(1, retries + 1):
            try:
                address = await self.ensure_address()
                interface = ToiletSystemInterface()
                if not await interface.connect(address, require_trust=require_trust):
                    raise RuntimeError("Failed to connect for GET_LOGS")
                try:
                    logs = await interface.get_logs()
                    if logs is None:
                        raise RuntimeError("GET_LOGS returned None")
                    return logs
                finally:
                    await interface.disconnect()
            except Exception as exc:
                last_error = exc
                if attempt < retries:
                    self.log(f"GET_LOGS attempt {attempt}/{retries} failed: {exc}; retrying...")
                    await asyncio.sleep(3.0)
        raise RuntimeError(f"GET_LOGS failed after {retries} attempts: {last_error}")

    async def ensure_dev_mode(self, enabled: bool) -> None:
        address = await self.ensure_address()
        interface = ToiletSystemInterface()
        if not await interface.connect(address, require_trust=False):
            raise RuntimeError("Failed to connect for DEV mode setup")
        try:
            status = await interface.get_dev_mode_status()
            target = 1 if enabled else 0
            if status != target:
                self.log(f"Setting DEV mode to {target}...")
                if not await interface.set_dev_mode(target):
                    raise RuntimeError(f"Failed to set DEV mode to {target}")
        finally:
            await interface.disconnect()

    async def run_ota_prep_trim(self) -> str:
        address = await self.ensure_address()
        interface = ToiletSystemInterface()
        if not await interface.connect(address, require_trust=False):
            raise RuntimeError("Failed to connect for OTA prep")
        try:
            if not await interface.set_dev_mode(1):
                raise RuntimeError("Failed to enable DEV mode for OTA prep")
            if not await interface.prepare_ota_for_update():
                raise RuntimeError("prepare_ota_for_update failed")
            await asyncio.sleep(1.0)
            logs = await interface.get_logs()
            if logs is None:
                raise RuntimeError("GET_LOGS returned None after OTA prep")
            return logs
        finally:
            await interface.disconnect()

    def seed_log(self, size_bytes: int, *, label: str) -> None:
        if self.backup_image is None:
            raise RuntimeError("SPIFFS backup missing; run preflight first")
        work_dir = self.build_dir / self.run_id / label
        self.log(f"Seeding errors.txt to {size_bytes} bytes via SPIFFS flash...")
        if self.dry_run:
            self.log(f"[dry-run] Would seed {size_bytes} bytes to SPIFFS")
            return
        spiffs_tools.seed_log_size(
            self.port,
            self.chip,
            self.backup_image,
            size_bytes,
            work_dir,
        )

    def reboot_device(self) -> None:
        if self.dry_run:
            self.log("[dry-run] Would esptool run (reboot)")
            return
        spiffs_tools.esptool_reset(self.port, self.chip)

    async def run_step(self, name: str, seeded: Optional[int], action) -> StepResult:
        started = utc_now()
        t0 = time.monotonic()
        result = StepResult(name=name, passed=False, seeded_bytes=seeded, started_at=started)
        self.log(f"\n=== Step: {name} ===")
        try:
            logs = await action()
            if logs is not None:
                result.observed_log_bytes = assertions.log_byte_length(logs)
                result.logs_excerpt = logs_excerpt(logs)
            result.passed = True
        except Exception as exc:
            msg = str(exc)
            result.errors.append(msg)
            self.failures.append(f"{name}: {msg}")
            self.log(f"FAIL {name}: {msg}")
        result.ended_at = utc_now()
        result.duration_s = round(time.monotonic() - t0, 2)
        status = "PASS" if result.passed else "FAIL"
        self.log(f"[{status}] {name} ({result.duration_s}s)")
        if result.observed_log_bytes is not None:
            self.log(f"  observed log bytes: {result.observed_log_bytes}")
        self.steps.append(result)
        return result

    async def run_preflight(self) -> None:
        if not self.should_run("preflight"):
            return
        self.build_dir.mkdir(parents=True, exist_ok=True)
        backup_path = self.build_dir / f"spiffs_backup_{self.run_id}.bin"
        self.log("Preflight: backing up SPIFFS partition...")
        if self.dry_run:
            self.log(f"[dry-run] Would backup SPIFFS to {backup_path}")
            self.backup_image = b"\xFF" * spiffs_tools.SPIFFS_SIZE
            self.backup_path = backup_path
            self.log("[dry-run] Skipping BLE preflight GET_LOGS")
            return

        self.backup_image = spiffs_tools.backup_spiffs(self.port, self.chip, backup_path)
        self.backup_path = backup_path
        self.log(f"SPIFFS backup saved: {backup_path} ({len(self.backup_image)} bytes)")

        await self.wait_for_ble()
        logs = await self.fetch_logs()
        self.log(f"Preflight GET_LOGS OK ({assertions.log_byte_length(logs)} bytes)")

    async def run_no_trim(self) -> None:
        if not self.should_run("no_trim"):
            return

        async def action() -> str:
            self.seed_log(SEED_NO_TRIM, label="no_trim")
            if not self.dry_run:
                await asyncio.sleep(3.0)
                await self.wait_for_ble()
            logs = await self.fetch_logs() if not self.dry_run else ""
            if not self.dry_run:
                assertions.assert_no_trim_under_threshold(logs, SEED_NO_TRIM, label="no_trim")
            return logs

        await self.run_step("no_trim", SEED_NO_TRIM, action)

    async def run_write_trim(self) -> None:
        if not self.should_run("write_trim"):
            return

        async def action() -> str:
            self.seed_log(SEED_WRITE_TRIM, label="write_trim")
            if not self.dry_run:
                await asyncio.sleep(3.0)
                await self.wait_for_ble()
            logs = await self.fetch_logs() if not self.dry_run else ""
            if not self.dry_run:
                assertions.assert_log_size_at_most(
                    logs, assertions.LOG_TRIM_TARGET, label="write_trim"
                )
                assertions.assert_newest_markers_kept(logs, label="write_trim")
                assertions.assert_oldest_markers_dropped(logs, label="write_trim")
                assertions.assert_line_aligned(logs, label="write_trim")
            return logs

        await self.run_step("write_trim", SEED_WRITE_TRIM, action)

    async def run_ota_trim(self) -> None:
        if not self.should_run("ota_trim"):
            return

        async def action() -> str:
            self.seed_log(SEED_OTA_TRIM, label="ota_trim")
            if self.dry_run:
                return ""
            await asyncio.sleep(3.0)
            await self.wait_for_ble()
            logs = await self.run_ota_prep_trim()
            assertions.assert_log_size_at_most(
                logs, assertions.OTA_PREP_LOG_TRIM_TARGET, label="ota_trim"
            )
            assertions.assert_newest_markers_kept(logs, label="ota_trim")
            assertions.assert_line_aligned(logs, label="ota_trim")
            return logs

        await self.run_step("ota_trim", SEED_OTA_TRIM, action)

    async def run_sleep_trim(self) -> None:
        if not self.should_run("sleep_trim"):
            return

        async def action() -> str:
            self.seed_log(SEED_SLEEP_TRIM, label="sleep_trim")
            if self.dry_run:
                return ""
            await asyncio.sleep(3.0)
            await self.wait_for_ble()
            self.log("Disabling DEV mode and disconnecting for inactivity sleep window...")
            await self.ensure_dev_mode(False)
            self.log(
                f"Waiting {self.sleep_wait_s:.0f}s for inactivity deep sleep "
                f"(do not press panel buttons or connect BLE)..."
            )
            await asyncio.sleep(self.sleep_wait_s)
            self.log(
                "Waiting for device after deep sleep (BLE poll; press panel button to wake if needed)..."
            )
            try:
                self.reboot_device()
                await asyncio.sleep(5.0)
            except RuntimeError as exc:
                self.log(f"esptool reboot unavailable ({exc}); relying on wake + BLE...")
            await self.wait_for_ble(timeout_s=max(self.boot_wait_s, 180.0))
            logs = await self.fetch_logs(retries=5)
            assertions.assert_log_size_at_most(
                logs, assertions.LOG_TRIM_TARGET, label="sleep_trim"
            )
            assertions.assert_newest_markers_kept(logs, label="sleep_trim")
            assertions.assert_oldest_markers_dropped(logs, label="sleep_trim")
            assertions.assert_line_aligned(logs, label="sleep_trim")
            return logs

        await self.run_step("sleep_trim", SEED_SLEEP_TRIM, action)

    async def run_restore(self) -> None:
        if not self.should_run("restore") or self.skip_restore:
            if self.skip_restore:
                self.log("Skipping SPIFFS restore (--skip-restore)")
            return

        async def action() -> None:
            if self.backup_path is None or self.backup_image is None:
                raise RuntimeError("No SPIFFS backup to restore")
            self.log(f"Restoring SPIFFS from {self.backup_path}...")
            if self.dry_run:
                self.log("[dry-run] Would restore SPIFFS backup")
                return ""
            tmp = self.build_dir / f"restore_{self.run_id}.bin"
            tmp.write_bytes(self.backup_image)
            spiffs_tools.flash_spiffs(self.port, self.chip, tmp)
            spiffs_tools.esptool_reset(self.port, self.chip)
            return ""

        await self.run_step("restore", None, action)

    def write_report(self, report: RegressionReport) -> None:
        self.results_dir.mkdir(parents=True, exist_ok=True)
        json_path = self.results_dir / f"log_trim_regression_{report.run_id}.json"
        txt_path = self.results_dir / f"log_trim_regression_{report.run_id}.txt"

        json_path.write_text(json.dumps(asdict(report), indent=2), encoding="utf-8")

        lines = [
            "Log Trim Regression Report",
            f"Run ID: {report.run_id}",
            f"Started: {report.started_at}",
            f"Ended: {report.ended_at}",
            f"Result: {'PASS' if report.passed else 'FAIL'}",
            f"Dry run: {report.dry_run}",
            f"Port: {report.port}",
            f"Chip: {report.chip}",
            f"BLE address: {report.address}",
            f"SPIFFS backup: {report.backup_path or '-'}",
            "",
        ]
        if report.failures:
            lines.append("Failures:")
            for failure in report.failures:
                lines.append(f"  - {failure}")
            lines.append("")

        for step in report.steps:
            status = "PASS" if step.passed else "FAIL"
            lines.append(f"[{status}] {step.name} ({step.duration_s}s)")
            if step.seeded_bytes is not None:
                lines.append(f"  seeded bytes: {step.seeded_bytes}")
            if step.observed_log_bytes is not None:
                lines.append(f"  observed log bytes: {step.observed_log_bytes}")
            for err in step.errors:
                lines.append(f"  error: {err}")
            lines.append("")

        txt_path.write_text("\n".join(lines), encoding="utf-8")
        self.log(f"Report written: {json_path}")
        self.log(f"Report written: {txt_path}")

    async def run(self) -> RegressionReport:
        started_at = utc_now()
        try:
            if self.should_run("preflight"):
                await self.run_preflight()
            elif self.backup_image is None and not self.dry_run:
                raise RuntimeError(
                    "preflight step required to capture SPIFFS backup, or pass --backup"
                )

            if self.should_run("no_trim"):
                await self.run_no_trim()
            if self.should_run("write_trim"):
                await self.run_write_trim()
            if self.should_run("ota_trim"):
                await self.run_ota_trim()
            if self.should_run("sleep_trim"):
                await self.run_sleep_trim()
        finally:
            if self.should_run("restore") and not self.skip_restore:
                try:
                    await self.run_restore()
                except Exception as exc:
                    self.failures.append(f"restore: {exc}")
                    self.log(f"FAIL restore: {exc}")

        passed = not self.failures and all(step.passed for step in self.steps if step.name != "restore")
        report = RegressionReport(
            run_id=self.run_id,
            started_at=started_at,
            ended_at=utc_now(),
            passed=passed,
            dry_run=self.dry_run,
            port=self.port,
            chip=self.chip,
            address=self.address,
            backup_path=str(self.backup_path) if self.backup_path else None,
            steps=self.steps,
            failures=self.failures,
        )
        self.write_report(report)
        return report


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Automated SPIFFS error-log trim regression harness"
    )
    parser.add_argument("--port", "-p", default="COM6", help="USB serial port (default: COM6)")
    parser.add_argument("--chip", "-c", default="esp32s3", help="ESP32 chip type (default: esp32s3)")
    parser.add_argument("--address", help="BLE MAC address (skip scan if provided)")
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Exercise orchestration without USB flash/BLE operations",
    )
    parser.add_argument(
        "--skip-restore",
        action="store_true",
        help="Do not restore SPIFFS backup at end (debug only)",
    )
    parser.add_argument(
        "--backup",
        help="Existing SPIFFS backup .bin for inject/restore (skip preflight backup read)",
    )
    parser.add_argument(
        "--steps",
        nargs="+",
        choices=STEP_ORDER,
        help=f"Run only these steps (default: all). Choices: {', '.join(STEP_ORDER)}",
    )
    parser.add_argument(
        "--build-dir",
        default=str(DEFAULT_BUILD_DIR),
        help="Directory for SPIFFS backups and staging artifacts",
    )
    parser.add_argument(
        "--results-dir",
        default=str(DEFAULT_RESULTS_DIR),
        help="Directory for JSON/text reports",
    )
    parser.add_argument("--scan-timeout", type=float, default=25.0, help="BLE scan timeout (seconds)")
    parser.add_argument("--boot-wait", type=float, default=90.0, help="Max wait for reboot BLE (seconds)")
    parser.add_argument(
        "--sleep-wait",
        type=float,
        default=DEFAULT_SLEEP_WAIT_S,
        help=f"Idle wait before sleep-trim check (default: {DEFAULT_SLEEP_WAIT_S:.0f}s)",
    )
    return parser


async def async_main() -> int:
    parser = build_arg_parser()
    args = parser.parse_args()
    runner = LogTrimRegressionRunner(args)
    report = await runner.run()

    print("\n" + "=" * 72)
    print(f"RESULT: {'PASS' if report.passed else 'FAIL'}")
    if report.failures:
        print("Failures:")
        for failure in report.failures:
            print(f"  - {failure}")
    print("=" * 72)
    return 0 if report.passed else 1


def main() -> int:
    try:
        import bleak  # noqa: F401
    except ImportError:
        print("Install dependencies: pip install bleak esptool", file=sys.stderr)
        return 1
    return asyncio.run(async_main())


if __name__ == "__main__":
    raise SystemExit(main())
