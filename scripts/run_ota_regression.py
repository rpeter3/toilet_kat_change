#!/usr/bin/env python3
"""
Automated OTA partition regression harness.

Sequence:
  Flash Factory -> OTA1 -> OTA2 -> Rollback -> OTA3 -> OTA4 -> Factory Reset -> OTA5

Wraps existing flash, BLE OTA, diagnostic, partition-read, and rollback tooling.
"""

from __future__ import annotations

import argparse
import asyncio
import hashlib
import json
import subprocess
import sys
import time
from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Optional

REPO_ROOT = Path(__file__).resolve().parent.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from bleak import BleakScanner  # noqa: E402

from get_ota_diag import find_device  # noqa: E402
from ota_update import OTAUpdater  # noqa: E402
from read_partitions import (  # noqa: E402
    OTADATA_OFFSET,
    OTADATA_SIZE,
    parse_otadata,
    read_flash,
)
from toilet_bluetooth_interface import DEVICE_NAME, ToiletSystemInterface  # noqa: E402

MAX_OTA_PARTITION_SIZE = 0x480000
OTA_GOOD_BOOT_STREAK_REQUIRED = 2
EXPECTED_PARTITION_SEQUENCE = [
    "factory",
    "ota_0",
    "ota_1",
    "ota_0",
    "ota_1",
    "ota_0",
    "factory",
    "ota_0",
]


@dataclass
class FirmwareSpec:
    label: str
    path: Path
    expected_sw: Optional[str] = None
    expected_build: Optional[str] = None
    md5: str = ""
    size: int = 0


@dataclass
class StepResult:
    name: str
    passed: bool
    expected_partition: str
    observed_partition: Optional[str] = None
    version: Optional[str] = None
    ota_diag: Optional[dict[str, str]] = None
    boot_info: Optional[str] = None
    logs_excerpt: Optional[str] = None
    errors: list[str] = field(default_factory=list)
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
    address: Optional[str]
    partition_sequence: list[str] = field(default_factory=list)
    steps: list[StepResult] = field(default_factory=list)
    failures: list[str] = field(default_factory=list)
    metadata: dict[str, Any] = field(default_factory=dict)


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def md5_file(path: Path) -> str:
    digest = hashlib.md5()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def parse_version_string(version_payload: str) -> dict[str, str]:
    parsed: dict[str, str] = {}
    for token in version_payload.split("|"):
        token = token.strip()
        if ":" not in token:
            continue
        key, value = token.split(":", 1)
        parsed[key.strip()] = value.strip()
    return parsed


def parse_ota_diag(response: Optional[str]) -> dict[str, str]:
    if not response or response == "OTA_DIAG:EMPTY":
        return {}
    payload = response
    if payload.startswith("OTA_DIAG:"):
        payload = payload.split(":", 1)[1]
        if payload.startswith("V1|"):
            payload = payload[3:]
    result: dict[str, str] = {}
    for token in payload.split("|"):
        if "=" not in token:
            continue
        key, value = token.split("=", 1)
        result[key.strip()] = value.strip()
    return result


def parse_boot_info(response: Optional[str]) -> dict[str, str]:
    if not response or not response.startswith("BOOT_INFO:"):
        return {}
    payload = response.split(":", 1)[1]
    result: dict[str, str] = {}
    for token in payload.split(","):
        if "=" not in token:
            continue
        key, value = token.split("=", 1)
        result[key.strip()] = value.strip()
    return result


def load_manifest(path: Path) -> tuple[FirmwareSpec, list[FirmwareSpec]]:
    data = json.loads(path.read_text(encoding="utf-8"))
    factory_data = data["factory"]
    factory = FirmwareSpec(
        label="factory",
        path=Path(factory_data["path"]),
        expected_sw=factory_data.get("expected_sw"),
        expected_build=factory_data.get("expected_build"),
    )
    ota_specs: list[FirmwareSpec] = []
    for index, entry in enumerate(data.get("ota", []), start=1):
        ota_specs.append(
            FirmwareSpec(
                label=f"ota{index}",
                path=Path(entry["path"]),
                expected_sw=entry.get("expected_sw"),
                expected_build=entry.get("expected_build"),
            )
        )
    if len(ota_specs) != 5:
        raise ValueError(f"Manifest must contain exactly 5 OTA binaries, found {len(ota_specs)}")
    return factory, ota_specs


def resolve_firmware_specs(args: argparse.Namespace) -> tuple[FirmwareSpec, list[FirmwareSpec]]:
    if args.manifest:
        factory, ota_specs = load_manifest(Path(args.manifest))
    else:
        factory = FirmwareSpec(
            label="factory",
            path=Path(args.factory_bin),
            expected_sw=args.factory_expected_sw,
            expected_build=args.factory_expected_build,
        )
        if len(args.ota_bin) != 5:
            raise ValueError(f"Expected 5 --ota-bin values, got {len(args.ota_bin)}")
        ota_specs = []
        for index, path_text in enumerate(args.ota_bin, start=1):
            expected_sw = None
            expected_build = None
            if args.ota_expected_sw and len(args.ota_expected_sw) >= index:
                expected_sw = args.ota_expected_sw[index - 1]
            if args.ota_expected_build and len(args.ota_expected_build) >= index:
                expected_build = args.ota_expected_build[index - 1]
            ota_specs.append(
                FirmwareSpec(
                    label=f"ota{index}",
                    path=Path(path_text),
                    expected_sw=expected_sw,
                    expected_build=expected_build,
                )
            )
    for spec in [factory, *ota_specs]:
        spec.path = spec.path.resolve()
        if not spec.path.exists():
            raise FileNotFoundError(f"Firmware binary not found: {spec.path}")
        spec.size = spec.path.stat().st_size
        spec.md5 = md5_file(spec.path)
        if spec.size > MAX_OTA_PARTITION_SIZE:
            raise ValueError(
                f"{spec.label} binary ({spec.size} bytes) exceeds slot size "
                f"({MAX_OTA_PARTITION_SIZE} bytes): {spec.path}"
            )
    return factory, ota_specs


def get_active_boot_partition(port: str, chip: str) -> str:
    ota_data = read_flash(port, chip, OTADATA_OFFSET, OTADATA_SIZE)
    ota = parse_otadata(ota_data)
    active = ota.get("active")
    if not active:
        return "factory"
    label = active.get("label", "")
    if label in ("", "(unspecified)"):
        seq = active.get("seq", 0)
        if seq in (0, 0xFFFFFFFF):
            return "factory"
        return "ota_unspecified"
    return label


def run_flash_factory(port: str, factory_bin: Path, boot_slot: str = "factory") -> None:
    script = REPO_ROOT / "scripts" / "flash-factory-app.ps1"
    cmd = [
        "powershell",
        "-NoProfile",
        "-ExecutionPolicy",
        "Bypass",
        "-File",
        str(script),
        "-Port",
        port,
        "-AppBin",
        str(factory_bin),
        "-BootSlot",
        boot_slot,
    ]
    result = subprocess.run(cmd, cwd=REPO_ROOT, capture_output=True, text=True)
    if result.stdout:
        print(result.stdout)
    if result.stderr:
        print(result.stderr, file=sys.stderr)
    if result.returncode != 0:
        raise RuntimeError(f"flash-factory-app.ps1 failed with exit code {result.returncode}")


def esptool_reset(port: str, chip: str) -> None:
    cmd = [
        sys.executable,
        "-m",
        "esptool",
        "--chip",
        chip,
        "--port",
        port,
        "run",
    ]
    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode != 0:
        raise RuntimeError(
            f"esptool run failed (exit {result.returncode}): {result.stderr or result.stdout}"
        )


class OtaRegressionRunner:
    def __init__(self, args: argparse.Namespace):
        self.args = args
        self.port = args.port
        self.chip = args.chip
        self.address = args.address
        self.dry_run = args.dry_run
        self.trust_timeout_s = args.trust_timeout
        self.boot_wait_s = args.boot_wait
        self.scan_timeout_s = args.scan_timeout
        self.pending_clear_timeout_s = args.pending_clear_timeout
        self.results_dir = Path(args.results_dir)
        self.factory_spec, self.ota_specs = resolve_firmware_specs(args)
        self.partition_sequence: list[str] = []
        self.steps: list[StepResult] = []
        self.failures: list[str] = []
        self.run_id = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")

    def log(self, message: str) -> None:
        print(message, flush=True)

    async def ensure_address(self) -> str:
        if self.address:
            return self.address
        address = await find_device(self.scan_timeout_s)
        if not address:
            raise RuntimeError("BLE device not found; provide --address or power on the toilet")
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
        raise TimeoutError(f"BLE device not visible within {timeout:.0f}s")

    async def collect_diagnostics(
        self,
        *,
        require_trust: bool = False,
        read_version: bool = True,
    ) -> dict[str, Any]:
        address = await self.ensure_address()
        interface = ToiletSystemInterface()
        if not await interface.connect(address, require_trust=require_trust):
            raise RuntimeError("Failed to connect for diagnostics")

        snapshot: dict[str, Any] = {}
        try:
            ota_diag_raw = await interface._send_command_and_read_response("GET_OTA_DIAG")
            boot_info_raw = await interface._send_command_and_read_response("GET_BOOT_INFO")
            logs = await interface.get_logs()
            version_raw = None
            if read_version:
                updater = OTAUpdater()
                updater.client = interface.client
                updater.connected = True
                version_raw = await updater.check_version()

            snapshot = {
                "ota_diag_raw": ota_diag_raw,
                "ota_diag": parse_ota_diag(ota_diag_raw),
                "boot_info_raw": boot_info_raw,
                "boot_info": parse_boot_info(boot_info_raw),
                "logs": logs or "",
                "version_raw": version_raw,
                "version": parse_version_string(version_raw or ""),
            }
        finally:
            await interface.disconnect()
        return snapshot

    async def wait_for_pending_clear(self) -> dict[str, Any]:
        deadline = time.monotonic() + self.pending_clear_timeout_s
        last_snapshot: dict[str, Any] = {}
        while time.monotonic() < deadline:
            await self.wait_for_ble()
            snapshot = await self.collect_diagnostics()
            last_snapshot = snapshot
            pending = snapshot["ota_diag"].get("pending", "1")
            streak = int(snapshot["ota_diag"].get("streak", "0") or 0)
            if pending == "0":
                return snapshot
            if streak < OTA_GOOD_BOOT_STREAK_REQUIRED:
                self.log(
                    f"OTA pending verify active (pending={pending}, streak={streak}); "
                    "triggering extra reboot via esptool..."
                )
                if not self.dry_run:
                    esptool_reset(self.port, self.chip)
                    await asyncio.sleep(8.0)
            else:
                await asyncio.sleep(5.0)
        raise TimeoutError(
            "OTA pending verification did not clear within "
            f"{self.pending_clear_timeout_s:.0f}s; last diag={last_snapshot.get('ota_diag')}"
        )

    def verify_version(self, spec: Optional[FirmwareSpec], version: dict[str, str], errors: list[str]) -> None:
        if not spec:
            return
        sw = version.get("SW")
        build = version.get("Build")
        if spec.expected_sw and sw != spec.expected_sw:
            errors.append(f"Expected SW {spec.expected_sw}, observed {sw!r}")
        if spec.expected_build and build != spec.expected_build:
            errors.append(f"Expected Build {spec.expected_build}, observed {build!r}")

    def verify_boot_info(self, boot_info: dict[str, str], errors: list[str]) -> None:
        reset = boot_info.get("reset", "")
        if reset in {"panic", "wdt", "task_wdt", "brownout"}:
            errors.append(f"Abnormal reset reason observed: {reset}")

    async def assert_post_step_state(
        self,
        step_name: str,
        expected_partition: str,
        version_spec: Optional[FirmwareSpec],
        *,
        require_pending_clear: bool = False,
        expect_rollback: bool = False,
    ) -> StepResult:
        started = utc_now()
        t0 = time.monotonic()
        errors: list[str] = []
        observed_partition: Optional[str] = None
        snapshot: dict[str, Any] = {}

        try:
            if self.dry_run:
                observed_partition = expected_partition
                snapshot = {"version": {}, "ota_diag": {}, "boot_info": {}, "logs": ""}
            else:
                await self.wait_for_ble()
                if require_pending_clear:
                    snapshot = await self.wait_for_pending_clear()
                else:
                    snapshot = await self.collect_diagnostics()

                observed_partition = get_active_boot_partition(self.port, self.chip)
                if observed_partition == "ota_unspecified":
                    self.log(
                        "USB otadata label unspecified; keeping BLE/version checks as secondary evidence."
                    )
                    observed_partition = expected_partition

                if observed_partition != expected_partition:
                    errors.append(
                        f"Expected active partition {expected_partition}, observed {observed_partition}"
                    )

                self.verify_version(version_spec, snapshot.get("version", {}), errors)
                self.verify_boot_info(snapshot.get("boot_info", {}), errors)

                if expect_rollback:
                    trigger = snapshot.get("ota_diag", {}).get("last_trigger", "")
                    if trigger not in {"manual", "auto", "validation"}:
                        errors.append(
                            f"Expected rollback diagnostics (last_trigger), observed {trigger!r}"
                        )

                if require_pending_clear:
                    pending = snapshot.get("ota_diag", {}).get("pending", "1")
                    if pending != "0":
                        errors.append(f"Expected pending=0 after OTA verify, observed pending={pending}")

            if not errors:
                self.partition_sequence.append(observed_partition or expected_partition)

        except Exception as exc:
            errors.append(str(exc))

        ended = utc_now()
        passed = not errors
        result = StepResult(
            name=step_name,
            passed=passed,
            expected_partition=expected_partition,
            observed_partition=observed_partition,
            version=(snapshot.get("version_raw") if snapshot else None),
            ota_diag=snapshot.get("ota_diag"),
            boot_info=snapshot.get("boot_info_raw"),
            logs_excerpt=(snapshot.get("logs", "")[:2000] if snapshot else None),
            errors=errors,
            started_at=started,
            ended_at=ended,
            duration_s=round(time.monotonic() - t0, 2),
        )
        self.steps.append(result)
        if not passed:
            self.failures.extend([f"{step_name}: {err}" for err in errors])
        return result

    async def perform_ota(self, spec: FirmwareSpec) -> None:
        if self.dry_run:
            self.log(f"[dry-run] Would OTA {spec.label} from {spec.path}")
            return

        address = await self.ensure_address()
        await self.wait_for_ble()
        updater = OTAUpdater()
        if not await updater.connect(address):
            raise RuntimeError(f"Failed to connect for OTA ({spec.label})")
        try:
            success = await updater.update_firmware(str(spec.path))
            if not success:
                raise RuntimeError(f"OTA update failed for {spec.label}")
        finally:
            if updater.connected:
                await updater.disconnect()
        await asyncio.sleep(10.0)

    async def perform_rollback_previous(self) -> None:
        if self.dry_run:
            self.log("[dry-run] Would send OTA_ROLLBACK_PREVIOUS")
            return

        address = await self.ensure_address()
        await self.wait_for_ble()
        interface = ToiletSystemInterface()
        interface.trust_timeout_s = self.trust_timeout_s
        if not await interface.connect(address, require_trust=True):
            raise RuntimeError("Failed to connect for rollback (trust required)")
        try:
            if not await interface.ota_rollback_previous():
                raise RuntimeError("OTA_ROLLBACK_PREVIOUS rejected")
        finally:
            if interface.connected:
                await interface.disconnect()
        await asyncio.sleep(10.0)

    async def perform_rollback_factory(self) -> None:
        if self.dry_run:
            self.log("[dry-run] Would send OTA_ROLLBACK_FACTORY")
            return

        address = await self.ensure_address()
        await self.wait_for_ble()
        interface = ToiletSystemInterface()
        interface.trust_timeout_s = self.trust_timeout_s
        if not await interface.connect(address, require_trust=True):
            raise RuntimeError("Failed to connect for factory rollback (trust required)")
        try:
            if not await interface.ota_rollback_factory():
                raise RuntimeError("OTA_ROLLBACK_FACTORY rejected")
        finally:
            if interface.connected:
                await interface.disconnect()
        await asyncio.sleep(10.0)

    async def run_preflight(self) -> dict[str, Any]:
        self.log("=== Preflight ===")
        preflight: dict[str, Any] = {
            "factory": {
                "path": str(self.factory_spec.path),
                "size": self.factory_spec.size,
                "md5": self.factory_spec.md5,
                "sha256": sha256_file(self.factory_spec.path),
            },
            "ota": [],
        }
        for spec in self.ota_specs:
            preflight["ota"].append(
                {
                    "label": spec.label,
                    "path": str(spec.path),
                    "size": spec.size,
                    "md5": spec.md5,
                    "sha256": sha256_file(spec.path),
                    "expected_sw": spec.expected_sw,
                    "expected_build": spec.expected_build,
                }
            )

        if self.dry_run:
            self.log("Dry-run mode: skipping serial/BLE connectivity checks.")
            return preflight

        try:
            partition = get_active_boot_partition(self.port, self.chip)
            preflight["initial_usb_partition"] = partition
            self.log(f"Initial USB boot partition: {partition}")
        except Exception as exc:
            raise RuntimeError(f"Serial port preflight failed for {self.port}: {exc}") from exc

        address = await self.ensure_address()
        preflight["ble_address"] = address
        baseline = await self.collect_diagnostics()
        preflight["baseline"] = {
            "version": baseline.get("version_raw"),
            "ota_diag": baseline.get("ota_diag_raw"),
            "boot_info": baseline.get("boot_info_raw"),
        }
        self.log("Preflight checks passed.")
        return preflight

    async def run(self) -> RegressionReport:
        started_at = utc_now()
        self.log("=" * 72)
        self.log("OTA Partition Regression Harness")
        self.log("=" * 72)

        try:
            preflight = await self.run_preflight()
        except Exception as exc:
            self.failures.append(f"preflight: {exc}")
            report = RegressionReport(
                run_id=self.run_id,
                started_at=started_at,
                ended_at=utc_now(),
                passed=False,
                dry_run=self.dry_run,
                port=self.port,
                address=self.address,
                failures=self.failures,
                metadata={"preflight_error": str(exc)},
            )
            self.write_report(report)
            return report

        if self.args.preflight_only:
            report = RegressionReport(
                run_id=self.run_id,
                started_at=started_at,
                ended_at=utc_now(),
                passed=True,
                dry_run=self.dry_run,
                port=self.port,
                address=self.address,
                metadata={"preflight": preflight, "preflight_only": True},
            )
            self.write_report(report)
            return report

        # 1. Flash factory
        self.log("\n--- Step: Flash Factory ---")
        if not self.dry_run:
            run_flash_factory(self.port, self.factory_spec.path, boot_slot="factory")
            await asyncio.sleep(12.0)
        await self.assert_post_step_state("flash_factory", "factory", self.factory_spec)

        # 2. OTA1 -> ota_0
        self.log("\n--- Step: OTA1 ---")
        await self.perform_ota(self.ota_specs[0])
        await self.assert_post_step_state(
            "ota1", "ota_0", self.ota_specs[0], require_pending_clear=True
        )

        # 3. OTA2 -> ota_1
        self.log("\n--- Step: OTA2 ---")
        await self.perform_ota(self.ota_specs[1])
        await self.assert_post_step_state(
            "ota2", "ota_1", self.ota_specs[1], require_pending_clear=True
        )

        # 4. Rollback -> ota_0 (OTA1)
        self.log("\n--- Step: Rollback Previous ---")
        self.log("Trust handshake required: press a control panel button when prompted.")
        await self.perform_rollback_previous()
        await self.assert_post_step_state(
            "rollback_previous",
            "ota_0",
            self.ota_specs[0],
            expect_rollback=True,
        )

        # 5. OTA3 -> ota_1
        self.log("\n--- Step: OTA3 ---")
        await self.perform_ota(self.ota_specs[2])
        await self.assert_post_step_state(
            "ota3", "ota_1", self.ota_specs[2], require_pending_clear=True
        )

        # 6. OTA4 -> ota_0
        self.log("\n--- Step: OTA4 ---")
        await self.perform_ota(self.ota_specs[3])
        await self.assert_post_step_state(
            "ota4", "ota_0", self.ota_specs[3], require_pending_clear=True
        )

        # 7. Factory reset
        self.log("\n--- Step: Factory Reset ---")
        self.log("Trust handshake required: press a control panel button when prompted.")
        await self.perform_rollback_factory()
        await self.assert_post_step_state(
            "factory_reset", "factory", self.factory_spec
        )

        # 8. OTA5 -> ota_0
        self.log("\n--- Step: OTA5 ---")
        await self.perform_ota(self.ota_specs[4])
        await self.assert_post_step_state(
            "ota5", "ota_0", self.ota_specs[4], require_pending_clear=True
        )

        sequence_ok = self.partition_sequence == EXPECTED_PARTITION_SEQUENCE
        if not sequence_ok:
            self.failures.append(
                "Partition sequence mismatch: "
                f"expected {EXPECTED_PARTITION_SEQUENCE}, observed {self.partition_sequence}"
            )

        passed = not self.failures
        report = RegressionReport(
            run_id=self.run_id,
            started_at=started_at,
            ended_at=utc_now(),
            passed=passed,
            dry_run=self.dry_run,
            port=self.port,
            address=self.address,
            partition_sequence=self.partition_sequence,
            steps=self.steps,
            failures=self.failures,
            metadata={"preflight": preflight},
        )
        self.write_report(report)
        return report

    def write_report(self, report: RegressionReport) -> None:
        self.results_dir.mkdir(parents=True, exist_ok=True)
        json_path = self.results_dir / f"ota_regression_{report.run_id}.json"
        txt_path = self.results_dir / f"ota_regression_{report.run_id}.txt"

        payload = asdict(report)
        json_path.write_text(json.dumps(payload, indent=2), encoding="utf-8")

        lines = [
            "OTA Partition Regression Report",
            f"Run ID: {report.run_id}",
            f"Started: {report.started_at}",
            f"Ended: {report.ended_at}",
            f"Result: {'PASS' if report.passed else 'FAIL'}",
            f"Dry run: {report.dry_run}",
            f"Port: {report.port}",
            f"BLE address: {report.address}",
            "",
            f"Partition sequence: {' -> '.join(report.partition_sequence)}",
            f"Expected sequence: {' -> '.join(EXPECTED_PARTITION_SEQUENCE)}",
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
            lines.append(f"  expected partition: {step.expected_partition}")
            lines.append(f"  observed partition: {step.observed_partition}")
            if step.version:
                lines.append(f"  version: {step.version}")
            if step.errors:
                for err in step.errors:
                    lines.append(f"  error: {err}")
            lines.append("")

        txt_path.write_text("\n".join(lines), encoding="utf-8")
        self.log(f"Report written: {json_path}")
        self.log(f"Report written: {txt_path}")


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Automated OTA partition regression harness for ESP32 Toilet firmware"
    )
    parser.add_argument("--port", "-p", default="COM6", help="USB serial port (default: COM6)")
    parser.add_argument("--chip", "-c", default="esp32s3", help="ESP32 chip type (default: esp32s3)")
    parser.add_argument("--address", help="BLE MAC address (skip scan if provided)")
    parser.add_argument("--manifest", help="JSON manifest with factory + 5 OTA binary specs")
    parser.add_argument("--factory-bin", help="Factory firmware .bin (required without --manifest)")
    parser.add_argument("--ota-bin", action="append", default=[], help="OTA firmware .bin (repeat 5x)")
    parser.add_argument("--factory-expected-sw", help="Expected factory SW version string")
    parser.add_argument("--factory-expected-build", help="Expected factory build date (YYYY-MM-DD)")
    parser.add_argument(
        "--ota-expected-sw",
        action="append",
        default=[],
        help="Expected SW version for each OTA binary (repeat up to 5x)",
    )
    parser.add_argument(
        "--ota-expected-build",
        action="append",
        default=[],
        help="Expected build date for each OTA binary (repeat up to 5x)",
    )
    parser.add_argument(
        "--results-dir",
        default=str(REPO_ROOT / "test-results" / "ota-regression"),
        help="Directory for JSON/text reports",
    )
    parser.add_argument("--scan-timeout", type=float, default=25.0, help="BLE scan timeout (seconds)")
    parser.add_argument("--boot-wait", type=float, default=90.0, help="Max wait for reboot (seconds)")
    parser.add_argument(
        "--pending-clear-timeout",
        type=float,
        default=180.0,
        help="Max wait for OTA pending verification to clear (seconds)",
    )
    parser.add_argument(
        "--trust-timeout",
        type=float,
        default=120.0,
        help="Trust handshake timeout for rollback steps (seconds)",
    )
    parser.add_argument(
        "--preflight-only",
        action="store_true",
        help="Run connectivity/binary checks only, then exit",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Exercise orchestration without USB flash/BLE operations",
    )
    return parser


async def async_main() -> int:
    parser = build_arg_parser()
    args = parser.parse_args()

    if not args.manifest and not args.factory_bin:
        parser.error("Provide --manifest or --factory-bin")
    if not args.manifest and len(args.ota_bin) != 5:
        parser.error("Provide exactly five --ota-bin values or use --manifest")

    runner = OtaRegressionRunner(args)
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
        print("ERROR: bleak is required. Install with: pip install bleak")
        return 1
    return asyncio.run(async_main())


if __name__ == "__main__":
    raise SystemExit(main())
