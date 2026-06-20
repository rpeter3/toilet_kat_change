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
DEFAULT_AUTO_MANIFEST = REPO_ROOT / "test-builds" / "ota-regression" / "manifest.json"
AUTO_BUILD_SCRIPT = REPO_ROOT / "scripts" / "build_ota_regression_firmware.py"
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from bleak import BleakScanner  # noqa: E402

from get_ota_diag import find_device, report_visible_ble_scan  # noqa: E402
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

REGRESSION_STEP_ORDER = [
    "flash_factory",
    "ota1",
    "ota2",
    "rollback_previous",
    "ota3",
    "ota4",
    "factory_reset",
    "ota5",
]

PRIOR_PARTITION_SEQUENCE = {
    "flash_factory": [],
    "ota1": ["factory"],
    "ota2": ["factory", "ota_0"],
    "rollback_previous": ["factory", "ota_0", "ota_1"],
    "ota3": ["factory", "ota_0", "ota_1", "ota_0"],
    "ota4": ["factory", "ota_0", "ota_1", "ota_0", "ota_1"],
    "factory_reset": ["factory", "ota_0", "ota_1", "ota_0", "ota_1", "ota_0"],
    "ota5": ["factory", "ota_0", "ota_1", "ota_0", "ota_1", "ota_0", "factory"],
}

FULL_STEP_TRANSPORT = {
    "flash_factory": "serial",
    "ota1": "ble",
    "ota2": "ble",
    "rollback_previous": "ble",
    "ota3": "ble",
    "ota4": "ble",
    "factory_reset": "ble",
    "ota5": "ble",
}

FAST_STEP_TRANSPORT = {
    "flash_factory": "serial",
    "ota1": "ble",
    "ota2": "ble",
    "rollback_previous": "serial_sim",
    "ota3": "serial_sim",
    "ota4": "serial_sim",
    "factory_reset": "serial_sim",
    "ota5": "serial_sim",
}


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
    transport: str = "ble"
    pending_clear_required: bool = False
    expected_sw: Optional[str] = None
    observed_sw: Optional[str] = None
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
    fast_mode: bool
    auto_build: bool
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


def version_dict_from_hw_component(row: dict[str, str]) -> dict[str, str]:
    """Map GET_HW_COMPONENT payload to CHECK_VERSION-style SW/Build keys."""
    version: dict[str, str] = {}
    current = row.get("current_version")
    if current:
        version["SW"] = current
    install_date = row.get("install_date")
    if install_date:
        version["Build"] = install_date
    return version


def merge_version_info(*sources: dict[str, str]) -> dict[str, str]:
    merged: dict[str, str] = {}
    for source in sources:
        for key, value in source.items():
            if value and not merged.get(key):
                merged[key] = value
    return merged


async def read_firmware_version(interface: ToiletSystemInterface) -> tuple[Optional[str], dict[str, str]]:
    """Read firmware SW/Build via OTA CHECK_VERSION, with HW matrix fallback."""
    updater = OTAUpdater()
    updater.client = interface.client
    updater.connected = True
    version_raw = await updater.check_version()
    version = parse_version_string(version_raw or "")

    if not version.get("SW"):
        hw_row = await interface.get_hw_component("SOFTWARE_VERSION_NUMBER")
        if hw_row:
            version = merge_version_info(version, version_dict_from_hw_component(hw_row))
            if version.get("SW"):
                version_raw = (
                    f"HW:Uninitialized|SW:{version['SW']}|Build:{version.get('Build', '')}"
                )

    return version_raw, version


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


def resolve_repo_path(path_text: str) -> Path:
    path = Path(path_text)
    if not path.is_absolute():
        path = REPO_ROOT / path
    return path.resolve()


def run_auto_build_script(*, skip_firmware_build: bool = False) -> None:
    cmd = [sys.executable, str(AUTO_BUILD_SCRIPT)]
    if skip_firmware_build:
        cmd.append("--skip-firmware-build")
    print("Running OTA regression firmware build...")
    result = subprocess.run(cmd, cwd=REPO_ROOT)
    if result.returncode != 0:
        raise RuntimeError(f"build_ota_regression_firmware.py failed with exit code {result.returncode}")
    if not DEFAULT_AUTO_MANIFEST.exists():
        raise RuntimeError(f"Auto-build did not produce manifest: {DEFAULT_AUTO_MANIFEST}")


def load_manifest(path: Path) -> tuple[FirmwareSpec, list[FirmwareSpec], dict[str, Any]]:
    data = json.loads(path.read_text(encoding="utf-8"))
    factory_data = data["factory"]
    factory = FirmwareSpec(
        label="factory",
        path=resolve_repo_path(factory_data["path"]),
        expected_sw=factory_data.get("expected_sw"),
        expected_build=factory_data.get("expected_build"),
    )
    ota_specs: list[FirmwareSpec] = []
    for index, entry in enumerate(data.get("ota", []), start=1):
        ota_specs.append(
            FirmwareSpec(
                label=entry.get("label", f"ota{index}"),
                path=resolve_repo_path(entry["path"]),
                expected_sw=entry.get("expected_sw"),
                expected_build=entry.get("expected_build"),
            )
        )
    if len(ota_specs) != 5:
        raise ValueError(f"Manifest must contain exactly 5 OTA binaries, found {len(ota_specs)}")
    return factory, ota_specs, data


def resolve_firmware_specs(args: argparse.Namespace) -> tuple[FirmwareSpec, list[FirmwareSpec], dict[str, Any]]:
    manifest_meta: dict[str, Any] = {}
    if args.manifest:
        factory, ota_specs, manifest_meta = load_manifest(Path(args.manifest))
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
    return factory, ota_specs, manifest_meta


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


def get_current_ota_seq(port: str, chip: str) -> int:
    ota_data = read_flash(port, chip, OTADATA_OFFSET, OTADATA_SIZE)
    ota = parse_otadata(ota_data)
    active = ota.get("active")
    if not active:
        return 0
    seq = active.get("seq", 0)
    if seq in (0, 0xFFFFFFFF):
        return 0
    return int(seq)


def next_seq_for_slot(current_seq: int, target_partition: str) -> int:
    """Return the next valid otadata seq for the target slot (odd=ota_0, even=ota_1)."""
    want_even = target_partition == "ota_1"
    candidate = max(current_seq + 1, 1)
    while (candidate % 2 == 0) != want_even:
        candidate += 1
    return candidate


def run_flash_ota_slot(
    port: str,
    boot_slot: str,
    app_bin: Optional[Path] = None,
    ota_seq: int = 0,
    *,
    otadata_only: bool = False,
) -> None:
    script = REPO_ROOT / "scripts" / "flash-ota-slot.ps1"
    cmd = [
        "powershell",
        "-NoProfile",
        "-ExecutionPolicy",
        "Bypass",
        "-File",
        str(script),
        "-Port",
        port,
        "-BootSlot",
        boot_slot,
    ]
    if app_bin is not None:
        cmd.extend(["-AppBin", str(app_bin)])
    if ota_seq > 0:
        cmd.extend(["-OtaSeq", str(ota_seq)])
    if otadata_only:
        cmd.append("-OtadataOnly")
    result = subprocess.run(cmd, cwd=REPO_ROOT, capture_output=True, text=True)
    if result.stdout:
        print(result.stdout)
    if result.stderr:
        print(result.stderr, file=sys.stderr)
    if result.returncode != 0:
        raise RuntimeError(f"flash-ota-slot.ps1 failed with exit code {result.returncode}")


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


def run_flash_sarah_factory(port: str) -> None:
    script = REPO_ROOT / "scripts" / "flash-sarah-factory.ps1"
    cmd = [
        "powershell",
        "-NoProfile",
        "-ExecutionPolicy",
        "Bypass",
        "-File",
        str(script),
        "-Port",
        port,
    ]
    result = subprocess.run(cmd, cwd=REPO_ROOT, capture_output=True, text=True)
    if result.stdout:
        print(result.stdout)
    if result.stderr:
        print(result.stderr, file=sys.stderr)
    if result.returncode != 0:
        raise RuntimeError(f"flash-sarah-factory.ps1 failed with exit code {result.returncode}")


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
        self.fast_mode = args.fast
        self.auto_build = args.auto_build
        self.ensure_dev_mode = args.ensure_dev_mode
        self.trust_timeout_s = args.trust_timeout
        self.boot_wait_s = args.boot_wait
        self.scan_timeout_s = args.scan_timeout
        self.pending_clear_timeout_s = args.pending_clear_timeout
        self.results_dir = Path(args.results_dir)
        self.factory_spec, self.ota_specs, self.manifest_meta = resolve_firmware_specs(args)
        self.partition_sequence: list[str] = []
        self.steps: list[StepResult] = []
        self.failures: list[str] = []
        self.run_id = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        self.step_transport = FAST_STEP_TRANSPORT if self.fast_mode else FULL_STEP_TRANSPORT
        self.start_from = getattr(args, "start_from", "flash_factory") or "flash_factory"
        if self.start_from not in REGRESSION_STEP_ORDER:
            raise ValueError(f"Invalid --start-from {self.start_from!r}")
        self.partition_sequence = list(PRIOR_PARTITION_SEQUENCE[self.start_from])

    def should_run_step(self, step_name: str) -> bool:
        return REGRESSION_STEP_ORDER.index(step_name) >= REGRESSION_STEP_ORDER.index(self.start_from)

    def step_transport_for(self, step_name: str) -> str:
        return self.step_transport[step_name]

    def log(self, message: str) -> None:
        print(message, flush=True)

    async def ensure_address(self) -> str:
        if self.address:
            return self.address
        address = await find_device(self.scan_timeout_s)
        if not address:
            raise RuntimeError(
                "BLE device not found; provide --address or power on the toilet "
                "(see BLE scan output above)"
            )
        self.address = address
        return address

    async def report_target_not_advertising(self, reason: str) -> None:
        await report_visible_ble_scan(
            min(self.scan_timeout_s, 15.0),
            expected_address=self.address,
            reason=reason,
        )

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
        await self.report_target_not_advertising("device not visible before timeout")
        target = self.address or "ESP32 Toilet"
        raise TimeoutError(f"BLE device not visible within {timeout:.0f}s ({target})")

    async def collect_diagnostics(
        self,
        *,
        require_trust: bool = False,
        read_version: bool = True,
    ) -> dict[str, Any]:
        address = await self.ensure_address()
        interface = ToiletSystemInterface()
        if not await interface.connect(address, require_trust=require_trust):
            await self.report_target_not_advertising("BLE connect failed during diagnostics")
            raise RuntimeError("Failed to connect for diagnostics")

        snapshot: dict[str, Any] = {}
        try:
            ota_diag_raw = await interface._send_command_and_read_response("GET_OTA_DIAG")
            boot_info_raw = await interface._send_command_and_read_response("GET_BOOT_INFO")
            logs = await interface.get_logs()
            version_raw = None
            version: dict[str, str] = {}
            if read_version:
                version_raw, version = await read_firmware_version(interface)

            snapshot = {
                "ota_diag_raw": ota_diag_raw,
                "ota_diag": parse_ota_diag(ota_diag_raw),
                "boot_info_raw": boot_info_raw,
                "boot_info": parse_boot_info(boot_info_raw),
                "logs": logs or "",
                "version_raw": version_raw,
                "version": version if read_version else {},
            }
        finally:
            await interface.disconnect()
        return snapshot

    async def ensure_dev_mode_enabled(self) -> dict[str, Any]:
        """Connect without trust, ensure DEV mode is on for automated BLE privileged commands."""
        address = await self.ensure_address()
        interface = ToiletSystemInterface()
        last_error: Optional[Exception] = None
        for attempt in range(1, 6):
            try:
                if attempt > 1:
                    self.log(f"DEV mode connect retry {attempt}/5...")
                    await asyncio.sleep(5.0)
                await self.wait_for_ble(timeout_s=min(self.boot_wait_s, 60.0))
                if not await interface.connect(address, require_trust=False):
                    raise RuntimeError("Failed to connect to enable DEV mode")
                break
            except asyncio.CancelledError as exc:
                last_error = exc
                if interface.connected:
                    await interface.disconnect()
            except Exception as exc:
                last_error = exc
                if interface.connected:
                    await interface.disconnect()
        else:
            await self.report_target_not_advertising("BLE connect failed during DEV mode setup")
            raise RuntimeError(f"Failed to connect for DEV mode setup: {last_error}") from last_error

        result: dict[str, Any] = {"address": address}
        try:
            dev_mode = await interface.get_dev_mode_status()
            result["initial_dev_mode"] = dev_mode
            if dev_mode == 1:
                self.log("DEV mode already enabled (BLE trust bypass active).")
                return result
            if not self.ensure_dev_mode:
                self.log("WARN: DEV mode is off; rollback steps may require manual trust.")
                return result
            self.log("DEV mode is off; sending SET_DEV_MODE:1 for automated testing...")
            if not await interface.set_dev_mode(1):
                raise RuntimeError("Failed to enable DEV mode via SET_DEV_MODE:1")
            dev_mode = await interface.get_dev_mode_status()
            result["dev_mode_after"] = dev_mode
            if dev_mode != 1:
                raise RuntimeError("DEV mode still off after SET_DEV_MODE:1")
            self.log("DEV mode enabled for this test run.")
        finally:
            await interface.disconnect()
        return result

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
        if spec.expected_sw and spec.expected_sw != "unknown" and sw != spec.expected_sw:
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
        transport: str = "ble",
        require_pending_clear: bool = False,
        expect_rollback: bool = False,
    ) -> StepResult:
        started = utc_now()
        t0 = time.monotonic()
        errors: list[str] = []
        observed_partition: Optional[str] = None
        expected_sw = version_spec.expected_sw if version_spec else None
        observed_sw: Optional[str] = None
        snapshot: dict[str, Any] = {}

        try:
            if self.dry_run:
                observed_partition = expected_partition
                observed_sw = expected_sw
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
                observed_sw = snapshot.get("version", {}).get("SW")

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
            transport=transport,
            pending_clear_required=require_pending_clear,
            expected_sw=expected_sw,
            observed_sw=observed_sw,
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

    def perform_serial_slot_update(self, spec: FirmwareSpec, target_partition: str) -> None:
        if self.dry_run:
            self.log(
                f"[dry-run] Would serial-flash {spec.label} to {target_partition} from {spec.path}"
            )
            return

        current_seq = get_current_ota_seq(self.port, self.chip)
        ota_seq = next_seq_for_slot(current_seq, target_partition)
        self.log(
            f"Serial sim: flash {spec.label} -> {target_partition} "
            f"(seq {current_seq} -> {ota_seq})"
        )
        run_flash_ota_slot(self.port, target_partition, spec.path, ota_seq)

    def perform_serial_rollback_previous(self, target_partition: str) -> None:
        if self.dry_run:
            self.log(f"[dry-run] Would serial-sim rollback to {target_partition}")
            return

        current_seq = get_current_ota_seq(self.port, self.chip)
        ota_seq = next_seq_for_slot(current_seq, target_partition)
        self.log(
            f"Serial sim: rollback boot -> {target_partition} "
            f"(seq {current_seq} -> {ota_seq}, image unchanged)"
        )
        run_flash_ota_slot(self.port, target_partition, ota_seq=ota_seq, otadata_only=True)

    def perform_serial_factory_reset(self) -> None:
        if self.dry_run:
            self.log("[dry-run] Would serial-sim factory reset (otadata erase)")
            return

        self.log("Serial sim: factory reset via otadata erase")
        run_flash_ota_slot(self.port, "factory")

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
            "fast_mode": self.fast_mode,
            "auto_build": self.auto_build,
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

        if self.manifest_meta:
            preflight["manifest"] = self.manifest_meta

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
        if not self.dry_run:
            preflight["dev_mode"] = await self.ensure_dev_mode_enabled()
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
        if self.auto_build:
            self.log("Mode: AUTO-BUILD (SARAH factory + regA/regB from current source)")
        if self.fast_mode:
            self.log("Mode: FAST (OTA1/OTA2 BLE; other hops serial-simulated)")
        else:
            self.log("Mode: FULL (all OTA/rollback steps over BLE)")
        if self.start_from != "flash_factory":
            self.log(
                f"Resuming from {self.start_from} "
                f"(assuming prior partitions: {' -> '.join(self.partition_sequence) or 'none'})"
            )
        self.log("=" * 72)

        if (
            self.start_from != "flash_factory"
            and not self.dry_run
            and not self.args.skip_preflight
            and self.args.boot_settle > 0
        ):
            self.log(f"Waiting {self.args.boot_settle:.0f}s for full boot before BLE preflight...")
            await asyncio.sleep(self.args.boot_settle)

        try:
            if self.args.skip_preflight:
                self.log("=== Preflight (skipped) ===")
                preflight = {"skipped": True, "start_from": self.start_from}
                if self.address:
                    preflight["ble_address"] = self.address
            else:
                preflight = await self.run_preflight()
        except (Exception, asyncio.CancelledError) as exc:
            if isinstance(exc, asyncio.CancelledError):
                exc = RuntimeError("BLE connection cancelled during preflight")
            self.failures.append(f"preflight: {exc}")
            report = RegressionReport(
                run_id=self.run_id,
                started_at=started_at,
                ended_at=utc_now(),
                passed=False,
                dry_run=self.dry_run,
                fast_mode=self.fast_mode,
                auto_build=self.auto_build,
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
                fast_mode=self.fast_mode,
                auto_build=self.auto_build,
                port=self.port,
                address=self.address,
                metadata={"preflight": preflight, "preflight_only": True},
            )
            self.write_report(report)
            return report

        try:
            await self._run_steps(preflight)
        except Exception as exc:
            self.failures.append(str(exc))
            self.log(f"Harness aborted: {exc}")

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
            fast_mode=self.fast_mode,
            auto_build=self.auto_build,
            port=self.port,
            address=self.address,
            partition_sequence=self.partition_sequence,
            steps=self.steps,
            failures=self.failures,
            metadata={
                "preflight": preflight,
                "manifest": self.manifest_meta if self.auto_build else None,
                "start_from": self.start_from,
            },
        )
        self.write_report(report)
        return report

    async def _run_steps(self, preflight: dict[str, Any]) -> None:
        # 1. Flash factory
        if self.should_run_step("flash_factory"):
            self.log("\n--- Step: Flash Factory ---")
            use_sarah_factory = self.auto_build or bool(self.manifest_meta.get("auto_build"))
            if not self.dry_run:
                if use_sarah_factory:
                    run_flash_sarah_factory(self.port)
                else:
                    run_flash_factory(self.port, self.factory_spec.path, boot_slot="factory")
                await asyncio.sleep(12.0)
            await self.assert_post_step_state(
                "flash_factory",
                "factory",
                self.factory_spec,
                transport=self.step_transport_for("flash_factory"),
            )

        # 2. OTA1 -> ota_0
        if self.should_run_step("ota1"):
            self.log("\n--- Step: OTA1 ---")
            await self.perform_ota(self.ota_specs[0])
            await self.assert_post_step_state(
                "ota1",
                "ota_0",
                self.ota_specs[0],
                transport=self.step_transport_for("ota1"),
                require_pending_clear=True,
            )

        # 3. OTA2 -> ota_1
        if self.should_run_step("ota2"):
            self.log("\n--- Step: OTA2 ---")
            if not self.dry_run:
                self.log("Pause 20s before OTA2 to let device settle...")
                await asyncio.sleep(20.0)
            await self.perform_ota(self.ota_specs[1])
            await self.assert_post_step_state(
                "ota2",
                "ota_1",
                self.ota_specs[1],
                transport=self.step_transport_for("ota2"),
                require_pending_clear=True,
            )

        # 4. Rollback -> ota_0 (OTA1)
        if self.should_run_step("rollback_previous"):
            self.log("\n--- Step: Rollback Previous ---")
            rollback_transport = self.step_transport_for("rollback_previous")
            if rollback_transport == "ble":
                if not self.fast_mode:
                    self.log("Rollback over BLE (DEV mode auto-grants trust when enabled).")
                await self.perform_rollback_previous()
            else:
                self.perform_serial_rollback_previous("ota_0")
                if not self.dry_run:
                    await asyncio.sleep(12.0)
            await self.assert_post_step_state(
                "rollback_previous",
                "ota_0",
                self.ota_specs[0],
                transport=rollback_transport,
                expect_rollback=(rollback_transport == "ble"),
            )

        # 5. OTA3 -> ota_1
        if self.should_run_step("ota3"):
            self.log("\n--- Step: OTA3 ---")
            ota3_transport = self.step_transport_for("ota3")
            if ota3_transport == "ble":
                await self.perform_ota(self.ota_specs[2])
            else:
                self.perform_serial_slot_update(self.ota_specs[2], "ota_1")
                if not self.dry_run:
                    await asyncio.sleep(12.0)
            await self.assert_post_step_state(
                "ota3",
                "ota_1",
                self.ota_specs[2],
                transport=ota3_transport,
                require_pending_clear=(ota3_transport == "ble"),
            )

        # 6. OTA4 -> ota_0
        if self.should_run_step("ota4"):
            self.log("\n--- Step: OTA4 ---")
            ota4_transport = self.step_transport_for("ota4")
            if ota4_transport == "ble":
                await self.perform_ota(self.ota_specs[3])
            else:
                self.perform_serial_slot_update(self.ota_specs[3], "ota_0")
                if not self.dry_run:
                    await asyncio.sleep(12.0)
            await self.assert_post_step_state(
                "ota4",
                "ota_0",
                self.ota_specs[3],
                transport=ota4_transport,
                require_pending_clear=(ota4_transport == "ble"),
            )

        # 7. Factory reset
        if self.should_run_step("factory_reset"):
            self.log("\n--- Step: Factory Reset ---")
            factory_transport = self.step_transport_for("factory_reset")
            if factory_transport == "ble":
                if not self.fast_mode:
                    self.log("Factory rollback over BLE (DEV mode auto-grants trust when enabled).")
                await self.perform_rollback_factory()
            else:
                self.perform_serial_factory_reset()
                if not self.dry_run:
                    await asyncio.sleep(12.0)
            await self.assert_post_step_state(
                "factory_reset",
                "factory",
                self.factory_spec,
                transport=factory_transport,
            )

        # 8. OTA5 -> ota_0
        if self.should_run_step("ota5"):
            self.log("\n--- Step: OTA5 ---")
            ota5_transport = self.step_transport_for("ota5")
            if ota5_transport == "ble":
                await self.perform_ota(self.ota_specs[4])
            else:
                self.perform_serial_slot_update(self.ota_specs[4], "ota_0")
                if not self.dry_run:
                    await asyncio.sleep(12.0)
            await self.assert_post_step_state(
                "ota5",
                "ota_0",
                self.ota_specs[4],
                transport=ota5_transport,
                require_pending_clear=(ota5_transport == "ble"),
            )

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
            f"Fast mode: {report.fast_mode}",
            f"Auto-build: {report.auto_build}",
            f"Port: {report.port}",
            f"BLE address: {report.address}",
            "",
            f"Partition sequence: {' -> '.join(report.partition_sequence)}",
            f"Expected sequence: {' -> '.join(EXPECTED_PARTITION_SEQUENCE)}",
            "",
        ]

        if report.steps:
            lines.append("Version evolution:")
            lines.append(f"{'Step':<20} {'Partition':<10} {'Expected SW':<24} {'Observed SW':<24} Pass")
            lines.append("-" * 90)
            for step in report.steps:
                sw_pass = (
                    step.expected_sw is None
                    or (step.observed_sw == step.expected_sw and step.passed)
                )
                sw_status = "PASS" if sw_pass else "FAIL"
                lines.append(
                    f"{step.name:<20} {step.expected_partition:<10} "
                    f"{step.expected_sw or '-':<24} {step.observed_sw or '-':<24} {sw_status}"
                )
            lines.append("")

        if report.metadata.get("manifest"):
            manifest = report.metadata["manifest"]
            lines.append("Auto-build manifest:")
            lines.append(f"  base_version: {manifest.get('base_version', '-')}")
            lines.append(f"  build_date: {manifest.get('build_date', '-')}")
            if manifest.get("factory"):
                lines.append(f"  factory SW: {manifest['factory'].get('expected_sw', '-')}")
            lines.append("")

        if report.failures:
            lines.append("Failures:")
            for failure in report.failures:
                lines.append(f"  - {failure}")
            lines.append("")

        for step in report.steps:
            status = "PASS" if step.passed else "FAIL"
            lines.append(f"[{status}] {step.name} ({step.duration_s}s)")
            lines.append(f"  transport: {step.transport}")
            if step.pending_clear_required:
                lines.append("  pending_clear_required: yes")
            lines.append(f"  expected partition: {step.expected_partition}")
            lines.append(f"  observed partition: {step.observed_partition}")
            if step.expected_sw or step.observed_sw:
                lines.append(f"  expected SW: {step.expected_sw}")
                lines.append(f"  observed SW: {step.observed_sw}")
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
    parser.add_argument(
        "--auto-build",
        action="store_true",
        help="Build regA/regB from current source, extract SARAH factory, and use generated manifest",
    )
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
        "--boot-settle",
        type=float,
        default=0.0,
        help="Seconds to wait for full boot before BLE preflight (use ~90 when resuming after power cycle)",
    )
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
    parser.add_argument(
        "--fast",
        action="store_true",
        help="Fast mode: OTA1/OTA2 over BLE; OTA3-5, rollback, and factory reset serial-simulated",
    )
    parser.add_argument(
        "--skip-preflight",
        action="store_true",
        help="Skip USB/BLE preflight (use when resuming and device is already ready)",
    )
    parser.add_argument(
        "--start-from",
        choices=REGRESSION_STEP_ORDER,
        default="flash_factory",
        help="Resume harness from this step (assumes prior steps already completed)",
    )
    parser.add_argument(
        "--ensure-dev-mode",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Enable DEV mode over BLE before test if off (default: on; grants trust bypass)",
    )
    return parser


async def async_main() -> int:
    parser = build_arg_parser()
    args = parser.parse_args()

    if args.auto_build:
        skip_build = args.dry_run and DEFAULT_AUTO_MANIFEST.exists()
        if skip_build:
            print(f"Auto-build: reusing existing manifest ({DEFAULT_AUTO_MANIFEST})")
        else:
            run_auto_build_script(skip_firmware_build=False)
        args.manifest = str(DEFAULT_AUTO_MANIFEST)
    elif not args.manifest and not args.factory_bin:
        parser.error("Provide --auto-build, --manifest, or --factory-bin")
    if not args.manifest and len(args.ota_bin) != 5:
        parser.error("Provide exactly five --ota-bin values or use --manifest / --auto-build")

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
