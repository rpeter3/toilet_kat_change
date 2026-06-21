"""Live diagnostics for OTA regression (USB serial + otadata + optional BLE)."""

from __future__ import annotations

import asyncio
import re
import threading
import time
from collections import deque
from typing import Callable, Optional

try:
    import serial
except ImportError:
    serial = None  # type: ignore[assignment]

from read_partitions import OTADATA_OFFSET, OTADATA_SIZE, parse_otadata, read_flash

LogFn = Callable[[str], None]

_INTERESTING_SERIAL = re.compile(
    r"(OTA|UPDATE|ENABLE|BOOT|panic|WDT|brownout|rollback|partition|MD5|ERROR|WARN|reboot|WiFi|BLE)",
    re.IGNORECASE,
)


def read_usb_boot_summary(port: str, chip: str) -> str:
    """Read otadata over USB; return a one-line boot summary."""
    try:
        ota_data = read_flash(port, chip, OTADATA_OFFSET, OTADATA_SIZE)
        ota = parse_otadata(ota_data)
        active = ota.get("active")
        if not active:
            return "boot=factory (no valid otadata seq)"
        label = active.get("label", "")
        seq = active.get("seq", 0)
        if label in ("", "(unspecified)"):
            if seq in (0, 0xFFFFFFFF):
                return "boot=factory"
            return f"boot=ota_unspecified seq={seq}"
        return f"boot={label} seq={seq}"
    except Exception as exc:
        return f"usb_unavailable ({exc})"


async def read_ble_advertising_status(address: str, timeout_s: float = 2.0) -> str:
    """Scan only — never opens a GATT connection (safe during active OTA sessions)."""
    from bleak import BleakScanner

    try:
        devices = await BleakScanner.discover(timeout=timeout_s)
        for device in devices:
            if device.address.lower() == address.lower():
                name = device.name or "?"
                return f"ble=advertising name={name}"
        return "ble=not_advertising"
    except Exception as exc:
        return f"ble_scan_error ({exc})"


async def read_ble_summary(address: str, timeout_s: float = 8.0) -> str:
    """Best-effort BLE snapshot; opens a short diagnostic connection."""
    from bleak import BleakScanner

    from toilet_bluetooth_interface import ToiletSystemInterface

    try:
        adv = await read_ble_advertising_status(address, timeout_s=min(timeout_s, 3.0))
        if adv == "ble=not_advertising":
            return adv

        interface = ToiletSystemInterface()
        if not await interface.connect(address, require_trust=False):
            return "ble=visible,connect_failed"

        try:
            ota_diag = await interface._send_command_and_read_response("GET_OTA_DIAG")
            boot_info = await interface._send_command_and_read_response("GET_BOOT_INFO")
            parts = [f"ble=connected ota_diag={_short(ota_diag, 80)}"]
            if boot_info:
                parts.append(f"boot_info={_short(boot_info, 60)}")
            return " ".join(parts)
        finally:
            await interface.disconnect()
    except Exception as exc:
        return f"ble_error ({exc})"


def _short(text: Optional[str], limit: int) -> str:
    if not text:
        return "-"
    text = text.replace("\n", " ").strip()
    if len(text) <= limit:
        return text
        return text[: limit - 3] + "..."


_ACTIVE_BLE_SESSION_STATES = frozenset(
    {
        "connecting",
        "connected",
        "ota_enable",
        "ota_enable_wait",
        "ota_prepare",
        "ota_start",
        "ota_transfer",
        "ota_finalize",
    }
)


class BleConnectionTracker:
    """Logs BLE connection / OTA phase transitions immediately."""

    def __init__(self, log: LogFn):
        self.log = log
        self.state = "idle"
        self.detail = ""

    @property
    def session_active(self) -> bool:
        return self.state in _ACTIVE_BLE_SESSION_STATES

    def set(self, state: str, detail: str = "") -> None:
        if state == self.state and detail == self.detail:
            return
        self.state = state
        self.detail = detail
        msg = f"[ble] {state}"
        if detail:
            msg += f" | {detail}"
        self.log(msg)

    def fail(self, operation: str, reason: str) -> None:
        self.state = "failed"
        self.detail = reason
        self.log(f"[ble] FAILED {operation} | {reason}")

    def disconnected(self, reason: str = "") -> None:
        self.state = "disconnected"
        self.detail = reason
        msg = "[ble] disconnected"
        if reason:
            msg += f" | {reason}"
        self.log(msg)


class SerialLineMonitor:
    """Background USB-serial reader; keeps recent lines for heartbeat logs."""

    def __init__(self, port: str, baud: int = 115200, *, log: Optional[LogFn] = None):
        self.port = port
        self.baud = baud
        self._log = log
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._ser: Optional["serial.Serial"] = None
        self._recent: deque[str] = deque(maxlen=8)
        self._line_buf = ""

    @property
    def running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def start(self) -> bool:
        if self.running:
            return True
        if serial is None:
            if self._log:
                self._log("[serial] pyserial not installed; USB log capture disabled")
            return False
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name=f"serial-{self.port}", daemon=True)
        self._thread.start()
        return True

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=3.0)
            self._thread = None
        if self._ser and self._ser.is_open:
            try:
                self._ser.close()
            except Exception:
                pass
        self._ser = None

    def recent(self, *, max_chars: int = 200) -> str:
        if not self._recent:
            return "(no serial output yet)"
        text = " | ".join(self._recent)
        if len(text) > max_chars:
            return "..." + text[-max_chars:]
        return text

    def _emit_line(self, line: str) -> None:
        line = line.strip()
        if not line:
            return
        self._recent.append(line)
        if self._log and _INTERESTING_SERIAL.search(line):
            self._log(f"[serial] {line}")

    def _run(self) -> None:
        try:
            self._ser = serial.Serial(self.port, self.baud, timeout=0.3)
        except Exception as exc:
            if self._log:
                self._log(f"[serial] Could not open {self.port}: {exc}")
            return

        if self._log:
            self._log(f"[serial] Monitoring {self.port} @ {self.baud}")

        while not self._stop.is_set():
            try:
                chunk = self._ser.read(4096)
            except Exception as exc:
                if self._log:
                    self._log(f"[serial] Read error: {exc}")
                break
            if not chunk:
                continue
            self._line_buf += chunk.decode("utf-8", errors="replace")
            while "\n" in self._line_buf:
                line, self._line_buf = self._line_buf.split("\n", 1)
                self._emit_line(line)

        if self._ser and self._ser.is_open:
            try:
                self._ser.close()
            except Exception:
                pass


class DiagnosticsHeartbeat:
    """Periodic status lines combining serial tail, USB otadata, and BLE."""

    def __init__(
        self,
        *,
        port: str,
        chip: str,
        log: LogFn,
        interval_s: float = 10.0,
        address: Optional[str] = None,
        serial_monitor: Optional[SerialLineMonitor] = None,
        include_usb: bool = True,
        include_ble: bool = True,
        ble_tracker: Optional[BleConnectionTracker] = None,
    ):
        self.port = port
        self.chip = chip
        self.log = log
        self.interval_s = interval_s
        self.address = address
        self.serial_monitor = serial_monitor
        self.include_usb = include_usb
        self.include_ble = include_ble
        self.ble_tracker = ble_tracker
        self._phase = "idle"
        self._extra = ""
        self._t0 = time.monotonic()
        self._last_emit = self._t0
        self._tick = 0

    def set_phase(self, phase: str, extra: str = "") -> None:
        self._phase = phase
        self._extra = extra
        self._t0 = time.monotonic()
        self._last_emit = self._t0
        self._tick = 0
        parts = [f"[diag] phase={phase}"]
        if extra:
            parts.append(extra)
        self.log(" ".join(parts))

    def maybe_emit(self, force: bool = False) -> None:
        now = time.monotonic()
        if not force and (now - self._last_emit) < self.interval_s:
            return
        self._tick += 1
        self._last_emit = now
        elapsed = now - self._t0
        parts = [f"[diag +{elapsed:.0f}s] phase={self._phase} tick=#{self._tick}"]
        if self._extra:
            parts.append(self._extra)
        if self.serial_monitor and self.serial_monitor.running:
            parts.append(f"serial={self.serial_monitor.recent()}")
        if self.include_usb:
            if self.serial_monitor and self.serial_monitor.running:
                parts.append("usb=(deferred; serial open)")
            else:
                parts.append(f"usb={read_usb_boot_summary(self.port, self.chip)}")
        self.log(" ".join(parts))

    async def maybe_emit_async(self, force: bool = False) -> None:
        now = time.monotonic()
        if not force and (now - self._last_emit) < self.interval_s:
            return
        self._tick += 1
        self._last_emit = now
        elapsed = now - self._t0
        parts = [f"[diag +{elapsed:.0f}s] phase={self._phase} tick=#{self._tick}"]
        if self._extra:
            parts.append(self._extra)
        if self.serial_monitor and self.serial_monitor.running:
            parts.append(f"serial={self.serial_monitor.recent()}")
        if self.include_usb:
            if self.serial_monitor and self.serial_monitor.running:
                parts.append("usb=(deferred; serial open)")
            else:
                parts.append(f"usb={read_usb_boot_summary(self.port, self.chip)}")
        if self.include_ble and self.address:
            if self.ble_tracker and self.ble_tracker.session_active:
                ble_part = f"ble=session:{self.ble_tracker.state}"
                if self.ble_tracker.detail:
                    ble_part += f" {self.ble_tracker.detail}"
                parts.append(ble_part)
            else:
                parts.append(await read_ble_advertising_status(self.address))
        self.log(" ".join(parts))

    async def sleep(self, seconds: float, phase: Optional[str] = None) -> None:
        if phase:
            self.set_phase(phase)
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            await self.maybe_emit_async()
            remaining = deadline - time.monotonic()
            await asyncio.sleep(min(self.interval_s, max(0.1, remaining), 2.0))
