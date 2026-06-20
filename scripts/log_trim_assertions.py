#!/usr/bin/env python3
"""Assertions for SPIFFS error-log trim regression (mirrors toilet_kat_change.ino constants)."""

from __future__ import annotations

# Source of truth: toilet_kat_change/toilet_kat_change.ino
MAX_LOG_SIZE = 76800
LOG_HEADROOM = 10240
LOG_SLEEP_TRIM_TRIGGER = MAX_LOG_SIZE - LOG_HEADROOM  # 66560
LOG_TRIM_TARGET = 51200
OTA_PREP_LOG_TRIM_TARGET = 40960
LOG_LINE_MAX_LEN = 200

# Line-boundary trim + one boot line slack.
LINE_SLACK = 512

MARKER_OLD_PREFIX = "TRIM_MARKER_OLD_"
MARKER_NEW_PREFIX = "TRIM_MARKER_NEW_"
MARKER_OLD_FIRST = f"{MARKER_OLD_PREFIX}000001"
MARKER_NEW_LAST = f"{MARKER_NEW_PREFIX}999999"


class LogTrimAssertionError(AssertionError):
    pass


def log_byte_length(logs: str) -> int:
    return len(logs.encode("utf-8"))


def assert_log_size_at_most(logs: str, max_bytes: int, *, label: str = "") -> None:
    size = log_byte_length(logs)
    limit = max_bytes + LINE_SLACK
    if size > limit:
        prefix = f"{label}: " if label else ""
        raise LogTrimAssertionError(
            f"{prefix}log size {size} exceeds max {max_bytes} (+ slack {LINE_SLACK})"
        )


def assert_log_size_at_least(logs: str, min_bytes: int, *, label: str = "") -> None:
    size = log_byte_length(logs)
    if size < min_bytes:
        prefix = f"{label}: " if label else ""
        raise LogTrimAssertionError(f"{prefix}log size {size} below min {min_bytes}")


def assert_marker_present(logs: str, marker: str, *, label: str = "") -> None:
    if marker not in logs:
        prefix = f"{label}: " if label else ""
        raise LogTrimAssertionError(f"{prefix}expected marker {marker!r} not found")


def assert_marker_absent(logs: str, marker: str, *, label: str = "") -> None:
    if marker in logs:
        prefix = f"{label}: " if label else ""
        raise LogTrimAssertionError(f"{prefix}marker {marker!r} should have been trimmed away")


def assert_newest_markers_kept(logs: str, *, label: str = "") -> None:
    assert_marker_present(logs, MARKER_NEW_LAST, label=label)


def assert_oldest_markers_dropped(logs: str, *, label: str = "") -> None:
    assert_marker_absent(logs, MARKER_OLD_FIRST, label=label)


def assert_line_aligned(logs: str, *, label: str = "") -> None:
    if not logs:
        return
    if logs.startswith("\n"):
        prefix = f"{label}: " if label else ""
        raise LogTrimAssertionError(f"{prefix}log begins with stray newline (partial line trim?)")
    if not logs.endswith("\n"):
        prefix = f"{label}: " if label else ""
        raise LogTrimAssertionError(f"{prefix}log missing trailing newline on last line")


def assert_no_trim_under_threshold(logs: str, seeded_bytes: int, *, label: str = "") -> None:
    """After reboot with sub-threshold seed, oldest markers should remain."""
    assert_marker_present(logs, MARKER_OLD_FIRST, label=label)
    # Boot may append ota_boot; allow modest growth only.
    assert_log_size_at_most(logs, seeded_bytes + 2048, label=label)
