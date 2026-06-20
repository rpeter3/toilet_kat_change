#!/usr/bin/env python3
"""SPIFFS backup, extract, inject, repack, and flash helpers for log trim regression."""

from __future__ import annotations

import shutil
import struct
import subprocess
import sys
import tempfile
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
SPIFFSGEN = Path(__file__).resolve().parent / "vendor" / "spiffsgen.py"

# partitions.csv storage partition
SPIFFS_OFFSET = 0xDA0000
SPIFFS_SIZE = 0x260000
SPIFFS_PARTITION_LABEL = "storage"

LOG_FILE_SPIFFS_PATH = "/errors.txt"
ERRORS_TXT_STAGING_NAME = "errors.txt"

KNOWN_SPIFFS_FILES = (
    ERRORS_TXT_STAGING_NAME,
    "hwcfg_active.bin",
    "hwcfg_lkg.bin",
    "errors.tmp",
)

# ESP32 Arduino 3.3.3 / IDF 5.5 sdkconfig defaults
SPIFFS_PAGE_SIZE = 256
SPIFFS_BLOCK_SIZE = 4096
SPIFFS_OBJ_NAME_LEN = 32
SPIFFS_META_LEN = 4
SPIFFS_OBJ_ID_LEN = 2
SPIFFS_SPAN_IX_LEN = 2

SPIFFS_PH_FLAG_USED_FINAL_INDEX = 0xF8
SPIFFS_PH_FLAG_USED_FINAL = 0xFC
SPIFFS_TYPE_FILE = 1

SPIFFS_DATA_HEADER_LEN = SPIFFS_OBJ_ID_LEN + SPIFFS_SPAN_IX_LEN + 1  # 5
SPIFFS_DATA_HEADER_ALIGNED = 8  # index pages only
SPIFFS_DATA_CONTENT_LEN = SPIFFS_PAGE_SIZE - SPIFFS_DATA_HEADER_LEN  # 251

SPIFFS_INDEX_HEADER_LEN = (
    SPIFFS_DATA_HEADER_ALIGNED + 4 + 1 + SPIFFS_OBJ_NAME_LEN + SPIFFS_META_LEN
)  # 49


def _import_read_flash():
    if str(REPO_ROOT) not in sys.path:
        sys.path.insert(0, str(REPO_ROOT))
    from scripts.esp_flash_tools import read_flash

    return read_flash


def is_erased_partition(data: bytes) -> bool:
    sample = data[:4096]
    return not sample or all(b == 0xFF for b in sample)


def _page_header(page: bytes) -> tuple[int, int, int]:
    if len(page) < 5:
        return 0, 0, 0xFF
    obj_id = struct.unpack_from("<H", page, 0)[0]
    span_ix = struct.unpack_from("<H", page, 2)[0]
    flags = page[4]
    return obj_id, span_ix, flags


def _is_index_page(flags: int) -> bool:
    return flags == SPIFFS_PH_FLAG_USED_FINAL_INDEX


def _is_data_page(flags: int) -> bool:
    return flags == SPIFFS_PH_FLAG_USED_FINAL


def _normalize_obj_id(obj_id: int) -> int:
    return obj_id & ~(1 << 15)


def _read_index_page_indices(page: bytes, span_ix: int) -> list[int]:
    if span_ix == 0:
        start = SPIFFS_INDEX_HEADER_LEN
    else:
        start = SPIFFS_DATA_HEADER_ALIGNED
    indices: list[int] = []
    pos = start
    while pos + 2 <= SPIFFS_PAGE_SIZE:
        value = struct.unpack_from("<H", page, pos)[0]
        if value == 0xFFFF:
            break
        indices.append(value)
        pos += 2
    return indices


def _parse_index_page_span0(page: bytes) -> tuple[str, int, list[int]] | None:
    obj_id, span_ix, flags = _page_header(page)
    if not _is_index_page(flags) or span_ix != 0:
        return None
    if SPIFFS_INDEX_HEADER_LEN > len(page):
        return None
    size = struct.unpack_from("<I", page, SPIFFS_DATA_HEADER_ALIGNED)[0]
    obj_type = page[SPIFFS_DATA_HEADER_ALIGNED + 4]
    if obj_type != SPIFFS_TYPE_FILE:
        return None
    name_start = SPIFFS_DATA_HEADER_ALIGNED + 5
    name_bytes = page[name_start : name_start + SPIFFS_OBJ_NAME_LEN]
    name = name_bytes.split(b"\x00")[0].decode("utf-8", errors="replace")
    indices = _read_index_page_indices(page, 0)
    return name, size, indices


def extract_spiffs_files(image: bytes) -> dict[str, bytes]:
    """Extract regular files from a SPIFFS partition image."""
    if is_erased_partition(image):
        return {}

    num_pages = len(image) // SPIFFS_PAGE_SIZE
    index_pages: dict[int, dict[int, tuple[str, int, list[int]]]] = {}

    for page_ix in range(num_pages):
        page = image[page_ix * SPIFFS_PAGE_SIZE : (page_ix + 1) * SPIFFS_PAGE_SIZE]
        if all(b == 0xFF for b in page[:16]):
            continue
        obj_id, span_ix, flags = _page_header(page)
        if not _is_index_page(flags):
            continue
        norm_id = _normalize_obj_id(obj_id)
        if span_ix == 0:
            parsed = _parse_index_page_span0(page)
            if parsed is None:
                continue
            name, size, indices = parsed
            index_pages.setdefault(norm_id, {})[0] = (name, size, indices)
        else:
            indices = _read_index_page_indices(page, span_ix)
            index_pages.setdefault(norm_id, {})[span_ix] = ("", 0, indices)

    files: dict[str, bytes] = {}
    for norm_id, spans in index_pages.items():
        if 0 not in spans:
            continue
        name, size, _ = spans[0]
        if not name or not name.startswith("/"):
            continue

        all_indices: list[int] = []
        for span_ix in sorted(spans):
            _, _, indices = spans[span_ix]
            all_indices.extend(indices)

        chunks: list[bytes] = []
        remaining = size
        for page_num in all_indices:
            if remaining <= 0:
                break
            offset = page_num * SPIFFS_PAGE_SIZE
            if offset + SPIFFS_PAGE_SIZE > len(image):
                break
            data_page = image[offset : offset + SPIFFS_PAGE_SIZE]
            _, _, flags = _page_header(data_page)
            if not _is_data_page(flags):
                continue
            payload = data_page[SPIFFS_DATA_HEADER_LEN:]
            take = min(len(payload), remaining)
            chunks.append(payload[:take])
            remaining -= take

        content = b"".join(chunks)
        if len(content) > size:
            content = content[:size]
        staging_name = name.lstrip("/")
        files[staging_name] = content

    return files


def spiffs_staging_name(path: str) -> str:
    return path.lstrip("/")


def generate_errors_txt(target_bytes: int) -> bytes:
    """Build a newline-terminated errors.txt of exactly target_bytes."""
    if target_bytes <= 0:
        return b""

    header = "trim_test,1,0,TRIM_MARKER_OLD_000001\n"
    footer = "trim_test,999999,0,TRIM_MARKER_NEW_999999\n"
    if target_bytes < len(header) + len(footer):
        raise ValueError(f"target_bytes {target_bytes} too small for trim markers")

    mid_lines: list[str] = []
    seq = 2
    mid_line_width = 80
    budget = target_bytes - len(header) - len(footer)
    min_tail_line = 20

    n_full = budget // mid_line_width
    remainder = budget % mid_line_width
    if remainder and remainder < min_tail_line:
        if n_full == 0:
            raise ValueError(f"target_bytes {target_bytes} too small for structured fill")
        n_full -= 1
        remainder += mid_line_width

    for _ in range(n_full):
        prefix = f"trim_test,{seq:06d},0,"
        pad_len = mid_line_width - len(prefix) - 1
        line = f"{prefix}{'P' * pad_len}\n"
        if len(line) != mid_line_width:
            raise ValueError(f"mid line width mismatch: {len(line)} != {mid_line_width}")
        mid_lines.append(line)
        seq += 1

    if remainder:
        prefix = f"trim_test,{seq:06d},0,"
        pad_len = remainder - len(prefix) - 1
        if pad_len < 0:
            raise ValueError(f"cannot fit remaining {remainder} bytes into a valid log line")
        line = f"{prefix}{'Q' * pad_len}\n"
        if len(line) != remainder:
            raise ValueError(f"tail filler line size mismatch: {len(line)} != {remainder}")
        mid_lines.append(line)

    body = (header + "".join(mid_lines) + footer).encode("utf-8")
    if len(body) != target_bytes:
        raise ValueError(f"generated errors.txt size {len(body)} != target {target_bytes}")
    if any(len(line) > 200 for line in body.decode("utf-8").splitlines()):
        raise ValueError("generated line exceeds LOG_LINE_MAX_LEN (200)")
    return body


def build_spiffs_image(staging_dir: Path, output_path: Path) -> None:
    if not SPIFFSGEN.exists():
        raise RuntimeError(f"spiffsgen.py not found at {SPIFFSGEN}")
    staging_dir.mkdir(parents=True, exist_ok=True)
    cmd = [
        sys.executable,
        str(SPIFFSGEN),
        hex(SPIFFS_SIZE),
        str(staging_dir),
        str(output_path),
        "--page-size",
        str(SPIFFS_PAGE_SIZE),
        "--block-size",
        str(SPIFFS_BLOCK_SIZE),
        "--obj-name-len",
        str(SPIFFS_OBJ_NAME_LEN),
        "--meta-len",
        str(SPIFFS_META_LEN),
        "--use-magic",
        "--use-magic-len",
    ]
    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode != 0:
        raise RuntimeError(
            f"spiffsgen failed (exit {result.returncode}): {result.stderr or result.stdout}"
        )
    if not output_path.exists() or output_path.stat().st_size != SPIFFS_SIZE:
        raise RuntimeError(f"spiffsgen produced invalid image at {output_path}")


def unpack_spiffs_image(image: bytes, staging_dir: Path) -> dict[str, bytes]:
    staging_dir.mkdir(parents=True, exist_ok=True)
    extracted = extract_spiffs_files(image)
    for name, content in extracted.items():
        out = staging_dir / name
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_bytes(content)
    return extracted


def inject_errors_txt_from_backup(
    backup_image: bytes,
    target_log_bytes: int,
    work_dir: Path,
) -> Path:
    """Read-modify-write: preserve existing SPIFFS files, replace errors.txt, repack."""
    staging_dir = work_dir / "staging"
    if staging_dir.exists():
        shutil.rmtree(staging_dir)
    staging_dir.mkdir(parents=True)

    extracted = unpack_spiffs_image(backup_image, staging_dir)
    errors_path = staging_dir / ERRORS_TXT_STAGING_NAME
    errors_path.write_bytes(generate_errors_txt(target_log_bytes))

    output_path = work_dir / f"errors_{target_log_bytes}.bin"
    build_spiffs_image(staging_dir, output_path)

    # Round-trip sanity check when extraction succeeded.
    if extracted:
        rebuilt = extract_spiffs_files(output_path.read_bytes())
        if ERRORS_TXT_STAGING_NAME not in rebuilt:
            raise RuntimeError("repacked SPIFFS image missing errors.txt after inject")
        if abs(len(rebuilt[ERRORS_TXT_STAGING_NAME]) - target_log_bytes) > 4:
            raise RuntimeError(
                "repacked errors.txt size mismatch: "
                f"expected {target_log_bytes}, got {len(rebuilt[ERRORS_TXT_STAGING_NAME])}"
            )
    return output_path


def backup_spiffs(port: str, chip: str, dest: Path) -> bytes:
    read_flash = _import_read_flash()
    data = read_flash(port, chip, SPIFFS_OFFSET, SPIFFS_SIZE)
    dest.parent.mkdir(parents=True, exist_ok=True)
    dest.write_bytes(data)
    return data


def flash_spiffs(port: str, chip: str, image_path: Path) -> None:
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
        "write-flash",
        hex(SPIFFS_OFFSET),
        str(image_path),
    ]
    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode != 0:
        raise RuntimeError(
            f"esptool write-flash SPIFFS failed (exit {result.returncode}): "
            f"{result.stderr or result.stdout}"
        )


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


def seed_log_size(
    port: str,
    chip: str,
    backup_image: bytes,
    target_log_bytes: int,
    work_dir: Path,
) -> None:
    image_path = inject_errors_txt_from_backup(backup_image, target_log_bytes, work_dir)
    flash_spiffs(port, chip, image_path)
    esptool_reset(port, chip)
