# SPIKE_PIO — Bootloader ROM overwrite spike

Sacrificial-hardware-only test: ROM-erase and write the proven-good bootloader @0x0 while running under the SARAH bootloader.

**Not** the production `toilet_kat_change` app. Uses its own `platformio.ini` and configs.

## Prerequisites

- PlatformIO (`pio` in PATH or `~/.platformio/penv/Scripts/pio.exe`)
- Python + `esptool` (`python -m esptool`)
- First build downloads pioarduino platform + Arduino 3.3.2 (may take several minutes)
- Script auto-installs `click<8.2` in the PIO venv (esptool 5.x requirement on Windows)

## Build only

```powershell
.\scripts\spike-pio-overwrite.ps1 -BuildOnly
```

## Build

## Flash + test

From repo root (device on USB):

```powershell
.\scripts\spike-pio-overwrite.ps1 -Port COM5
```

Do **not** use `pio run -t upload` — that flashes the PIO bootloader @0x0 instead of SARAH.

## Recovery

Reflash SARAH bootloader @0x0 via `..\scripts\flash-spike-factory-sarah.ps1` or esptool.
