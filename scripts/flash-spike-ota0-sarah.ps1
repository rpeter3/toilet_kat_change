# Flash SARAH bootloader + spike app to ota_0 for realistic Phase 0 testing.
param(
    [string]$Port = "COM5"
)

$ErrorActionPreference = "Stop"
$Root = Split-Path -Parent $PSScriptRoot
$SpikeBuild = Join-Path $Root "SPIKE_BootloaderRomWrite\build\esp32.esp32.esp32s3"
$AppBin = Join-Path $SpikeBuild "SPIKE_BootloaderRomWrite.ino.bin"
$SarahMerged = Join-Path $Root "Produciton_backups\toilet_kat_change.ino.merged_OG_VERSION.bin"
$SarahBoot = Join-Path $Root "SPIKE_BootloaderRomWrite\sarah.bootloader.bin"
$OtaData = Join-Path $env:TEMP "spike_otadata_ota0.bin"

if (-not (Test-Path $AppBin)) {
    throw "Spike app not found at $AppBin. Build SPIKE_BootloaderRomWrite in Arduino IDE first (Export Compiled Binary)."
}

python (Join-Path $PSScriptRoot "extract_bootloader.py") $SarahMerged $SarahBoot
python (Join-Path $PSScriptRoot "make_otadata.py") ota_0 $OtaData

Write-Host "Flashing SARAH bootloader @0x0, otadata, spike app @ota_0 on $Port..."
python -m esptool --chip esp32s3 --port $Port --baud 921600 write-flash `
    0x0 $SarahBoot `
    0xE000 $OtaData `
    0x490000 $AppBin

Write-Host "Done. Reset board and open Serial Monitor @115200 (USB CDC On Boot enabled)."
Write-Host "Verifying otadata on device..."
python (Join-Path $Root "read_partitions.py") --port $Port
