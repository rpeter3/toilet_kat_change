# Flash SARAH bootloader + spike app to FACTORY for ROM patch test.
# Factory boot can write flash under SARAH; ota_0 cannot (confirmed by spike).
param(
    [string]$Port = "COM5"
)

$ErrorActionPreference = "Stop"
$Root = Split-Path -Parent $PSScriptRoot
$SpikeBuild = Join-Path $Root "SPIKE_BootloaderRomWrite\build\esp32.esp32.esp32s3"
$AppBin = Join-Path $SpikeBuild "SPIKE_BootloaderRomWrite.ino.bin"
$SarahMerged = Join-Path $Root "Produciton_backups\toilet_kat_change.ino.merged_OG_VERSION.bin"
$SarahBoot = Join-Path $Root "SPIKE_BootloaderRomWrite\sarah.bootloader.bin"
$OtaData = Join-Path $env:TEMP "spike_otadata_factory.bin"

if (-not (Test-Path $AppBin)) {
    throw "Spike app not found at $AppBin. Export Compiled Binary first."
}

python (Join-Path $PSScriptRoot "extract_bootloader.py") $SarahMerged $SarahBoot
python (Join-Path $PSScriptRoot "make_otadata.py") factory $OtaData

Write-Host "Flashing SARAH bootloader @0x0, cleared otadata, spike app @factory on $Port..."
python -m esptool --chip esp32s3 --port $Port --baud 921600 write-flash `
    0x0 $SarahBoot `
    0xE000 $OtaData `
    0x10000 $AppBin

Write-Host "Done. Device boots from factory. Reset and open Serial Monitor @115200."
python (Join-Path $Root "read_partitions.py") --port $Port
