# Build, flash, capture serial, and verify bootloader region via esptool.
param(
    [string]$Port = "COM5",
    [ValidateSet("factory", "ota_0")]
    [string]$BootSlot = "factory"
)

$ErrorActionPreference = "Stop"
$Root = Split-Path -Parent $PSScriptRoot
$SketchDir = Join-Path $Root "SPIKE_BootloaderEraseTest"
$BuildDir = Join-Path $SketchDir "build\esp32.esp32.esp32s3"
$Cli = "C:\Program Files\Arduino CLI\arduino-cli.exe"
$LogPath = Join-Path $Root "spike-erase-test-$BootSlot.log"
$PhysPath = Join-Path $env:TEMP "spike_erase_phys_4k.bin"

if (-not (Test-Path $Cli)) {
    throw "Arduino CLI not found at $Cli"
}

$PatchScript = Join-Path $PSScriptRoot "patch-sdkconfig-wdt.ps1"
$SdkconfigPath = Join-Path $BuildDir "sdkconfig"

Write-Host "Building SPIKE_BootloaderEraseTest (TWDT disabled in sdkconfig export)..."
& $Cli compile --fqbn esp32:esp32:esp32s3 $SketchDir `
    --build-path $BuildDir `
    --build-property "build.partitions_file=partitions.csv" `
    --build-property "build.flash_size=16MB" `
    --build-property "upload.flash_size=16MB" `
    --build-property "build.cdc_on_boot=1" `
    --clean

# Arduino ESP32 3.x ignores sdkconfig.defaults; it copies a prebuilt sdkconfig here.
# Patch the export so the spike audit matches sdkconfig.defaults intent.
& $PatchScript -SdkconfigPath $SdkconfigPath

Write-Host "sdkconfig WDT lines:"
Select-String -Path $SdkconfigPath -Pattern 'CONFIG_ESP_(TASK_WDT_EN|INT_WDT)'

$AppBin = Join-Path $BuildDir "SPIKE_BootloaderEraseTest.ino.bin"
if (-not (Test-Path $AppBin)) {
    throw "Build output not found: $AppBin"
}

$SarahMerged = Join-Path $Root "Produciton_backups\toilet_kat_change.ino.merged_OG_VERSION.bin"
$SarahBoot = Join-Path $SketchDir "sarah.bootloader.bin"
python (Join-Path $PSScriptRoot "extract_bootloader.py") $SarahMerged $SarahBoot

if ($BootSlot -eq "factory") {
    $OtaData = Join-Path $env:TEMP "spike_erase_otadata_factory.bin"
    python (Join-Path $PSScriptRoot "make_otadata.py") factory $OtaData
    Write-Host "Flashing SARAH BL + factory app on $Port..."
    python -m esptool --chip esp32s3 --port $Port --baud 921600 write-flash `
        0x0 $SarahBoot `
        0xE000 $OtaData `
        0x10000 $AppBin
} else {
    $OtaData = Join-Path $env:TEMP "spike_erase_otadata_ota0.bin"
    python (Join-Path $PSScriptRoot "make_otadata.py") ota_0 $OtaData
    Write-Host "Flashing SARAH BL + ota_0 app on $Port..."
    python -m esptool --chip esp32s3 --port $Port --baud 921600 write-flash `
        0x0 $SarahBoot `
        0xE000 $OtaData `
        0x490000 $AppBin
}

Write-Host "Capturing serial for 45s -> $LogPath"
Start-Sleep -Seconds 2
python (Join-Path $PSScriptRoot "capture_serial.py") $Port $LogPath 45
$serialExit = $LASTEXITCODE

Write-Host "Reading physical first 4KB @0x0 via esptool..."
python -m esptool --chip esp32s3 --port $Port read-flash 0 4096 $PhysPath
python -c @"
from pathlib import Path
d = Path(r'$PhysPath').read_bytes()
all_ff = all(b == 0xFF for b in d)
print(f'physical_first_4k_all_ff={all_ff}')
print('first_16_bytes:', ' '.join(f'{b:02X}' for b in d[:16]))
"@

Write-Host ""
Write-Host "=== Serial log tail ==="
Get-Content $LogPath -Tail 25

if (Select-String -Path $LogPath -Pattern "\[PASS\] in-app ROM erase" -Quiet) {
    exit 0
}
exit $(if ($serialExit -eq 0) { 0 } else { 1 })
