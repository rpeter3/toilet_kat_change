# Build SPIKE_PIO via PlatformIO, flash SARAH+app, capture serial, verify bootloader MD5.
param(
    [string]$Port = "COM5",
    [ValidateSet("factory", "ota_0")]
    [string]$BootSlot = "factory",
    [switch]$BuildOnly
)

$ErrorActionPreference = "Stop"
$Root = Split-Path -Parent $PSScriptRoot
$SpikeDir = Join-Path $Root "SPIKE_PIO"
$EnvName = "esp32-s3-devkitc-1"
$BuildDir = Join-Path $SpikeDir ".pio\build\$EnvName"
$AppBin = Join-Path $BuildDir "firmware.bin"
$SdkconfigPath = Join-Path $BuildDir "sdkconfig"
$LogPath = Join-Path $Root "spike-pio-overwrite-$BootSlot.log"
$PhysPath = Join-Path $env:TEMP "spike_pio_phys_bl.bin"
$GoodMd5 = "e536c176c97b5905286ed980da47abe8"
$BootloaderLen = 19984

if (-not (Test-Path $SpikeDir)) {
    throw "SPIKE_PIO not found: $SpikeDir"
}

# pioarduino platform bundles esptool 5.x; click 8.2+ breaks its CLI on Windows.
$pioPip = Join-Path $env:USERPROFILE ".platformio\penv\Scripts\pip.exe"
if (Test-Path $pioPip) {
    & $pioPip install "click<8.2" | Out-Null
}

$pio = Get-Command pio -ErrorAction SilentlyContinue
if (-not $pio) {
    $pioExe = Join-Path $env:USERPROFILE ".platformio\penv\Scripts\pio.exe"
    if (-not (Test-Path $pioExe)) {
        throw "PlatformIO (pio) not found in PATH or $pioExe"
    }
    $pio = $pioExe
}

Write-Host "Building SPIKE_PIO (WDT disabled via sdkconfig.defaults)..."
Push-Location $SpikeDir
try {
    & $pio run -t clean
  if ($LASTEXITCODE -ne 0) { throw "pio clean failed" }
    & $pio run
  if ($LASTEXITCODE -ne 0) { throw "pio build failed" }
}
finally {
    Pop-Location
}

if (-not (Test-Path $AppBin)) {
    throw "Build output not found: $AppBin"
}

Write-Host "sdkconfig WDT audit:"
if (Test-Path $SdkconfigPath) {
    $wdtLines = Select-String -Path $SdkconfigPath -Pattern 'CONFIG_ESP_(TASK_WDT_EN|INT_WDT)'
    $wdtLines | ForEach-Object { Write-Host $_.Line }
    if (-not (Select-String -Path $SdkconfigPath -Pattern 'CONFIG_ESP_TASK_WDT_EN=n' -Quiet)) {
        throw "CONFIG_ESP_TASK_WDT_EN=n not set in $SdkconfigPath - aborting before flash"
    }
    if (-not (Select-String -Path $SdkconfigPath -Pattern 'CONFIG_ESP_INT_WDT=n' -Quiet)) {
        throw "CONFIG_ESP_INT_WDT=n not set in $SdkconfigPath - aborting before flash"
    }
} else {
    $DefaultsPath = Join-Path $SpikeDir "sdkconfig.defaults"
    Write-Host "  (no exported sdkconfig; checking $DefaultsPath)"
    if (-not (Test-Path $DefaultsPath)) {
        throw "sdkconfig.defaults not found: $DefaultsPath"
    }
    Get-Content $DefaultsPath | Where-Object { $_ -match 'CONFIG_ESP_(TASK_WDT_EN|INT_WDT)' } | ForEach-Object { Write-Host "  $_" }
    if (-not (Select-String -Path $DefaultsPath -Pattern 'CONFIG_ESP_TASK_WDT_EN=n' -Quiet)) {
        throw "CONFIG_ESP_TASK_WDT_EN=n not in sdkconfig.defaults"
    }
    if (-not (Select-String -Path $DefaultsPath -Pattern 'CONFIG_ESP_INT_WDT=n' -Quiet)) {
        throw "CONFIG_ESP_INT_WDT=n not in sdkconfig.defaults"
    }
    Write-Host "  note: pioarduino may not export sdkconfig to .pio/build; defaults were used at configure time"
}

if ($BuildOnly) {
    Write-Host "BuildOnly: skipping flash and serial capture."
    Write-Host "Firmware: $AppBin"
    exit 0
}

$SarahMerged = Join-Path $Root "Produciton_backups\toilet_kat_change.ino.merged_OG_VERSION.bin"
$SarahBoot = Join-Path $SpikeDir "sarah.bootloader.bin"
python (Join-Path $PSScriptRoot "extract_bootloader.py") $SarahMerged $SarahBoot

if ($BootSlot -eq "factory") {
    $OtaData = Join-Path $env:TEMP "spike_pio_otadata_factory.bin"
    python (Join-Path $PSScriptRoot "make_otadata.py") factory $OtaData
    Write-Host "Flashing SARAH BL + factory SPIKE_PIO app on $Port..."
    python -m esptool --chip esp32s3 --port $Port --baud 921600 write-flash `
        0x0 $SarahBoot `
        0xE000 $OtaData `
        0x10000 $AppBin
} else {
    $OtaData = Join-Path $env:TEMP "spike_pio_otadata_ota0.bin"
    python (Join-Path $PSScriptRoot "make_otadata.py") ota_0 $OtaData
    Write-Host "Flashing SARAH BL + ota_0 SPIKE_PIO app on $Port..."
    python -m esptool --chip esp32s3 --port $Port --baud 921600 write-flash `
        0x0 $SarahBoot `
        0xE000 $OtaData `
        0x490000 $AppBin
}

if ($LASTEXITCODE -ne 0) {
    throw "esptool write-flash failed"
}

Write-Host "Capturing serial for 60s -> $LogPath"
Start-Sleep -Seconds 2
python (Join-Path $PSScriptRoot "capture_serial.py") $Port $LogPath 60
$serialExit = $LASTEXITCODE

Write-Host "Reading physical bootloader @0x0 ($BootloaderLen bytes) via esptool..."
python -m esptool --chip esp32s3 --port $Port read-flash 0 $BootloaderLen $PhysPath
if ($LASTEXITCODE -ne 0) {
    throw "esptool read-flash failed"
}

$md5Out = python -c @"
import hashlib
from pathlib import Path
expected = '$GoodMd5'
d = Path(r'$PhysPath').read_bytes()
md5 = hashlib.md5(d).hexdigest()
all_ff = all(b == 0xFF for b in d)
print(f'physical_bl_md5={md5}')
print(f'expected_md5={expected}')
print(f'md5_match={md5 == expected}')
print(f'all_ff={all_ff}')
print('first_16_bytes:', ' '.join(f'{b:02X}' for b in d[:16]))
"@
Write-Host $md5Out

Write-Host ""
Write-Host "=== Serial log tail ==="
Get-Content $LogPath -Tail 30

$passSerial = Select-String -Path $LogPath -Pattern "\[PASS\] ROM overwrite of 0x0\.\.0x7FFF completed" -Quiet
$passMd5 = $md5Out -match 'md5_match=True'

if ($passSerial -and $passMd5) {
    Write-Host ""
    Write-Host "=== SPIKE_PIO PASS ==="
    exit 0
}

Write-Host ""
Write-Host "=== SPIKE_PIO FAIL ==="
Write-Host "pass_serial=$passSerial pass_md5=$passMd5"
exit $(if ($serialExit -eq 0) { 1 } else { $serialExit })
