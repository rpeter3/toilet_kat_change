# Phase 0 spike: build, flash, and capture serial output for ROM bootloader write test.
param(
    [string]$Port = "COM5",
    [ValidateSet("factory", "ota_0")]
    [string]$BootSlot = "factory"
)

$ErrorActionPreference = "Stop"
$Root = Split-Path -Parent $PSScriptRoot
$SpikeDir = Join-Path $Root "SPIKE_BootloaderRomWrite"
$Cli = "C:\Program Files\Arduino CLI\arduino-cli.exe"

if (-not (Test-Path $Cli)) {
    throw "Arduino CLI not found at $Cli"
}

function Get-Esptool {
    $cmd = Get-Command esptool.py -ErrorAction SilentlyContinue
    if ($cmd) { return "esptool.py" }
    $pio = Join-Path $env:USERPROFILE ".platformio\penv\Scripts\esptool.py"
    if (Test-Path $pio) { return $pio }
    return "python -m esptool"
}

Push-Location $SpikeDir
$BuildDir = Join-Path $SpikeDir "build\esp32.esp32.esp32s3"
try {
    Write-Host "Building SPIKE_BootloaderRomWrite..."
    & $Cli compile `
        --fqbn esp32:esp32:esp32s3 . `
        --build-path $BuildDir `
        --build-property "build.partitions_file=partitions.csv" `
        --build-property "upload.flash_size=16MB" `
        --build-property "build.flash_size=16MB" `
        --build-property "build.cdc_on_boot=1"
}
finally {
    Pop-Location
}

$AppBin = Join-Path $BuildDir "SPIKE_BootloaderRomWrite.ino.bin"
$BootBin = Join-Path $BuildDir "SPIKE_BootloaderRomWrite.ino.bootloader.bin"
$PartBin = Join-Path $BuildDir "SPIKE_BootloaderRomWrite.ino.partitions.bin"
$BootApp0 = Join-Path $env:LOCALAPPDATA "Arduino15\packages\esp32\hardware\esp32\3.3.10\tools\partitions\boot_app0.bin"

foreach ($f in @($AppBin, $BootBin, $PartBin)) {
    if (-not (Test-Path $f)) {
        throw "Missing build artifact: $f"
    }
}

$esptool = Get-Esptool
Write-Host "Flashing spike app to $BootSlot on $Port..."

if ($BootSlot -eq "factory") {
    if ($esptool -eq "python -m esptool") {
        & python -m esptool --chip esp32s3 --port $Port --baud 921600 write_flash `
            0x0 $BootBin `
            0x8000 $PartBin `
            0xE000 $BootApp0 `
            0x10000 $AppBin
    } else {
        & $esptool --chip esp32s3 --port $Port --baud 921600 write_flash `
            0x0 $BootBin `
            0x8000 $PartBin `
            0xE000 $BootApp0 `
            0x10000 $AppBin
    }
} else {
    $otaData = Join-Path $env:TEMP "spike_otadata_$BootSlot.bin"
    python (Join-Path $PSScriptRoot "make_otadata.py") $BootSlot $otaData

    if ($esptool -eq "python -m esptool") {
        & python -m esptool --chip esp32s3 --port $Port --baud 921600 write_flash `
            0xE000 $otaData `
            0x490000 $AppBin
    } else {
        & $esptool --chip esp32s3 --port $Port --baud 921600 write_flash `
            0xE000 $otaData `
            0x490000 $AppBin
    }
}

Write-Host "Waiting for reboot..."
Start-Sleep -Seconds 3

$logPath = Join-Path $Root "spike-phase0-$BootSlot.log"
$CaptureScript = Join-Path $PSScriptRoot "capture_serial.py"
Write-Host "Capturing serial log to $logPath (15s)..."

python $CaptureScript $Port $logPath 15
exit $LASTEXITCODE
