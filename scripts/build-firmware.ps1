# Build production firmware for BLE OTA / S1 app bundling.
# Requires Arduino CLI with esp32:esp32 core 3.x installed.

$ErrorActionPreference = "Stop"
$Root = Split-Path -Parent $PSScriptRoot
$SketchDir = Join-Path $Root "toilet_kat_change"
$Cli = "C:\Program Files\Arduino CLI\arduino-cli.exe"

if (-not (Test-Path $Cli)) {
    throw "Arduino CLI not found at $Cli"
}

Push-Location $SketchDir
try {
    & $Cli compile `
        --fqbn esp32:esp32:esp32s3 . `
        --build-property "build.partitions_file=partitions.csv" `
        --build-property "upload.flash_size=16MB" `
        --build-property "build.flash_size=16MB" `
        --build-property "build.sdkconfig.defaults=sdkconfig.defaults"
}
finally {
    Pop-Location
}

$OutBin = Join-Path $SketchDir "build\esp32.esp32.esp32s3\toilet_kat_change.ino.bin"
if (-not (Test-Path $OutBin)) {
    throw "Build succeeded but firmware binary not found at $OutBin"
}

Write-Host "Firmware built: $OutBin"
Write-Host "Size: $((Get-Item $OutBin).Length) bytes"
