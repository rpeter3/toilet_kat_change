# Build production firmware for BLE OTA / S1 app bundling.
# Toolchain pins are defined in scripts/firmware-toolchain.json.
#
# Output: toilet_kat_change/build/esp32.esp32.esp32s3/toilet_kat_change.ino.bin
#
# To build and bundle into CompoCloset-S1-app:
#   ..\CompoCloset-S1-app\scripts\build-firmware.ps1 [version]

$ErrorActionPreference = "Stop"
$Root = Split-Path -Parent $PSScriptRoot
$ToolchainPath = Join-Path $PSScriptRoot "firmware-toolchain.json"
if (-not (Test-Path $ToolchainPath)) {
    throw "Firmware toolchain config not found: $ToolchainPath"
}
$Toolchain = Get-Content $ToolchainPath -Raw | ConvertFrom-Json
$CoreVersion = $Toolchain.arduinoEsp32Core
$IdfLibsTag = $Toolchain.espIdfLibsTag
$SketchDir = Join-Path $Root "toilet_kat_change"
$BuildDir = Join-Path $SketchDir "build\esp32.esp32.esp32s3"
$Cli = "C:\Program Files\Arduino CLI\arduino-cli.exe"

if (-not (Test-Path $Cli)) {
    throw "Arduino CLI not found at $Cli"
}

& $Cli core update-index
& $Cli core install "esp32:esp32@$CoreVersion"

$SketchBuildDir = Join-Path $SketchDir "build"
if (Test-Path $SketchBuildDir) {
    Remove-Item -Recurse -Force $SketchBuildDir
}

Push-Location $SketchDir
try {
    & $Cli compile `
        --clean `
        --build-path $BuildDir `
        --fqbn esp32:esp32:esp32s3 . `
        --build-property "build.partitions_file=partitions.csv" `
        --build-property "upload.flash_size=16MB" `
        --build-property "build.flash_size=16MB" `
        --build-property "build.cdc_on_boot=1" `
        --build-property "build.sdkconfig.defaults=sdkconfig.defaults"
    if ($LASTEXITCODE -ne 0) {
        throw "arduino-cli compile failed with exit code $LASTEXITCODE"
    }
}
finally {
    Pop-Location
}

$OutBin = Join-Path $BuildDir "toilet_kat_change.ino.bin"
if (-not (Test-Path $OutBin)) {
    throw "Build succeeded but firmware binary not found at $OutBin"
}

$MapPath = Join-Path $BuildDir "toilet_kat_change.ino.map"
if (-not (Test-Path $MapPath)) {
    throw "Build map file not found at $MapPath"
}
$mapText = Get-Content $MapPath -Raw
$corePath = "hardware\esp32\$CoreVersion"
if ($mapText -notmatch [regex]::Escape($corePath) -and $mapText -notmatch [regex]::Escape($IdfLibsTag)) {
    throw "Build used unexpected toolchain; expected $corePath or $IdfLibsTag in map file"
}

Write-Host "Firmware built: $OutBin"
Write-Host "Size: $((Get-Item $OutBin).Length) bytes"
Write-Host "Core: esp32:esp32@$CoreVersion"
Write-Host "IDF:  $($Toolchain.espIdfVersion)"
