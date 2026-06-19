# Flash SARAH bootloader + extracted factory app + canonical partition table.
param(
    [string]$Port = "COM5",
    [string]$ArtifactsDir = "",
    [string]$Label = "sarah"
)

$ErrorActionPreference = "Stop"
$Root = Split-Path -Parent $PSScriptRoot
if (-not $ArtifactsDir) {
    $ArtifactsDir = Join-Path $Root "test-builds\ota-regression"
}
$ArtifactsDir = (Resolve-Path $ArtifactsDir).Path

$BootBin = Join-Path $ArtifactsDir "${Label}_bootloader.bin"
$PartBin = Join-Path $ArtifactsDir "${Label}_partitions.bin"
$AppBin = Join-Path $ArtifactsDir "${Label}_factory_app.bin"
$OtaData = Join-Path $env:TEMP "flash_sarah_factory_otadata.bin"

foreach ($path in @($BootBin, $PartBin, $AppBin)) {
    if (-not (Test-Path $path)) {
        throw "Missing artifact: $path (run build_ota_regression_firmware.py first)"
    }
}

python (Join-Path $PSScriptRoot "make_otadata.py") factory $OtaData
if ($LASTEXITCODE -ne 0) {
    throw "make_otadata.py failed"
}

Write-Host "Flashing SARAH factory on $Port..."
python -m esptool --chip esp32s3 --port $Port --baud 921600 write-flash `
    0x0 $BootBin `
    0x8000 $PartBin `
    0xE000 $OtaData `
    0x10000 $AppBin
if ($LASTEXITCODE -ne 0) {
    throw "esptool write-flash failed with exit code $LASTEXITCODE"
}

python -m esptool --chip esp32s3 --port $Port run
if ($LASTEXITCODE -ne 0) {
    throw "esptool run failed with exit code $LASTEXITCODE"
}

Write-Host "Done. Verifying partition table..."
python (Join-Path $Root "read_partitions.py") --port $Port
if ($LASTEXITCODE -ne 0) {
    throw "read_partitions.py failed with exit code $LASTEXITCODE"
}

Write-Host "SARAH factory flashed."
