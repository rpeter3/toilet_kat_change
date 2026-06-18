# Flash an app.bin to the factory partition using build artifacts from a pinned toolchain build.
# Does not assume any specific merged image or bootloader identity.
#
# Example (factory image preserved, boot from ota_0 for rollback testing):
#   .\scripts\build-firmware.ps1
#   .\scripts\flash-factory-app.ps1 -Port COM5 `
#       -AppBin .\toilet_kat_change\build\esp32.esp32.esp32s3\toilet_kat_change.ino.bin `
#       -BootSlot ota_0

param(
    [string]$Port = "COM5",
    [Parameter(Mandatory = $true)]
    [string]$AppBin,
    [string]$BuildDir,
    [ValidateSet("factory", "ota_0", "ota_1")]
    [string]$BootSlot = "ota_0"
)

$ErrorActionPreference = "Stop"
$Root = Split-Path -Parent $PSScriptRoot
$ToolchainPath = Join-Path $PSScriptRoot "firmware-toolchain.json"
$PartitionsCsv = Join-Path $Root "toilet_kat_change\partitions.csv"

if (-not (Test-Path $AppBin)) {
    throw "App binary not found: $AppBin"
}
$AppBin = (Resolve-Path $AppBin).Path

if (-not $BuildDir) {
    $BuildDir = Split-Path -Parent $AppBin
}
if (-not (Test-Path $BuildDir)) {
    throw "Build directory not found: $BuildDir"
}
$BuildDir = (Resolve-Path $BuildDir).Path

if (-not (Test-Path $ToolchainPath)) {
    throw "Firmware toolchain config not found: $ToolchainPath"
}
if (-not (Test-Path $PartitionsCsv)) {
    throw "Partition table not found: $PartitionsCsv"
}

$Toolchain = Get-Content $ToolchainPath -Raw | ConvertFrom-Json
$CoreVersion = $Toolchain.arduinoEsp32Core
$Fqbn = $Toolchain.fqbn

function Get-PartitionOffset {
    param([string]$Name)
    foreach ($line in Get-Content $PartitionsCsv) {
        $trimmed = $line.Trim()
        if (-not $trimmed -or $trimmed.StartsWith("#")) { continue }
        $fields = $trimmed -split ",\s*"
        if ($fields.Count -lt 4) { continue }
        if ($fields[0].Trim() -eq $Name) {
            $offsetText = $fields[3].Trim()
            return [Convert]::ToInt32($offsetText, 16)
        }
    }
    throw "Partition '$Name' not found in $PartitionsCsv"
}

$FactoryOffset = Get-PartitionOffset "factory"
$Ota0Offset = Get-PartitionOffset "ota_0"
$Ota1Offset = Get-PartitionOffset "ota_1"

Write-Host "Validating app image with esptool image-info..."
python -m esptool image-info $AppBin | Out-Host
if ($LASTEXITCODE -ne 0) {
    throw "esptool image-info failed for $AppBin"
}

$CompileCommandsPath = Join-Path $BuildDir "compile_commands.json"
$BuildOptionsPath = Join-Path $BuildDir "build.options.json"
if ((Test-Path $CompileCommandsPath) -and (Test-Path $BuildOptionsPath)) {
    Write-Host "Checking build profile in $BuildDir..."
    $compileText = Get-Content $CompileCommandsPath -Raw
    $buildOptions = Get-Content $BuildOptionsPath -Raw | ConvertFrom-Json
    if ($Fqbn -and $buildOptions.fqbn -ne $Fqbn) {
        throw "Build profile mismatch: expected fqbn '$Fqbn' in build.options.json"
    }
    $profileChecks = @(
        "CORE_DEBUG_LEVEL=3",
        "BOARD_HAS_PSRAM",
        "ARDUINO_USB_CDC_ON_BOOT=1",
        "esp32s3/dio_opi/"
    )
    foreach ($pattern in $profileChecks) {
        if ($compileText -notmatch [regex]::Escape($pattern)) {
            throw "Build profile mismatch: expected '$pattern' in compile_commands.json"
        }
    }
    Write-Host "Build profile checks passed."
} else {
    Write-Host "WARN: compile_commands.json or build.options.json missing; skipping profile checks."
}

$BootBin = Get-ChildItem -Path $BuildDir -Filter "*.bootloader.bin" | Select-Object -First 1
$PartBin = Get-ChildItem -Path $BuildDir -Filter "*.partitions.bin" | Select-Object -First 1
if (-not $BootBin) { throw "No *.bootloader.bin in $BuildDir" }
if (-not $PartBin) { throw "No *.partitions.bin in $BuildDir" }

$BootApp0 = Join-Path $env:LOCALAPPDATA "Arduino15\packages\esp32\hardware\esp32\$CoreVersion\tools\partitions\boot_app0.bin"
if (-not (Test-Path $BootApp0)) {
    throw "boot_app0.bin not found at $BootApp0 (install esp32 core $CoreVersion)"
}

$OtaDataPath = Join-Path $env:TEMP "flash_factory_otadata_$BootSlot.bin"
if ($BootSlot -eq "factory") {
    python (Join-Path $PSScriptRoot "make_otadata.py") factory $OtaDataPath
} else {
    python (Join-Path $PSScriptRoot "make_otadata.py") $BootSlot $OtaDataPath
}
if ($LASTEXITCODE -ne 0) {
    throw "make_otadata.py failed"
}

$flashArgs = @(
    "--chip", "esp32s3",
    "--port", $Port,
    "--baud", "921600",
    "write-flash",
    "0x0", $BootBin.FullName,
    "0x8000", $PartBin.FullName,
    "0xE000", $OtaDataPath,
    ("0x{0:X}" -f $FactoryOffset), $AppBin
)

if ($BootSlot -eq "ota_0") {
    $flashArgs += ("0x{0:X}" -f $Ota0Offset), $AppBin
} elseif ($BootSlot -eq "ota_1") {
    $flashArgs += ("0x{0:X}" -f $Ota1Offset), $AppBin
}

Write-Host "Flashing factory app @0x$($FactoryOffset.ToString('X')) (boot slot: $BootSlot) on $Port..."
python -m esptool @flashArgs
if ($LASTEXITCODE -ne 0) {
    throw "esptool write-flash failed with exit code $LASTEXITCODE"
}

Write-Host "Done. Verifying partition table on device..."
python (Join-Path $Root "read_partitions.py") --port $Port
if ($LASTEXITCODE -ne 0) {
    throw "read_partitions.py failed with exit code $LASTEXITCODE"
}

Write-Host "Factory app flashed. Reset the device to boot from $BootSlot."
