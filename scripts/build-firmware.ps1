# Build production firmware for BLE OTA / S1 app bundling.
# Toolchain pins and FQBN are defined in scripts/firmware-toolchain.json.
#
# Output: toilet_kat_change/build/esp32.esp32.esp32s3/toilet_kat_change.ino.bin
#
# To build and bundle into CompoCloset-S1-app:
#   npm run bundle-firmware -- <path-to-bin> 4.0.16

param(
    # Keep software_version_build.h so OTA regression can bake in -regA/-regB versions.
    # Production / app-bundling builds must omit this and strip any leftover override.
    [switch]$AllowVersionOverride,
    # Optional alternate build output dir (avoids locking fights with Arduino IDE's sketch/build).
    [string]$BuildPath = ""
)

$ErrorActionPreference = "Stop"
$Root = Split-Path -Parent $PSScriptRoot
$ToolchainPath = Join-Path $PSScriptRoot "firmware-toolchain.json"
if (-not (Test-Path $ToolchainPath)) {
    throw "Firmware toolchain config not found: $ToolchainPath"
}
$Toolchain = Get-Content $ToolchainPath -Raw | ConvertFrom-Json
$CoreVersion = $Toolchain.arduinoEsp32Core
$IdfLibsTag = $Toolchain.espIdfLibsTag
$Fqbn = $Toolchain.fqbn
if (-not $Fqbn) {
    throw "firmware-toolchain.json must define fqbn"
}
$SketchDir = Join-Path $Root "toilet_kat_change"
if ($BuildPath) {
    $BuildDir = $BuildPath
} else {
    $BuildDir = Join-Path $SketchDir "build\esp32.esp32.esp32s3"
}
$VersionOverrideHeader = Join-Path $SketchDir "software_version_build.h"
if ((Test-Path $VersionOverrideHeader) -and -not $AllowVersionOverride) {
    Remove-Item -Force $VersionOverrideHeader
    Write-Host "Removed dev-only version override: software_version_build.h"
} elseif ((Test-Path $VersionOverrideHeader) -and $AllowVersionOverride) {
    Write-Host "Keeping version override: software_version_build.h"
}
$Cli = "C:\Program Files\Arduino CLI\arduino-cli.exe"

if (-not (Test-Path $Cli)) {
    throw "Arduino CLI not found at $Cli"
}

& $Cli core update-index
& $Cli core install "esp32:esp32@$CoreVersion"

# Prefer a dedicated build path for automated runs; only wipe the default sketch/build
# when we are actually building into it (Arduino IDE often locks that tree).
if (-not $BuildPath) {
    $SketchBuildDir = Join-Path $SketchDir "build"
    if (Test-Path $SketchBuildDir) {
        $removed = $false
        foreach ($attempt in 1..5) {
            try {
                Remove-Item -Recurse -Force $SketchBuildDir -ErrorAction Stop
                $removed = $true
                break
            } catch {
                Write-Host "WARN: could not remove $SketchBuildDir (attempt $attempt/5): $($_.Exception.Message)"
                Start-Sleep -Seconds 2
            }
        }
        if (-not $removed) {
            Write-Host "WARN: leaving locked sketch build dir in place; arduino-cli --clean will refresh outputs"
        }
    }
} else {
    New-Item -ItemType Directory -Force -Path $BuildDir | Out-Null
}

Push-Location $SketchDir
try {
    & $Cli compile `
        --clean `
        --build-path $BuildDir `
        --fqbn $Fqbn . `
        --build-property "build.partitions_file=partitions.csv" `
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

$CompileCommandsPath = Join-Path $BuildDir "compile_commands.json"
if (-not (Test-Path $CompileCommandsPath)) {
    throw "compile_commands.json not found at $CompileCommandsPath"
}
$compileText = Get-Content $CompileCommandsPath -Raw
$profileChecks = @(
    @{ Label = "CORE_DEBUG_LEVEL=3"; Pattern = "CORE_DEBUG_LEVEL=3" },
    @{ Label = "BOARD_HAS_PSRAM"; Pattern = "BOARD_HAS_PSRAM" },
    @{ Label = "ARDUINO_USB_CDC_ON_BOOT=1"; Pattern = "ARDUINO_USB_CDC_ON_BOOT=1" }
)
foreach ($check in $profileChecks) {
    if ($compileText -notmatch [regex]::Escape($check.Pattern)) {
        throw "Build profile mismatch: expected $($check.Label) in compile_commands.json"
    }
}

$SdkConfigPath = Join-Path $BuildDir "sdkconfig"
$BuildOptionsPath = Join-Path $BuildDir "build.options.json"
if (-not (Test-Path $BuildOptionsPath)) {
    throw "build.options.json not found at $BuildOptionsPath"
}
$buildOptions = Get-Content $BuildOptionsPath -Raw | ConvertFrom-Json
if ($buildOptions.fqbn -ne $Fqbn) {
    throw "Build profile mismatch: expected fqbn '$Fqbn' in build.options.json"
}
if ($compileText -notmatch "esp32s3/dio_opi/") {
    throw "Build profile mismatch: expected dio_opi prebuilt libs in compile_commands.json"
}

$AppBinItem = Get-Item $OutBin
$AppBinMd5 = (Get-FileHash $OutBin -Algorithm MD5).Hash.ToLowerInvariant()
$Metadata = [ordered]@{
    fqbn               = $Fqbn
    arduinoEsp32Core   = $CoreVersion
    espIdfVersion      = $Toolchain.espIdfVersion
    espIdfLibsTag      = $IdfLibsTag
    appBinSize         = $AppBinItem.Length
    appBinMd5          = $AppBinMd5
    factoryOffset      = "0x10000"
    builtAt            = (Get-Date).ToUniversalTime().ToString("o")
}
$MetadataPath = Join-Path $BuildDir "build-metadata.json"
$utf8NoBom = New-Object System.Text.UTF8Encoding $false
[System.IO.File]::WriteAllText($MetadataPath, ($Metadata | ConvertTo-Json -Depth 4), $utf8NoBom)

Write-Host "Firmware built: $OutBin"
Write-Host "Metadata: $MetadataPath"
Write-Host "Size: $((Get-Item $OutBin).Length) bytes"
Write-Host "Core: esp32:esp32@$CoreVersion"
Write-Host "IDF:  $($Toolchain.espIdfVersion)"
Write-Host "FQBN: $Fqbn"
