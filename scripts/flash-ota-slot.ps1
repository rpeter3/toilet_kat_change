# Flash an app image to a single OTA slot and set otadata to boot from it.
# Does not rewrite bootloader, partition table, or factory.
#
# Examples:
#   .\scripts\flash-ota-slot.ps1 -Port COM5 -BootSlot ota_0 -AppBin .\build\ota1.bin -OtaSeq 4
#   .\scripts\flash-ota-slot.ps1 -Port COM5 -BootSlot factory

param(
    [string]$Port = "COM5",
    [Parameter(Mandatory = $true)]
    [ValidateSet("ota_0", "ota_1", "factory")]
    [string]$BootSlot,
    [string]$AppBin = "",
    [int]$OtaSeq = 0,
    [switch]$OtadataOnly
)

$ErrorActionPreference = "Stop"
$Root = Split-Path -Parent $PSScriptRoot
$PartitionsCsv = Join-Path $Root "toilet_kat_change\partitions.csv"

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

if ($BootSlot -ne "factory" -and -not $OtadataOnly) {
    if (-not $AppBin) {
        throw "AppBin is required when BootSlot is $BootSlot (unless -OtadataOnly)"
    }
    if (-not (Test-Path $AppBin)) {
        throw "App binary not found: $AppBin"
    }
    $AppBin = (Resolve-Path $AppBin).Path
}

$OtaDataPath = Join-Path $env:TEMP "flash_ota_slot_${BootSlot}_$([Guid]::NewGuid().ToString('N')).bin"
$MakeOtadata = Join-Path $PSScriptRoot "make_otadata.py"

if ($BootSlot -eq "factory") {
    python $MakeOtadata factory $OtaDataPath
} elseif ($OtaSeq -gt 0) {
    python $MakeOtadata $BootSlot $OtaDataPath --seq $OtaSeq
} else {
    python $MakeOtadata $BootSlot $OtaDataPath
}
if ($LASTEXITCODE -ne 0) {
    throw "make_otadata.py failed"
}

$flashArgs = @(
    "--chip", "esp32s3",
    "--port", $Port,
    "--baud", "921600",
    "write-flash",
    "0xE000", $OtaDataPath
)

if ($BootSlot -ne "factory" -and -not $OtadataOnly) {
    $SlotOffset = Get-PartitionOffset $BootSlot
    $flashArgs += ("0x{0:X}" -f $SlotOffset), $AppBin
}

Write-Host "Flashing boot slot $BootSlot on $Port..."
python -m esptool @flashArgs
if ($LASTEXITCODE -ne 0) {
    throw "esptool write-flash failed with exit code $LASTEXITCODE"
}

Write-Host "Resetting device..."
python -m esptool --chip esp32s3 --port $Port run
if ($LASTEXITCODE -ne 0) {
    throw "esptool run failed with exit code $LASTEXITCODE"
}

Write-Host "Done. Boot slot set to $BootSlot."
