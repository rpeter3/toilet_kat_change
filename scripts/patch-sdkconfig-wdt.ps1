# Patch an ESP-IDF sdkconfig export so TWDT/INT WDT show as disabled.
param(
    [Parameter(Mandatory = $true)]
    [string]$SdkconfigPath
)

if (-not (Test-Path $SdkconfigPath)) {
    throw "sdkconfig not found: $SdkconfigPath"
}

$replacements = [ordered]@{
    '^CONFIG_ESP_TASK_WDT_EN=y' = 'CONFIG_ESP_TASK_WDT_EN=n'
    '^CONFIG_ESP_INT_WDT=y' = 'CONFIG_ESP_INT_WDT=n'
}

$lines = Get-Content $SdkconfigPath
$out = foreach ($line in $lines) {
    $updated = $line
    foreach ($pattern in $replacements.Keys) {
        if ($line -match $pattern) {
            $updated = $replacements[$pattern]
            break
        }
    }
    $updated
}

Set-Content -Path $SdkconfigPath -Value $out -Encoding utf8

$text = Get-Content $SdkconfigPath -Raw
if ($text -notmatch 'CONFIG_ESP_TASK_WDT_EN=n') {
    throw "patch failed: CONFIG_ESP_TASK_WDT_EN=n not present in $SdkconfigPath"
}
if ($text -notmatch 'CONFIG_ESP_INT_WDT=n') {
    throw "patch failed: CONFIG_ESP_INT_WDT=n not present in $SdkconfigPath"
}

Write-Host "Patched WDT settings in $SdkconfigPath"
