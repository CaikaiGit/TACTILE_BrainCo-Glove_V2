param(
    [string]$SourceBin,
    [string]$TargetBin,
    [int]$CropBytes = 31744
)

if ([string]::IsNullOrWhiteSpace($SourceBin) -or [string]::IsNullOrWhiteSpace($TargetBin)) {
    exit 1
}

if (-not (Test-Path $SourceBin)) {
    exit 2
}

$bytes = [System.IO.File]::ReadAllBytes($SourceBin)
if ($bytes.Length -le $CropBytes) {
    exit 3
}

[System.IO.File]::WriteAllBytes($TargetBin, $bytes[$CropBytes..($bytes.Length - 1)])
exit 0
