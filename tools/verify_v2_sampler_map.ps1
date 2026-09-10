param(
    [string]$Root = 'E:\项目\强脑手套\V2.0\源码\TACTILE_BrainCo-Glove_V2'
)

$projects = @('plam-hc32-base')
$expectedMasks = @(
    '0x18U, 0x18U, 0x91U, 0x91U, 0x11U, 0xD3U, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU',
    '0x0CU, 0x0CU, 0x08U, 0x49U, 0x49U, 0x48U, 0xFFU, 0x3FU, 0x3FU, 0xFFU, 0xFFU',
    '0x0CU, 0x08U, 0x29U, 0x29U, 0x28U, 0xFFU, 0x3FU, 0x3FU, 0x3FU, 0x00U, 0x00U'
)
$expectedPointCounts = @(57, 49, 49, 49, 37)

function Get-MaskPointCount([string]$Mask) {
    $count = 0
    foreach ($value in [regex]::Matches($Mask, '0x[0-9A-F]+')) {
        $bits = [Convert]::ToInt32($value.Value.Substring(2), 16)
        while ($bits -ne 0) {
            $count += ($bits -band 1)
            $bits = $bits -shr 1
        }
    }
    return $count
}

if (@(Get-MaskPointCount $expectedMasks[0]) +
    @(Get-MaskPointCount $expectedMasks[1]) +
    @(Get-MaskPointCount $expectedMasks[1]) +
    @(Get-MaskPointCount $expectedMasks[1]) +
    @(Get-MaskPointCount $expectedMasks[2]) -join ',' -ne ($expectedPointCounts -join ',')) {
    throw 'Expected V2 point matrices do not add up to 57/49/49/49/37.'
}

foreach ($project in $projects) {
    $sampler = Join-Path $Root "$project\application\sampler.c"
    $header = Join-Path $Root "$project\application\sampler.h"
    if (-not (Test-Path -LiteralPath $sampler) -or
        -not (Test-Path -LiteralPath $header)) {
        throw "$project is missing its sampler source or header."
    }
    $source = Get-Content -Raw -LiteralPath $sampler
    $headerText = Get-Content -Raw -LiteralPath $header

    foreach ($required in @(
        'SAMPLER_MAX_Y_COUNT',
        'm_astcColumnPins',
        'm_au8FingerYCounts',
        'GPIO_PORT_A, GPIO_PIN_00',
        'ADC_CH0',
        'ADC_PGA_PIN_ADC1_PA0',
        'sampler_select_finger',
        'GPIO_PORT_E, GPIO_PIN_01',
        'GPIO_PORT_D, GPIO_PIN_06',
        'GPIO_PORT_D, GPIO_PIN_09',
        'GPIO_PORT_D, GPIO_PIN_08',
        'GPIO_PORT_C, GPIO_PIN_11',
        'GPIO_PORT_C, GPIO_PIN_04',
        'GPIO_PORT_A, GPIO_PIN_05',
        'GPIO_PORT_C, GPIO_PIN_03')) {
        if (-not ($source + $headerText).Contains($required)) {
            throw "$project missing required V2 sampler symbol: $required"
        }
    }

    foreach ($mask in $expectedMasks) {
        if (-not $source.Contains($mask)) {
            throw "$project missing expected point mask: $mask"
        }
    }

    if ($source -match 'm_astcFingerAdcPins' -or $source -match 'm_au8FingerAdcChannels') {
        throw "$project still contains V1 per-finger ADC tables"
    }
}

Write-Output 'V2 sampler map contract passed for the unified hand project.'
