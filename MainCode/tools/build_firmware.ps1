[CmdletBinding()]
param(
    [ValidateSet('Debug', 'Release')]
    [string]$Configuration = 'Release'
)

$ErrorActionPreference = 'Stop'
$projectRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..')).Path
$cubeCltRoot = 'C:\ST\STM32CubeCLT_1.22.0'

if (Test-Path -LiteralPath $cubeCltRoot) {
    $toolDirectories = @(
        (Join-Path $cubeCltRoot 'GNU-tools-for-STM32\bin'),
        (Join-Path $cubeCltRoot 'Ninja\bin'),
        (Join-Path $cubeCltRoot 'CMake\bin')
    )
    $env:PATH = ($toolDirectories -join [IO.Path]::PathSeparator) +
        [IO.Path]::PathSeparator + $env:PATH
}

$cmake = Get-Command cmake -ErrorAction Stop
$preset = 'firmware-' + $Configuration.ToLowerInvariant()

Push-Location $projectRoot
try {
    & $cmake.Source --preset $preset
    if ($LASTEXITCODE -ne 0) {
        throw "CMake configure failed with exit code $LASTEXITCODE"
    }
    & $cmake.Source --build --preset $preset --parallel
    if ($LASTEXITCODE -ne 0) {
        throw "Firmware build failed with exit code $LASTEXITCODE"
    }

    $buildDirectory = Join-Path $projectRoot `
        (Join-Path 'build' ('main-' + $Configuration.ToLowerInvariant()))
    $outputDirectory = Join-Path $projectRoot 'output\firmware'
    New-Item -ItemType Directory -Force -Path $outputDirectory | Out-Null
    $artifactNames = @(
        'ChallengeCup_Main.elf',
        'ChallengeCup_Main.hex',
        'ChallengeCup_Main.bin',
        'ChallengeCup_Main.map'
    )
    foreach ($name in $artifactNames) {
        Copy-Item -LiteralPath (Join-Path $buildDirectory $name) `
            -Destination (Join-Path $outputDirectory $name) -Force
    }
    Write-Host "Firmware artifacts: $outputDirectory"
} finally {
    Pop-Location
}
