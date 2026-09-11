[CmdletBinding()]
param(
    [string]$Artifact = '',
    [ValidateSet('UR', 'HOTPLUG', 'NORMAL')]
    [string]$ConnectMode = 'UR'
)

$ErrorActionPreference = 'Stop'
$projectRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..')).Path
if ([string]::IsNullOrWhiteSpace($Artifact)) {
    $Artifact = Join-Path $projectRoot 'output\firmware\ChallengeCup_Main.hex'
}
$resolvedArtifact = (Resolve-Path -LiteralPath $Artifact).Path
$extension = [IO.Path]::GetExtension($resolvedArtifact).ToLowerInvariant()
if ($extension -notin @('.hex', '.bin', '.elf')) {
    throw 'Artifact must be an .hex, .bin, or .elf file.'
}

$defaultProgrammer =
    'C:\ST\STM32CubeCLT_1.22.0\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe'
if (Test-Path -LiteralPath $defaultProgrammer) {
    $programmer = $defaultProgrammer
} else {
    $programmer = (Get-Command STM32_Programmer_CLI.exe -ErrorAction Stop).Source
}

$arguments = @('-c', "port=SWD mode=$ConnectMode reset=HWrst", '-w', $resolvedArtifact)
if ($extension -eq '.bin') {
    $arguments += '0x08000000'
}
$arguments += @('-v', '-rst')

Write-Host "Flashing $resolvedArtifact over ST-LINK/SWD"
& $programmer @arguments
if ($LASTEXITCODE -ne 0) {
    throw "STM32CubeProgrammer failed with exit code $LASTEXITCODE"
}
