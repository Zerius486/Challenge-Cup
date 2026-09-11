[CmdletBinding()]
param(
    [ValidateSet('All', 'eRob', 'RobStride', 'Force', 'UDP')]
    [string]$Project = 'All',
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

$cmake = (Get-Command cmake -ErrorAction Stop).Source
$toolchain = Join-Path $projectRoot 'cmake\arm-none-eabi-gcc.cmake'
$allProjects = @(
    [pscustomobject]@{
        Selector = 'eRob'
        Directory = 'erob_motor'
        Artifact = 'eRob_Motor_Test'
    },
    [pscustomobject]@{
        Selector = 'RobStride'
        Directory = 'robstride_motor'
        Artifact = 'RobStride_Motor_Test'
    },
    [pscustomobject]@{
        Selector = 'Force'
        Directory = 'force_sensor'
        Artifact = 'Force_Sensor_Test'
    },
    [pscustomobject]@{
        Selector = 'UDP'
        Directory = 'udp_communication'
        Artifact = 'UDP_Communication_Test'
    }
)

$selectedProjects = if ($Project -eq 'All') {
    $allProjects
} else {
    @($allProjects | Where-Object Selector -eq $Project)
}

foreach ($entry in $selectedProjects) {
    $sourceDirectory = Join-Path $projectRoot `
        (Join-Path 'Projects\ModuleTests' $entry.Directory)
    $buildDirectory = Join-Path $projectRoot `
        (Join-Path 'build' ('standalone-' + $entry.Directory + '-' +
                            $Configuration.ToLowerInvariant()))
    $outputDirectory = Join-Path $projectRoot `
        (Join-Path 'output\standalone_tests' $entry.Directory)

    $configureArguments = @(
        '-S', $sourceDirectory,
        '-B', $buildDirectory,
        '-G', 'Ninja',
        "-DCMAKE_BUILD_TYPE=$Configuration"
    )
    if (-not (Test-Path -LiteralPath (Join-Path $buildDirectory 'CMakeCache.txt'))) {
        $configureArguments += "-DCMAKE_TOOLCHAIN_FILE=$toolchain"
    }
    & $cmake @configureArguments
    if ($LASTEXITCODE -ne 0) {
        throw "CMake configure failed for $($entry.Selector)"
    }
    & $cmake --build $buildDirectory --parallel
    if ($LASTEXITCODE -ne 0) {
        throw "Build failed for $($entry.Selector)"
    }

    New-Item -ItemType Directory -Force -Path $outputDirectory | Out-Null
    $artifactNames = @(
        ($entry.Artifact + '.elf')
        ($entry.Artifact + '.hex')
        ($entry.Artifact + '.bin')
        ($entry.Artifact + '.map')
    )
    foreach ($artifactName in $artifactNames) {
        Copy-Item -LiteralPath (Join-Path $buildDirectory $artifactName) `
            -Destination (Join-Path $outputDirectory $artifactName) -Force
    }
    Write-Host "$($entry.Selector) artifacts: $outputDirectory"
}
