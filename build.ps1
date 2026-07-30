# build.ps1 — build (and run) a box3d-minc program.
#
# Usage:
#   ./build.ps1                      # build + run the sample browser
#   ./build.ps1 run <file.mc>        # build + run your own program
#   ./build.ps1 wasm [<file.mc>]     # build + serve in the browser
#   ./build.ps1 wasm -NoRun          # serve without opening the browser
#   ./build.ps1 build [<file.mc>]    # compile only
#   ./build.ps1 clean
#
# A program is ONE compilation unit: `import box3d;` pulls in the
# physics + runtime from lib/, and `import sokol_all;` etc. resolve
# against the minc install when building from this directory.
#
# The minc compiler is taken from $env:MINC, then PATH, then next to
# this script. Install minc from https://minc.dev.

param(
    [Parameter(Position=0)]
    [string]$Command = 'run',
    [Parameter(Position=1)]
    [string]$Source,
    [switch]$NoRun
)

$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $MyInvocation.MyCommand.Path

# Treat a .mc first argument as `run <file>`.
if ($Command -like '*.mc') { $Source = $Command; $Command = 'run' }

if ($Command -eq 'clean') {
    if (Test-Path (Join-Path $root 'build')) {
        Remove-Item -Recurse -Force (Join-Path $root 'build')
    }
    Write-Host 'clean.'
    exit 0
}

# Locate minc: $env:MINC (install dir, or a direct binary path),
# then PATH, then next to this script.
$minc = $env:MINC
if ($minc -and (Test-Path $minc -PathType Container)) { $minc = Join-Path $minc 'minc.exe' }
if (-not $minc) {
    $minc = (Get-Command minc.exe -ErrorAction SilentlyContinue).Source
}
if (-not $minc) { $minc = Join-Path $root 'minc.exe' }
if (-not (Test-Path $minc)) {
    Write-Host ''
    Write-Host 'minc compiler not found.' -ForegroundColor Red
    Write-Host 'Install it:  powershell -c "irm minc.dev/install.ps1 | iex"'
    Write-Host 'or set $env:MINC (see install_minc.md).'
    Write-Host 'See README.md (Quickstart) and LICENSE.md.'
    exit 1
}

if (-not $Source) { $Source = Join-Path $root 'sample_browser\main.mc' }
if (-not [System.IO.Path]::IsPathRooted($Source)) { $Source = Join-Path $root $Source }
# A directory means "<dir>/main.mc".
if (Test-Path $Source -PathType Container) { $Source = Join-Path $Source 'main.mc' }
if (-not (Test-Path $Source)) { Write-Error "no such file: $Source"; exit 1 }
# Name the output after the program's folder when it is a main.mc.
$name = [System.IO.Path]::GetFileNameWithoutExtension($Source)
if ($name -eq 'main') { $name = Split-Path -Leaf (Split-Path -Parent $Source) }

$libDir = Join-Path $root 'lib'
if (-not (Test-Path (Join-Path $libDir 'box3d.mc'))) {
    Write-Error "missing lib\box3d.mc — dist is incomplete"; exit 1
}

$buildDir = Join-Path $root 'build'
if (-not (Test-Path $buildDir)) { New-Item -ItemType Directory -Path $buildDir | Out-Null }

Push-Location $root   # `import box3d;` resolves against ./lib from here
try {
    if ($Command -eq 'wasm') {
        $webDir = Join-Path $buildDir 'web'
        New-Item -ItemType Directory -Force $webDir | Out-Null
        $wasm = Join-Path $webDir "$name.wasm"
        Write-Host "building + serving $name for the web (wasm)..."
        $args = @('run', '--target', 'wasm', $Source, '-o', $wasm)
        if ($NoRun) { $args += '--no-browser' }
        & $minc @args
        exit $LASTEXITCODE
    }

    $exe = Join-Path $buildDir "$name.exe"
    Write-Host "building $name..."
    & $minc $Source -o $exe
    if ($LASTEXITCODE -ne 0 -or -not (Test-Path $exe)) {
        Write-Error 'minc compile failed'; exit 1
    }
    Write-Host "built $exe"
    if ($Command -eq 'run' -and -not $NoRun) { & $exe }
} finally {
    Pop-Location
}
