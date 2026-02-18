# Usage examples:
#   pwsh -File tools/make_tar.ps1
#   pwsh -File tools/make_tar.ps1 -Env esp32-s3-devkitc-1 -Out LuminaGrowX-package.tar
#   pwsh -File tools/make_tar.ps1 -NoFirmware

[CmdletBinding()]
param(
  [string]$Env = "esp32s3",
  [string]$Out = "LuminaGrowX-package.tar",
  [switch]$NoFirmware
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

function Exec($exe, $arguments) {
  Write-Host "» $exe $arguments"
  $p = Start-Process -FilePath $exe -ArgumentList $arguments -NoNewWindow -PassThru -Wait
  if ($p.ExitCode -ne 0) {
    throw "Command failed ($exe) with code $($p.ExitCode)"
  }
}

$root = Split-Path -Parent $PSScriptRoot
$releaseDir = Join-Path $root 'release'
$stagingDir = Join-Path $root 'temp_staging'
$wwwDir = Join-Path $stagingDir 'www'

Write-Host "==> Preparing staging folder" -ForegroundColor Cyan
Remove-Item -Recurse -Force $stagingDir -ErrorAction SilentlyContinue | Out-Null
New-Item -ItemType Directory -Path $stagingDir | Out-Null
New-Item -ItemType Directory -Path $wwwDir | Out-Null

if (!(Test-Path $releaseDir)) {
  New-Item -ItemType Directory -Path $releaseDir | Out-Null
}

# Copy Web assets
$dataDir = Join-Path $root 'data'
if (Test-Path $dataDir) {
  Write-Host "==> Copy web assets from 'data/'" -ForegroundColor Cyan
  Copy-Item (Join-Path $dataDir '*') $wwwDir -Recurse -Force
} else {
  Write-Warning "data/ not found; package will contain only firmware (if present)"
}

# Copy CHANGELOG.md to www
$changelogPath = Join-Path $root 'CHANGELOG.md'
if (Test-Path $changelogPath) {
  Write-Host "==> Include CHANGELOG.md" -ForegroundColor Cyan
  Copy-Item $changelogPath (Join-Path $wwwDir 'CHANGELOG.md') -Force
}

$fwIncluded = $false
if (-not $NoFirmware) {
  $fwPath = Join-Path $root (".pio/build/{0}/firmware.bin" -f $Env)
  if ($true) {
    Write-Host "==> Building firmware for '$Env' (pio run)" -ForegroundColor Cyan
    $pioExe = "$env:USERPROFILE\.platformio\penv\Scripts\pio.exe"
    if (!(Test-Path $pioExe)) { $pioExe = "pio" }
    Exec $pioExe "run -e $Env"
  }
  if (Test-Path $fwPath) {
    Write-Host "==> Include firmware.bin" -ForegroundColor Cyan
    Copy-Item $fwPath (Join-Path $stagingDir 'firmware.bin') -Force
    $fwIncluded = $true
  } else {
    Write-Warning "firmware.bin not found; continuing without firmware"
  }
}

# Create TAR
$outTar = Join-Path $releaseDir 'LuminaGrowX_Update.tar'
if (Test-Path $outTar) { Remove-Item $outTar -Force -ErrorAction SilentlyContinue }

Write-Host "==> Creating TAR: $outTar" -ForegroundColor Cyan
$tarCmd = Get-Command tar -ErrorAction SilentlyContinue
if ($tarCmd) {
  # Wir entfernen Out-Null, um Fehler besser zu sehen
  if ($fwIncluded) {
    & tar -cvf $outTar -C $stagingDir firmware.bin www
  } else {
    & tar -cvf $outTar -C $stagingDir www
  }
} else {
  $seven = Get-Command 7z -ErrorAction SilentlyContinue
  if (-not $seven) { throw "Neither 'tar' nor '7z' found in PATH" }
  Push-Location $stagingDir
  try {
    if ($fwIncluded) {
      & 7z a -ttar $outTar firmware.bin www
    } else {
      & 7z a -ttar $outTar www
    }
  } finally { Pop-Location }
}

# Cleanup staging (mit kurzer Verzögerung und Fehlertoleranz)
Start-Sleep -Milliseconds 500
Remove-Item -Recurse -Force $stagingDir -ErrorAction SilentlyContinue

if (-not (Test-Path $outTar)) { throw "TAR not created: $outTar" }
$size = (Get-Item $outTar).Length
Write-Host ("==> Done: LuminaGrowX_Update.tar ({0:N0} bytes)" -f $size) -ForegroundColor Green

