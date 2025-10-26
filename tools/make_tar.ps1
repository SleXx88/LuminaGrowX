# Usage examples:
#   pwsh -File tools/make_tar.ps1
#   pwsh -File tools/make_tar.ps1 -Env esp32-s3-devkitc-1 -Out LuminaGrowX-package.tar
#   pwsh -File tools/make_tar.ps1 -NoFirmware

[CmdletBinding()]
param(
  [string]$Env = "esp32-s3-devkitc-1",
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
$wwwDir = Join-Path $releaseDir 'www'

Write-Host "==> Preparing release folder" -ForegroundColor Cyan
Remove-Item -Recurse -Force $releaseDir -ErrorAction SilentlyContinue | Out-Null
New-Item -ItemType Directory -Path $releaseDir | Out-Null
New-Item -ItemType Directory -Path $wwwDir | Out-Null

# Copy Web assets
$dataDir = Join-Path $root 'data'
if (Test-Path $dataDir) {
  Write-Host "==> Copy web assets from 'data/'" -ForegroundColor Cyan
  Copy-Item (Join-Path $dataDir '*') $wwwDir -Recurse -Force
} else {
  Write-Warning "data/ not found; package will contain only firmware (if present)"
}

$fwIncluded = $false
if (-not $NoFirmware) {
  $fwPath = Join-Path $root (".pio/build/{0}/firmware.bin" -f $Env)
  if (-not (Test-Path $fwPath)) {
    Write-Host "==> Building firmware for '$Env' (pio run)" -ForegroundColor Cyan
    Exec 'pio' "run -e $Env"
  }
  if (Test-Path $fwPath) {
    Write-Host "==> Include firmware.bin" -ForegroundColor Cyan
    Copy-Item $fwPath (Join-Path $releaseDir 'firmware.bin') -Force
    $fwIncluded = $true
  } else {
    Write-Warning "firmware.bin not found; continuing without firmware"
  }
}

# Create TAR
$outTar = Join-Path $root $Out
if (Test-Path $outTar) { Remove-Item $outTar -Force }

Write-Host "==> Creating TAR: $Out" -ForegroundColor Cyan
$tarCmd = Get-Command tar -ErrorAction SilentlyContinue
if ($tarCmd) {
  if ($fwIncluded) {
    & tar -cvf $outTar -C $releaseDir firmware.bin www | Out-Null
  } else {
    & tar -cvf $outTar -C $releaseDir www | Out-Null
  }
} else {
  $seven = Get-Command 7z -ErrorAction SilentlyContinue
  if (-not $seven) { throw "Neither 'tar' nor '7z' found in PATH" }
  Push-Location $releaseDir
  try {
    if ($fwIncluded) {
      & 7z a -ttar $outTar firmware.bin www | Out-Null
    } else {
      & 7z a -ttar $outTar www | Out-Null
    }
  } finally { Pop-Location }
}

if (-not (Test-Path $outTar)) { throw "TAR not created: $outTar" }
$size = (Get-Item $outTar).Length
Write-Host ("==> Done: {0} ({1:N0} bytes)" -f $Out, $size) -ForegroundColor Green

