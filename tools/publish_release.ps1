# Skript zum automatischen Bauen und VerÃ¶ffentlichen eines GitHub-Releases
# BenÃ¶tigt: GitHub CLI (gh)

$ErrorActionPreference = "Stop"

# 1. Version aus include/version.h auslesen
$versionFile = "include/version.h"
$versionRaw = Get-Content $versionFile | Select-String -Pattern 'FW_VERSION\s+"([^"]+)"'
if ($versionRaw -match '"([^"]+)"') {
    $tag = $matches[1]
} else {
    Write-Error "Konnte Version nicht in $versionFile finden."
    exit 1
}

Write-Host "==> Starte Release-Prozess fÃ¼r Version $tag" -ForegroundColor Cyan

# 2. Bauen der Pakete
Write-Host "==> Baue Update-Paket (.tar)..." -ForegroundColor Gray
powershell -ExecutionPolicy Bypass -File tools\make_tar.ps1
if ($LASTEXITCODE -ne 0) { exit 1 }

Write-Host "==> Baue Factory-Image (.bin)..." -ForegroundColor Gray
powershell -ExecutionPolicy Bypass -File tools\make_factory_bin.ps1
if ($LASTEXITCODE -ne 0) { exit 1 }

# 3. Changelog vorbereiten (nimmt den neuesten Teil der CHANGELOG.md bis zur nÃ¤chsten Version)
$changelog = "LuminaGrowX LisaPro Stable Release $tag"
if (Test-Path "CHANGELOG.md") {
    # Einfache Extraktion: Alles unter der aktuellen Version bis zur nÃ¤chsten Überschrift
    # In diesem Fall senden wir einfach die ganze Datei oder einen festen Text
    $changelog = Get-Content "CHANGELOG.md" -Raw
}

# 4. GitHub Release erstellen
Write-Host "==> Erstelle GitHub Release $tag..." -ForegroundColor Yellow

# PrÃ¼fen ob gh eingeloggt ist
& gh auth status
if ($LASTEXITCODE -ne 0) {
    Write-Error "Bitte zuerst 'gh auth login' ausfÃ¼hren."
    exit 1
}

# Release erstellen und Dateien hochladen
# -d: Entwurf (Draft), falls du es erst noch checken willst. Entferne -d fÃ¼r direktes Live-gehen.
& gh release create $tag "release/LuminaGrowX_Update.tar" "release/LuminaGrowX_Factory.bin" --title "LuminaGrowX $tag" --notes "$changelog"

if ($LASTEXITCODE -eq 0) {
    Write-Host "`nERFOLG! Release $tag wurde auf GitHub verÃ¶ffentlicht." -ForegroundColor Green
} else {
    Write-Error "Fehler beim Erstellen des GitHub Releases."
}
