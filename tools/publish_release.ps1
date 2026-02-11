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

# 3. Changelog vorbereiten
$changelogFile = "release/notes.md"
if (!(Test-Path "release")) { New-Item -ItemType Directory "release" | Out-Null }

if (Test-Path "CHANGELOG.md") {
    # Wir lesen die Datei explizit als UTF8
    $content = Get-Content "CHANGELOG.md" -Raw -Encoding UTF8
    
    # Optional: Nur den neuesten Block extrahieren (zwischen ## [V...] und dem nächsten ##)
    if ($content -match "(?s)(## \[$tag\].*?)(?=\r?\n## |$)") {
        $notes = $matches[1]
    } else {
        $notes = $content
    }
    # Speichern als UTF8 ohne BOM (für gh CLI am besten)
    [System.IO.File]::WriteAllLines((Resolve-Path ".").Path + "\$changelogFile", $notes)
} else {
    "LuminaGrowX LisaPro Stable Release $tag" | Out-File $changelogFile -Encoding UTF8
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
# Wir nutzen --notes-file statt --notes, um Encoding-Probleme bei der Argument-Übergabe zu vermeiden
& gh release create $tag "release/LuminaGrowX_Update.tar" "release/LuminaGrowX_Factory.bin" --title "LuminaGrowX $tag" --notes-file $changelogFile

if ($LASTEXITCODE -eq 0) {
    Write-Host "`nERFOLG! Release $tag wurde auf GitHub verÃ¶ffentlicht." -ForegroundColor Green
} else {
    Write-Error "Fehler beim Erstellen des GitHub Releases."
}
