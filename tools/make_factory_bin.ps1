# Skript zum Erstellen eines "Factory Images" (Single Binary) für Web-Flasher
# Kombiniert: Bootloader + Partition Table + BootApp + Firmware + LittleFS

$ErrorActionPreference = "Stop"

# Pfade
$pioBuildDir = ".pio\build\esp32s3"
$releaseDir = "release"
$outFile = "$releaseDir\LuminaGrowX_Factory.bin"

# Python Interpreter von PlatformIO suchen
$python = "$env:USERPROFILE\.platformio\penv\Scripts\python.exe"
if (!(Test-Path $python)) {
    # Fallback: Versuche python3 im Standardpfad
    $python = "$env:USERPROFILE\.platformio\python3\python.exe"
}

# Esptool suchen (in packages/tool-esptoolpy)
$esptoolDir = Resolve-Path "$env:USERPROFILE\.platformio\packages\tool-esptoolpy*" | Select-Object -First 1
if (!$esptoolDir) {
    Write-Error "Konnte esptool-Package nicht finden. Bitte 'pio run' einmal ausführen."
    exit 1
}
$espTool = "$esptoolDir\esptool.py"

# Framework Package suchen (für boot_app0.bin)
$frameworkDir = Resolve-Path "$env:USERPROFILE\.platformio\packages\framework-arduinoespressif32*" | Select-Object -First 1
if (!$frameworkDir) {
    Write-Error "Konnte framework-arduinoespressif32 nicht finden."
    exit 1
}

Write-Host "Python: $python" -ForegroundColor Gray
Write-Host "Esptool: $espTool" -ForegroundColor Gray

# 1. Release Ordner sicherstellen
if (!(Test-Path $releaseDir)) {
    New-Item -ItemType Directory -Force -Path $releaseDir | Out-Null
}

# 1.5 CHANGELOG.md nach data/ kopieren (für buildfs)
Write-Host "Kopiere CHANGELOG.md nach data/..." -ForegroundColor Gray
Copy-Item "CHANGELOG.md" "data/CHANGELOG.md" -Force

Write-Host "1. Baue Firmware..." -ForegroundColor Cyan
& "$env:USERPROFILE\.platformio\penv\Scripts\pio.exe" run
if ($LASTEXITCODE -ne 0) { Write-Error "Build failed"; exit 1 }

Write-Host "2. Baue Dateisystem (LittleFS)..." -ForegroundColor Cyan
& "$env:USERPROFILE\.platformio\penv\Scripts\pio.exe" run -t buildfs
if ($LASTEXITCODE -ne 0) { Write-Error "BuildFS failed"; exit 1 }

# Sicherstellen, dass boot_app0.bin existiert
$bootApp0Src = "$frameworkDir\tools\partitions\boot_app0.bin"
$bootApp0Dst = "$pioBuildDir\boot_app0.bin"
if (!(Test-Path $bootApp0Dst)) {
    Write-Host "Kopiere boot_app0.bin..." -ForegroundColor Gray
    if (Test-Path $bootApp0Src) {
        Copy-Item $bootApp0Src $bootApp0Dst
    } else {
        Write-Error "boot_app0.bin nicht gefunden unter: $bootApp0Src"
        exit 1
    }
}

Write-Host "3. Erstelle Factory Image (Merge)..." -ForegroundColor Cyan

$argsList = @(
    "$espTool",
    "--chip", "esp32s3",
    "merge_bin",
    "-o", "$outFile",
    "--flash_mode", "dio",
    "--flash_freq", "40m",
    "--flash_size", "16MB",
    "0x0000", "$pioBuildDir\bootloader.bin",
    "0x8000", "$pioBuildDir\partitions.bin",
    "0xe000", "$pioBuildDir\boot_app0.bin",
    "0x10000", "$pioBuildDir\firmware.bin",
    "0x418000", "$pioBuildDir\littlefs.bin"
)

& $python $argsList

if ($LASTEXITCODE -eq 0) {
    Write-Host "`nERFOLG! Das Factory-Image liegt hier:" -ForegroundColor Green
    Write-Host "$outFile" -ForegroundColor White
    Write-Host "`nVerwendung mit Adafruit WebSerial ESPTool:" -ForegroundColor Yellow
    Write-Host "1. Öffne https://adafruit.github.io/Adafruit_WebSerial_ESPTool/"
    Write-Host "2. Verbinde den ESP32-S3 (Boot-Button halten, Reset drücken, Boot loslassen)."
    Write-Host "3. Klicke 'Connect'."
    Write-Host "4. Wähle bei Offset '0x0' die Datei 'LuminaGrowX_Factory.bin' aus."
    Write-Host "5. Klicke 'Program'."
} else {
    Write-Error "Merge failed"
}
