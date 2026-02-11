@echo off
echo ======================================================
echo Starte FULL RELEASE PROZESS (V1.0.0)
echo ======================================================
echo Dieses Skript baut die Firmware und laedt sie auf GitHub hoch.
echo Benötigt: GitHub CLI (gh) installiert und eingeloggt.
echo.
powershell -ExecutionPolicy Bypass -File tools\publish_release.ps1
echo.
pause
