@echo off
echo Starte Factory-Image Erstellung...
powershell -ExecutionPolicy Bypass -File tools\make_factory_bin.ps1
pause
