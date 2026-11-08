@echo off
cd /d "%~dp0"
powershell -NoProfile -ExecutionPolicy Bypass -File instalar_ia.ps1
pause
