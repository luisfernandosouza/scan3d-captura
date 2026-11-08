@echo off
cd /d "%~dp0"
powershell -NoProfile -ExecutionPolicy Bypass -File instalar.ps1
pause
