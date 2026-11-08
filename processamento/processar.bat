@echo off
cd /d "%~dp0"
if "%~1"=="" (
    echo Arraste o arquivo .zip da captura para cima deste arquivo.
    pause
    exit /b 1
)
py -3 processar.py "%~1"
pause
