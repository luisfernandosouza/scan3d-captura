@echo off
cd /d "%~dp0"
if "%~1"=="" (
    echo Arraste o arquivo .zip da captura para cima deste arquivo.
    pause
    exit /b 1
)
py -3 -c "import numpy, plyfile" 2>nul
if errorlevel 1 (
    echo Instalando bibliotecas Python que faltam...
    py -3 -m pip install -r requirements.txt
)
py -3 processar.py "%~1"
pause
