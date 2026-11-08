@echo off
cd /d "%~dp0"
if "%~1"=="" (
    echo Arraste o arquivo .zip da captura para cima deste arquivo.
    pause
    exit /b 1
)
for /f "delims=" %%i in ('py -3 -c "import sys; print(sys.executable)"') do set "PY=%%i"
echo Python: %PY%
"%PY%" -c "import numpy, plyfile" 2>nul
if errorlevel 1 (
    echo Instalando bibliotecas Python que faltam...
    "%PY%" -m pip install -r requirements.txt
)
"%PY%" processar.py %*
pause
