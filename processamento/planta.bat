@echo off
cd /d "%~dp0"
if "%~1"=="" (
    echo Uso: planta.bat NOME_DA_CAPTURA   ^(ex.: planta.bat quarto^)
    pause
    exit /b 1
)
for /f "delims=" %%i in ('py -3 -c "import sys; print(sys.executable)"') do set "PY=%%i"
"%PY%" -c "import numpy, plyfile" 2>nul
if errorlevel 1 "%PY%" -m pip install -r requirements.txt
"%PY%" planta.py %*
pause
