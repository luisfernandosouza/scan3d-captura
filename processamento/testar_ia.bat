@echo off
cd /d "%~dp0"
set "CAP=%~1"
if "%CAP%"=="" set "CAP=quarto"
for /f "delims=" %%i in ('py -3 -c "import sys; print(sys.executable)"') do set "PY=%%i"
set KMP_DUPLICATE_LIB_OK=TRUE
echo Testando a IA de escala em "%CAP%" (o log vai para ia_log.txt)...
"%PY%" -X faulthandler escala_ia.py "%CAP%" > ia_log.txt 2>&1
echo Codigo de saida: %ERRORLEVEL% >> ia_log.txt
type ia_log.txt
pause
