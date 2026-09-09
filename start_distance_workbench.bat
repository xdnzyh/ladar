@echo off
setlocal
cd /d "%~dp0CCD_Distance_App_v1_2\source"
python app.py
if errorlevel 1 py -3 app.py
endlocal
