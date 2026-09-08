@echo off
setlocal
cd /d "%~dp0"
where python >nul 2>nul
if errorlevel 1 (
  py -3 hardware_app.py --view radar
) else (
  python hardware_app.py --view radar
)
if errorlevel 1 pause
endlocal
