@echo off
setlocal
cd /d "%~dp0"
where python >nul 2>nul
if %errorlevel%==0 (
  python hardware_app.py --view radar
) else (
  py -3 hardware_app.py --view radar
)
endlocal

