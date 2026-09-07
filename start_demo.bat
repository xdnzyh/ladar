@echo off
setlocal
cd /d "%~dp0"
where python >nul 2>nul
if %errorlevel%==0 (
  python radar_app.py --demo
) else (
  py -3 radar_app.py --demo
)
endlocal
