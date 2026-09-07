@echo off
setlocal
cd /d "%~dp0"
where python >nul 2>nul
if %errorlevel%==0 (
  python simulation_app.py --view navigation
) else (
  py -3 simulation_app.py --view navigation
)
endlocal

