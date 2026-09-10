@echo off
setlocal
set "CAR_CONFIG_EXE="
set /a CAR_CONFIG_COUNT=0 >nul
for /d %%D in ("%~dp0control\*") do for %%F in ("%%~fD\*_CONFIG1.exe") do if exist "%%~fF" (
  set "CAR_CONFIG_EXE=%%~fF"
  set /a CAR_CONFIG_COUNT+=1 >nul
)
if not "%CAR_CONFIG_COUNT%"=="1" (
  echo Expected exactly one CONFIG1 executable under control.
  echo Found: %CAR_CONFIG_COUNT%
  pause
  exit /b 1
)
"%CAR_CONFIG_EXE%" %*
set "CAR_CONFIG_RESULT=%ERRORLEVEL%"
if not "%CAR_CONFIG_RESULT%"=="0" (
  echo CONFIG1 exited with error %CAR_CONFIG_RESULT%.
  pause
)
exit /b %CAR_CONFIG_RESULT%
