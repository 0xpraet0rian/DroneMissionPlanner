@echo off
echo Building DroneMissionPlanner.exe ...
py -m pip install --upgrade pyinstaller pywebview >nul

set ICONARG=
if exist ico.ico set ICONARG=--icon ico.ico

py -m PyInstaller --onefile --noconsole --name DroneMissionPlanner %ICONARG% mission_planner.py

if exist dist\DroneMissionPlanner.exe (
  move /Y dist\DroneMissionPlanner.exe DroneMissionPlanner.exe >nul
  rmdir /S /Q build dist >nul 2>&1
  del /Q DroneMissionPlanner.spec >nul 2>&1
  echo Done: DroneMissionPlanner.exe
) else (
  echo Build failed - check the log above.
)
pause
