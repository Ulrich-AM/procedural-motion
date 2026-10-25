@echo off
setlocal EnableExtensions
cd /d "%~dp0"

set "EXE="
if exist "%~dp0proc_motion_lab.exe" set "EXE=%~dp0proc_motion_lab.exe"
if not defined EXE if exist "%~dp0build\Release\proc_motion_lab.exe" set "EXE=%~dp0build\Release\proc_motion_lab.exe"
if not defined EXE if exist "%~dp0build\proc_motion_lab.exe" set "EXE=%~dp0build\proc_motion_lab.exe"

if not defined EXE (
  echo The PROC MOTION LAB executable was not found.
  echo.
  echo Build the Release configuration first by running:
  echo   build_windows.bat
  echo.
  echo This script looked for:
  echo   %~dp0proc_motion_lab.exe
  echo   %~dp0build\Release\proc_motion_lab.exe
  echo   %~dp0build\proc_motion_lab.exe
  echo.
  pause
  exit /b 1
)

echo Starting %EXE%
echo The working directory is the repository root so assets\ can be found.
"%EXE%"
if errorlevel 1 (
  echo.
  echo The application exited with an error.
  pause
  exit /b 1
)
exit /b 0
