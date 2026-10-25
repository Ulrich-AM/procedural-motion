@echo off
setlocal EnableExtensions
cd /d "%~dp0"

set "LOG=%~dp0build_log.txt"
echo PROC MOTION LAB Release build > "%LOG%"
echo Started %DATE% %TIME% >> "%LOG%"
echo. >> "%LOG%"

where cmake >nul 2>&1
if errorlevel 1 (
  echo CMake was not found on PATH.
  echo Install CMake and Visual Studio with the C++ workload, then run this script again.
  echo CMake was not found on PATH.>> "%LOG%"
  pause
  exit /b 1
)

echo Configuring Release...
echo ===== CONFIGURE =====>> "%LOG%"
cmake -S "%~dp0." -B "%~dp0build" -DCMAKE_BUILD_TYPE=Release >> "%LOG%" 2>&1
if errorlevel 1 (
  type "%LOG%"
  echo.
  echo CONFIGURE FAILED. See build_log.txt
  pause
  exit /b 1
)
echo Configure finished.

echo Building Release...
echo ===== BUILD =====>> "%LOG%"
cmake --build "%~dp0build" --config Release --parallel >> "%LOG%" 2>&1
if errorlevel 1 (
  type "%LOG%"
  echo.
  echo BUILD FAILED. See build_log.txt
  pause
  exit /b 1
)

set "EXE="
if exist "%~dp0build\Release\proc_motion_lab.exe" set "EXE=%~dp0build\Release\proc_motion_lab.exe"
if not defined EXE if exist "%~dp0build\proc_motion_lab.exe" set "EXE=%~dp0build\proc_motion_lab.exe"

if not defined EXE (
  echo The build reported success but proc_motion_lab.exe was not found.
  echo The build reported success but proc_motion_lab.exe was not found.>> "%LOG%"
  echo Looked for:
  echo   %~dp0build\Release\proc_motion_lab.exe
  echo   %~dp0build\proc_motion_lab.exe
  pause
  exit /b 1
)

echo BUILD SUCCEEDED
echo Executable: %EXE%
echo BUILD SUCCEEDED>> "%LOG%"
echo Executable: %EXE%>> "%LOG%"
echo.
echo Tests: build\Release\proc_motion_tests.exe
exit /b 0
