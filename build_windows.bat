@echo off
setlocal EnableExtensions
cd /d "%~dp0"

where cmake >nul 2>&1
if errorlevel 1 (
  echo CMake was not found on PATH.
  echo Install CMake 3.24 or newer and Visual Studio 2022 or newer with the Desktop C++ workload.
  exit /b 1
)

if exist "build\CMakeCache.txt" goto do_build

call :configure "Visual Studio 17 2022"
if not errorlevel 1 goto do_build
call :configure "Visual Studio 18 2026"
if not errorlevel 1 goto do_build
echo Falling back to the default CMake generator.
call :configure_default
if errorlevel 1 exit /b 1

:do_build
cmake --build build --config Release --parallel
if errorlevel 1 exit /b 1

echo.
echo Build finished.
if exist "build\Release\procedural_motion.exe" (
  echo Run: build\Release\procedural_motion.exe
  echo Test: build\Release\procedural_motion.exe --self-test
) else if exist "build\procedural_motion.exe" (
  echo Run: build\procedural_motion.exe
  echo Test: build\procedural_motion.exe --self-test
) else (
  echo The executable was built. Look under the build directory.
)
exit /b 0

:configure
if exist build rmdir /s /q build
echo Configuring with %~1
cmake -S . -B build -G "%~1" -A x64
exit /b %errorlevel%

:configure_default
if exist build rmdir /s /q build
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
exit /b %errorlevel%
