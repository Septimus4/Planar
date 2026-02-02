@echo off
REM Build script for Planar Desktop Controller on Windows
REM
REM This script builds a standalone Windows executable using PyInstaller.
REM
REM Prerequisites:
REM   - Python 3.10 or later
REM   - Virtual environment with dependencies installed
REM
REM Usage:
REM   build-desktop.bat

echo ===================================
echo Planar Desktop Controller Builder
echo ===================================
echo.

REM Check if Python is available
python --version >nul 2>&1
if %errorlevel% neq 0 (
    echo ERROR: Python not found in PATH
    echo Please install Python 3.10 or later from https://www.python.org/
    exit /b 1
)

echo [1/5] Installing build dependencies...
python -m pip install --upgrade pip setuptools wheel
python -m pip install pyinstaller

echo.
echo [2/5] Installing desktop dependencies...
python -m pip install -e .[desktop]

echo.
echo [3/5] Running PyInstaller...
pyinstaller planar-desktop.spec --clean --noconfirm

if %errorlevel% neq 0 (
    echo ERROR: PyInstaller build failed
    exit /b 1
)

echo.
echo [4/5] Testing executable...
dist\planar-desktop.exe --help >nul 2>&1
if %errorlevel% neq 0 (
    echo WARNING: Executable test failed, but build completed
) else (
    echo Executable test passed!
)

echo.
echo [5/5] Build complete!
echo.
echo Executable location: dist\planar-desktop.exe
echo Size: 
for %%A in (dist\planar-desktop.exe) do echo   %%~zA bytes
echo.
echo To test the application:
echo   dist\planar-desktop.exe --help
echo   dist\planar-desktop.exe --host PI_IP_ADDRESS status
echo.

exit /b 0
