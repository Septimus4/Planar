#!/usr/bin/env pwsh
# Build script for Planar Desktop Controller on Windows
#
# This script builds a standalone Windows executable using PyInstaller.
#
# Prerequisites:
#   - Python 3.10 or later
#   - Virtual environment with dependencies installed
#
# Usage:
#   .\build-desktop.ps1

$ErrorActionPreference = "Stop"

Write-Host "===================================" -ForegroundColor Cyan
Write-Host "Planar Desktop Controller Builder" -ForegroundColor Cyan
Write-Host "===================================" -ForegroundColor Cyan
Write-Host ""

# Check if Python is available
try {
    $pythonVersion = python --version 2>&1
    Write-Host "Using $pythonVersion" -ForegroundColor Green
} catch {
    Write-Host "ERROR: Python not found in PATH" -ForegroundColor Red
    Write-Host "Please install Python 3.10 or later from https://www.python.org/" -ForegroundColor Yellow
    exit 1
}

# Step 1: Install build dependencies
Write-Host "[1/5] Installing build dependencies..." -ForegroundColor Yellow
python -m pip install --upgrade pip setuptools wheel | Out-Null
python -m pip install pyinstaller | Out-Null

# Step 2: Install desktop dependencies
Write-Host ""
Write-Host "[2/5] Installing desktop dependencies..." -ForegroundColor Yellow
python -m pip install -e ".[desktop]" | Out-Null

# Step 3: Run PyInstaller
Write-Host ""
Write-Host "[3/5] Running PyInstaller..." -ForegroundColor Yellow
pyinstaller planar-desktop.spec --clean --noconfirm

if ($LASTEXITCODE -ne 0) {
    Write-Host "ERROR: PyInstaller build failed" -ForegroundColor Red
    exit 1
}

# Step 4: Test executable
Write-Host ""
Write-Host "[4/5] Testing executable..." -ForegroundColor Yellow
$testOutput = & "dist\planar-desktop.exe" --help 2>&1
if ($LASTEXITCODE -ne 0) {
    Write-Host "WARNING: Executable test failed, but build completed" -ForegroundColor Yellow
} else {
    Write-Host "Executable test passed!" -ForegroundColor Green
}

# Step 5: Report results
Write-Host ""
Write-Host "[5/5] Build complete!" -ForegroundColor Green
Write-Host ""
Write-Host "Executable location: dist\planar-desktop.exe" -ForegroundColor Cyan
$exeSize = (Get-Item "dist\planar-desktop.exe").Length
$exeSizeMB = [math]::Round($exeSize / 1MB, 2)
Write-Host "Size: $exeSizeMB MB ($exeSize bytes)" -ForegroundColor Cyan
Write-Host ""
Write-Host "To test the application:" -ForegroundColor White
Write-Host "  .\dist\planar-desktop.exe --help" -ForegroundColor Gray
Write-Host "  .\dist\planar-desktop.exe --host PI_IP_ADDRESS status" -ForegroundColor Gray
Write-Host ""
