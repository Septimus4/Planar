# -*- mode: python ; coding: utf-8 -*-
"""
PyInstaller spec file for Planar Desktop Controller.

This builds a single-file executable for Windows that bundles:
- Desktop controller CLI (desktop.controller)
- Desktop client library (desktop.client)
- All required dependencies

Usage:
    pyinstaller planar-desktop.spec
"""

import sys
from PyInstaller.utils.hooks import collect_data_files, collect_submodules

block_cipher = None

# Collect all data files from the package
datas = []

# Collect hidden imports
hiddenimports = [
    'desktop',
    'desktop.controller',
    'desktop.client',
    'requests',
    'websocket',
]

a = Analysis(
    ['desktop/controller.py'],
    pathex=[],
    binaries=[],
    datas=datas,
    hiddenimports=hiddenimports,
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludes=[
        # Exclude capture-specific modules (Raspberry Pi only)
        'capture',
        'pyserial',
        'smbus2',
        'aiohttp',
        # Exclude processing-specific modules
        'processing',
        'open3d',
        'scikit-learn',
        # Exclude simulation
        'simulation',
        # Exclude test modules
        'pytest',
        'pytest_cov',
        'pytest_asyncio',
    ],
    win_no_prefer_redirects=False,
    win_private_assemblies=False,
    cipher=block_cipher,
    noarchive=False,
)

pyz = PYZ(a.pure, a.zipped_data, cipher=block_cipher)

exe = EXE(
    pyz,
    a.scripts,
    a.binaries,
    a.zipfiles,
    a.datas,
    [],
    name='planar-desktop',
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=True,
    upx_exclude=[],
    runtime_tmpdir=None,
    console=True,
    disable_windowed_traceback=False,
    argv_emulation=False,
    target_arch=None,
    codesign_identity=None,
    entitlements_file=None,
    icon=None,  # Add icon path if available: 'resources/icon.ico'
)
