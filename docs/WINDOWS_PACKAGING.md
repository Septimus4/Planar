# Windows Packaging Guide

This guide explains how to create Windows installer packages (MSIX or MSI) for the Planar Desktop Controller and distribute them via GitHub Releases.

## Quick Start for Windows 11 Users

If you're on Windows 11 and just want to get started:

1. **Download the pre-built executable**:
   - Visit [GitHub Releases](https://github.com/Septimus4/Planar/releases)
   - Download `planar-desktop-windows-<version>.zip`
   - Extract and run `planar-desktop.exe`

2. **Or build from source**:
   ```powershell
   git clone https://github.com/Septimus4/Planar.git
   cd Planar
   .\scripts\windows\build-desktop.ps1
   ```

3. **Connect to your Raspberry Pi**:
   ```cmd
   planar-desktop.exe --host YOUR_PI_IP status
   ```

For creating MSIX installers (modern Windows 11 format), continue reading below.

## Overview

The Planar Desktop Controller can be packaged as a standalone Windows installer using the following workflow:

1. **PyInstaller**: Bundle Python application into a single executable
2. **WiX Toolset or MSIX Packaging Tool**: Create MSI or MSIX installer
3. **GitHub Actions**: Automate builds and attach installers to releases

## Prerequisites

### For Building Executable

- **Python 3.10 or later** - [Download from python.org](https://www.python.org/downloads/)
- **PyInstaller** - Installed automatically by build script
- **Git** - For cloning the repository

### For Creating Installers

Choose one of the following:

#### Option A: MSIX Installer (Recommended for Windows 11)

- **Windows 10/11** - MSIX packaging is built into Windows
- **MSIX Packaging Tool** - [Download from Microsoft Store](https://www.microsoft.com/store/productId/9N5LW3JBCXKF)
- **Code Signing Certificate** - Optional but recommended for distribution

#### Option B: MSI Installer (Traditional)

- **WiX Toolset v3.11+** - [Download from wixtoolset.org](https://wixtoolset.org/releases/)
- **Visual Studio** - Optional, but helpful for WiX development
- **Code Signing Certificate** - Optional but recommended for distribution

## Step 1: Build the Executable

### Using the Build Script (Recommended)

**Command Prompt:**
```cmd
cd Planar
scripts\windows\build-desktop.bat
```

**PowerShell:**
```powershell
cd Planar
.\scripts\windows\build-desktop.ps1
```

The executable will be created at: `dist\planar-desktop.exe`

### Manual Build

If you prefer to build manually:

```cmd
# Install dependencies
python -m pip install pyinstaller
python -m pip install -e .[desktop]

# Run PyInstaller
pyinstaller planar-desktop.spec --clean --noconfirm
```

### Verify the Build

Test the executable:

```cmd
dist\planar-desktop.exe --help
dist\planar-desktop.exe discover
```

## Step 2: Create Windows Installer

### Option A: Create MSIX Package

MSIX is the modern Windows packaging format, recommended for Windows 10/11 users.

#### Using MSIX Packaging Tool (GUI)

1. **Launch MSIX Packaging Tool** from the Start menu

2. **Select "Application package"** as the package type

3. **Choose "Create package on this computer"**

4. **Prepare packaging environment:**
   - Ensure the machine is clean (no pending reboots)
   - Disable Windows Defender temporarily
   - Close all unnecessary applications

5. **Select installer:**
   - Choose "I don't have an installer" (since we have a single executable)
   - Browse to `dist\planar-desktop.exe`

6. **Fill in package information:**
   - **Package name**: `Planar.DesktopController`
   - **Publisher**: Your organization name
   - **Version**: Match your release version (e.g., `0.1.0.0`)
   - **Package display name**: `Planar Desktop Controller`
   - **Publisher display name**: Your organization name

7. **Configure installation:**
   - Set installation location: `C:\Program Files\Planar\`
   - Add Start menu shortcut
   - Set working directory

8. **Select capabilities:**
   - `Internet (Client)` - Required for connecting to Raspberry Pi
   - `Private Networks (Client & Server)` - For local network discovery

9. **Review and create package**

10. **Sign the package** (if you have a code signing certificate)

#### Using Command Line (Advanced)

Create an `AppxManifest.xml`:

```xml
<?xml version="1.0" encoding="utf-8"?>
<Package xmlns="http://schemas.microsoft.com/appx/manifest/foundation/windows10"
         xmlns:uap="http://schemas.microsoft.com/appx/manifest/uap/windows10"
         xmlns:rescap="http://schemas.microsoft.com/appx/manifest/foundation/windows10/restrictedcapabilities">
  <Identity Name="Planar.DesktopController"
            Publisher="CN=YourPublisher"
            Version="0.1.0.0"
            ProcessorArchitecture="x64" />
  <Properties>
    <DisplayName>Planar Desktop Controller</DisplayName>
    <PublisherDisplayName>Your Organization</PublisherDisplayName>
    <Logo>Assets\StoreLogo.png</Logo>
    <Description>Remote control client for Planar LiDAR capture system</Description>
  </Properties>
  <Resources>
    <Resource Language="en-us" />
  </Resources>
  <Dependencies>
    <TargetDeviceFamily Name="Windows.Desktop" MinVersion="10.0.17763.0" MaxVersionTested="10.0.22000.0" />
  </Dependencies>
  <Capabilities>
    <Capability Name="internetClient" />
    <Capability Name="privateNetworkClientServer" />
  </Capabilities>
  <Applications>
    <Application Id="PlanarDesktop" Executable="planar-desktop.exe" EntryPoint="Windows.FullTrustApplication">
      <uap:VisualElements DisplayName="Planar Desktop Controller"
                          Description="Control Planar capture system from your desktop"
                          BackgroundColor="transparent"
                          Square150x150Logo="Assets\Square150x150Logo.png"
                          Square44x44Logo="Assets\Square44x44Logo.png">
      </uap:VisualElements>
    </Application>
  </Applications>
</Package>
```

Build the MSIX:

```powershell
# Create package layout
New-Item -ItemType Directory -Path package\Assets -Force
Copy-Item dist\planar-desktop.exe package\
Copy-Item AppxManifest.xml package\

# Create MSIX package
MakeAppx pack /d package /p PlanarDesktop-0.1.0.msix /l

# Sign the package (requires certificate)
SignTool sign /fd SHA256 /a /f YourCertificate.pfx /p YourPassword PlanarDesktop-0.1.0.msix
```

### Option B: Create MSI Installer with WiX

WiX is a traditional Windows installer toolkit that creates MSI files.

#### Create WiX Configuration

Create `installer\planar-desktop.wxs`:

```xml
<?xml version='1.0' encoding='windows-1252'?>
<Wix xmlns='http://schemas.microsoft.com/wix/2006/wi'>
  <Product Name='Planar Desktop Controller' 
           Id='*' 
           UpgradeCode='PUT-GUID-HERE'
           Language='1033' 
           Codepage='1252' 
           Version='0.1.0' 
           Manufacturer='Your Organization'>

    <Package Id='*' 
             Keywords='Installer' 
             Description="Planar Desktop Controller Installer"
             Manufacturer='Your Organization' 
             InstallerVersion='200' 
             Languages='1033' 
             Compressed='yes' 
             SummaryCodepage='1252' />

    <Media Id='1' Cabinet='planar.cab' EmbedCab='yes' />

    <Directory Id='TARGETDIR' Name='SourceDir'>
      <Directory Id='ProgramFilesFolder' Name='PFiles'>
        <Directory Id='INSTALLDIR' Name='Planar'>
          <Component Id='MainExecutable' Guid='PUT-GUID-HERE'>
            <File Id='PlanarDesktopEXE' 
                  Name='planar-desktop.exe' 
                  DiskId='1' 
                  Source='dist\planar-desktop.exe' 
                  KeyPath='yes'>
              <Shortcut Id="startmenuPlanar" 
                        Directory="ProgramMenuDir" 
                        Name="Planar Desktop Controller"
                        WorkingDirectory='INSTALLDIR' 
                        Icon="PlanarIcon.exe" 
                        IconIndex="0" 
                        Advertise="yes" />
            </File>
          </Component>
        </Directory>
      </Directory>

      <Directory Id="ProgramMenuFolder" Name="Programs">
        <Directory Id="ProgramMenuDir" Name="Planar">
          <Component Id="ProgramMenuDir" Guid="PUT-GUID-HERE">
            <RemoveFolder Id='ProgramMenuDir' On='uninstall' />
            <RegistryValue Root='HKCU' 
                          Key='Software\[Manufacturer]\[ProductName]' 
                          Type='string' 
                          Value='' 
                          KeyPath='yes' />
          </Component>
        </Directory>
      </Directory>
    </Directory>

    <Feature Id='Complete' Level='1'>
      <ComponentRef Id='MainExecutable' />
      <ComponentRef Id='ProgramMenuDir' />
    </Feature>

    <Icon Id="PlanarIcon.exe" SourceFile="dist\planar-desktop.exe" />

  </Product>
</Wix>
```

**Note**: Replace `PUT-GUID-HERE` with actual GUIDs generated using `guidgen` or an online GUID generator.

#### Build MSI

```cmd
# Compile WiX source
candle.exe installer\planar-desktop.wxs -out installer\planar-desktop.wixobj

# Link and create MSI
light.exe installer\planar-desktop.wixobj -out PlanarDesktop-0.1.0.msi

# Sign the MSI (optional but recommended)
signtool sign /f YourCertificate.pfx /p YourPassword /d "Planar Desktop Controller" PlanarDesktop-0.1.0.msi
```

## Step 3: Test the Installer

### For MSIX

```powershell
# Install
Add-AppxPackage -Path PlanarDesktop-0.1.0.msix

# Test
planar-desktop --help

# Uninstall
Remove-AppxPackage -Package Planar.DesktopController_0.1.0.0_x64__<publisher-hash>
```

### For MSI

```cmd
# Install
msiexec /i PlanarDesktop-0.1.0.msi /qb

# Test
"C:\Program Files\Planar\planar-desktop.exe" --help

# Uninstall
msiexec /x PlanarDesktop-0.1.0.msi /qb
```

## Step 4: Distribute via GitHub Releases

### Manual Upload

1. Go to your repository on GitHub
2. Navigate to **Releases** → **Create a new release**
3. Tag version (e.g., `v0.1.0`)
4. Upload your installer file(s):
   - `PlanarDesktop-0.1.0.msix` or
   - `PlanarDesktop-0.1.0.msi`
5. Add release notes
6. Publish release

### Automated with GitHub Actions

Create `.github/workflows/windows-release.yml`:

```yaml
name: Build Windows Installer

on:
  push:
    tags:
      - 'v*'
  workflow_dispatch:

jobs:
  build:
    runs-on: windows-latest
    
    steps:
    - uses: actions/checkout@v4
    
    - name: Set up Python
      uses: actions/setup-python@v5
      with:
        python-version: '3.11'
    
    - name: Install dependencies
      run: |
        python -m pip install --upgrade pip
        python -m pip install pyinstaller
        python -m pip install -e .[desktop]
    
    - name: Build executable with PyInstaller
      run: |
        pyinstaller planar-desktop.spec --clean --noconfirm
    
    - name: Test executable
      run: |
        dist\planar-desktop.exe --help
    
    - name: Create MSIX package
      run: |
        # Add MSIX packaging steps here
        # This is a placeholder - actual implementation depends on your setup
    
    - name: Upload artifact
      uses: actions/upload-artifact@v4
      with:
        name: planar-desktop-windows
        path: dist/planar-desktop.exe
    
    - name: Create Release
      if: startsWith(github.ref, 'refs/tags/')
      uses: softprops/action-gh-release@v1
      with:
        files: |
          dist/planar-desktop.exe
          PlanarDesktop-*.msix
          PlanarDesktop-*.msi
      env:
        GITHUB_TOKEN: ${{ secrets.GITHUB_TOKEN }}
```

## Code Signing

### Why Sign Your Installer?

- **Trust**: Windows SmartScreen won't flag your installer as untrusted
- **Security**: Users can verify the installer hasn't been tampered with
- **Professionalism**: Shows attention to security and quality

### Getting a Code Signing Certificate

1. **Purchase from a Certificate Authority** (e.g., DigiCert, Sectigo)
2. **For Open Source**: Consider [SignPath Foundation](https://about.signpath.io/product/signpath-foundation) for free code signing
3. **Self-Signed** (testing only): Not recommended for distribution

### Signing Command

```cmd
# Sign with certificate file
signtool sign /f certificate.pfx /p password /fd SHA256 /t http://timestamp.digicert.com PlanarDesktop-0.1.0.msi

# Sign with certificate from store
signtool sign /n "Your Certificate Name" /fd SHA256 /tr http://timestamp.digicert.com /td SHA256 PlanarDesktop-0.1.0.msi
```

## Troubleshooting

### PyInstaller Issues

**Problem**: Missing modules at runtime
- **Solution**: Add missing modules to `hiddenimports` in `planar-desktop.spec`

**Problem**: Executable is too large
- **Solution**: Use `--exclude-module` for unnecessary packages

**Problem**: Antivirus flags executable as malware
- **Solution**: Sign the executable, add exclusions, or submit to antivirus vendors as false positive

### MSIX Issues

**Problem**: Cannot install unsigned MSIX
- **Solution**: Enable Developer Mode in Windows Settings, or sign the package

**Problem**: MSIX won't run after installation
- **Solution**: Check AppxManifest capabilities match required permissions

### MSI Issues

**Problem**: WiX compiler errors
- **Solution**: Verify all GUIDs are unique, check XML syntax

**Problem**: Installer fails with error 2755
- **Solution**: Ensure all referenced files exist in source paths

## Additional Resources

- [PyInstaller Documentation](https://pyinstaller.org/en/stable/)
- [MSIX Packaging Tool Documentation](https://learn.microsoft.com/en-us/windows/msix/packaging-tool/tool-overview)
- [WiX Toolset Tutorial](https://www.firegiant.com/wix/tutorial/)
- [Windows Code Signing Guide](https://learn.microsoft.com/en-us/windows/win32/seccrypto/cryptography-tools)

## Support

For issues with packaging or distribution, please open an issue on GitHub:
https://github.com/Septimus4/Planar/issues
