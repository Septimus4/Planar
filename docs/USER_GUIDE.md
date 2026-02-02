# Planar User Guide

Complete user documentation for the Planar 2D floor-plan scanning system.

## Table of Contents

1. [Introduction](#introduction)
2. [Hardware Setup](#hardware-setup)
3. [Software Installation](#software-installation)
4. [Quick Start](#quick-start)
5. [Scanning Workflow](#scanning-workflow)
6. [Optimization Guide](#optimization-guide)
7. [Troubleshooting](#troubleshooting)
8. [Quick Reference](#quick-reference)

---

## Introduction

Planar is a tripod-based 2D floor-plan scanner that uses an RPLidar S3 sensor and BMI160 IMU to capture accurate room layouts. The system consists of:

- **Raspberry Pi** running the capture daemon
- **Desktop/Laptop** running the control software and processing pipeline
- **RPLidar S3** for distance measurements
- **BMI160 IMU** for orientation tracking between stations

### How It Works

1. Set up the tripod at a position with good visibility of the room
2. Capture a 360° scan from that position
3. Move the tripod to overlapping positions around the room
4. Process all scans to merge them into a complete floor plan
5. Export to DXF for use in CAD software

---

## Hardware Setup

### Bill of Materials

| Item | Model | Notes |
|------|-------|-------|
| Single-board computer | Raspberry Pi 4 (4GB+) | Pi 5 also works |
| LiDAR sensor | Slamtec RPLidar S3 | 40m range, 32K samples/sec |
| IMU | BMI160 breakout | I2C interface |
| Tripod | Standard camera tripod | Stable, adjustable height |
| USB cable | USB-A to Micro-B | For LiDAR power/data |
| Jumper wires | Female-to-female | For IMU connection |
| Power bank | 5V 3A output | For portable operation |

### Assembly Instructions

#### 1. Prepare the Raspberry Pi

1. Flash Raspberry Pi OS Lite (64-bit) to an SD card
2. Enable SSH and configure WiFi in `boot/` partition:
   ```bash
   # Create empty ssh file to enable SSH
   touch /boot/ssh
   
   # Create wpa_supplicant.conf for WiFi
   cat > /boot/wpa_supplicant.conf << EOF
   country=US
   ctrl_interface=DIR=/var/run/wpa_supplicant GROUP=netdev
   update_config=1
   network={
       ssid="YOUR_WIFI_NAME"
       psk="YOUR_WIFI_PASSWORD"
   }
   EOF
   ```

3. Boot the Pi and SSH into it:
   ```bash
   ssh pi@raspberrypi.local
   # Default password: raspberry
   ```

#### 2. Connect the RPLidar S3

1. Connect the RPLidar to a USB port on the Pi
2. The device will appear as `/dev/ttyUSB0` or similar
3. Create a udev rule for consistent naming:
   ```bash
   sudo cp provisioning/udev/99-rplidar.rules /etc/udev/rules.d/
   sudo udevadm control --reload-rules
   sudo udevadm trigger
   ```
4. The device will now always appear as `/dev/rplidar`

#### 3. Connect the BMI160 IMU

Wire the BMI160 to the Pi's GPIO header:

| BMI160 Pin | Pi GPIO Pin | Pi Physical Pin |
|------------|-------------|-----------------|
| VCC (3.3V) | 3.3V        | Pin 1           |
| GND        | GND         | Pin 6           |
| SDA        | GPIO 2 (SDA)| Pin 3           |
| SCL        | GPIO 3 (SCL)| Pin 5           |
| SDO        | GND         | Pin 9           |

> **Note:** SDO to GND sets the I2C address to 0x68. If SDO is left floating or connected to VCC, the address will be 0x69.

4. Enable I2C on the Pi:
   ```bash
   sudo raspi-config
   # Interface Options → I2C → Enable
   ```

5. Verify the IMU is detected:
   ```bash
   sudo apt install -y i2c-tools
   i2cdetect -y 1
   # Should show device at address 68 or 69
   ```

#### 4. Mount on Tripod

1. 3D print or fabricate a mounting plate for the Pi
2. Mount the LiDAR on top for clear 360° view
3. Mount the IMU securely to avoid vibration
4. Ensure all cables are secured and won't snag

**Mounting Diagram:**

```
    ┌─────────────────┐
    │    RPLidar S3   │  ← Top (clear view)
    └────────┬────────┘
             │
    ┌────────┴────────┐
    │  Raspberry Pi   │  ← Middle
    │  + BMI160 IMU   │
    └────────┬────────┘
             │
    ┌────────┴────────┐
    │    Tripod       │  ← Bottom
    └─────────────────┘
```

#### 5. Power Considerations

- **Tethered:** Use a 5V 3A USB power supply
- **Portable:** Use a power bank with 5V 3A output
- The RPLidar draws significant power (~2W); ensure adequate supply
- Consider a battery level indicator for field work

---

## Software Installation

### Raspberry Pi Setup

1. SSH into the Raspberry Pi:
   ```bash
   ssh pi@raspberrypi.local
   ```

2. Update the system:
   ```bash
   sudo apt update && sudo apt upgrade -y
   ```

3. Install dependencies:
   ```bash
   sudo apt install -y python3-pip python3-venv git i2c-tools
   ```

4. Clone the Planar repository:
   ```bash
   git clone https://github.com/your-repo/planar.git
   cd planar
   ```

5. Create virtual environment and install:
   ```bash
   python3 -m venv .venv
   source .venv/bin/activate
   pip install -e .
   ```

6. Install as a service (optional):
   ```bash
   sudo cp provisioning/systemd/planar-capture.service /etc/systemd/system/
   sudo systemctl daemon-reload
   sudo systemctl enable planar-capture
   sudo systemctl start planar-capture
   ```

### Desktop/Laptop Setup

1. Install Python 3.11+ and Git

2. Clone the repository:
   ```bash
   git clone https://github.com/your-repo/planar.git
   cd planar
   ```

3. Create virtual environment and install:
   ```bash
   python3 -m venv .venv
   source .venv/bin/activate  # On Windows: .venv\Scripts\activate
   pip install -e .
   ```

4. Install optional dependencies for processing:
   ```bash
   pip install open3d  # For faster ICP registration
   ```

---

## Quick Start

### 1. Start the Capture Daemon (on Pi)

```bash
cd planar
source .venv/bin/activate
python -m capture.daemon
```

Or if installed as a service:
```bash
sudo systemctl start planar-capture
```

### 2. Open the Web UI (on Desktop)

1. Open a web browser
2. Navigate to `http://raspberrypi.local:8080`
3. Or open `webui/index.html` and enter the Pi's address

### 3. Connect and Verify

1. Click "Connect" in the web UI
2. Verify LiDAR shows "Connected"
3. Verify IMU shows "Connected"
4. Check the level indicator - adjust tripod until centered

### 4. Capture a Session

1. Click "Start Session"
2. Start both LiDAR and IMU
3. Wait for preview to show room scan
4. Click "Mark Station" to save current scan
5. Move tripod to new position (maintain overlap!)
6. Repeat until room is covered
7. Click "Stop Session"

### 5. Process the Session

```bash
# On desktop
python -m processing.pipeline --session sessions/my_scan --out output/my_scan
```

### 6. View the Result

Open the generated DXF file in any CAD software or online viewer.

---

## Scanning Workflow

### Planning Your Scan

Before starting, walk through the space and plan:

1. **Station Positions:** Choose 4-8 positions around the room perimeter
2. **Overlap:** Ensure at least 30-50% overlap between adjacent stations
3. **Line of Sight:** Each station should see common features (walls, corners)
4. **Obstacles:** Note furniture, pillars, and other obstructions

**Example Station Layout:**

```
    ┌────────────────────────────┐
    │                            │
    │  ①────────────②           │
    │  │            │           │
    │  │   Room     │           │
    │  │            │           │
    │  ④────────────③           │
    │                            │
    └────────────────────────────┘
    
    Arrows show recommended scanning direction
```

### Step-by-Step Capture

#### Station Setup

1. **Position the Tripod**
   - Set height so LiDAR is ~1.0-1.2m above floor
   - Keep away from walls (at least 30cm)
   - Ensure stable footing

2. **Level the System**
   - Use the level indicator in the web UI
   - Adjust tripod legs until bubble is centered
   - Important: Tilt affects scan registration accuracy

3. **Wait for Stability**
   - Let vibrations settle (2-3 seconds)
   - Keep hands off tripod during capture

#### During Capture

1. **Start the Scan**
   - Click "Mark Station" when ready
   - Wait for scan completion (typically 2-3 rotations)
   - Status will show "Station X captured"

2. **Verify Quality**
   - Check preview shows clear walls/features
   - Points should be dense and consistent
   - Low-quality areas (glass, mirrors) are normal

#### Moving Between Stations

1. **Note Current Orientation**
   - IMU tracks rotation during movement
   - Smooth, deliberate movement is best

2. **Transport Carefully**
   - Lift tripod, don't drag
   - Keep sensor assembly level if possible
   - Avoid bumps and sudden movements

3. **Position at Next Station**
   - Overlap with previous station by 30-50%
   - Similar height as previous station
   - Level again before capturing

### Complex Spaces

#### Multi-Room Scanning

For connected rooms:
1. Position stations at doorways (can see both rooms)
2. Capture both rooms visible from doorway
3. Maintain overlap chain through all rooms

#### Handling Obstacles

- **Furniture:** Scan from multiple angles
- **Pillars:** Position stations to see all sides
- **Glass walls:** May need manual verification
- **Mirrors:** Can cause phantom points (ok, filtered in processing)

---

## Optimization Guide

### Quality Settings

#### LiDAR Configuration

| Setting | Default | Range | Effect |
|---------|---------|-------|--------|
| `scan_mode` | Standard | Standard/DenseBoost | Standard more reliable |
| `scan_frequency_hz` | 10 | 5-20 | Higher = faster, less dense |
| `baudrate` | 1000000 | 256000-1000000 | Keep at 1Mbaud |

**Recommendation:** Use default settings for best reliability.

#### IMU Configuration

| Setting | Default | Effect |
|---------|---------|--------|
| `sample_rate_hz` | 100 | Higher = better rotation tracking |
| `gyro_range` | 250 | Keep low for precision |
| `accel_range` | 2 | Keep low for precision |

### Improving Accuracy

#### Station Placement

1. **More Stations = Better**
   - 4-6 stations minimum per room
   - Add extra at long walls or complex areas
   - Corners are excellent registration features

2. **Maximize Overlap**
   - 50% overlap is ideal
   - At least 30% minimum
   - Overlap should include distinct features

3. **Consistent Height**
   - Same height for all stations
   - Cross-sectional plane captures walls accurately
   - Varying heights can cause wall misalignment

#### Environmental Factors

| Factor | Impact | Mitigation |
|--------|--------|------------|
| Sunlight | Can interfere with LiDAR | Close blinds, scan in shade |
| Glass | Transparent to LiDAR | Manual wall addition in CAD |
| Mirrors | Reflection causes phantoms | Mark in post-processing |
| Dark surfaces | Lower reflectivity | May appear as gaps |
| Moving objects | Creates artifacts | Remove people/pets |

### Processing Parameters

Adjust in `processing_config.json`:

```json
{
    "imu": {
        "drift_rate_deg_per_sec": 0.1,
        "bias_estimation_seconds": 2.0
    },
    "registration": {
        "voxel_size_m": 0.02,
        "max_correspondence_distance_m": 0.5,
        "icp_max_iterations": 50
    },
    "wall_extraction": {
        "dbscan_eps_m": 0.1,
        "dbscan_min_samples": 10,
        "ransac_threshold_m": 0.03,
        "min_wall_length_m": 0.5,
        "merge_distance_m": 0.1,
        "merge_angle_deg": 5.0
    }
}
```

**Key Tuning Parameters:**

- `voxel_size_m`: Smaller = finer detail, slower processing
- `max_correspondence_distance_m`: Larger = more forgiving alignment
- `min_wall_length_m`: Larger = ignores short segments (furniture)
- `merge_angle_deg`: Larger = more aggressive wall merging

### Speed Optimization

1. **Reduce Station Count**
   - Minimum needed for coverage
   - Focus on feature-rich areas

2. **Lower Resolution**
   - `voxel_size_m: 0.05` for faster processing
   - Acceptable for room-scale plans

3. **Skip Optional Steps**
   - Disable debug output
   - Skip visualization exports

---

## Troubleshooting

### Connection Issues

#### LiDAR Not Detected

```
Error: Could not open /dev/rplidar
```

**Solutions:**
1. Check USB connection
2. Verify udev rule is installed
3. Check permissions: `sudo chmod 666 /dev/ttyUSB0`
4. Try different USB port

#### IMU Not Detected

```
Error: No I2C device at address 0x69
```

**Solutions:**
1. Check wiring (especially SDA/SCL)
2. Verify I2C enabled: `sudo raspi-config`
3. Check address: `i2cdetect -y 1`
4. Try address 0x68 if SDO is grounded

#### Network Connection Failed

```
WebSocket connection failed
```

**Solutions:**
1. Verify Pi is on network: `ping raspberrypi.local`
2. Check firewall allows port 8080
3. Try IP address instead of hostname
4. Restart capture daemon

### Scan Quality Issues

#### Noisy/Scattered Points

**Causes:**
- Unstable tripod
- Reflective surfaces nearby
- Electrical interference

**Solutions:**
- Use heavier tripod or weight
- Move away from glass/mirrors
- Check for EMI sources

#### Missing Walls

**Causes:**
- Transparent surfaces (glass)
- Dark/absorbing materials
- Out of range (>40m)

**Solutions:**
- Use manual wall addition in CAD
- Add more stations for dark areas
- LiDAR range limit is 40m

#### Registration Failed

**Causes:**
- Insufficient overlap
- All stations too similar (featureless room)
- IMU drift excessive

**Solutions:**
- Add intermediate stations
- Ensure distinct features in overlap
- Check IMU calibration

### Processing Errors

#### Memory Error

```
MemoryError: Unable to allocate array
```

**Solutions:**
- Reduce `voxel_size_m`
- Process fewer stations at once
- Use machine with more RAM

#### ICP Convergence Failed

```
Warning: ICP did not converge
```

**Solutions:**
- Increase `max_correspondence_distance_m`
- Increase `icp_max_iterations`
- Check station overlap

---

## Quick Reference

### Commands

```bash
# Start capture daemon (Pi)
python -m capture.daemon

# Check status (Desktop)
python -m desktop.controller --host PI_IP status

# Start session (Desktop)
python -m desktop.controller --host PI_IP session start my_scan

# Mark station (Desktop)
python -m desktop.controller --host PI_IP session mark

# Stop session (Desktop)
python -m desktop.controller --host PI_IP session stop

# Process session (Desktop)
python -m processing.pipeline --session sessions/my_scan --out output/my_scan
```

### Web UI Shortcuts

| Action | Description |
|--------|-------------|
| Connect | Connect to Pi at specified address |
| Start/Stop LiDAR | Toggle LiDAR scanning |
| Start/Stop IMU | Toggle IMU streaming |
| Start Session | Begin new capture session |
| Mark Station | Capture current scan position |
| Stop Session | End session and save data |

### File Locations

| Path | Contents |
|------|----------|
| `sessions/<name>/` | Captured session data |
| `sessions/<name>/metadata.json` | Session info and config |
| `sessions/<name>/lidar_station_*.csv` | LiDAR point data |
| `sessions/<name>/imu_log.csv` | IMU samples |
| `output/<name>/floor_plan.dxf` | Final floor plan |
| `output/<name>/debug/` | Debug visualizations |

### LED Indicators (RPLidar S3)

| LED State | Meaning |
|-----------|---------|
| Solid green | Connected, idle |
| Blinking green | Scanning |
| Red | Error state |
| Off | No power |

### Checklist Before Scanning

- [ ] Battery charged / power connected
- [ ] Pi booted and daemon running
- [ ] LiDAR connected and responding
- [ ] IMU connected and calibrated
- [ ] Tripod stable and level
- [ ] Room clear of moving objects
- [ ] Station positions planned

### Output Quality Targets

| Metric | Good | Acceptable | Poor |
|--------|------|------------|------|
| Wall accuracy | <2cm | <5cm | >5cm |
| Registration error | <1cm | <3cm | >3cm |
| Overlap between stations | >50% | 30-50% | <30% |
| Points per station | >5000 | 2000-5000 | <2000 |

---

## Support

- **Issues:** GitHub Issues at project repository
- **Documentation:** `/docs` folder in repository
- **Examples:** `/examples` folder for sample sessions

---

*Planar v0.1.0 - Open-source floor plan scanning*
