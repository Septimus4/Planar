# Connecting the RPLidar S3Connecting the RPLIDAR S3 (CP210x USB-to-TTL) and capturing example data

===============================================================

This guide covers connecting and configuring the SLAMTEC RPLidar S3 (with CP210x USB-to-TTL bridge) for use with the Planar capture system.

This note shows where to find the CP210x USB-UART adapter on Linux and how to do a simple functional test and raw capture at the RPLIDAR S3 required serial settings (1,000,000 baud, 8N1).

## Hardware Overview

Hardware identification

| Component | Specification |-----------------------

|-----------|--------------|

| Model | RPLIDAR S3 |If your USB adapter is attached you'll see it in `lsusb`. Example you provided:

| Interface | USB (CP2102 USB-UART Bridge) |

| Baud Rate | 1,000,000 (1Mbaud) |Bus 001 Device 002: ID 10c4:ea60 Silicon Labs CP210x UART Bridge

| Scan Mode | Standard (360° scans) |

| Scan Rate | ~10 Hz |Relevant identifiers:

| Range | 0.1m - 40m |- Vendor ID (VID): 10c4

- Product ID (PID): ea60

## Hardware Identification

Use these to find the kernel device and create stable rules.

### Check USB Connection

Find the kernel device node

```bash---------------------------

# List USB devices

lsusb | grep -i silicon1. Check kernel messages after plugging in:

# Expected: Bus 001 Device 002: ID 10c4:ea60 Silicon Labs CP210x UART Bridge

```   dmesg | tail -n 50 | grep -i cp210



Key identifiers:2. List serial devices produced by USB adapters:

- **Vendor ID (VID)**: `10c4`

- **Product ID (PID)**: `ea60`   ls -l /dev/serial/by-id



### Find Device NodeTypical output contains a symlink such as:



```bash   usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_XXXXXXXX-if00-port0 -> /dev/ttyUSB0

# Check kernel messages after plugging in

dmesg | tail -n 20 | grep -i cp210If `by-id` is empty, fallback to checking ttyUSB/ttyACM:



# List serial devices   ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || true

ls -l /dev/serial/by-id/

# Example: usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_XXXXXXXX-if00-port0 -> /dev/ttyUSB0Persistent udev symlink (recommended)

-----------------------------------

# Or check directly

ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/nullCreate a udev rule so the adapter always appears as `/dev/rplidar_s3`:

```

Create `/etc/udev/rules.d/99-rplidar.rules` (root):

## Setup

  ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="rplidar_s3", MODE="0666"

### 1. Install udev Rules (Recommended)

Then reload rules and replug:

Create a stable device symlink at `/dev/rplidar`:

  sudo udevadm control --reload-rules && sudo udevadm trigger

```bash

# Copy the provided udev ruleThis creates `/dev/rplidar_s3` which you can use in place of `/dev/ttyUSB0`.

sudo cp provisioning/udev/99-rplidar.rules /etc/udev/rules.d/

Permissions

# Reload rules-----------

sudo udevadm control --reload-rules

sudo udevadm triggerIf you get permission denied when opening `/dev/ttyUSB0` or the udev symlink, add your user to the `dialout` group:



# Unplug and replug the LiDAR  sudo usermod -aG dialout $USER

# Verify symlink  # then either logout/login or run:

ls -la /dev/rplidar  newgrp dialout

```

Quick functional test (interactive)

The rule creates `/dev/rplidar` which the Planar drivers use by default.-----------------------------------



### 2. Set PermissionsYou can open the serial port in a terminal to check whether the device responds. The S3 expects 1,000,000 baud, 8N1.



Add your user to the `dialout` group:Use `picocom` (install with `sudo apt install picocom`):



```bash  picocom -b 1000000 /dev/rplidar_s3

sudo usermod -aG dialout $USER

Or use `screen`:

# Either logout/login or run:

newgrp dialout  screen /dev/rplidar_s3 1000000

```

Note: RPLIDAR S3 uses a binary protocol (Slamtec protocol). You will likely see non-printable characters; the point of this step is to confirm the port opens without errors and that activity appears in the port when the lidar is running.

### 3. Verify Connection

Raw capture to file

Quick test with the Planar driver:-------------------



```bashIf you only need to record the raw serial bytes for later decoding or for handing to a vendor SDK, capture directly to a file (will contain binary data):

cd /path/to/Planar

source .venv/bin/activate  sudo cat /dev/rplidar_s3 > /path/to/capture.raw



# Run hardware testsStop with Ctrl-C. This captures an ongoing serial stream until you stop it. Use file timestamps or rotate files for longer sessions.

python -m pytest tests/test_lidar_driver.py -v -m hardware

```Python example: capture raw bytes to file

---------------------------------------

Expected output:

```This small script uses `pyserial` to open the port (1M baud) and write raw frames to a file with timestamps.

tests/test_lidar_driver.py::TestRPLidarHardware::test_hardware_connect_and_get_info PASSED

tests/test_lidar_driver.py::TestRPLidarHardware::test_hardware_scan_frames PASSEDSave as `tools/capture_raw.py` and run inside a virtualenv with `pip install pyserial`:

tests/test_lidar_driver.py::TestRPLidarHardware::test_hardware_scan_data_validity PASSED

``````python

import serial

## Using with Planarimport time



### ConfigurationSERIAL_PORT = '/dev/rplidar_s3'  # or '/dev/ttyUSB0'

BAUD = 1000000

The LiDAR configuration is defined in `capture/config.py`:

outpath = 'capture.raw'

```pythonwith serial.Serial(SERIAL_PORT, BAUD, timeout=1) as s, open(outpath, 'wb') as f:

@dataclass    print('capturing to', outpath)

class LidarConfig:    t_start = time.time()

    device: str = "/dev/rplidar"    try:

    baudrate: int = 1000000        while True:

    scan_mode: str = "Standard"            data = s.read(4096)

    scan_frequency_hz: float = 10.0            if data:

    enabled: bool = True                # prefix each block with a simple timestamp and length header (optional)

```                ts = time.time() - t_start

                # write a short text header then raw bytes

### Starting the Capture Daemon                hdr = f"TS:{ts:.6f}\n".encode('ascii')

                f.write(hdr)

```bash                f.write(data)

# With defaults                f.flush()

planar-capture    except KeyboardInterrupt:

        print('stopped')

# With custom device path

planar-capture --lidar-device /dev/ttyUSB0```



# With debug loggingNotes on protocol and decoding

planar-capture --debug------------------------------

```

- The RPLIDAR S3 uses Slamtec's high-speed protocol (binary frames). Decoding the frames manually is possible but error-prone; prefer using the Slamtec SDK or an existing driver that supports S3's DenseBoost/32k mode.

### Python API- The driver must configure the serial port for 1,000,000 baud and the device must be left on a stable 3.3 V TTL connection (do not extend TTL wiring unnecessarily).



```pythonRecommended sanity checks before capture

from capture.lidar_driver import RPLidarDriver---------------------------------------



# Create driver1. Power: verify stable 5 V supply and that the lidar motor spins without protective shutdown.

driver = RPLidarDriver(port="/dev/rplidar", baudrate=1000000)2. Preheat: allow at least 2 minutes of spinning before trusting measurements (per vendor guidance).

3. Overlap: for multi-station capture, ensure overlapping geometry between stations.

# Connect

if driver.connect():Using the data with the processing pipeline

    print(f"Connected to: {driver.device_info}")-----------------------------------------

    

    # Start scanning with callback- The minimal processing pipeline in this repo (`processing.pipeline`) expects station CSVs of `timestamp,angle_deg,range_m`.

    def on_frame(frame):- If you capture raw bytes, you'll need to decode them into that CSV schema. For quick testing, you can run the vendor SDK or write a small parser that extracts angle+range samples and writes CSV per station.

        print(f"Frame {frame.frame_id}: {len(frame.points)} points")

    Safety and hardware notes

    driver.start_scan(callback=on_frame)-------------------------

    

    # ... do work ...- Use the CP210x USB adapter in USB mode (do not rewire to Pi GPIO for high-speed serial). The CP210x handles 1M baud reliably.

    - Keep TTL voltage at 3.3 V levels and ensure correct wiring to S3's RX/TX pins.

    driver.stop_scan()- Respect the datasheet warning about M2.5 screw depth when mounting the S3.

    driver.disconnect()

```If you want, I can:

- add a small parser that decodes the slamtec binary frames into `lidar_station_*.csv` (requires protocol reference or SDK), or

## Scan Modes- add a systemd service template and udev rules into the repo for Pi provisioning.



The RPLidar S3 supports multiple scan modes. Planar uses **Standard** mode by default:

| Mode | Description | Notes |
|------|-------------|-------|
| Standard | Basic 360° scan | Reliable, recommended |
| Express | Faster sampling | Higher data rate |
| DenseBoost | Dense point cloud | May require specific firmware |
| Sensitivity | Enhanced weak signal detection | Good for reflective surfaces |

**Note**: DenseBoost mode may not work with all firmware versions. The Planar driver automatically falls back to Standard mode if express scan fails.

## Troubleshooting

### Permission Denied

```bash
# Check group membership
groups

# If dialout not listed, add yourself and re-login
sudo usermod -aG dialout $USER
```

### Device Not Found

```bash
# Check if device exists
ls -la /dev/rplidar /dev/ttyUSB*

# Check dmesg for USB errors
dmesg | tail -30 | grep -i usb

# Replug the device
sudo udevadm trigger
```

### USB Timeout Errors

If you see `cp210x: failed set request 0x12 status: -110`:

1. Try a different USB port (prefer USB 2.0 ports)
2. Use a powered USB hub
3. Try a different USB cable
4. Check power supply is adequate

### Motor Not Spinning

- Verify 5V power supply is stable
- Check that motor control is enabled in driver
- Ensure no obstruction in the optical path

### Scan Data Invalid

- Allow 2 minutes for device to warm up
- Check for reflective surfaces causing interference
- Verify distance to objects is within 0.1m - 40m range

## Quick Functional Test

### Using picocom

```bash
sudo apt install picocom
picocom -b 1000000 /dev/rplidar
# You'll see binary data - this confirms the port works
# Exit with Ctrl-A Ctrl-X
```

### Using SDK Utilities

If you've built the RPLidar SDK (see [SDK_SETUP.md](SDK_SETUP.md)):

```bash
# Ultra simple test
sudo ./third_party/rplidar_sdk/output/Linux/Release/ultra_simple \
    --channel --serial /dev/rplidar 1000000

# Simple grabber with ASCII visualization
sudo ./third_party/rplidar_sdk/output/Linux/Release/simple_grabber \
    --channel --serial /dev/rplidar 1000000
```

## Data Format

The Planar driver captures scan data as `ScanFrame` objects:

```python
@dataclass
class ScanPoint:
    angle_deg: float    # 0-360 degrees
    distance_m: float   # Distance in meters
    quality: int        # Signal quality (0-255)
    timestamp: float    # Monotonic timestamp

@dataclass
class ScanFrame:
    frame_id: int
    timestamp: float
    scan_rate_hz: float
    points: List[ScanPoint]
```

Session CSV format (`lidar_station_N.csv`):

```csv
timestamp,angle_deg,distance_m,quality
1706869810.001,0.0,2.345,47
1706869810.001,0.5,2.350,48
...
```

## Safety Notes

- **Power**: Ensure stable 5V supply; insufficient power causes erratic behavior
- **Warm-up**: Allow 2+ minutes before trusting measurements
- **Mounting**: Use M2.5 screws with correct depth per datasheet
- **Eye Safety**: Class 1 laser, but avoid direct eye exposure
- **Ventilation**: Ensure adequate airflow around the unit

## See Also

- [Architecture Overview](ARCHITECTURE.md) - System architecture
- [SDK Setup Guide](SDK_SETUP.md) - Building the RPLidar SDK
- [RPLidar S3 Datasheet](https://www.slamtec.com/en/Lidar/S3) - Official specifications
