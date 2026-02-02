# Planar Remote Capture System

This document describes the remote control architecture for the Planar floor-plan scanner.

## Overview

The Planar capture system consists of two main components:

1. **Capture Daemon** (`capture/`) - Runs on the Raspberry Pi
2. **Desktop Controller** (`desktop/`) - Runs on the desktop computer

```
┌─────────────────────┐          ┌─────────────────────┐
│   Desktop Client    │          │   Raspberry Pi      │
│                     │   HTTP   │                     │
│  desktop.controller │◄────────►│  capture.daemon     │
│                     │ WebSocket│                     │
│   - Status monitor  │          │   - RPLidar S3      │
│   - Configuration   │          │   - BMI160 IMU      │
│   - Session control │          │   - Session storage │
└─────────────────────┘          └─────────────────────┘
```

## Hardware Setup

### RPLidar S3
- **Connection**: USB to /dev/rplidar (via udev symlink)
- **Baudrate**: 1,000,000
- **Scan Mode**: DenseBoost (32 kHz sample rate)
- **Scan Frequency**: 10 Hz (configurable)

### BMI160 IMU (Gravity I2C)
- **Connection**: I2C bus 1
- **Address**: 0x69 (default for Gravity module)
- **Chip ID**: 0xD1
- **Sample Rate**: 100 Hz (configurable)

## Installation

### Raspberry Pi Setup

1. Copy the Planar project to the Pi:
   ```bash
   scp -r Planar/ pi@<PI_IP>:/home/pi/planar
   ```

2. Run the setup script:
   ```bash
   cd /home/pi/planar
   sudo ./provisioning/setup_pi.sh
   ```

3. Reboot to enable I2C:
   ```bash
   sudo reboot
   ```

4. Verify hardware:
   ```bash
   python3 scripts/diagnose_hardware.py
   ```

5. Start the service:
   ```bash
   sudo systemctl start planar-capture
   sudo systemctl status planar-capture
   ```

### Desktop Setup

1. Install dependencies:
   ```bash
   pip install requests websocket-client
   ```

2. Verify connection:
   ```bash
   python -m desktop.controller --host <PI_IP> status
   ```

## API Reference

### HTTP Endpoints

| Method | Endpoint | Description |
|--------|----------|-------------|
| GET | `/health` | Health check |
| GET | `/api/status` | Overall system status |
| GET | `/api/lidar/status` | LiDAR diagnostics |
| POST | `/api/lidar/start` | Start scanning |
| POST | `/api/lidar/stop` | Stop scanning |
| POST | `/api/lidar/configure` | Configure LiDAR |
| GET | `/api/imu/status` | IMU diagnostics |
| POST | `/api/imu/start` | Start streaming |
| POST | `/api/imu/stop` | Stop streaming |
| POST | `/api/imu/selftest` | Run self-test |
| POST | `/api/imu/configure` | Configure IMU |
| GET | `/api/session/list` | List sessions |
| POST | `/api/session/start` | Start session |
| POST | `/api/session/stop` | Stop session |
| GET | `/api/config` | Get configuration |
| POST | `/api/config` | Update configuration |

### WebSocket Streams

| Endpoint | Description | Data Format |
|----------|-------------|-------------|
| `/ws/lidar` | Real-time scan frames | `{type: "lidar_frame", data: {points: [...], frame_id, scan_rate_hz}}` |
| `/ws/imu` | Real-time IMU samples | `{type: "imu_sample", data: {gyro: {x,y,z}, accel: {x,y,z}}}` |
| `/ws/events` | System events | `{type: "event", event: "...", data: {...}}` |

## Desktop CLI Usage

```bash
# Check system status
python -m desktop.controller --host 192.168.1.100 status

# Start/stop LiDAR
python -m desktop.controller --host 192.168.1.100 lidar start
python -m desktop.controller --host 192.168.1.100 lidar stop

# Start/stop IMU
python -m desktop.controller --host 192.168.1.100 imu start
python -m desktop.controller --host 192.168.1.100 imu stop

# Run IMU self-test
python -m desktop.controller --host 192.168.1.100 imu selftest

# Configure LiDAR
python -m desktop.controller --host 192.168.1.100 lidar configure --mode DenseBoost --frequency 10

# Configure IMU
python -m desktop.controller --host 192.168.1.100 imu configure --gyro-range 250 --sample-rate 100

# Start a capture session
python -m desktop.controller --host 192.168.1.100 session start my_floor_scan

# Stop session
python -m desktop.controller --host 192.168.1.100 session stop

# List sessions
python -m desktop.controller --host 192.168.1.100 session list

# Interactive mode
python -m desktop.controller --host 192.168.1.100 interactive

# Live monitoring
python -m desktop.controller --host 192.168.1.100 monitor

# Discover devices on network
python -m desktop.controller discover
```

## Python Client API

```python
from desktop.client import PlanarClient

# Connect to Pi
client = PlanarClient("192.168.1.100", port=8080)

# Check connection
if client.ping():
    print("Connected!")

# Get status
status = client.get_status()
print(f"LiDAR: {status['lidar']['status']}")
print(f"IMU: {status['imu']['status']}")

# Configure and start sensors
client.configure_lidar(scan_mode="DenseBoost", scan_frequency_hz=10)
client.configure_imu(sample_rate_hz=100, gyro_range_dps=250)
client.start_lidar()
client.start_imu()

# Start a capture session
client.start_session("floor_scan_001")

# Stream data with callbacks
def on_lidar_frame(frame):
    print(f"Frame {frame['frame_id']}: {frame['point_count']} points")

def on_imu_sample(sample):
    print(f"Gyro Z: {sample['gyro']['z']:.4f} rad/s")

client.on_lidar_frame = on_lidar_frame
client.on_imu_sample = on_imu_sample
client.connect_streams()

# ... capture data ...

# Stop session and sensors
client.stop_session()
client.stop_lidar()
client.stop_imu()
client.close()
```

## Configuration

The capture daemon reads configuration from `/etc/planar/capture.json`:

```json
{
  "lidar": {
    "device": "/dev/rplidar",
    "baudrate": 1000000,
    "scan_mode": "DenseBoost",
    "scan_frequency_hz": 10.0,
    "enabled": true
  },
  "imu": {
    "i2c_bus": 1,
    "i2c_address": 105,
    "sample_rate_hz": 100.0,
    "gyro_range": 250,
    "accel_range": 2,
    "enabled": true
  },
  "server": {
    "host": "0.0.0.0",
    "http_port": 8080,
    "auth_token": null
  },
  "session": {
    "session_dir": "/var/planar/sessions",
    "max_sessions": 100
  }
}
```

## Troubleshooting

### LiDAR Issues

1. **Device not found**
   ```bash
   ls -l /dev/rplidar
   # If missing, check udev rules:
   cat /etc/udev/rules.d/99-rplidar.rules
   sudo udevadm control --reload-rules
   sudo udevadm trigger
   ```

2. **Permission denied**
   ```bash
   sudo usermod -aG dialout $USER
   # Log out and back in
   ```

3. **No response from device**
   - Power cycle the LiDAR
   - Check USB cable
   - Try different baudrate

### IMU Issues

1. **I2C bus not found**
   ```bash
   # Enable I2C
   sudo raspi-config
   # Interface Options -> I2C -> Enable
   sudo reboot
   ```

2. **Device not detected**
   ```bash
   sudo i2cdetect -y 1
   # Should show 69 (or 68 for alternate address)
   ```

3. **Permission denied**
   ```bash
   sudo usermod -aG i2c $USER
   ```

4. **Wrong chip ID**
   - Verify wiring (SDA, SCL, VCC, GND)
   - Check I2C address jumper on module

### Run diagnostics

```bash
# On the Pi:
python3 scripts/diagnose_hardware.py

# From desktop:
python -m desktop.controller --host <PI_IP> imu selftest
```

## Session Data Format

Sessions are stored in `/var/planar/sessions/<session_name>/`:

- `metadata.json` - Session metadata and device info
- `lidar_station_0.csv` - LiDAR scan data (timestamp, angle_deg, range_m, quality)
- `imu_log.csv` - IMU samples (timestamp, gyro_x/y/z, accel_x/y/z)
- `events.json` - Timeline of capture events

This format is compatible with the processing-agent for generating floor plans.
