# Planar# Planar

PLANAR is a tripod-based 2D indoor scanning system that captures precise floor plans using stationary LiDAR scans. It combines mechanical leveling, optional IMU yaw hinting, and automated scan registration to export clean CAD-ready DXF wall layouts without SLAM or complex robotics.

**PLANAR** is a tripod-based 2D indoor scanning system that captures precise floor plans using stationary LiDAR scans. It combines mechanical leveling, optional IMU yaw hinting, and automated scan registration to export clean CAD-ready DXF wall layouts without SLAM or complex robotics.

## Features

- 📡 **RPLidar S3 Integration** - High-resolution 2D LiDAR scanning at 1Mbaud with Standard scan mode
- 🔄 **BMI160 IMU Support** - 6-axis IMU for yaw reference and orientation tracking
- 🌐 **Remote Control** - Full remote operation from desktop to Raspberry Pi via HTTP/WebSocket
- 📊 **Session Management** - Automatic session recording with LiDAR frames, IMU data, and events
- 🏗️ **DXF Export** - Clean CAD-ready floor plan exports via ezdxf
- 🧪 **Comprehensive Testing** - 122 unit and integration tests with hardware test support

## System Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                          Desktop Client                              │
│  ┌──────────────────┐    ┌──────────────────┐                       │
│  │   Controller     │    │   Preview UI     │                       │
│  │   (desktop/)     │    │   (WebSocket)    │                       │
│  └────────┬─────────┘    └────────┬─────────┘                       │
└───────────┼───────────────────────┼─────────────────────────────────┘
            │ HTTP API              │ WebSocket
            ▼                       ▼
┌─────────────────────────────────────────────────────────────────────┐
│                     Raspberry Pi (Capture Daemon)                    │
│  ┌──────────────────────────────────────────────────────────────┐   │
│  │                    capture/daemon.py                          │   │
│  │  ┌────────────┐  ┌────────────┐  ┌────────────────────────┐  │   │
│  │  │ LiDAR      │  │ IMU        │  │ Server (aiohttp)       │  │   │
│  │  │ Driver     │  │ Driver     │  │ - REST API             │  │   │
│  │  │ (serial)   │  │ (I2C)      │  │ - WebSocket preview    │  │   │
│  │  └─────┬──────┘  └─────┬──────┘  └────────────────────────┘  │   │
│  │        │               │                                      │   │
│  │        ▼               ▼                                      │   │
│  │  ┌──────────────────────────────────────────────────────┐    │   │
│  │  │              Session Storage                          │    │   │
│  │  │  sessions/<id>/                                       │    │   │
│  │  │    ├── metadata.json                                  │    │   │
│  │  │    ├── events.json                                    │    │   │
│  │  │    ├── lidar_station_0.csv                            │    │   │
│  │  │    └── imu_log.csv                                    │    │   │
│  │  └──────────────────────────────────────────────────────┘    │   │
│  └──────────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────────┘
            │
            ▼
┌─────────────────────────────────────────────────────────────────────┐
│                      Processing Pipeline                             │
│  ┌────────────┐  ┌────────────┐  ┌────────────┐  ┌────────────┐    │
│  │ IMU Yaw    │→ │ Station    │→ │ Pose Graph │→ │ Wall       │    │
│  │ Prior      │  │ Reg (ICP)  │  │ Optimize   │  │ Extract    │    │
│  └────────────┘  └────────────┘  └────────────┘  └─────┬──────┘    │
│                                                         │           │
│                                                         ▼           │
│                                                  ┌────────────┐     │
│                                                  │ DXF Export │     │
│                                                  └────────────┘     │
└─────────────────────────────────────────────────────────────────────┘
```

## Hardware Requirements

### Capture Device (Raspberry Pi)
- **Raspberry Pi 4/5** (or compatible SBC)
- **RPLidar S3** - Connected via USB (CP2102 USB-UART bridge)
- **BMI160 IMU** - Connected via I2C (address 0x69)

### Desktop
- Any machine running Python 3.10+ for remote control and processing

## Installation

### Using uv (Recommended)

```bash
# Clone the repository
git clone https://github.com/Septimus4/Planar.git
cd Planar

# Create virtual environment and install dependencies
uv venv
source .venv/bin/activate

# Install for capture daemon (Raspberry Pi)
uv pip install -e ".[capture]"

# Install for desktop client
uv pip install -e ".[desktop]"

# Install for processing pipeline
uv pip install -e ".[processing]"

# Install everything including dev tools
uv pip install -e ".[all]"
```

### Using pip

```bash
# Clone the repository
git clone https://github.com/Septimus4/Planar.git
cd Planar

# Create virtual environment
python -m venv .venv
source .venv/bin/activate

# Install dependencies
pip install -e ".[all]"
```

## Quick Start

### 1. Set up Hardware (Raspberry Pi)

```bash
# Install udev rules for RPLidar
sudo cp provisioning/udev/99-rplidar.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger

# Add user to required groups
sudo usermod -aG dialout,i2c $USER
# Log out and back in for group changes to take effect
```

### 2. Start Capture Daemon (Raspberry Pi)

```bash
# Using the CLI entry point
planar-capture --port 8080 --lidar-device /dev/rplidar --imu-bus 1 --imu-address 0x69

# Or directly
python -m capture.daemon --port 8080
```

### 3. Control from Desktop

```python
from desktop.client import PlanarClient
import asyncio

async def main():
    client = PlanarClient(host="raspberrypi.local", port=8080)
    
    # Get system status
    status = await client.get_status()
    print(f"Daemon running: {status['daemon']['running']}")
    
    # Start LiDAR
    await client.start_lidar()
    
    # Start IMU
    await client.start_imu()
    
    # Begin a capture session
    await client.start_session("my_floor_plan")
    
    # ... move tripod to stations, mark each one ...
    
    # Stop session
    result = await client.stop_session()
    print(f"Captured {result['lidar_frames']} frames, {result['imu_samples']} IMU samples")

asyncio.run(main())
```

### 4. Process Captured Data

```bash
# Run processing pipeline on captured session
planar-process --session sessions/my_floor_plan --output artifacts/

# Or simulate a session for testing
planar-simulate --rooms 3 --output sessions/test_session
```

## Project Structure

```
Planar/
├── capture/                 # Capture daemon (Raspberry Pi)
│   ├── config.py           # Configuration management
│   ├── daemon.py           # Main capture daemon
│   ├── lidar_driver.py     # RPLidar S3 driver
│   ├── imu_driver.py       # BMI160 IMU driver
│   └── server.py           # HTTP/WebSocket server
├── desktop/                 # Desktop client
│   ├── client.py           # Async HTTP/WebSocket client
│   └── controller.py       # CLI controller
├── processing/              # Processing pipeline
│   ├── pipeline.py         # Main processing pipeline
│   └── dxf_exporter.py     # DXF export utilities
├── simulation/              # Synthetic data generation
│   └── generate_synthetic.py
├── tests/                   # Test suite (122 tests)
│   ├── test_config.py
│   ├── test_daemon.py
│   ├── test_lidar_driver.py
│   ├── test_imu_driver.py
│   ├── test_server.py
│   └── test_integration.py
├── docs/                    # Documentation
├── provisioning/            # System setup (udev, systemd)
├── third_party/             # RPLidar SDK
├── pyproject.toml           # Project configuration
└── AGENTS.md                # Agent contracts for automation
```

## API Reference

### Capture Daemon REST API

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/api/health` | GET | Health check |
| `/api/status` | GET | Full system status |
| `/api/lidar/status` | GET | LiDAR diagnostics |
| `/api/lidar/start` | POST | Start LiDAR scanning |
| `/api/lidar/stop` | POST | Stop LiDAR scanning |
| `/api/lidar/configure` | POST | Configure scan mode |
| `/api/imu/status` | GET | IMU diagnostics |
| `/api/imu/start` | POST | Start IMU streaming |
| `/api/imu/stop` | POST | Stop IMU streaming |
| `/api/imu/selftest` | POST | Run IMU self-test |
| `/api/session/list` | GET | List sessions |
| `/api/session/start` | POST | Start new session |
| `/api/session/stop` | POST | Stop current session |
| `/api/config` | GET/POST | Get/set configuration |

### WebSocket Streams

- `/ws/preview` - Real-time LiDAR frame preview
- `/ws/imu` - Real-time IMU samples

## Testing

```bash
# Run all tests
pytest tests/ -v

# Run without hardware tests
pytest tests/ -v -m "not hardware"

# Run with coverage
pytest tests/ --cov=capture --cov=desktop --cov-report=html
```

## Configuration

Configuration can be provided via JSON file or CLI arguments:

```json
{
  "lidar": {
    "device": "/dev/rplidar",
    "baudrate": 1000000,
    "scan_mode": "Standard",
    "enabled": true
  },
  "imu": {
    "i2c_bus": 1,
    "i2c_address": 105,
    "sample_rate_hz": 100,
    "gyro_range": 500,
    "accel_range": 4,
    "enabled": true
  },
  "server": {
    "host": "0.0.0.0",
    "http_port": 8080,
    "ws_port": 8081
  },
  "session": {
    "session_dir": "/var/planar/sessions"
  }
}
```

## Documentation

- [Architecture Overview](docs/ARCHITECTURE.md)
- [Connecting the RPLidar](docs/CONNECT_LIDAR.md)
- [SDK Setup Guide](docs/SDK_SETUP.md)
- [ROS2 Setup (Optional)](docs/ROS2_SETUP.md)
- [Agent Contracts](AGENTS.md)

## Contributing

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Run tests (`pytest tests/ -v`)
4. Commit changes (`git commit -m 'Add amazing feature'`)
5. Push to branch (`git push origin feature/amazing-feature`)
6. Open a Pull Request

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Acknowledgments

- [SLAMTEC](https://www.slamtec.com/) for the RPLidar SDK
- [Bosch Sensortec](https://www.bosch-sensortec.com/) for the BMI160 IMU
- [ezdxf](https://ezdxf.mozman.at/) for DXF export capabilities
