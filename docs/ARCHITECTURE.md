# Planar - Architecture Overview

This document describes the software architecture of the Planar floor-plan scanning system.

## System Overview

Planar is a distributed system with three main components:

1. **Capture Daemon** - Runs on Raspberry Pi, manages hardware, records sessions
2. **Desktop Client** - Remote control and monitoring from a desktop/laptop
3. **Processing Pipeline** - Transforms captured data into DXF floor plans

```
┌─────────────────────────────────────────────────────────────────┐
│                        DESKTOP                                   │
│  ┌─────────────────────────────────────────────────────────┐    │
│  │                  Desktop Client                          │    │
│  │  • PlanarClient (async HTTP/WebSocket)                   │    │
│  │  • Controller CLI                                        │    │
│  │  • Preview rendering                                     │    │
│  └─────────────────────────────────────────────────────────┘    │
└───────────────────────────┬─────────────────────────────────────┘
                            │ Network (HTTP/WebSocket)
                            ▼
┌─────────────────────────────────────────────────────────────────┐
│                     RASPBERRY PI                                 │
│  ┌─────────────────────────────────────────────────────────┐    │
│  │                 Capture Daemon                           │    │
│  │  ┌───────────┐ ┌───────────┐ ┌─────────────────────┐    │    │
│  │  │  LiDAR    │ │   IMU     │ │   CaptureServer     │    │    │
│  │  │  Driver   │ │  Driver   │ │   (aiohttp)         │    │    │
│  │  └─────┬─────┘ └─────┬─────┘ │  • REST API         │    │    │
│  │        │             │       │  • WebSocket        │    │    │
│  │        │             │       └─────────────────────┘    │    │
│  │        ▼             ▼                                  │    │
│  │  ┌─────────────────────────────────────────────────┐    │    │
│  │  │             Session Manager                      │    │    │
│  │  │  • LiDAR frame recording                         │    │    │
│  │  │  • IMU sample logging                            │    │    │
│  │  │  • Event timeline                                │    │    │
│  │  │  • Station markers                               │    │    │
│  │  └─────────────────────────────────────────────────┘    │    │
│  └─────────────────────────────────────────────────────────┘    │
│                              │                                   │
│                              ▼                                   │
│  ┌─────────────────────────────────────────────────────────┐    │
│  │                 Session Storage                          │    │
│  │  /var/planar/sessions/<session-id>/                      │    │
│  │    ├── metadata.json                                     │    │
│  │    ├── events.json                                       │    │
│  │    ├── lidar_station_0.csv                               │    │
│  │    └── imu_log.csv                                       │    │
│  └─────────────────────────────────────────────────────────┘    │
└─────────────────────────────────────────────────────────────────┘
```

## Component Details

### Capture Daemon (`capture/`)

The capture daemon is the core runtime component deployed on the Raspberry Pi.

#### Configuration (`config.py`)

Manages all system configuration with dataclass-based schemas:

```python
@dataclass
class LidarConfig:
    device: str = "/dev/rplidar"
    baudrate: int = 1000000
    scan_mode: str = "Standard"
    scan_frequency_hz: float = 10.0
    enabled: bool = True

@dataclass  
class ImuConfig:
    i2c_bus: int = 1
    i2c_address: int = 0x69
    sample_rate_hz: int = 100
    gyro_range: int = 500  # ±500 °/s
    accel_range: int = 4   # ±4g
    enabled: bool = True

@dataclass
class CaptureConfig:
    lidar: LidarConfig
    imu: ImuConfig
    server: ServerConfig
    session: SessionConfig
```

#### LiDAR Driver (`lidar_driver.py`)

Pure Python driver for SLAMTEC RPLidar S3:

- **Protocol**: Binary serial at 1,000,000 baud
- **Scan Mode**: Standard (360° scans at ~10Hz)
- **Data Format**: `ScanFrame` containing `ScanPoint` objects
- **Features**:
  - Async frame callbacks
  - Motor PWM control
  - Health monitoring
  - Express scan protocol support

```python
class ScanPoint:
    angle_deg: float      # 0-360
    distance_m: float     # meters
    quality: int          # signal quality (0-255)
    timestamp: float      # monotonic time

class ScanFrame:
    frame_id: int
    timestamp: float
    scan_rate_hz: float
    points: List[ScanPoint]
```

#### IMU Driver (`imu_driver.py`)

Driver for Bosch BMI160 6-axis IMU:

- **Interface**: I2C (SMBus2)
- **Address**: 0x69 (Gravity BMI160 default)
- **Data Rate**: Up to 400Hz
- **Features**:
  - Gyroscope: ±125 to ±2000 °/s
  - Accelerometer: ±2g to ±16g
  - Built-in self-test
  - Temperature sensor

```python
class ImuSample:
    timestamp: float
    gyro_x: float   # rad/s
    gyro_y: float
    gyro_z: float
    accel_x: float  # m/s²
    accel_y: float
    accel_z: float
    temperature: float  # °C
```

#### Daemon (`daemon.py`)

Orchestrates all components:

```python
class CaptureDaemon:
    # Core components
    config: CaptureConfig
    lidar: RPLidarDriver
    imu: BMI160Driver
    server: CaptureServer
    session: SessionState
    
    # Control methods
    def start_lidar() -> bool
    def stop_lidar()
    def start_imu() -> bool
    def stop_imu()
    def start_session(name: str) -> bool
    def stop_session() -> Dict[str, Any]
    def mark_station() -> int
    def update_config(params: Dict) -> bool
    
    # Lifecycle
    async def run()
    async def shutdown()
```

#### Server (`server.py`)

HTTP/WebSocket server using aiohttp:

**REST API Endpoints:**
| Endpoint | Method | Description |
|----------|--------|-------------|
| `/api/health` | GET | Health check |
| `/api/status` | GET | Full system status |
| `/api/lidar/status` | GET | LiDAR diagnostics |
| `/api/lidar/start` | POST | Start scanning |
| `/api/lidar/stop` | POST | Stop scanning |
| `/api/lidar/configure` | POST | Set scan mode |
| `/api/imu/status` | GET | IMU diagnostics |
| `/api/imu/start` | POST | Start streaming |
| `/api/imu/stop` | POST | Stop streaming |
| `/api/imu/selftest` | POST | Run self-test |
| `/api/session/list` | GET | List sessions |
| `/api/session/start` | POST | Start recording |
| `/api/session/stop` | POST | Stop recording |
| `/api/config` | GET/POST | Configuration |

**WebSocket Endpoints:**
- `/ws/preview` - Real-time LiDAR frame stream
- `/ws/imu` - Real-time IMU sample stream

### Desktop Client (`desktop/`)

#### Client (`client.py`)

Async HTTP/WebSocket client for remote control:

```python
class PlanarClient:
    async def get_status() -> Dict
    async def start_lidar() -> Dict
    async def stop_lidar() -> Dict
    async def start_imu() -> Dict
    async def stop_imu() -> Dict
    async def start_session(name: str) -> Dict
    async def stop_session() -> Dict
    async def get_config() -> Dict
    async def set_config(config: Dict) -> Dict
    
    # WebSocket streams
    async def stream_preview(callback: Callable)
    async def stream_imu(callback: Callable)
```

#### Controller (`controller.py`)

CLI interface for common operations:

```bash
planar-controller connect <host> <port>
planar-controller status
planar-controller start-scan
planar-controller stop-scan
planar-controller session start <name>
planar-controller session stop
```

### Processing Pipeline (`processing/`)

#### Pipeline (`pipeline.py`)

Transforms captured sessions into floor plans:

1. **Load Session** - Parse CSV/JSON files from session directory
2. **IMU Yaw Prior** - Integrate gyro-Z to compute relative yaw between stations
3. **Pairwise Registration** - ICP refinement with coarse correlative initialization
4. **Pose Graph Optimization** - Factor graph with loop closures
5. **Point Cloud Merge** - Transform and combine all station scans
6. **Wall Extraction** - RANSAC line fitting with clustering
7. **DXF Export** - Generate CAD-ready output

#### DXF Exporter (`dxf_exporter.py`)

Creates industry-standard DXF files using ezdxf:

- Wall lines with configurable layer names
- Dimension annotations
- Station markers
- Scale and coordinate system options

### Simulation (`simulation/`)

#### Synthetic Generator (`generate_synthetic.py`)

Creates test sessions for development and CI:

```bash
planar-simulate --rooms 3 --noise 0.01 --output sessions/test
```

Generates:
- Realistic room geometries
- Simulated LiDAR scans with noise
- IMU samples with drift
- Event timeline

## Data Formats

### Session Directory Structure

```
sessions/<session-id>/
├── metadata.json       # Session metadata and config
├── events.json         # Timeline of events
├── lidar_station_0.csv # LiDAR data for station 0
├── lidar_station_1.csv # LiDAR data for station 1
└── imu_log.csv         # Continuous IMU samples
```

### metadata.json

```json
{
  "project": "Planar",
  "created": "2026-02-02T10:30:00Z",
  "name": "office_scan",
  "stations": 5,
  "lidar": {
    "model": "RPLIDAR S3",
    "firmware": "1.02",
    "serial": "ABC123"
  },
  "imu": {
    "chip_id": "0xD1",
    "bus": 1,
    "address": "0x69"
  },
  "config": { ... }
}
```

### events.json

```json
[
  {"type": "session_start", "timestamp": 1706869800.0},
  {"type": "station_captured", "station": 1, "timestamp": 1706869810.0},
  {"type": "station_captured", "station": 2, "timestamp": 1706869850.0},
  {"type": "session_stop", "timestamp": 1706869900.0}
]
```

### lidar_station_N.csv

```csv
timestamp,angle_deg,distance_m,quality
1706869810.001,0.0,2.345,47
1706869810.001,0.5,2.350,48
1706869810.002,1.0,2.348,47
...
```

### imu_log.csv

```csv
timestamp,gyro_x,gyro_y,gyro_z,accel_x,accel_y,accel_z,temperature
1706869800.000,0.001,-0.002,0.015,0.05,-0.03,9.81,25.3
1706869800.010,0.002,-0.001,0.014,0.04,-0.02,9.80,25.3
...
```

## Agent Orchestration

Planar supports automated agent workflows as defined in `AGENTS.md`:

### Agent Roles

- **capture-agent**: Hardware control and session recording
- **sim-agent**: Synthetic session generation
- **processing-agent**: Data processing and DXF export
- **ui-agent**: Preview and monitoring
- **ops-agent**: CI/CD and quality gates

### CLI Contracts

All agents follow consistent CLI patterns:

```bash
# Capture
planar-capture --config config.json --session-dir /var/planar/sessions

# Simulate  
planar-simulate --scene scene.json --output sessions/<id>

# Process
planar-process --session sessions/<id> --output artifacts/<id>
```

### Exit Codes

- `0`: Success (JSON summary written)
- `1`: Fatal error (error details in stderr)
- `2`: Configuration error

## Quality Gates

### Registration Quality

- **Fitness threshold**: Minimum ICP fitness score for edge acceptance
- **Overlap check**: Estimate station overlap before registration
- **Outlier rejection**: RANSAC-based robust estimation

### Data Validation

- **LiDAR health**: Motor status, scan rate verification
- **IMU calibration**: Self-test, bias estimation
- **Session integrity**: File completeness checks

## Testing

The project maintains comprehensive test coverage:

```
tests/
├── test_config.py       # Configuration tests (10 tests)
├── test_daemon.py       # Daemon unit tests (24 tests)
├── test_lidar_driver.py # LiDAR driver tests (18 tests)
├── test_imu_driver.py   # IMU driver tests (29 tests)
├── test_server.py       # Server API tests (27 tests)
├── test_integration.py  # Integration tests (13 tests)
└── test_simulator.py    # Simulator tests (1 test)
```

**Total: 122 tests**

Run tests:
```bash
# All tests
pytest tests/ -v

# Exclude hardware tests
pytest tests/ -v -m "not hardware"

# With coverage
pytest tests/ --cov=capture --cov=desktop
```

## Deployment

### Systemd Service

Install the capture daemon as a system service:

```bash
sudo cp provisioning/systemd/planar-capture.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable planar-capture
sudo systemctl start planar-capture
```

### udev Rules

Install device rules for stable device names:

```bash
sudo cp provisioning/udev/99-rplidar.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

## Future Enhancements

- [ ] Web-based preview UI
- [ ] Multi-floor support
- [ ] Point cloud visualization
- [ ] Automatic room segmentation
- [ ] Door/window detection
- [ ] Export to additional CAD formats
