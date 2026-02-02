# Planar System Description

> **Document Version**: 2.0  
> **Last Updated**: February 2026  
> **Status**: Active Development (V1 Complete, V2 In Progress)

---

## Product

Tripod-based 2D indoor floor-plan scanner built around **RPLIDAR S3**. The system captures a small number of **stationary 360° scans** (one per tripod placement), uses **IMU yaw priors** to stabilize registration, automatically stitches scans into a single global 2D point set, extracts wall lines, and exports **DXF** for AutoCAD/Rhino.

## Operating Mode

* **Stationary scan stations** only.
* Manual tripod repositioning between stations.
* No rolling platform, no autonomous motion, no camera/tag infrastructure.

## Core Constraints Enforced by Design

* Scan plane must remain horizontal across stations (mechanical leveling).
* Overlap between consecutive stations must exist (software overlap guard).
* Registration must be validated and rejected when inconsistent (quality gates).
* Capture must remain reliable even if Wi‑Fi/UI disconnects (Pi-first architecture).

---

# Implementation Status

## Summary

| Component | Status | Notes |
|-----------|--------|-------|
| LiDAR Driver | ✅ Complete | RPLidar S3, Standard mode, 1Mbaud |
| IMU Driver | ✅ Complete | BMI160, I2C, full 6-axis |
| Capture Daemon | ✅ Complete | Full state machine, session management |
| Remote Server | ✅ Complete | HTTP API + WebSocket preview |
| Desktop Client | ✅ Complete | Async client + CLI controller |
| Configuration | ✅ Complete | Dataclass-based, JSON serialization |
| Test Suite | ✅ Complete | 122 tests, hardware tests included |
| Processing Pipeline | 🔄 Partial | Structure in place, needs ICP/pose graph |
| DXF Export | 🔄 Partial | Basic export ready, wall extraction pending |
| Simulator | ✅ Complete | Synthetic session generation |
| Web UI | ⏳ Planned | Currently CLI/API only |

## What We Learned

### LiDAR Findings

1. **Scan Mode**: Use **Standard** mode, not DenseBoost/Express
   - DenseBoost mode didn't respond on our S3 unit (firmware dependent)
   - Standard mode is reliable at ~10Hz with good point density
   - Express scan protocol may require specific firmware versions

2. **Baud Rate**: Confirmed **1,000,000 baud** works reliably via CP2102

3. **Connection**: USB-TTL (CP2102) is the correct approach
   - Avoid GPIO UART for high-speed serial
   - udev rules essential for stable `/dev/rplidar` symlink

4. **Startup**: Device needs motor spin-up time before reliable scans

### IMU Findings

1. **Address**: Gravity BMI160 defaults to **0x69** (not 0x68)
   - Some boards have AD0/SA0 pin for address selection
   - Confirmed chip ID is 0xD1

2. **Self-Test**: Built-in self-test works and validates gyro/accel

3. **I2C**: Standard 100kHz I2C speed is sufficient for 100-400Hz sampling

4. **Permissions**: User must be in `i2c` group (not just `dialout`)

### Architecture Findings

1. **Async Design**: aiohttp-based server works well for concurrent HTTP + WebSocket

2. **Session Format**: JSON metadata + CSV data files is maintainable and debuggable

3. **Configuration**: Dataclass-based config with JSON serialization is clean

4. **Testing**: Hardware tests with `@pytest.mark.hardware` marker allow flexible CI

---

# Build List

## A. Sensor and Compute (Verified Working)

| Item | Specification | Status |
|------|---------------|--------|
| RPLIDAR S3 | 2D 360° TOF, 10Hz, 1Mbaud UART | ✅ Working |
| Raspberry Pi 5 (8GB) | Capture server + storage | ✅ Working |
| UPS HAT + 18650 cells | 5.1V / 5A power stability | ✅ Recommended |
| USB-TTL (CP2102) | S3 kit adapter, 1Mbaud capable | ✅ Working |
| BMI160 IMU | I2C, address 0x69, gyro Z for yaw | ✅ Working |

## B. Mechanical Mounting (Unchanged)

| Item | Notes |
|------|-------|
| Video tripod with leveling bowl | Heavy, stiff legs, center spreader |
| Cheese plate (top deck) | Aluminum, 1/4"-20 / 3/8"-16 points |
| S3 fasteners | 4× M2.5 screws, **≤4mm depth** |
| 2-axis bubble level | Mounted on top deck |
| Electronics shelf | Below head, Pi + UPS |

## C. Scan-Plane Indicator (Unchanged)

| Item | Notes |
|------|-------|
| Laser line module | ~90° fan, class 1/2, 3-12V |
| Laser mount | 9-12mm clamp or L-bracket |
| Laser control | Switch or MOSFET (on in positioning, off in capture) |

## D. Storage and Accessories

| Item | Notes |
|------|-------|
| Storage | NVMe preferred, high-endurance microSD acceptable |
| Status LED/buzzer | Ready / capturing / error feedback |
| Transport | Carry bag + optical window cover |

---

# Software Stack

## Implemented Components

### 1. Capture Module (`capture/`)

```
capture/
├── __init__.py
├── config.py          # ✅ CaptureConfig, LidarConfig, ImuConfig, etc.
├── daemon.py          # ✅ CaptureDaemon state machine
├── lidar_driver.py    # ✅ RPLidarDriver (Standard mode)
├── imu_driver.py      # ✅ BMI160Driver (full 6-axis)
└── server.py          # ✅ CaptureServer (HTTP + WebSocket)
```

**State Machine** (implemented in `daemon.py`):
- `IDLE` → No session active
- Session management via `start_session()` / `stop_session()`
- Station marking via `mark_station()`
- Event recording to `events.json`

**Verified Operational Parameters**:
- LiDAR: Standard scan mode at ~10Hz
- IMU: 100Hz sample rate, ±500°/s gyro, ±4g accel
- Server: aiohttp on configurable port (default 8080)

### 2. Desktop Module (`desktop/`)

```
desktop/
├── __init__.py
├── client.py          # ✅ PlanarClient (async HTTP/WS)
└── controller.py      # ✅ CLI controller
```

**API Coverage**:
- Status polling
- LiDAR/IMU start/stop
- Session management
- Configuration get/set
- WebSocket preview streaming

### 3. Processing Module (`processing/`)

```
processing/
├── __init__.py
├── pipeline.py        # 🔄 Structure in place
└── dxf_exporter.py    # 🔄 Basic export ready
```

**Implemented**:
- Session loading
- Basic coordinate transforms
- DXF file creation with ezdxf

**Pending**:
- IMU yaw prior computation
- ICP registration
- Pose graph optimization
- Wall extraction (RANSAC)

### 4. Simulation Module (`simulation/`)

```
simulation/
└── generate_synthetic.py  # ✅ Synthetic session generation
```

**Implemented**:
- Room geometry generation
- Simulated LiDAR scans with noise
- IMU sample generation
- Session directory structure matching capture format

### 5. Test Suite (`tests/`)

```
tests/
├── conftest.py           # ✅ Fixtures
├── test_config.py        # ✅ 10 tests
├── test_daemon.py        # ✅ 24 tests
├── test_lidar_driver.py  # ✅ 18 tests (3 hardware)
├── test_imu_driver.py    # ✅ 29 tests (7 hardware)
├── test_server.py        # ✅ 27 tests
├── test_integration.py   # ✅ 13 tests (2 hardware)
└── test_simulator.py     # ✅ 1 test
```

**Total**: 122 tests, all passing

---

# REST API Reference

| Endpoint | Method | Status | Description |
|----------|--------|--------|-------------|
| `/api/health` | GET | ✅ | Health check |
| `/api/status` | GET | ✅ | Full system status |
| `/api/lidar/status` | GET | ✅ | LiDAR diagnostics |
| `/api/lidar/start` | POST | ✅ | Start LiDAR scanning |
| `/api/lidar/stop` | POST | ✅ | Stop LiDAR scanning |
| `/api/lidar/configure` | POST | ✅ | Configure scan mode |
| `/api/imu/status` | GET | ✅ | IMU diagnostics |
| `/api/imu/start` | POST | ✅ | Start IMU streaming |
| `/api/imu/stop` | POST | ✅ | Stop IMU streaming |
| `/api/imu/selftest` | POST | ✅ | Run IMU self-test |
| `/api/session/list` | GET | ✅ | List sessions |
| `/api/session/start` | POST | ✅ | Start new session |
| `/api/session/stop` | POST | ✅ | Stop current session |
| `/api/session/mark` | POST | ✅ | Mark station |
| `/api/config` | GET | ✅ | Get configuration |
| `/api/config` | POST | ✅ | Update configuration |

**WebSocket Endpoints**:
- `/ws/preview` - Real-time LiDAR frame preview ✅
- `/ws/imu` - Real-time IMU sample stream ✅

---

# Session Data Format

## Directory Structure

```
sessions/<session-id>/
├── metadata.json         # Session metadata
├── events.json           # Event timeline
├── lidar_station_0.csv   # LiDAR data per station
└── imu_log.csv           # Continuous IMU samples
```

## File Formats

### metadata.json
```json
{
  "project": "Planar",
  "created": "2026-02-02T10:30:00Z",
  "name": "session_name",
  "stations": 3,
  "lidar": {"model": "RPLIDAR S3", "firmware": "1.02"},
  "imu": {"chip_id": "0xD1", "bus": 1, "address": "0x69"},
  "config": { ... }
}
```

### events.json
```json
[
  {"type": "session_start", "timestamp": 1706869800.0},
  {"type": "station_captured", "station": 1, "timestamp": 1706869810.0},
  {"type": "session_stop", "timestamp": 1706869900.0}
]
```

### lidar_station_N.csv
```csv
timestamp,angle_deg,distance_m,quality
1706869810.001,0.0,2.345,47
```

### imu_log.csv
```csv
timestamp,gyro_x,gyro_y,gyro_z,accel_x,accel_y,accel_z,temperature
1706869800.000,0.001,-0.002,0.015,0.05,-0.03,9.81,25.3
```

---

# Functional Scope (Deliverables)

## V0 — Simulation-first ✅ COMPLETE

- [x] Simulator generates synthetic station scans + IMU logs
- [x] Session format matches real capture format
- [x] Test suite validates format compatibility

## V1 — Hardware Capture + Remote Control ✅ COMPLETE

- [x] Pi capture daemon logs real S3 + IMU + events
- [x] Remote HTTP API for control
- [x] WebSocket preview streaming
- [x] Session management (start/stop/list)
- [x] Station marking
- [x] Configuration management
- [x] Comprehensive test suite (122 tests)
- [x] Hardware integration tests

## V2 — Automated Registration 🔄 IN PROGRESS

- [ ] IMU yaw prior computation from gyro Z integration
- [ ] Coarse correlative scan matching
- [ ] ICP refinement
- [ ] Pose graph optimization
- [ ] Quality gating (fitness thresholds, sanity bounds)
- [ ] One-command processing produces merged output

## V3 — Wall Extraction & DXF ⏳ PLANNED

- [ ] Point cloud clustering (DBSCAN)
- [ ] RANSAC line fitting
- [ ] Collinear segment merging
- [ ] Endpoint snapping
- [ ] DXF export with layers (WALLS, DEBUG_POINTS, DEBUG_POSES)

## V4 — UX Hardening ⏳ PLANNED

- [ ] Web-based preview UI
- [ ] Positioning mode (laser on + preview + overlap feedback)
- [ ] Capture mode (laser off + auto-stop after N revolutions)
- [ ] Health checks (power/disk/temp monitoring)
- [ ] Robust reconnect behavior
- [ ] Overlap indicator (GOOD / MARGINAL / BAD)

---

# Implementation Plan (Updated)

## Phase 1 — Mechanical and Electrical ⏳ PENDING

Hardware assembly per original spec (no changes needed).

## Phase 2 — Pi Base Software ✅ COMPLETE

| Task | Status |
|------|--------|
| OS provisioning (Lite 64-bit) | ✅ Done |
| LiDAR driver (1M baud, Standard mode) | ✅ Done |
| IMU driver (I2C, 0x69) | ✅ Done |
| Log format finalization | ✅ Done |
| Atomic writes + session structure | ✅ Done |
| Status polling | ✅ Done |

**Findings**:
- Use Standard scan mode (not DenseBoost)
- BMI160 at 0x69 (not 0x68)
- CP2102 USB-TTL confirmed working

## Phase 3 — Remote Control and Preview ✅ COMPLETE

| Task | Status |
|------|--------|
| REST API server | ✅ Done |
| WebSocket preview stream | ✅ Done |
| Session tooling (list/download) | ✅ Done |
| Async desktop client | ✅ Done |
| CLI controller | ✅ Done |
| Capture continues if UI disconnects | ✅ Done |
| Preview never blocks capture | ✅ Done |

## Phase 4 — Processing Pipeline 🔄 IN PROGRESS

| Task | Status |
|------|--------|
| Parser + normalization | ✅ Done |
| IMU yaw prior | ⏳ Pending |
| Coarse correlative matching | ⏳ Pending |
| ICP refinement | ⏳ Pending |
| Pose graph optimization | ⏳ Pending |
| Quality gates | ⏳ Pending |
| Wall extraction | ⏳ Pending |
| DXF export | 🔄 Basic ready |

## Phase 5 — Field Hardening ⏳ PLANNED

| Task | Status |
|------|--------|
| Operator workflow tuning | ⏳ Pending |
| Error handling (power droop, etc.) | ⏳ Pending |
| Performance validation | ⏳ Pending |
| Acceptance testing | ⏳ Pending |

---

# Operating Defaults (Verified)

| Parameter | Value | Notes |
|-----------|-------|-------|
| Scan frequency | ~10 Hz | Standard mode default |
| Scan mode | Standard | DenseBoost not reliable |
| Baud rate | 1,000,000 | Confirmed working |
| IMU sample rate | 100 Hz | Configurable up to 400 Hz |
| IMU address | 0x69 | Gravity BMI160 default |
| Gyro range | ±500 °/s | Good for tripod rotation |
| Accel range | ±4g | Sufficient for leveling check |
| Preheat | ≥2 minutes | Before trusted measurements |
| Power | 5V stable | UPS recommended |

---

# Quality Gates (To Be Implemented)

| Gate | Threshold | Action |
|------|-----------|--------|
| ICP fitness | < configurable | Reject edge |
| Translation sanity | > max expected | Reject edge |
| Rotation sanity | > max expected | Reject edge |
| Overlap estimate | BAD | Skip registration |
| Power voltage | < 4.8V | Warning |
| Disk space | < 1GB | Warning |

---

# Next Steps

1. **Immediate**: Implement IMU yaw prior computation
2. **Short-term**: Add ICP registration with Open3D
3. **Medium-term**: Pose graph optimization
4. **Medium-term**: Wall extraction pipeline
5. **Long-term**: Web UI for preview and control

---

# References

- [Architecture Overview](docs/ARCHITECTURE.md)
- [Hardware Setup](docs/SDK_SETUP.md)
- [LiDAR Connection](docs/CONNECT_LIDAR.md)
- [Agent Contracts](AGENTS.md)
- [RPLidar S3 Datasheet](https://www.slamtec.com/en/Lidar/S3)
- [BMI160 Datasheet](https://www.bosch-sensortec.com/products/motion-sensors/imus/bmi160/)
