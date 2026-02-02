# Planar Agent Definitions

## Overview

This repository contains the Planar project: a tripod-based 2D floor-plan scanner system built around RPLidar S3 and BMI160 IMU.

## Purpose of this file

This document defines autonomous AI agent roles, responsibilities, and how agents should operate on the repository. It is intended to enable automated agents to run simulations, run processing, and produce DXF artifacts with minimal manual steps.

## Agent roles

### capture-agent
- **Module**: `capture.daemon`
- **Runs on**: Raspberry Pi target
- **Responsibilities**: Read LiDAR & IMU streams, maintain session directories, atomically write station logs and events, publish preview frames over WebSocket, expose REST API for remote control.
- **Inputs**: Device handles (serial `/dev/rplidar`, I2C bus 1 address 0x69), configuration files.
- **Outputs**: Session directory with station scan logs, IMU logs, events, metadata.
- **Status**: ✅ Implemented

### desktop-agent
- **Module**: `desktop.controller`
- **Runs on**: Desktop/laptop (macOS, Linux, Windows)
- **Responsibilities**: Remote control of capture-agent via HTTP API, real-time monitoring via WebSocket, session management, device discovery.
- **Inputs**: Pi host address, user commands.
- **Outputs**: Status display, downloaded session data.
- **Status**: ✅ Implemented

### sim-agent
- **Module**: `simulation.generate_synthetic`
- **Runs on**: Local development machine or CI
- **Responsibilities**: Generate synthetic sessions for development and CI testing.
- **Inputs**: Scene description (rooms, walls), station placements, noise params.
- **Outputs**: Session directory compatible with processing-agent.
- **Status**: ✅ Implemented

### processing-agent
- **Module**: `processing.pipeline`
- **Runs on**: Desktop or CI
- **Responsibilities**: Load session directories, compute IMU yaw priors, register stations, optimize pose graph, extract wall line segments, write DXF.
- **Inputs**: Session directory, processing config (thresholds, gates).
- **Outputs**: Merged point cloud, pose graph, DXF file, debug artifacts.
- **Status**: 🔄 Partial (structure ready, ICP/pose graph pending)

### ui-agent
- **Module**: `webui/`
- **Runs on**: Browser (connects to capture-agent)
- **Responsibilities**: Provide real-time preview rendering and session controls via web UI. Uses preview frames from capture-agent (WebSocket).
- **Status**: ✅ Implemented

### ops-agent
- **Runs on**: CI/CD environment
- **Responsibilities**: Maintain CI, run tests (122 tests), run smoke tests, and deploy artifacts. Ensures quality gates are enforced in pipelines.
- **Status**: 🔄 Partial (tests implemented, CI pipeline pending)

## Data layout

```
sessions/<session-id>/
├── metadata.json         # Device versions, capture config, session info
├── events.json           # Timeline (session_start, station_captured, session_stop)
├── lidar_station_<n>.csv # Polar/Cartesian per sweep (timestamp, angle_deg, distance_m, quality)
└── imu_log.csv           # Full 6-axis samples (timestamp, gyro_xyz, accel_xyz, temperature)
```

## Hardware Configuration

| Device | Interface | Address/Path | Notes |
|--------|-----------|--------------|-------|
| RPLidar S3 | USB-TTL | `/dev/rplidar` | 1Mbaud, **Standard mode** (DenseBoost unreliable) |
| BMI160 IMU | I2C | Bus 1, 0x69 | Chip ID 0xD1, 100Hz sample rate |

## Agent contracts

- All agents must accept CLI flags for input/output directories and config files.
- All agents must return non-zero exit codes on fatal errors.
- All agents should write a machine-readable JSON summary on success to stdout or to a file.
- Capture-agent exposes REST API (see `agents/agents.yaml` for endpoints).

## Running locally

1. **Development**: Start with sim-agent to create synthetic sessions:
   ```bash
   python -m simulation.generate_synthetic --scene scene.json --out sessions/test
   ```

2. **Processing**: Run processing-agent on the produced session:
   ```bash
   python -m processing.pipeline --session sessions/test --out artifacts/test
   ```

3. **Real capture**: Start capture daemon on Pi, control from desktop:
   ```bash
   # On Pi:
   python -m capture.daemon --port 8080
   
   # On desktop:
   python -m desktop.controller --host PI_IP status
   ```

## Related Documentation

- [System Description](SYSTEM.md) - Full system spec with implementation status
- [Architecture](docs/ARCHITECTURE.md) - System architecture details
- [Hardware Setup](docs/SDK_SETUP.md) - LiDAR and IMU setup guide
- [Agent YAML](agents/agents.yaml) - Machine-readable agent definitions
