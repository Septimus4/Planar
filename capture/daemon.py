"""Planar capture daemon - main orchestration service.

This daemon runs on the Raspberry Pi and:
- Manages LiDAR and IMU sensor connections
- Exposes REST API and WebSocket server for remote control
- Records capture sessions to disk
- Broadcasts real-time sensor data for preview

Usage:
    python -m capture.daemon --config /etc/planar/capture.json
    python -m capture.daemon --session-dir /var/planar/sessions
"""

import argparse
import asyncio
import json
import logging
import os
import signal
import sys
import time
import csv
from datetime import datetime
from typing import Optional, Dict, Any, List
from dataclasses import dataclass, asdict
import threading

from .config import CaptureConfig, load_config
from .lidar_driver import RPLidarDriver, ScanFrame, LidarStatus
from .imu_driver import BMI160Driver, ImuSample, ImuStatus
from .server import CaptureServer

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s"
)
logger = logging.getLogger(__name__)


@dataclass
class SessionState:
    """Current capture session state."""
    active: bool = False
    name: str = ""
    path: str = ""
    start_time: float = 0.0
    station_count: int = 0
    lidar_points: int = 0
    imu_samples: int = 0
    events: List[Dict[str, Any]] = None
    
    def __post_init__(self):
        if self.events is None:
            self.events = []


class CaptureDaemon:
    """Main capture daemon orchestrating sensors and server."""
    
    def __init__(self, config: CaptureConfig):
        self.config = config
        
        # Initialize drivers
        self.lidar = RPLidarDriver(
            port=config.lidar.device,
            baudrate=config.lidar.baudrate
        )
        self.imu = BMI160Driver(
            bus=config.imu.i2c_bus,
            address=config.imu.i2c_address,
            gyro_range_dps=config.imu.gyro_range,
            accel_range_g=config.imu.accel_range,
            sample_rate_hz=config.imu.sample_rate_hz
        )
        
        # Initialize server
        self.server = CaptureServer(
            host=config.server.host,
            http_port=config.server.http_port,
            ws_port=config.server.ws_port,
            auth_token=config.server.auth_token
        )
        
        # Session state
        self.session = SessionState()
        self._session_lidar_file = None
        self._session_imu_file = None
        self._session_lidar_writer = None
        self._session_imu_writer = None
        
        # Event loop reference
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._running = False
        
        # Wire up server callbacks
        self._setup_server_callbacks()
    
    def _setup_server_callbacks(self):
        """Connect server callbacks to daemon methods."""
        # Status providers
        self.server.get_status = self.get_status
        self.server.get_lidar_status = self.lidar.get_diagnostics
        self.server.get_imu_status = self.imu.get_diagnostics
        self.server.get_sessions = self.list_sessions
        self.server.get_config = lambda: self.config.to_dict()
        
        # Action handlers
        self.server.on_lidar_start = self.start_lidar
        self.server.on_lidar_stop = self.stop_lidar
        self.server.on_lidar_configure = self.configure_lidar
        self.server.on_imu_start = self.start_imu
        self.server.on_imu_stop = self.stop_imu
        self.server.on_imu_configure = self.configure_imu
        self.server.on_imu_selftest = self.imu.self_test
        self.server.on_session_start = self.start_session
        self.server.on_session_stop = self.stop_session
        self.server.on_config_update = self.update_config
    
    def get_status(self) -> Dict[str, Any]:
        """Get overall system status."""
        return {
            "daemon": {
                "running": self._running,
                "uptime": time.time() - self._start_time if hasattr(self, '_start_time') else 0,
            },
            "lidar": {
                "status": self.lidar.status.value,
                "connected": self.lidar.is_connected,
                "scanning": self.lidar.is_scanning,
            },
            "imu": {
                "status": self.imu.status.value,
                "connected": self.imu.is_connected,
                "streaming": self.imu.is_streaming,
            },
            "session": {
                "active": self.session.active,
                "name": self.session.name,
                "duration": time.time() - self.session.start_time if self.session.active else 0,
                "station_count": self.session.station_count,
                "lidar_points": self.session.lidar_points,
                "imu_samples": self.session.imu_samples,
            },
            "server": {
                "clients": self.server.client_count,
            },
            "timestamp": time.time(),
        }
    
    def start_lidar(self) -> bool:
        """Start LiDAR scanning."""
        if not self.lidar.is_connected:
            if not self.lidar.connect():
                logger.error("Failed to connect to LiDAR")
                return False
        
        self.lidar.configure(
            scan_mode=self.config.lidar.scan_mode,
            scan_frequency_hz=self.config.lidar.scan_frequency_hz
        )
        
        return self.lidar.start_scan(callback=self._on_lidar_frame)
    
    def stop_lidar(self):
        """Stop LiDAR scanning."""
        self.lidar.stop_scan()
    
    def configure_lidar(self, params: Dict[str, Any]) -> bool:
        """Configure LiDAR parameters."""
        if "device" in params:
            self.config.lidar.device = params["device"]
        if "baudrate" in params:
            self.config.lidar.baudrate = params["baudrate"]
        if "scan_mode" in params:
            self.config.lidar.scan_mode = params["scan_mode"]
        if "scan_frequency_hz" in params:
            self.config.lidar.scan_frequency_hz = params["scan_frequency_hz"]
        
        return self.lidar.configure(
            scan_mode=self.config.lidar.scan_mode,
            scan_frequency_hz=self.config.lidar.scan_frequency_hz
        )
    
    def start_imu(self) -> bool:
        """Start IMU streaming."""
        if not self.imu.is_connected:
            if not self.imu.connect():
                logger.error("Failed to connect to IMU")
                return False
        
        return self.imu.start_streaming(callback=self._on_imu_sample)
    
    def stop_imu(self):
        """Stop IMU streaming."""
        self.imu.stop_streaming()
    
    def configure_imu(self, params: Dict[str, Any]) -> bool:
        """Configure IMU parameters."""
        gyro_range = params.get("gyro_range_dps", self.config.imu.gyro_range)
        accel_range = params.get("accel_range_g", self.config.imu.accel_range)
        sample_rate = params.get("sample_rate_hz", self.config.imu.sample_rate_hz)
        
        self.config.imu.gyro_range = gyro_range
        self.config.imu.accel_range = accel_range
        self.config.imu.sample_rate_hz = sample_rate
        
        return self.imu.configure(
            gyro_range_dps=gyro_range,
            accel_range_g=accel_range,
            sample_rate_hz=sample_rate
        )
    
    def _on_lidar_frame(self, frame: ScanFrame):
        """Handle incoming LiDAR frame."""
        # Broadcast to WebSocket clients
        frame_data = {
            "frame_id": frame.frame_id,
            "timestamp": frame.timestamp,
            "scan_rate_hz": frame.scan_rate_hz,
            "point_count": len(frame.points),
            "points": [
                {"angle": p.angle_deg, "distance": p.distance_m, "quality": p.quality}
                for p in frame.points[::4]  # Subsample for preview
            ]
        }
        
        if self._loop:
            self._loop.call_soon_threadsafe(
                lambda: asyncio.create_task(self._broadcast_lidar(frame_data))
            )
        
        # Record to session if active
        if self.session.active and self._session_lidar_writer:
            for p in frame.points:
                self._session_lidar_writer.writerow([
                    f"{p.timestamp:.6f}",
                    f"{p.angle_deg:.3f}",
                    f"{p.distance_m:.6f}",
                    p.quality
                ])
                self.session.lidar_points += 1
    
    def _on_imu_sample(self, sample: ImuSample):
        """Handle incoming IMU sample."""
        # Broadcast to WebSocket clients (throttled)
        if hasattr(self, '_last_imu_broadcast'):
            if time.time() - self._last_imu_broadcast < 0.02:  # 50Hz max
                pass  # Skip broadcast but still record
            else:
                self._broadcast_imu_sample(sample)
                self._last_imu_broadcast = time.time()
        else:
            self._broadcast_imu_sample(sample)
            self._last_imu_broadcast = time.time()
        
        # Record to session if active
        if self.session.active and self._session_imu_writer:
            self._session_imu_writer.writerow([
                f"{sample.timestamp:.6f}",
                f"{sample.gyro_x:.6f}",
                f"{sample.gyro_y:.6f}",
                f"{sample.gyro_z:.6f}",
                f"{sample.accel_x:.6f}",
                f"{sample.accel_y:.6f}",
                f"{sample.accel_z:.6f}"
            ])
            self.session.imu_samples += 1
    
    def _broadcast_imu_sample(self, sample: ImuSample):
        """Broadcast IMU sample to clients."""
        sample_data = {
            "timestamp": sample.timestamp,
            "gyro": {"x": sample.gyro_x, "y": sample.gyro_y, "z": sample.gyro_z},
            "accel": {"x": sample.accel_x, "y": sample.accel_y, "z": sample.accel_z},
            "temperature": sample.temperature
        }
        
        if self._loop:
            self._loop.call_soon_threadsafe(
                lambda: asyncio.create_task(self._broadcast_imu(sample_data))
            )
    
    async def _broadcast_lidar(self, data: Dict):
        """Async broadcast for LiDAR data."""
        await self.server._broadcast_to_clients(
            self.server._lidar_clients,
            {"type": "lidar_frame", "data": data, "timestamp": time.time()}
        )
    
    async def _broadcast_imu(self, data: Dict):
        """Async broadcast for IMU data."""
        await self.server._broadcast_to_clients(
            self.server._imu_clients,
            {"type": "imu_sample", "data": data, "timestamp": time.time()}
        )
    
    def list_sessions(self) -> List[Dict[str, Any]]:
        """List available sessions."""
        sessions = []
        session_dir = self.config.session.session_dir
        
        if not os.path.exists(session_dir):
            return sessions
        
        for name in os.listdir(session_dir):
            path = os.path.join(session_dir, name)
            if os.path.isdir(path):
                metadata_path = os.path.join(path, "metadata.json")
                if os.path.exists(metadata_path):
                    try:
                        with open(metadata_path) as f:
                            metadata = json.load(f)
                        sessions.append({
                            "name": name,
                            "path": path,
                            "created": metadata.get("created"),
                            "stations": metadata.get("stations", 0)
                        })
                    except Exception:
                        sessions.append({"name": name, "path": path})
        
        return sorted(sessions, key=lambda x: x.get("created", ""), reverse=True)
    
    def start_session(self, name: str) -> bool:
        """Start a new capture session."""
        if self.session.active:
            logger.warning("Session already active")
            return False
        
        session_path = os.path.join(self.config.session.session_dir, name)
        os.makedirs(session_path, exist_ok=True)
        
        self.session = SessionState(
            active=True,
            name=name,
            path=session_path,
            start_time=time.time(),
            events=[]
        )
        
        # Open LiDAR log file
        lidar_path = os.path.join(session_path, "lidar_station_0.csv")
        self._session_lidar_file = open(lidar_path, "w", newline="")
        self._session_lidar_writer = csv.writer(self._session_lidar_file)
        self._session_lidar_writer.writerow(["timestamp", "angle_deg", "range_m", "quality"])
        
        # Open IMU log file
        imu_path = os.path.join(session_path, "imu_log.csv")
        self._session_imu_file = open(imu_path, "w", newline="")
        self._session_imu_writer = csv.writer(self._session_imu_file)
        self._session_imu_writer.writerow([
            "timestamp", "gyro_x_rad_s", "gyro_y_rad_s", "gyro_z_rad_s",
            "accel_x_m_s2", "accel_y_m_s2", "accel_z_m_s2"
        ])
        
        # Record session start event
        self.session.events.append({
            "type": "session_started",
            "timestamp": time.time()
        })
        
        # Write initial metadata
        self._save_session_metadata()
        
        logger.info(f"Session started: {name}")
        return True
    
    def stop_session(self) -> Dict[str, Any]:
        """Stop the current capture session."""
        if not self.session.active:
            return {"error": "No active session"}
        
        # Record session end event
        self.session.events.append({
            "type": "session_stopped",
            "timestamp": time.time()
        })
        
        # Close files
        if self._session_lidar_file:
            self._session_lidar_file.close()
            self._session_lidar_file = None
            self._session_lidar_writer = None
        
        if self._session_imu_file:
            self._session_imu_file.close()
            self._session_imu_file = None
            self._session_imu_writer = None
        
        # Save final metadata and events
        self._save_session_metadata()
        self._save_session_events()
        
        result = {
            "name": self.session.name,
            "path": self.session.path,
            "duration": time.time() - self.session.start_time,
            "lidar_points": self.session.lidar_points,
            "imu_samples": self.session.imu_samples,
            "station_count": self.session.station_count,
        }
        
        logger.info(f"Session stopped: {self.session.name}")
        self.session = SessionState()
        
        return result
    
    def _save_session_metadata(self):
        """Save session metadata to file."""
        metadata = {
            "project": "Planar",
            "created": datetime.utcnow().isoformat() + "Z",
            "name": self.session.name,
            "stations": self.session.station_count,
            "lidar": self.lidar.get_diagnostics().get("info", {}),
            "imu": {
                "chip_id": f"0x{self.imu.info.chip_id:02X}" if self.imu.info.chip_id else None,
                "bus": self.imu.bus,
                "address": f"0x{self.imu.address:02X}"
            },
            "config": self.config.to_dict()
        }
        
        with open(os.path.join(self.session.path, "metadata.json"), "w") as f:
            json.dump(metadata, f, indent=2)
    
    def _save_session_events(self):
        """Save session events to file."""
        with open(os.path.join(self.session.path, "events.json"), "w") as f:
            json.dump(self.session.events, f, indent=2)
    
    def mark_station(self) -> int:
        """Mark a new station in the current session."""
        if not self.session.active:
            return -1
        
        self.session.station_count += 1
        station_num = self.session.station_count
        
        self.session.events.append({
            "type": "station_captured",
            "station": station_num,
            "timestamp": time.time()
        })
        
        # If we want separate files per station, we could rotate here
        # For now, all data goes into station_0
        
        logger.info(f"Station {station_num} marked")
        return station_num
    
    def update_config(self, params: Dict[str, Any]) -> bool:
        """Update daemon configuration."""
        try:
            if "lidar" in params:
                for k, v in params["lidar"].items():
                    if hasattr(self.config.lidar, k):
                        setattr(self.config.lidar, k, v)
            if "imu" in params:
                for k, v in params["imu"].items():
                    if hasattr(self.config.imu, k):
                        setattr(self.config.imu, k, v)
            if "session" in params:
                for k, v in params["session"].items():
                    if hasattr(self.config.session, k):
                        setattr(self.config.session, k, v)
            return True
        except Exception as e:
            logger.error(f"Config update failed: {e}")
            return False
    
    async def run(self):
        """Main daemon run loop."""
        self._start_time = time.time()
        self._running = True
        self._loop = asyncio.get_running_loop()
        
        # Set up signal handlers
        for sig in (signal.SIGTERM, signal.SIGINT):
            self._loop.add_signal_handler(sig, lambda: asyncio.create_task(self.shutdown()))
        
        logger.info("Planar capture daemon starting...")
        
        # Ensure session directory exists
        os.makedirs(self.config.session.session_dir, exist_ok=True)
        
        # Connect to sensors if enabled
        if self.config.lidar.enabled:
            logger.info("Connecting to LiDAR...")
            if self.lidar.connect():
                logger.info("LiDAR connected")
            else:
                logger.warning("LiDAR connection failed - will retry on start command")
        
        if self.config.imu.enabled:
            logger.info("Connecting to IMU...")
            if self.imu.connect():
                logger.info("IMU connected")
            else:
                logger.warning("IMU connection failed - will retry on start command")
        
        # Start server
        await self.server.start()
        
        logger.info(f"Capture daemon running on http://{self.config.server.host}:{self.config.server.http_port}")
        logger.info("Ready for remote connections")
        
        # Run until shutdown
        while self._running:
            await asyncio.sleep(1)
    
    async def shutdown(self):
        """Graceful shutdown."""
        logger.info("Shutting down...")
        self._running = False
        
        # Stop any active session
        if self.session.active:
            self.stop_session()
        
        # Stop sensors
        self.lidar.stop_scan()
        self.imu.stop_streaming()
        self.lidar.disconnect()
        self.imu.disconnect()
        
        # Stop server
        await self.server.stop()
        
        logger.info("Shutdown complete")


def main():
    """CLI entry point."""
    parser = argparse.ArgumentParser(description="Planar capture daemon")
    parser.add_argument("--config", "-c", help="Path to config file")
    parser.add_argument("--session-dir", help="Session storage directory")
    parser.add_argument("--port", "-p", type=int, default=8080, help="HTTP port")
    parser.add_argument("--host", default="0.0.0.0", help="Bind address")
    parser.add_argument("--lidar-device", default="/dev/rplidar", help="LiDAR device path")
    parser.add_argument("--imu-bus", type=int, default=1, help="I2C bus number")
    parser.add_argument("--imu-address", type=lambda x: int(x, 0), default=0x69, help="IMU I2C address")
    parser.add_argument("--debug", action="store_true", help="Enable debug logging")
    args = parser.parse_args()
    
    if args.debug:
        logging.getLogger().setLevel(logging.DEBUG)
    
    # Load config
    config = load_config(args.config)
    
    # Override with CLI args
    if args.session_dir:
        config.session.session_dir = args.session_dir
    if args.port:
        config.server.http_port = args.port
    if args.host:
        config.server.host = args.host
    if args.lidar_device:
        config.lidar.device = args.lidar_device
    if args.imu_bus:
        config.imu.i2c_bus = args.imu_bus
    if args.imu_address:
        config.imu.i2c_address = args.imu_address
    
    # Create and run daemon
    daemon = CaptureDaemon(config)
    
    try:
        asyncio.run(daemon.run())
    except KeyboardInterrupt:
        pass
    
    return 0


if __name__ == "__main__":
    sys.exit(main())
