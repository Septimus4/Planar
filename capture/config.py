"""Configuration settings for the capture daemon."""

from dataclasses import dataclass, field
from typing import Optional
import json
import os


@dataclass
class LidarConfig:
    """RPLidar S3 configuration."""
    device: str = "/dev/rplidar"
    baudrate: int = 1_000_000
    scan_mode: str = "DenseBoost"
    scan_frequency_hz: float = 10.0
    enabled: bool = True


@dataclass
class ImuConfig:
    """BMI160 IMU configuration."""
    i2c_bus: int = 1
    i2c_address: int = 0x69
    sample_rate_hz: float = 100.0
    gyro_range: int = 250  # degrees per second (125, 250, 500, 1000, 2000)
    accel_range: int = 2   # g (2, 4, 8, 16)
    enabled: bool = True


@dataclass
class ServerConfig:
    """Remote control server configuration."""
    host: str = "0.0.0.0"
    http_port: int = 8080
    ws_port: int = 8081
    auth_token: Optional[str] = None  # Optional auth token for security


@dataclass
class SessionConfig:
    """Session storage configuration."""
    session_dir: str = "/var/planar/sessions"
    max_sessions: int = 100
    auto_cleanup: bool = True


@dataclass
class CaptureConfig:
    """Main capture daemon configuration."""
    lidar: LidarConfig = field(default_factory=LidarConfig)
    imu: ImuConfig = field(default_factory=ImuConfig)
    server: ServerConfig = field(default_factory=ServerConfig)
    session: SessionConfig = field(default_factory=SessionConfig)

    @classmethod
    def from_file(cls, path: str) -> "CaptureConfig":
        """Load configuration from JSON file."""
        with open(path) as f:
            data = json.load(f)
        
        config = cls()
        if "lidar" in data:
            for k, v in data["lidar"].items():
                if hasattr(config.lidar, k):
                    setattr(config.lidar, k, v)
        if "imu" in data:
            for k, v in data["imu"].items():
                if hasattr(config.imu, k):
                    setattr(config.imu, k, v)
        if "server" in data:
            for k, v in data["server"].items():
                if hasattr(config.server, k):
                    setattr(config.server, k, v)
        if "session" in data:
            for k, v in data["session"].items():
                if hasattr(config.session, k):
                    setattr(config.session, k, v)
        return config

    def to_dict(self) -> dict:
        """Convert to dictionary."""
        return {
            "lidar": {
                "device": self.lidar.device,
                "baudrate": self.lidar.baudrate,
                "scan_mode": self.lidar.scan_mode,
                "scan_frequency_hz": self.lidar.scan_frequency_hz,
                "enabled": self.lidar.enabled,
            },
            "imu": {
                "i2c_bus": self.imu.i2c_bus,
                "i2c_address": self.imu.i2c_address,
                "sample_rate_hz": self.imu.sample_rate_hz,
                "gyro_range": self.imu.gyro_range,
                "accel_range": self.imu.accel_range,
                "enabled": self.imu.enabled,
            },
            "server": {
                "host": self.server.host,
                "http_port": self.server.http_port,
                "ws_port": self.server.ws_port,
            },
            "session": {
                "session_dir": self.session.session_dir,
                "max_sessions": self.session.max_sessions,
                "auto_cleanup": self.session.auto_cleanup,
            },
        }

    def save(self, path: str):
        """Save configuration to JSON file."""
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w") as f:
            json.dump(self.to_dict(), f, indent=2)


# Default config file locations
DEFAULT_CONFIG_PATHS = [
    "/etc/planar/capture.json",
    os.path.expanduser("~/.config/planar/capture.json"),
    "./capture_config.json",
]


def load_config(path: Optional[str] = None) -> CaptureConfig:
    """Load configuration from file or return defaults."""
    if path and os.path.exists(path):
        return CaptureConfig.from_file(path)
    
    for p in DEFAULT_CONFIG_PATHS:
        if os.path.exists(p):
            return CaptureConfig.from_file(p)
    
    return CaptureConfig()
