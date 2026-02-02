"""Tests for capture configuration module."""

import json
import os
import tempfile
import pytest

from capture.config import (
    CaptureConfig,
    LidarConfig,
    ImuConfig,
    ServerConfig,
    SessionConfig,
    load_config,
)


class TestLidarConfig:
    """Tests for LidarConfig dataclass."""

    def test_default_values(self):
        config = LidarConfig()
        assert config.device == "/dev/rplidar"
        assert config.baudrate == 1_000_000
        assert config.scan_mode == "DenseBoost"
        assert config.scan_frequency_hz == 10.0
        assert config.enabled is True

    def test_custom_values(self):
        config = LidarConfig(
            device="/dev/ttyUSB0",
            baudrate=115200,
            scan_mode="Standard",
            scan_frequency_hz=5.0,
            enabled=False,
        )
        assert config.device == "/dev/ttyUSB0"
        assert config.baudrate == 115200
        assert config.scan_mode == "Standard"
        assert config.scan_frequency_hz == 5.0
        assert config.enabled is False


class TestImuConfig:
    """Tests for ImuConfig dataclass."""

    def test_default_values(self):
        config = ImuConfig()
        assert config.i2c_bus == 1
        assert config.i2c_address == 0x69
        assert config.sample_rate_hz == 100.0
        assert config.gyro_range == 250
        assert config.accel_range == 2
        assert config.enabled is True

    def test_custom_values(self):
        config = ImuConfig(
            i2c_bus=0,
            i2c_address=0x68,
            sample_rate_hz=200.0,
            gyro_range=500,
            accel_range=4,
        )
        assert config.i2c_bus == 0
        assert config.i2c_address == 0x68
        assert config.sample_rate_hz == 200.0
        assert config.gyro_range == 500
        assert config.accel_range == 4


class TestCaptureConfig:
    """Tests for CaptureConfig dataclass."""

    def test_default_nested_configs(self):
        config = CaptureConfig()
        assert isinstance(config.lidar, LidarConfig)
        assert isinstance(config.imu, ImuConfig)
        assert isinstance(config.server, ServerConfig)
        assert isinstance(config.session, SessionConfig)

    def test_to_dict(self):
        config = CaptureConfig()
        d = config.to_dict()
        
        assert "lidar" in d
        assert "imu" in d
        assert "server" in d
        assert "session" in d
        
        assert d["lidar"]["device"] == "/dev/rplidar"
        assert d["imu"]["i2c_address"] == 0x69
        assert d["server"]["http_port"] == 8080

    def test_from_file(self):
        with tempfile.NamedTemporaryFile(mode="w", suffix=".json", delete=False) as f:
            json.dump({
                "lidar": {"device": "/dev/custom", "baudrate": 500000},
                "imu": {"i2c_bus": 2, "sample_rate_hz": 50.0},
            }, f)
            f.flush()
            
            config = CaptureConfig.from_file(f.name)
            
            assert config.lidar.device == "/dev/custom"
            assert config.lidar.baudrate == 500000
            assert config.imu.i2c_bus == 2
            assert config.imu.sample_rate_hz == 50.0
            # Defaults should still be set for unspecified values
            assert config.lidar.scan_mode == "DenseBoost"
            assert config.imu.gyro_range == 250
            
            os.unlink(f.name)

    def test_save_and_load_roundtrip(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            config_path = os.path.join(tmpdir, "config.json")
            
            original = CaptureConfig()
            original.lidar.device = "/dev/test"
            original.imu.sample_rate_hz = 200.0
            original.save(config_path)
            
            loaded = CaptureConfig.from_file(config_path)
            
            assert loaded.lidar.device == "/dev/test"
            assert loaded.imu.sample_rate_hz == 200.0


class TestLoadConfig:
    """Tests for load_config function."""

    def test_load_nonexistent_returns_defaults(self):
        config = load_config("/nonexistent/path.json")
        assert isinstance(config, CaptureConfig)
        assert config.lidar.device == "/dev/rplidar"

    def test_load_existing_file(self):
        with tempfile.NamedTemporaryFile(mode="w", suffix=".json", delete=False) as f:
            json.dump({"lidar": {"device": "/dev/loaded"}}, f)
            f.flush()
            
            config = load_config(f.name)
            assert config.lidar.device == "/dev/loaded"
            
            os.unlink(f.name)
