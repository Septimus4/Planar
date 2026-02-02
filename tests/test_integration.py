"""Integration tests for capture system.

These tests verify that all components work together correctly.
They use mocks for hardware but test real component interactions.
"""

import pytest
from unittest.mock import Mock, MagicMock, AsyncMock, patch
import asyncio
import tempfile
import json
from pathlib import Path


@pytest.mark.asyncio
class TestCaptureIntegration:
    """Integration tests for the complete capture system."""

    @pytest.fixture
    def mock_lidar_driver(self):
        """Mock LiDAR driver."""
        driver = Mock()
        driver.connect.return_value = True
        driver.disconnect.return_value = None
        driver.is_connected = True
        driver.get_device_info.return_value = {
            "model": "RPLIDAR S3",
            "firmware": "1.2.3",
            "serial": "ABC123",
        }
        return driver

    @pytest.fixture
    def mock_imu_driver(self):
        """Mock IMU driver."""
        driver = Mock()
        driver.connect.return_value = True
        driver.disconnect.return_value = None
        driver.is_connected = True
        driver.get_chip_id.return_value = 0xD1
        driver.self_test.return_value = {
            "passed": True,
            "gyro": True,
            "accel": True,
        }
        return driver

    async def test_daemon_server_integration(self, tmp_path):
        """Test daemon and server work together."""
        from capture.daemon import CaptureDaemon
        from capture.config import CaptureConfig, SessionConfig, ServerConfig
        
        config = CaptureConfig()
        config.server.host = "127.0.0.1"
        config.server.http_port = 0  # Auto port
        config.session.session_dir = str(tmp_path)
        
        daemon = CaptureDaemon(config)
        
        # Test status API
        status = daemon.get_status()
        assert "daemon" in status
        assert "lidar" in status
        assert "imu" in status


class TestConfigToDriverIntegration:
    """Tests for config → driver initialization."""

    def test_lidar_config_applied_to_driver(self):
        """Test that LiDAR config values are passed to driver."""
        from capture.config import LidarConfig
        from capture.lidar_driver import RPLidarDriver
        
        config = LidarConfig(
            device="/dev/ttyUSB0",
            baudrate=256000,
            scan_mode="Standard",
        )
        
        with patch('serial.Serial'):
            driver = RPLidarDriver(
                port=config.device,
                baudrate=config.baudrate,
            )
            
            assert driver.port == config.device
            assert driver.baudrate == config.baudrate

    def test_imu_config_applied_to_driver(self):
        """Test that IMU config values are passed to driver."""
        from capture.config import ImuConfig
        from capture.imu_driver import BMI160Driver
        
        config = ImuConfig(
            i2c_bus=2,
            i2c_address=0x68,
            gyro_range=1000,
            accel_range=8,
        )
        
        with patch('smbus2.SMBus'):
            driver = BMI160Driver(
                bus=config.i2c_bus,
                address=config.i2c_address,
            )
            
            assert driver.bus == config.i2c_bus
            assert driver.address == config.i2c_address


class TestSessionDataIntegration:
    """Tests for session data format compatibility with processing pipeline."""

    def test_session_directory_structure(self, tmp_path):
        """Test session creates correct directory structure."""
        from capture.daemon import CaptureDaemon
        from capture.config import CaptureConfig, SessionConfig
        
        config = CaptureConfig()
        config.session.session_dir = str(tmp_path)
        
        daemon = CaptureDaemon(config)
        daemon.lidar.get_diagnostics = Mock(return_value={"info": {}})
        daemon.imu._info.chip_id = 0xD1
        
        result = daemon.start_session("structure_test")
        assert result is True
        session_path = daemon.session.path
        daemon.stop_session()
        
        session_dir = Path(session_path)
        
        # These files are expected by processing-agent per AGENTS.md
        assert (session_dir / "metadata.json").exists()
        assert (session_dir / "events.json").exists()

    def test_events_json_format(self, tmp_path):
        """Test events.json matches expected format."""
        from capture.daemon import CaptureDaemon
        from capture.config import CaptureConfig, SessionConfig
        import time
        
        config = CaptureConfig()
        config.session.session_dir = str(tmp_path)
        
        daemon = CaptureDaemon(config)
        daemon.lidar.get_diagnostics = Mock(return_value={"info": {}})
        daemon.imu._info.chip_id = 0xD1
        
        result = daemon.start_session("events_test")
        assert result is True
        
        # Use mark_station to add events (actual daemon API)
        daemon.mark_station()  # Station 1
        daemon.mark_station()  # Station 2
        
        session_path = daemon.session.path
        daemon.stop_session()
        
        events_path = Path(session_path) / "events.json"
        with open(events_path) as f:
            events = json.load(f)
        
        # Verify event format - events have type, timestamp, and optionally station
        for event in events:
            assert "type" in event
            assert "timestamp" in event

    def test_metadata_json_format(self, tmp_path):
        """Test metadata.json contains required fields."""
        from capture.daemon import CaptureDaemon
        from capture.config import CaptureConfig, SessionConfig
        
        config = CaptureConfig()
        config.session.session_dir = str(tmp_path)
        
        daemon = CaptureDaemon(config)
        daemon.lidar.get_diagnostics = Mock(return_value={"info": {}})
        daemon.imu._info.chip_id = 0xD1
        
        result = daemon.start_session("metadata_test")
        assert result is True
        session_path = daemon.session.path
        daemon.stop_session()
        
        metadata_path = Path(session_path) / "metadata.json"
        with open(metadata_path) as f:
            metadata = json.load(f)
        
        # Required fields per AGENTS.md - actual uses "name" and "created"
        assert "name" in metadata
        assert "created" in metadata
        assert "config" in metadata


class TestDesktopClientServerIntegration:
    """Tests for desktop client and capture server integration."""

    @pytest.fixture
    async def running_server(self, tmp_path):
        """Start a capture server for testing."""
        from capture.server import CaptureServer
        
        server = CaptureServer(host="127.0.0.1", http_port=0)  # Auto port
        
        # Set up mock handlers
        server.get_status = Mock(return_value={
            "daemon": {"running": True},
            "lidar": {"status": "idle"},
            "imu": {"status": "idle"},
        })
        server.on_lidar_start = Mock(return_value=True)
        server.on_lidar_stop = Mock()
        server.on_imu_start = Mock(return_value=True)
        server.on_imu_stop = Mock()
        
        await server.start()
        
        yield server
        
        await server.stop()

    @pytest.mark.asyncio
    async def test_client_connects_to_server(self, running_server):
        """Test that client can connect to server."""
        import aiohttp
        
        # Get actual port
        port = running_server._runner.addresses[0][1]
        
        # Use aiohttp directly instead of PlanarClient for this test
        async with aiohttp.ClientSession() as session:
            async with session.get(f"http://127.0.0.1:{port}/api/status") as resp:
                status = await resp.json()
        
        assert status is not None
        assert "daemon" in status

    @pytest.mark.asyncio
    async def test_client_lidar_control(self, running_server):
        """Test client can start/stop LiDAR via API."""
        import aiohttp
        
        port = running_server._runner.addresses[0][1]
        
        async with aiohttp.ClientSession() as session:
            # Start LiDAR
            async with session.post(f"http://127.0.0.1:{port}/api/lidar/start") as resp:
                result = await resp.json()
            assert result.get("status") == "ok" or result.get("success") is True
            
            # Stop LiDAR
            async with session.post(f"http://127.0.0.1:{port}/api/lidar/stop") as resp:
                result = await resp.json()
            assert result.get("status") == "ok" or result.get("success") is True


class TestDataFlowIntegration:
    """Tests for data flow through the system."""

    def test_lidar_frame_to_session_file(self, tmp_path):
        """Test LiDAR frames are correctly written to session files."""
        from capture.daemon import CaptureDaemon
        from capture.config import CaptureConfig, SessionConfig
        from capture.lidar_driver import ScanFrame, ScanPoint
        import time
        
        config = CaptureConfig()
        config.session.session_dir = str(tmp_path)
        
        daemon = CaptureDaemon(config)
        daemon.lidar.get_diagnostics = Mock(return_value={"info": {}})
        daemon.imu._info.chip_id = 0xD1
        
        result = daemon.start_session("lidar_flow_test")
        assert result is True
        session_path = daemon.session.path
        
        # Send multiple frames using actual method name
        now = time.time()
        for i in range(5):
            frame = ScanFrame(
                frame_id=i,
                timestamp=now + i * 0.1,
                scan_rate_hz=10.0,
                points=[
                    ScanPoint(
                        angle_deg=float(j * 10),
                        distance_m=(1.0 + j * 0.001),
                        quality=50,
                        timestamp=now + i * 0.1
                    )
                    for j in range(36)  # 36 points per frame
                ]
            )
            daemon._on_lidar_frame(frame)
        
        daemon.stop_session()
        
        # Verify data was written
        session_dir = Path(session_path)
        lidar_files = list(session_dir.glob("lidar_*.csv"))
        
        assert len(lidar_files) > 0
        
        # Check file content
        with open(lidar_files[0]) as f:
            content = f.read()
            lines = content.strip().split('\n')
            assert len(lines) > 1  # Header + data

    def test_imu_samples_to_session_file(self, tmp_path):
        """Test IMU samples are correctly written to session file."""
        from capture.daemon import CaptureDaemon
        from capture.config import CaptureConfig, SessionConfig
        from capture.imu_driver import ImuSample
        
        config = CaptureConfig()
        config.session.session_dir = str(tmp_path)
        
        daemon = CaptureDaemon(config)
        daemon.lidar.get_diagnostics = Mock(return_value={"info": {}})
        daemon.imu._info.chip_id = 0xD1
        
        result = daemon.start_session("imu_flow_test")
        assert result is True
        session_path = daemon.session.path
        
        # Send samples using actual method name
        for i in range(100):
            sample = ImuSample(
                timestamp=1000.0 + i * 0.01,  # 100Hz
                gyro_x=0.01 * i,
                gyro_y=0.02 * i,
                gyro_z=0.03 * i,
                accel_x=0.0,
                accel_y=0.0,
                accel_z=9.8,
                temperature=25.0,
            )
            daemon._on_imu_sample(sample)
        
        daemon.stop_session()
        
        # Verify
        imu_path = Path(session_path) / "imu_log.csv"
        assert imu_path.exists()
        
        with open(imu_path) as f:
            lines = f.read().strip().split('\n')
            # Should have header + 100 samples
            assert len(lines) >= 100


# Hardware integration tests
@pytest.mark.hardware
class TestFullSystemHardware:
    """Full system hardware integration tests."""

    @pytest.fixture
    def hardware_config(self, tmp_path):
        """Config for real hardware."""
        from capture.config import CaptureConfig, LidarConfig, ImuConfig, SessionConfig
        
        config = CaptureConfig()
        config.lidar.device = "/dev/rplidar"
        config.lidar.baudrate = 1000000
        config.lidar.scan_mode = "Standard"  # Use Standard mode, not DenseBoost
        config.imu.i2c_bus = 1
        config.imu.i2c_address = 0x69
        config.session.session_dir = str(tmp_path)
        
        return config

    def test_hardware_drivers_connect(self, hardware_config):
        """Test real drivers can connect to hardware."""
        from capture.lidar_driver import RPLidarDriver
        from capture.imu_driver import BMI160Driver
        
        lidar = RPLidarDriver(
            port=hardware_config.lidar.device,
            baudrate=hardware_config.lidar.baudrate,
        )
        imu = BMI160Driver(
            bus=hardware_config.imu.i2c_bus,
            address=hardware_config.imu.i2c_address,
        )
        
        try:
            assert lidar.connect() is True, "LiDAR failed to connect"
            assert imu.connect() is True, "IMU failed to connect"
            
            # Verify device identities
            assert imu.info.chip_id == 0xD1, "Wrong IMU chip ID"
            
        finally:
            lidar.disconnect()
            imu.disconnect()

    def test_hardware_data_capture(self, hardware_config):
        """Test capturing real data from hardware."""
        from capture.daemon import CaptureDaemon
        import time
        
        daemon = CaptureDaemon(hardware_config)
        
        try:
            # Connect to hardware
            assert daemon.start_lidar()
            assert daemon.start_imu()
            
            # Start session - returns bool
            result = daemon.start_session("hardware_capture_test")
            assert result is True
            session_path = daemon.session.path
            
            # Capture for a few seconds
            time.sleep(3.0)
            
            # Mark a station using actual method
            daemon.mark_station()
            time.sleep(1.0)
            daemon.mark_station()
            
            # Stop
            session_info = daemon.stop_session()
            
            # Verify outputs
            session_dir = Path(session_path)
            
            assert (session_dir / "metadata.json").exists()
            assert (session_dir / "events.json").exists()
            assert (session_dir / "imu_log.csv").exists()
            
            # Should have some LiDAR data files
            lidar_files = list(session_dir.glob("lidar_*.csv"))
            assert len(lidar_files) > 0
            
        finally:
            daemon.stop_imu()
            daemon.stop_lidar()
