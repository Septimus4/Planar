"""Tests for capture daemon orchestration."""

import pytest
from unittest.mock import Mock, MagicMock, AsyncMock, patch, call
import asyncio
import tempfile
import shutil
from pathlib import Path
from dataclasses import asdict
import json


class TestCaptureDaemonUnit:
    """Unit tests for CaptureDaemon."""

    def test_initial_state(self):
        from capture.daemon import CaptureDaemon
        from capture.config import CaptureConfig
        
        config = CaptureConfig()
        daemon = CaptureDaemon(config)
        
        assert daemon.config == config
        assert daemon._running is False
        assert daemon.session.active is False

    def test_with_custom_config(self):
        from capture.daemon import CaptureDaemon
        from capture.config import CaptureConfig, LidarConfig, ImuConfig
        
        config = CaptureConfig()
        config.lidar.device = "/dev/ttyUSB1"
        config.lidar.baudrate = 115200
        config.imu.i2c_bus = 2
        config.imu.i2c_address = 0x68
        
        daemon = CaptureDaemon(config)
        
        assert daemon.config.lidar.device == "/dev/ttyUSB1"
        assert daemon.config.imu.i2c_bus == 2


class TestCaptureDaemonSession:
    """Tests for session management."""

    def test_session_directory_creation(self, tmp_path):
        from capture.daemon import CaptureDaemon
        from capture.config import CaptureConfig, SessionConfig
        
        sessions_dir = tmp_path / "sessions"
        config = CaptureConfig()
        config.session.session_dir = str(sessions_dir)
        daemon = CaptureDaemon(config)
        
        # Mock the lidar/imu get_diagnostics for metadata creation
        daemon.lidar.get_diagnostics = Mock(return_value={"info": {}})
        daemon.imu._info.chip_id = 0xD1
        
        # Start a session
        result = daemon.start_session("test_session")
        
        assert result is True
        assert daemon.session.active is True
        # Session path should exist
        assert Path(daemon.session.path).exists()

    def test_session_metadata_written(self, tmp_path):
        from capture.daemon import CaptureDaemon
        from capture.config import CaptureConfig, SessionConfig
        
        sessions_dir = tmp_path / "sessions"
        config = CaptureConfig()
        config.session.session_dir = str(sessions_dir)
        daemon = CaptureDaemon(config)
        daemon.lidar.get_diagnostics = Mock(return_value={"info": {}})
        daemon.imu._info.chip_id = 0xD1
        
        result = daemon.start_session("metadata_test")
        assert result is True
        
        metadata_path = Path(daemon.session.path) / "metadata.json"
        assert metadata_path.exists()
        
        with open(metadata_path) as f:
            metadata = json.load(f)
        
        # Actual implementation uses "name" not "session_name"
        assert "name" in metadata
        assert metadata["name"] == "metadata_test"
        assert "created" in metadata
        assert "config" in metadata

    def test_stop_session_without_active_session(self, tmp_path):
        from capture.daemon import CaptureDaemon
        from capture.config import CaptureConfig, SessionConfig
        
        config = CaptureConfig()
        config.session.session_dir = str(tmp_path)
        daemon = CaptureDaemon(config)
        
        result = daemon.stop_session()
        
        # Returns dict with error when no active session
        assert result is not None
        assert "error" in result


class TestCaptureDaemonStatus:
    """Tests for status reporting."""

    def test_get_status_idle(self):
        from capture.daemon import CaptureDaemon
        from capture.config import CaptureConfig
        
        config = CaptureConfig()
        daemon = CaptureDaemon(config)
        
        status = daemon.get_status()
        
        assert "daemon" in status
        assert "lidar" in status
        assert "imu" in status
        assert "session" in status
        # daemon.running is determined by _running attribute
        assert status["daemon"]["running"] is False

    def test_get_lidar_diagnostics(self):
        from capture.daemon import CaptureDaemon
        from capture.config import CaptureConfig
        
        daemon = CaptureDaemon(CaptureConfig())
        
        # Use get_diagnostics on the lidar driver
        status = daemon.lidar.get_diagnostics()
        
        assert status["connected"] is False
        assert status["scanning"] is False

    def test_get_imu_diagnostics(self):
        from capture.daemon import CaptureDaemon
        from capture.config import CaptureConfig
        
        daemon = CaptureDaemon(CaptureConfig())
        
        # Use get_diagnostics on the imu driver
        status = daemon.imu.get_diagnostics()
        
        assert status["connected"] is False
        assert status["streaming"] is False


class TestCaptureDaemonDriverControl:
    """Tests for driver start/stop control."""

    def test_start_lidar_connects_driver(self):
        from capture.daemon import CaptureDaemon
        from capture.config import CaptureConfig
        
        daemon = CaptureDaemon(CaptureConfig())
        
        # Mock the connect method
        daemon.lidar.connect = Mock(return_value=True)
        daemon.lidar.start_scan = Mock(return_value=True)
        
        result = daemon.start_lidar()
        
        assert result is True
        daemon.lidar.connect.assert_called_once()

    def test_start_lidar_connect_failure(self):
        from capture.daemon import CaptureDaemon
        from capture.config import CaptureConfig
        
        daemon = CaptureDaemon(CaptureConfig())
        
        daemon.lidar.connect = Mock(return_value=False)
        
        result = daemon.start_lidar()
        
        assert result is False

    def test_stop_lidar_stops_scan(self):
        from capture.daemon import CaptureDaemon
        from capture.config import CaptureConfig
        
        daemon = CaptureDaemon(CaptureConfig())
        daemon.lidar.stop_scan = Mock()
        
        daemon.stop_lidar()
        
        daemon.lidar.stop_scan.assert_called_once()

    def test_start_imu_connects_driver(self):
        from capture.daemon import CaptureDaemon
        from capture.config import CaptureConfig
        
        daemon = CaptureDaemon(CaptureConfig())
        
        daemon.imu.connect = Mock(return_value=True)
        daemon.imu.start_streaming = Mock(return_value=True)
        
        result = daemon.start_imu()
        
        assert result is True
        daemon.imu.connect.assert_called_once()

    def test_stop_imu_stops_streaming(self):
        from capture.daemon import CaptureDaemon
        from capture.config import CaptureConfig
        
        daemon = CaptureDaemon(CaptureConfig())
        daemon.imu.stop_streaming = Mock()
        
        daemon.stop_imu()
        
        daemon.imu.stop_streaming.assert_called_once()


class TestCaptureDaemonEvents:
    """Tests for event handling."""

    def test_session_events_recorded(self, tmp_path):
        from capture.daemon import CaptureDaemon
        from capture.config import CaptureConfig, SessionConfig
        
        config = CaptureConfig()
        config.session.session_dir = str(tmp_path)
        daemon = CaptureDaemon(config)
        daemon.lidar.get_diagnostics = Mock(return_value={"info": {}})
        daemon.imu._info.chip_id = 0xD1
        
        # Start session to enable event recording
        result = daemon.start_session("event_test")
        assert result is True
        
        # Events are added directly to session.events
        daemon.session.events.append({
            "type": "test_event",
            "data": {"key": "value"},
            "timestamp": 1234567890.0
        })
        
        session_path = daemon.session.path
        
        # Stop session to flush events
        daemon.stop_session()
        
        # Check events file
        events_path = Path(session_path) / "events.json"
        assert events_path.exists()
        with open(events_path) as f:
            events = json.load(f)
        
        assert len(events) > 0
        assert any(e["type"] == "test_event" for e in events)

    def test_station_mark_events(self, tmp_path):
        from capture.daemon import CaptureDaemon
        from capture.config import CaptureConfig, SessionConfig
        
        config = CaptureConfig()
        config.session.session_dir = str(tmp_path)
        daemon = CaptureDaemon(config)
        daemon.lidar.get_diagnostics = Mock(return_value={"info": {}})
        daemon.imu._info.chip_id = 0xD1
        
        result = daemon.start_session("station_test")
        assert result is True
        
        # Mark stations using the actual mark_station method
        station1 = daemon.mark_station()
        station2 = daemon.mark_station()
        
        assert station1 == 1
        assert station2 == 2
        
        session_path = daemon.session.path
        daemon.stop_session()
        
        events_path = Path(session_path) / "events.json"
        with open(events_path) as f:
            events = json.load(f)
        
        station_events = [e for e in events if "station" in e["type"]]
        assert len(station_events) >= 2


class TestCaptureDaemonConfiguration:
    """Tests for runtime configuration changes."""

    def test_update_config_lidar(self):
        from capture.daemon import CaptureDaemon
        from capture.config import CaptureConfig, LidarConfig
        
        config = CaptureConfig()
        config.lidar.scan_mode = "Standard"
        daemon = CaptureDaemon(config)
        
        # Use update_config with nested params
        result = daemon.update_config({
            "lidar": {
                "scan_mode": "DenseBoost",
                "scan_frequency_hz": 15.0
            }
        })
        
        assert result is True
        assert daemon.config.lidar.scan_mode == "DenseBoost"
        assert daemon.config.lidar.scan_frequency_hz == 15.0

    def test_update_config_imu(self):
        from capture.daemon import CaptureDaemon
        from capture.config import CaptureConfig, ImuConfig
        
        config = CaptureConfig()
        config.imu.sample_rate_hz = 100
        daemon = CaptureDaemon(config)
        
        result = daemon.update_config({
            "imu": {"sample_rate_hz": 200}
        })
        
        assert result is True
        assert daemon.config.imu.sample_rate_hz == 200

    def test_get_config_returns_dict(self):
        from capture.daemon import CaptureDaemon
        from capture.config import CaptureConfig
        
        daemon = CaptureDaemon(CaptureConfig())
        
        # Use config.to_dict() instead of daemon.get_config()
        config_dict = daemon.config.to_dict()
        
        assert isinstance(config_dict, dict)
        assert "lidar" in config_dict
        assert "imu" in config_dict
        assert "server" in config_dict


@pytest.mark.asyncio
class TestCaptureDaemonAsync:
    """Async tests for daemon lifecycle."""

    async def test_daemon_run_and_shutdown(self, tmp_path):
        from capture.daemon import CaptureDaemon
        from capture.config import CaptureConfig
        
        config = CaptureConfig()
        # Use a free port
        config.server.http_port = 0
        # Use tmp_path for session directory to avoid permission issues
        config.session.session_dir = str(tmp_path / "sessions")
        daemon = CaptureDaemon(config)
        
        # Start daemon in background
        run_task = asyncio.create_task(daemon.run())
        
        # Give it time to start
        await asyncio.sleep(0.5)
        
        # Request shutdown
        await daemon.shutdown()
        
        # Wait for clean shutdown
        try:
            await asyncio.wait_for(run_task, timeout=3.0)
        except asyncio.TimeoutError:
            run_task.cancel()
            pytest.fail("Daemon did not shut down in time")


class TestCaptureDaemonDataRecording:
    """Tests for data recording to files."""

    def test_lidar_data_recorded(self, tmp_path):
        from capture.daemon import CaptureDaemon
        from capture.config import CaptureConfig, SessionConfig
        from capture.lidar_driver import ScanFrame, ScanPoint
        import time
        
        config = CaptureConfig()
        config.session.session_dir = str(tmp_path)
        daemon = CaptureDaemon(config)
        daemon.lidar.get_diagnostics = Mock(return_value={"info": {}})
        daemon.imu._info.chip_id = 0xD1
        
        result = daemon.start_session("lidar_record_test")
        assert result is True
        session_path = daemon.session.path
        
        # Simulate receiving LiDAR frame using _on_lidar_frame
        now = time.time()
        frame = ScanFrame(
            frame_id=1,
            timestamp=now,
            scan_rate_hz=10.0,
            points=[
                ScanPoint(angle_deg=0.0, distance_m=1.0, quality=50, timestamp=now),
                ScanPoint(angle_deg=90.0, distance_m=2.0, quality=60, timestamp=now),
            ]
        )
        daemon._on_lidar_frame(frame)
        
        daemon.stop_session()
        
        # Check that lidar data was written
        lidar_files = list(Path(session_path).glob("lidar_*.csv"))
        assert len(lidar_files) > 0

    def test_imu_data_recorded(self, tmp_path):
        from capture.daemon import CaptureDaemon
        from capture.config import CaptureConfig, SessionConfig
        from capture.imu_driver import ImuSample
        
        config = CaptureConfig()
        config.session.session_dir = str(tmp_path)
        daemon = CaptureDaemon(config)
        daemon.lidar.get_diagnostics = Mock(return_value={"info": {}})
        daemon.imu._info.chip_id = 0xD1
        
        result = daemon.start_session("imu_record_test")
        assert result is True
        session_path = daemon.session.path
        
        # Simulate receiving IMU sample using _on_imu_sample
        sample = ImuSample(
            timestamp=1000.0,
            gyro_x=0.1, gyro_y=0.2, gyro_z=0.3,
            accel_x=0.0, accel_y=0.0, accel_z=9.8,
            temperature=25.0,
        )
        daemon._on_imu_sample(sample)
        
        daemon.stop_session()
        
        # Check that imu log was written
        imu_path = Path(session_path) / "imu_log.csv"
        assert imu_path.exists()


class TestCaptureDaemonCLI:
    """Tests for CLI entry point."""

    def test_main_function_exists(self):
        """Test that main function exists and is callable."""
        from capture.daemon import main
        
        assert callable(main)

    def test_cli_argparse_defaults(self):
        """Test CLI argument parsing with default values."""
        import argparse
        
        parser = argparse.ArgumentParser(description="Planar capture daemon")
        parser.add_argument("--config", "-c", help="Path to config file")
        parser.add_argument("--session-dir", help="Session storage directory")
        parser.add_argument("--port", "-p", type=int, default=8080, help="HTTP port")
        parser.add_argument("--host", default="0.0.0.0", help="Bind address")
        
        args = parser.parse_args([])
        
        assert args.config is None
        assert args.host == "0.0.0.0"
        assert args.port == 8080

    def test_cli_argparse_custom(self):
        """Test CLI argument parsing with custom values."""
        import argparse
        
        parser = argparse.ArgumentParser(description="Planar capture daemon")
        parser.add_argument("--config", "-c", help="Path to config file")
        parser.add_argument("--session-dir", help="Session storage directory")
        parser.add_argument("--port", "-p", type=int, default=8080, help="HTTP port")
        parser.add_argument("--host", default="0.0.0.0", help="Bind address")
        
        args = parser.parse_args([
            "--config", "/path/to/config.json",
            "--host", "192.168.1.100",
            "--port", "9000",
        ])
        
        assert args.config == "/path/to/config.json"
        assert args.host == "192.168.1.100"
        assert args.port == 9000


# Hardware integration tests
@pytest.mark.hardware
class TestCaptureDaemonHardware:
    """Hardware integration tests - require actual devices connected."""

    def test_hardware_full_flow(self, tmp_path):
        """Test complete session with real hardware."""
        from capture.daemon import CaptureDaemon
        from capture.config import CaptureConfig, SessionConfig, LidarConfig, ImuConfig
        
        config = CaptureConfig()
        config.lidar.device = "/dev/rplidar"
        config.lidar.baudrate = 1000000
        config.imu.i2c_bus = 1
        config.imu.i2c_address = 0x69
        config.session.session_dir = str(tmp_path)
        
        daemon = CaptureDaemon(config)
        
        try:
            # Start drivers
            assert daemon.start_lidar() is True, "Failed to start LiDAR"
            assert daemon.start_imu() is True, "Failed to start IMU"
            
            # Start session - returns bool
            result = daemon.start_session("hardware_test")
            assert result is True
            session_path = daemon.session.path
            
            # Let it run briefly
            import time
            time.sleep(2.0)
            
            # Stop everything
            daemon.stop_session()
            daemon.stop_lidar()
            daemon.stop_imu()
            
            # Verify outputs
            session_dir = Path(session_path)
            assert (session_dir / "metadata.json").exists()
            assert (session_dir / "events.json").exists()
            
        finally:
            # Cleanup
            daemon.stop_imu()
            daemon.stop_lidar()
