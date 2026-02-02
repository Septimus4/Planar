"""Tests for RPLidar driver module."""

import time
import struct
import pytest
from unittest.mock import Mock, MagicMock, patch, PropertyMock
from typing import List

from capture.lidar_driver import (
    RPLidarDriver,
    LidarStatus,
    ScanPoint,
    ScanFrame,
    LidarInfo,
)


class TestScanPoint:
    """Tests for ScanPoint dataclass."""

    def test_creation(self):
        point = ScanPoint(
            angle_deg=45.0,
            distance_m=2.5,
            quality=60,
            timestamp=1000.0,
        )
        assert point.angle_deg == 45.0
        assert point.distance_m == 2.5
        assert point.quality == 60
        assert point.timestamp == 1000.0


class TestScanFrame:
    """Tests for ScanFrame dataclass."""

    def test_creation(self):
        points = [
            ScanPoint(angle_deg=i * 10, distance_m=1.0, quality=50, timestamp=1000.0)
            for i in range(36)
        ]
        frame = ScanFrame(
            points=points,
            timestamp=1000.0,
            scan_rate_hz=10.0,
            frame_id=1,
        )
        assert len(frame.points) == 36
        assert frame.scan_rate_hz == 10.0
        assert frame.frame_id == 1


class TestRPLidarDriverUnit:
    """Unit tests for RPLidarDriver (mocked hardware)."""

    def test_initial_state(self):
        driver = RPLidarDriver(port="/dev/test", baudrate=115200)
        assert driver.status == LidarStatus.DISCONNECTED
        assert driver.is_connected is False
        assert driver.is_scanning is False
        assert driver.port == "/dev/test"
        assert driver.baudrate == 115200

    def test_configure_valid_mode(self):
        driver = RPLidarDriver()
        assert driver.configure(scan_mode="DenseBoost", scan_frequency_hz=10.0) is True
        assert driver._scan_mode == "DenseBoost"
        assert driver._scan_frequency == 10.0

    def test_configure_invalid_mode(self):
        driver = RPLidarDriver()
        assert driver.configure(scan_mode="InvalidMode") is False

    def test_configure_standard_mode(self):
        driver = RPLidarDriver()
        assert driver.configure(scan_mode="Standard", scan_frequency_hz=5.0) is True
        assert driver._scan_mode == "Standard"

    @patch("os.path.exists")
    def test_connect_device_not_found(self, mock_exists):
        mock_exists.return_value = False
        driver = RPLidarDriver(port="/dev/nonexistent")
        
        result = driver.connect()
        
        assert result is False
        assert driver.status == LidarStatus.ERROR

    @patch("serial.Serial")
    @patch("os.path.exists")
    def test_connect_success(self, mock_exists, mock_serial_class):
        mock_exists.return_value = True
        
        # Create mock serial instance
        mock_serial = MagicMock()
        mock_serial_class.return_value = mock_serial
        
        # Mock response for get_info command
        # Response descriptor: A5 5A [len 4 bytes] [type]
        info_descriptor = bytes([0xA5, 0x5A, 20, 0, 0, 0, 0x04])
        info_data = bytes([0x00] * 4 + [0x12, 0x34, 0x56, 0x78] * 4)  # 20 bytes
        
        # Mock response for health check
        health_descriptor = bytes([0xA5, 0x5A, 3, 0, 0, 0, 0x06])
        health_data = bytes([0x00, 0x00, 0x00])  # Status OK
        
        mock_serial.read.side_effect = [
            info_descriptor,
            info_data,
            health_descriptor,
            health_data,
        ]
        
        driver = RPLidarDriver(port="/dev/rplidar")
        result = driver.connect()
        
        assert result is True
        assert driver.status == LidarStatus.IDLE
        assert driver.is_connected is True

    def test_disconnect_when_not_connected(self):
        driver = RPLidarDriver()
        driver.disconnect()  # Should not raise
        assert driver.status == LidarStatus.DISCONNECTED

    def test_start_scan_when_not_connected(self):
        driver = RPLidarDriver()
        result = driver.start_scan()
        assert result is False

    def test_get_diagnostics(self):
        driver = RPLidarDriver(port="/dev/test", baudrate=500000)
        driver._scan_mode = "Standard"
        driver._scan_frequency = 5.0
        driver._frame_count = 100
        
        diag = driver.get_diagnostics()
        
        assert diag["port"] == "/dev/test"
        assert diag["baudrate"] == 500000
        assert diag["config"]["scan_mode"] == "Standard"
        assert diag["config"]["scan_frequency_hz"] == 5.0
        assert diag["stats"]["frames_captured"] == 100

    def test_add_remove_frame_callback(self):
        driver = RPLidarDriver()
        
        callback = Mock()
        driver.add_frame_callback(callback)
        assert callback in driver._frame_callbacks
        
        driver.remove_frame_callback(callback)
        assert callback not in driver._frame_callbacks

    def test_get_current_frame_initially_none(self):
        driver = RPLidarDriver()
        assert driver.get_current_frame() is None


class TestRPLidarProtocol:
    """Tests for RPLidar protocol handling."""

    def test_command_constants(self):
        assert RPLidarDriver.SYNC_BYTE == 0xA5
        assert RPLidarDriver.SYNC_BYTE2 == 0x5A
        assert RPLidarDriver.CMD_STOP == 0x25
        assert RPLidarDriver.CMD_GET_INFO == 0x50
        assert RPLidarDriver.CMD_GET_HEALTH == 0x52

    def test_scan_modes_defined(self):
        assert "Standard" in RPLidarDriver.SCAN_MODES
        assert "DenseBoost" in RPLidarDriver.SCAN_MODES
        assert RPLidarDriver.SCAN_MODES["DenseBoost"]["sample_rate"] == 32000


@pytest.mark.hardware
class TestRPLidarHardware:
    """Hardware integration tests for RPLidar.
    
    These tests require actual hardware connected.
    Run with: pytest -m hardware
    """

    @pytest.fixture
    def lidar(self):
        """Create and connect to real LiDAR."""
        import os
        if not os.path.exists("/dev/rplidar"):
            pytest.skip("LiDAR not connected (/dev/rplidar not found)")
        
        driver = RPLidarDriver(port="/dev/rplidar", baudrate=1_000_000)
        if not driver.connect():
            pytest.skip("Could not connect to LiDAR")
        
        yield driver
        driver.disconnect()

    def test_hardware_connect_and_get_info(self, lidar):
        """Test connecting to real hardware and reading device info."""
        assert lidar.is_connected
        assert lidar.info.serial_number != ""
        assert lidar.info.firmware_version != ""
        print(f"\nLiDAR Info:")
        print(f"  Model: {lidar.info.model}")
        print(f"  S/N: {lidar.info.serial_number}")
        print(f"  Firmware: {lidar.info.firmware_version}")
        print(f"  Health: {lidar.info.health_status}")

    def test_hardware_scan_frames(self, lidar):
        """Test receiving scan frames from real hardware."""
        frames_received = []
        
        def on_frame(frame):
            frames_received.append(frame)
        
        # Use Standard mode which works reliably
        lidar.configure(scan_mode="Standard", scan_frequency_hz=10.0)
        lidar.start_scan(callback=on_frame)
        
        # Wait for a few frames
        time.sleep(2.0)
        lidar.stop_scan()
        
        assert len(frames_received) > 0, "Should receive at least one frame"
        
        frame = frames_received[0]
        assert len(frame.points) > 100, "Frame should have many points"
        assert frame.scan_rate_hz > 0
        
        print(f"\nReceived {len(frames_received)} frames")
        print(f"First frame: {len(frame.points)} points, {frame.scan_rate_hz:.1f} Hz")

    def test_hardware_scan_data_validity(self, lidar):
        """Test that scan data is reasonable."""
        lidar.configure(scan_mode="Standard")
        lidar.start_scan()
        time.sleep(1.0)
        
        frame = lidar.get_current_frame()
        lidar.stop_scan()
        
        assert frame is not None
        
        # Check data validity
        angles = [p.angle_deg for p in frame.points]
        distances = [p.distance_m for p in frame.points]
        
        # Angles should span close to 360 degrees
        angle_range = max(angles) - min(angles)
        assert angle_range > 300, f"Angle range too small: {angle_range}"
        
        # Distances should be reasonable (0.1m to 40m for S3)
        valid_distances = [d for d in distances if 0.1 < d < 40.0]
        assert len(valid_distances) > len(distances) * 0.5, "Too many invalid distances"
        
        print(f"\nAngle range: {min(angles):.1f}° - {max(angles):.1f}°")
        print(f"Distance range: {min(valid_distances):.2f}m - {max(valid_distances):.2f}m")
