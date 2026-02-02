"""Tests for mock drivers and hardware test infrastructure."""

import pytest
import time
from typing import List

from tests.mocks import (
    MockLidarDriver,
    MockIMUDriver,
    create_lidar_driver,
    create_imu_driver,
    simulate_lidar_scan,
)
from capture.lidar_driver import ScanFrame, ScanPoint, LidarStatus
from capture.imu_driver import ImuSample


class TestMockLidarDriver:
    """Tests for the mock LiDAR driver."""
    
    def test_initial_state(self):
        """Test mock starts disconnected."""
        driver = MockLidarDriver()
        assert not driver.is_connected
        assert not driver.is_scanning
        assert driver.status == LidarStatus.DISCONNECTED
    
    def test_connect_disconnect(self):
        """Test connection lifecycle."""
        driver = MockLidarDriver()
        
        # Connect
        assert driver.connect()
        assert driver.is_connected
        assert driver.status == LidarStatus.IDLE
        
        # Disconnect
        driver.disconnect()
        assert not driver.is_connected
        assert driver.status == LidarStatus.DISCONNECTED
    
    def test_device_info(self):
        """Test device info is properly mocked."""
        driver = MockLidarDriver()
        driver.connect()
        
        info = driver.info
        assert "MOCK" in info.model
        assert info.serial_number is not None
        assert info.firmware_version is not None
        
        driver.disconnect()
    
    def test_start_stop_scan(self):
        """Test scanning lifecycle."""
        driver = MockLidarDriver()
        driver.connect()
        
        # Start scan
        assert driver.start_scan()
        assert driver.is_scanning
        assert driver.status == LidarStatus.SCANNING
        
        # Stop scan
        driver.stop_scan()
        assert not driver.is_scanning
        assert driver.status == LidarStatus.IDLE
        
        driver.disconnect()
    
    def test_scan_produces_frames(self):
        """Test that scanning produces frames."""
        driver = MockLidarDriver()
        driver.connect()
        driver.start_scan()
        
        # Wait for scan to produce at least one frame
        time.sleep(0.15)
        
        frame = driver.get_current_frame()
        assert frame is not None
        assert isinstance(frame, ScanFrame)
        assert len(frame.points) > 0
        assert frame.frame_id > 0
        
        driver.stop_scan()
        driver.disconnect()
    
    def test_scan_callback(self):
        """Test that scan callback is called."""
        driver = MockLidarDriver()
        driver.connect()
        
        received_frames: List[ScanFrame] = []
        
        def callback(frame: ScanFrame):
            received_frames.append(frame)
        
        driver.start_scan(callback)
        time.sleep(0.25)  # Wait for a couple of frames
        driver.stop_scan()
        
        assert len(received_frames) >= 2
        
        driver.disconnect()
    
    def test_frame_callbacks(self):
        """Test add/remove frame callbacks."""
        driver = MockLidarDriver()
        driver.connect()
        
        received: List[ScanFrame] = []
        
        def callback(frame: ScanFrame):
            received.append(frame)
        
        driver.add_frame_callback(callback)
        driver.start_scan()
        time.sleep(0.15)
        driver.stop_scan()
        
        assert len(received) >= 1
        
        # Remove callback
        driver.remove_frame_callback(callback)
        received.clear()
        
        driver.start_scan()
        time.sleep(0.15)
        driver.stop_scan()
        
        # Should not receive any frames after removal
        assert len(received) == 0
        
        driver.disconnect()
    
    def test_diagnostics(self):
        """Test diagnostics returns expected data."""
        driver = MockLidarDriver()
        driver.connect()
        
        diag = driver.get_diagnostics()
        assert "status" in diag
        assert "connected" in diag
        assert "scanning" in diag
        assert "info" in diag
        
        driver.disconnect()
    
    def test_custom_room_geometry(self):
        """Test scanning with custom room geometry."""
        # Small 2x2m room
        walls = [
            (0, 0, 2, 0),  # Bottom
            (2, 0, 2, 2),  # Right
            (2, 2, 0, 2),  # Top
            (0, 2, 0, 0),  # Left
        ]
        
        driver = MockLidarDriver(room_walls=walls, scanner_pos=(1, 1))
        driver.connect()
        driver.start_scan()
        time.sleep(0.12)
        
        frame = driver.get_current_frame()
        assert frame is not None
        
        # Find a point that should be ~1m away (center of 2x2 room)
        distances = [p.distance_m for p in frame.points if p.quality > 0]
        assert len(distances) > 0
        
        # Most distances should be around 1m (±0.5m for corners)
        avg_dist = sum(distances) / len(distances)
        assert 0.5 < avg_dist < 1.5
        
        driver.stop_scan()
        driver.disconnect()


class TestMockIMUDriver:
    """Tests for the mock IMU driver."""
    
    def test_initial_state(self):
        """Test mock starts disconnected."""
        driver = MockIMUDriver()
        assert not driver.is_connected
        assert not driver.is_streaming
    
    def test_connect_disconnect(self):
        """Test connection lifecycle."""
        driver = MockIMUDriver()
        
        # Connect
        assert driver.connect()
        assert driver.is_connected
        
        # Disconnect
        driver.disconnect()
        assert not driver.is_connected
    
    def test_chip_id(self):
        """Test chip ID matches BMI160."""
        driver = MockIMUDriver()
        driver.connect()
        
        chip_id = driver.get_chip_id()
        assert chip_id == MockIMUDriver.CHIP_ID_BMI160
        
        driver.disconnect()
    
    def test_read_sample(self):
        """Test reading a single sample."""
        driver = MockIMUDriver()
        driver.connect()
        
        sample = driver.read_sample()
        assert sample is not None
        assert isinstance(sample, ImuSample)
        
        # Check gyro values are reasonable (small noise around 0)
        assert -10 < sample.gyro_x < 10
        assert -10 < sample.gyro_y < 10
        assert -10 < sample.gyro_z < 10
        
        # Check accel has gravity (~9.8 on z-axis)
        assert -2 < sample.accel_x < 2
        assert -2 < sample.accel_y < 2
        assert 8 < sample.accel_z < 11
        
        # Check temperature is reasonable
        assert 15 < sample.temperature < 35
        
        driver.disconnect()
    
    def test_streaming(self):
        """Test streaming mode."""
        driver = MockIMUDriver()
        driver.connect()
        
        received: List[ImuSample] = []
        
        def callback(sample: ImuSample):
            received.append(sample)
        
        driver.start_streaming(callback)
        assert driver.is_streaming
        
        time.sleep(0.1)  # Collect ~10 samples at 100Hz
        
        driver.stop_streaming()
        assert not driver.is_streaming
        
        # Should have received several samples
        assert len(received) >= 5
        
        driver.disconnect()
    
    def test_set_rotation_rate(self):
        """Test simulated rotation affects gyro readings."""
        import math
        driver = MockIMUDriver()
        driver.connect()
        
        # Set a rotation rate
        driver.set_rotation_rate(50.0)  # 50 deg/s
        
        # Read samples and check gyro_z
        samples = [driver.read_sample() for _ in range(10)]
        # Gyro values are in radians/s, convert to deg/s
        avg_gyro_z = sum(math.degrees(s.gyro_z) for s in samples) / len(samples)
        
        # Should be approximately 50 deg/s (with some noise tolerance)
        assert 40 < avg_gyro_z < 60
        
        driver.disconnect()
    
    def test_set_gyro_bias(self):
        """Test simulated gyro bias."""
        import math
        driver = MockIMUDriver()
        driver.connect()
        
        # Set a gyro bias
        driver.set_gyro_bias(2.0)  # 2 deg/s bias
        
        # Read samples and check average gyro_z
        samples = [driver.read_sample() for _ in range(20)]
        # Gyro values are in radians/s, convert to deg/s
        avg_gyro_z = sum(math.degrees(s.gyro_z) for s in samples) / len(samples)
        
        # Should be approximately 2 deg/s (with some noise)
        assert 0.5 < avg_gyro_z < 3.5
        
        driver.disconnect()
    
    def test_diagnostics(self):
        """Test diagnostics returns expected data."""
        driver = MockIMUDriver()
        driver.connect()
        
        diag = driver.get_diagnostics()
        assert "connected" in diag
        assert "streaming" in diag
        assert "info" in diag
        assert "chip_id" in diag["info"]  # chip_id is nested under info
        
        driver.disconnect()
    
    def test_self_test(self):
        """Test self-test always passes for mock."""
        driver = MockIMUDriver()
        driver.connect()
        
        result = driver.self_test()
        # self_test returns a dict with passed, gyro, accel keys
        assert isinstance(result, dict)
        assert result["passed"] is True
        
        driver.disconnect()


class TestSimulateLidarScan:
    """Tests for the LiDAR scan simulation function."""
    
    def test_basic_scan(self):
        """Test basic scan produces points."""
        points = simulate_lidar_scan()
        assert len(points) > 0
        assert len(points) <= 360  # Default 1-degree resolution
    
    def test_point_format(self):
        """Test point format is (angle, distance, quality)."""
        points = simulate_lidar_scan()
        
        for point in points:
            assert len(point) == 3
            angle, distance, quality = point
            assert 0 <= angle < 360
            assert distance >= 0
            assert 0 <= quality <= 100
    
    def test_custom_room(self):
        """Test with custom room geometry."""
        # Long corridor
        walls = [
            (0, 0, 10, 0),   # Bottom
            (10, 0, 10, 1),  # Right
            (10, 1, 0, 1),   # Top
            (0, 1, 0, 0),    # Left
        ]
        
        points = simulate_lidar_scan(origin=(5, 0.5), walls=walls)
        
        # Points should exist
        assert len(points) > 0


class TestFactoryFunctions:
    """Tests for driver factory functions."""
    
    def test_create_lidar_driver_mock(self):
        """Test factory creates mock driver by default."""
        driver = create_lidar_driver()
        assert isinstance(driver, MockLidarDriver)
    
    def test_create_lidar_driver_explicit_mock(self):
        """Test factory creates mock when explicitly requested."""
        driver = create_lidar_driver(use_hardware=False)
        assert isinstance(driver, MockLidarDriver)
    
    def test_create_imu_driver_mock(self):
        """Test factory creates mock driver by default."""
        driver = create_imu_driver()
        assert isinstance(driver, MockIMUDriver)
    
    def test_create_imu_driver_explicit_mock(self):
        """Test factory creates mock when explicitly requested."""
        driver = create_imu_driver(use_hardware=False)
        assert isinstance(driver, MockIMUDriver)


class TestConfTestFixtures:
    """Tests that conftest fixtures work correctly with mocks."""
    
    def test_mock_lidar_fixture(self, mock_lidar):
        """Test the mock_lidar fixture."""
        assert mock_lidar.is_connected
        assert isinstance(mock_lidar, MockLidarDriver)
    
    def test_mock_imu_fixture(self, mock_imu):
        """Test the mock_imu fixture."""
        assert mock_imu.is_connected
        assert isinstance(mock_imu, MockIMUDriver)
    
    def test_lidar_driver_fixture_uses_mock(self, lidar_driver):
        """Test the lidar_driver fixture uses mock by default."""
        # Without --hardware flag, should be mock
        assert isinstance(lidar_driver, MockLidarDriver)
    
    def test_imu_driver_fixture_uses_mock(self, imu_driver):
        """Test the imu_driver fixture uses mock by default."""
        # Without --hardware flag, should be mock
        assert isinstance(imu_driver, MockIMUDriver)


@pytest.mark.hardware
class TestHardwareMockIntegration:
    """Integration tests that work with both mock and real hardware."""
    
    def test_lidar_scan_cycle(self, lidar_driver):
        """Test complete LiDAR scan cycle."""
        # Already connected by fixture
        assert lidar_driver.is_connected
        
        # Start scanning
        assert lidar_driver.start_scan()
        assert lidar_driver.is_scanning
        
        # Wait for frames
        time.sleep(0.2)
        
        # Get a frame
        frame = lidar_driver.get_current_frame()
        assert frame is not None
        assert len(frame.points) > 0
        
        # Stop scanning
        lidar_driver.stop_scan()
        assert not lidar_driver.is_scanning
    
    def test_imu_sample_reading(self, imu_driver):
        """Test IMU sample reading."""
        # Already connected by fixture
        assert imu_driver.is_connected
        
        # Read a sample
        sample = imu_driver.read_sample()
        assert sample is not None
        
        # Should have reasonable values
        assert sample.temperature > 0
        assert -1000 < sample.gyro_x < 1000
        assert -100 < sample.accel_z < 100
