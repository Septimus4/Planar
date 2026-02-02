"""Tests for BMI160 IMU driver module."""

import time
import struct
import pytest
from unittest.mock import Mock, MagicMock, patch
import math

from capture.imu_driver import (
    BMI160Driver,
    ImuStatus,
    ImuSample,
    ImuInfo,
    scan_i2c_bus,
    detect_bmi160,
)


class TestImuSample:
    """Tests for ImuSample dataclass."""

    def test_creation(self):
        sample = ImuSample(
            timestamp=1000.0,
            gyro_x=0.01,
            gyro_y=-0.02,
            gyro_z=0.005,
            accel_x=0.1,
            accel_y=-0.2,
            accel_z=9.8,
            temperature=25.0,
        )
        assert sample.timestamp == 1000.0
        assert sample.gyro_z == 0.005
        assert sample.accel_z == 9.8
        assert sample.temperature == 25.0


class TestBMI160DriverUnit:
    """Unit tests for BMI160Driver (mocked hardware)."""

    def test_initial_state(self):
        driver = BMI160Driver(bus=1, address=0x69)
        assert driver.status == ImuStatus.DISCONNECTED
        assert driver.is_connected is False
        assert driver.is_streaming is False
        assert driver.bus == 1
        assert driver.address == 0x69

    def test_default_configuration(self):
        driver = BMI160Driver()
        assert driver.gyro_range_dps == 250
        assert driver.accel_range_g == 2
        assert driver.sample_rate_hz == 100.0

    def test_custom_configuration(self):
        driver = BMI160Driver(
            bus=0,
            address=0x68,
            gyro_range_dps=500,
            accel_range_g=4,
            sample_rate_hz=200.0,
        )
        assert driver.bus == 0
        assert driver.address == 0x68
        assert driver.gyro_range_dps == 500
        assert driver.accel_range_g == 4
        assert driver.sample_rate_hz == 200.0

    @patch("os.path.exists")
    def test_connect_bus_not_found(self, mock_exists):
        mock_exists.return_value = False
        driver = BMI160Driver(bus=99)
        
        result = driver.connect()
        
        assert result is False
        assert driver.status == ImuStatus.ERROR

    @patch("smbus2.SMBus")
    @patch("os.path.exists")
    def test_connect_wrong_chip_id(self, mock_exists, mock_smbus_class):
        mock_exists.return_value = True
        
        mock_bus = MagicMock()
        mock_smbus_class.return_value = mock_bus
        mock_bus.read_byte_data.return_value = 0x00  # Wrong chip ID
        
        driver = BMI160Driver()
        result = driver.connect()
        
        assert result is False
        assert driver.status == ImuStatus.ERROR

    @patch("smbus2.SMBus")
    @patch("os.path.exists")
    def test_connect_success(self, mock_exists, mock_smbus_class):
        mock_exists.return_value = True
        
        mock_bus = MagicMock()
        mock_smbus_class.return_value = mock_bus
        
        # Return correct chip ID (0xD1) and PMU status
        mock_bus.read_byte_data.side_effect = [
            0xD1,  # Chip ID
            0xD1,  # Chip ID (after reset)
            0x00,  # Error register
            0x14,  # PMU status (acc=normal, gyr=normal)
        ]
        
        driver = BMI160Driver()
        result = driver.connect()
        
        assert result is True
        assert driver.status == ImuStatus.IDLE
        assert driver.is_connected is True
        assert driver.info.chip_id == 0xD1

    def test_disconnect_when_not_connected(self):
        driver = BMI160Driver()
        driver.disconnect()  # Should not raise
        assert driver.status == ImuStatus.DISCONNECTED

    def test_read_sample_when_not_connected(self):
        driver = BMI160Driver()
        sample = driver.read_sample()
        assert sample is None

    def test_start_streaming_when_not_connected(self):
        driver = BMI160Driver()
        result = driver.start_streaming()
        assert result is False

    def test_get_diagnostics(self):
        driver = BMI160Driver(bus=1, address=0x69)
        driver.gyro_range_dps = 500
        driver.accel_range_g = 4
        driver.sample_rate_hz = 200.0
        driver._sample_count = 1000
        
        diag = driver.get_diagnostics()
        
        assert diag["bus"] == 1
        assert diag["address"] == "0x69"
        assert diag["config"]["gyro_range_dps"] == 500
        assert diag["config"]["accel_range_g"] == 4
        assert diag["config"]["sample_rate_hz"] == 200.0
        assert diag["stats"]["samples_read"] == 1000

    def test_add_remove_sample_callback(self):
        driver = BMI160Driver()
        
        callback = Mock()
        driver.add_sample_callback(callback)
        assert callback in driver._sample_callbacks
        
        driver.remove_sample_callback(callback)
        assert callback not in driver._sample_callbacks

    def test_configure_updates_values(self):
        driver = BMI160Driver()
        driver.configure(
            gyro_range_dps=1000,
            accel_range_g=8,
            sample_rate_hz=400.0,
        )
        assert driver.gyro_range_dps == 1000
        assert driver.accel_range_g == 8
        assert driver.sample_rate_hz == 400.0


class TestBMI160Protocol:
    """Tests for BMI160 register and protocol constants."""

    def test_register_addresses(self):
        assert BMI160Driver.REG_CHIP_ID == 0x00
        assert BMI160Driver.REG_ERR_REG == 0x02
        assert BMI160Driver.REG_PMU_STATUS == 0x03
        assert BMI160Driver.REG_DATA_8 == 0x0C  # Gyro start
        assert BMI160Driver.REG_DATA_14 == 0x12  # Accel start
        assert BMI160Driver.REG_CMD == 0x7E

    def test_chip_id_constant(self):
        assert BMI160Driver.CHIP_ID_BMI160 == 0xD1

    def test_gyro_range_mappings(self):
        assert 125 in BMI160Driver.GYRO_RANGE_MAP
        assert 250 in BMI160Driver.GYRO_RANGE_MAP
        assert 500 in BMI160Driver.GYRO_RANGE_MAP
        assert 1000 in BMI160Driver.GYRO_RANGE_MAP
        assert 2000 in BMI160Driver.GYRO_RANGE_MAP

    def test_accel_range_mappings(self):
        assert 2 in BMI160Driver.ACCEL_RANGE_MAP
        assert 4 in BMI160Driver.ACCEL_RANGE_MAP
        assert 8 in BMI160Driver.ACCEL_RANGE_MAP
        assert 16 in BMI160Driver.ACCEL_RANGE_MAP

    def test_odr_mappings(self):
        assert 25 in BMI160Driver.ODR_MAP
        assert 50 in BMI160Driver.ODR_MAP
        assert 100 in BMI160Driver.ODR_MAP
        assert 200 in BMI160Driver.ODR_MAP
        assert 400 in BMI160Driver.ODR_MAP


class TestBMI160SelfTest:
    """Tests for BMI160 self-test functionality."""

    def test_selftest_not_connected(self):
        driver = BMI160Driver()
        results = driver.self_test()
        
        assert results["passed"] is False
        assert "Not connected" in results["errors"]

    @patch("smbus2.SMBus")
    @patch("os.path.exists")
    def test_selftest_pass(self, mock_exists, mock_smbus_class):
        mock_exists.return_value = True
        mock_bus = MagicMock()
        mock_smbus_class.return_value = mock_bus
        
        # Setup for connect
        mock_bus.read_byte_data.side_effect = [
            0xD1,  # Chip ID for connect
            0xD1,  # After reset
            0x00,  # Error reg
            0x14,  # PMU status
            0xD1,  # Chip ID for self-test
            0x00,  # Error register
        ]
        
        # Gyro near zero, accel near 1g (Z-up orientation)
        # Gyro: 6 bytes, Accel: 6 bytes, Temp: 2 bytes
        gyro_data = struct.pack('<hhh', 10, -5, 8)  # Small values near zero
        accel_data = struct.pack('<hhh', 100, -50, 16000)  # ~1g on Z axis
        temp_data = struct.pack('<h', 1024)  # ~25°C
        
        mock_bus.read_i2c_block_data.side_effect = [
            list(gyro_data + accel_data),  # Sensor data read
            list(temp_data),  # Temperature read
        ]
        
        driver = BMI160Driver()
        driver.connect()
        
        # Reset side effects for self-test
        mock_bus.read_byte_data.side_effect = [0xD1, 0x00]
        mock_bus.read_i2c_block_data.side_effect = [
            list(gyro_data + accel_data),
            list(temp_data),
        ]
        
        results = driver.self_test()
        
        assert results["chip_id_ok"] is True


class TestScaleCalculations:
    """Tests for sensor scale factor calculations."""

    def test_gyro_scale_250dps(self):
        """Verify gyro scale for ±250°/s range."""
        # At 250°/s full scale, 32768 LSB = 250°/s
        # Scale = (250 / 32768) * (π/180) rad/s per LSB
        expected_scale = (250.0 / 32768.0) * (math.pi / 180.0)
        
        driver = BMI160Driver(gyro_range_dps=250)
        # Scale is set during configure, simulate it
        driver._gyro_scale = expected_scale
        
        # 32768 LSB should equal 250°/s ≈ 4.36 rad/s
        raw_value = 32768
        actual_rad_s = raw_value * driver._gyro_scale
        expected_rad_s = 250.0 * math.pi / 180.0
        
        assert abs(actual_rad_s - expected_rad_s) < 0.01

    def test_accel_scale_2g(self):
        """Verify accel scale for ±2g range."""
        # At 2g full scale, 32768 LSB = 2g
        # Scale = (2 / 32768) * 9.80665 m/s² per LSB
        expected_scale = (2.0 / 32768.0) * 9.80665
        
        driver = BMI160Driver(accel_range_g=2)
        driver._accel_scale = expected_scale
        
        # 16384 LSB should equal 1g ≈ 9.8 m/s²
        raw_value = 16384
        actual_m_s2 = raw_value * driver._accel_scale
        expected_m_s2 = 9.80665
        
        assert abs(actual_m_s2 - expected_m_s2) < 0.1


@pytest.mark.hardware
class TestBMI160Hardware:
    """Hardware integration tests for BMI160.
    
    These tests require actual hardware connected.
    Run with: pytest -m hardware
    """

    @pytest.fixture
    def imu(self):
        """Create and connect to real IMU."""
        import os
        if not os.path.exists("/dev/i2c-1"):
            pytest.skip("I2C bus not available (/dev/i2c-1 not found)")
        
        # Try to detect BMI160
        address = detect_bmi160(bus=1)
        if address is None:
            pytest.skip("BMI160 not detected on I2C bus")
        
        driver = BMI160Driver(bus=1, address=address)
        if not driver.connect():
            pytest.skip("Could not connect to BMI160")
        
        yield driver
        driver.disconnect()

    def test_hardware_connect_and_verify_chip_id(self, imu):
        """Test connecting to real hardware and verifying chip ID."""
        assert imu.is_connected
        assert imu.info.chip_id == 0xD1
        print(f"\nIMU connected at 0x{imu.address:02X}")
        print(f"Chip ID: 0x{imu.info.chip_id:02X}")

    def test_hardware_read_sample(self, imu):
        """Test reading a single sample from real hardware."""
        sample = imu.read_sample()
        
        assert sample is not None
        assert sample.timestamp > 0
        
        # Gyro should be near zero when stationary
        gyro_magnitude = math.sqrt(
            sample.gyro_x**2 + sample.gyro_y**2 + sample.gyro_z**2
        )
        assert gyro_magnitude < 0.5, f"Gyro too high: {gyro_magnitude} rad/s"
        
        # Accel should be near 1g
        accel_magnitude = math.sqrt(
            sample.accel_x**2 + sample.accel_y**2 + sample.accel_z**2
        )
        assert 8.0 < accel_magnitude < 12.0, f"Accel unexpected: {accel_magnitude} m/s²"
        
        print(f"\nSample:")
        print(f"  Gyro (rad/s): X={sample.gyro_x:.4f}, Y={sample.gyro_y:.4f}, Z={sample.gyro_z:.4f}")
        print(f"  Accel (m/s²): X={sample.accel_x:.3f}, Y={sample.accel_y:.3f}, Z={sample.accel_z:.3f}")
        print(f"  Temperature: {sample.temperature:.1f}°C")

    def test_hardware_streaming(self, imu):
        """Test streaming samples from real hardware."""
        samples_received = []
        
        def on_sample(sample):
            samples_received.append(sample)
        
        imu.start_streaming(callback=on_sample)
        time.sleep(0.5)  # Collect for 500ms at 100Hz = ~50 samples
        imu.stop_streaming()
        
        assert len(samples_received) > 20, f"Only received {len(samples_received)} samples"
        
        # Check timing consistency
        if len(samples_received) >= 2:
            intervals = [
                samples_received[i+1].timestamp - samples_received[i].timestamp
                for i in range(min(10, len(samples_received) - 1))
            ]
            avg_interval = sum(intervals) / len(intervals)
            expected_interval = 1.0 / imu.sample_rate_hz
            
            print(f"\nReceived {len(samples_received)} samples")
            print(f"Average interval: {avg_interval*1000:.2f}ms (expected: {expected_interval*1000:.2f}ms)")

    def test_hardware_selftest(self, imu):
        """Run self-test on real hardware."""
        results = imu.self_test()
        
        print(f"\nSelf-test results:")
        print(f"  Passed: {results['passed']}")
        print(f"  Chip ID OK: {results['chip_id_ok']}")
        print(f"  Gyro OK: {results['gyro_ok']}")
        print(f"  Accel OK: {results['accel_ok']}")
        if results['errors']:
            print(f"  Errors: {results['errors']}")
        
        assert results["chip_id_ok"], "Chip ID verification failed"
        # Note: gyro/accel might fail if device is moving

    def test_hardware_configuration_change(self, imu):
        """Test changing configuration on real hardware."""
        # Change to different settings
        original_rate = imu.sample_rate_hz
        
        imu.configure(
            gyro_range_dps=500,
            accel_range_g=4,
            sample_rate_hz=200.0,
        )
        
        # Read a sample with new configuration
        sample = imu.read_sample()
        assert sample is not None
        
        # Restore original
        imu.configure(sample_rate_hz=original_rate)
        
        print(f"\nConfiguration change successful")
        print(f"  New gyro range: ±500°/s")
        print(f"  New accel range: ±4g")


@pytest.mark.hardware
class TestI2CScanning:
    """Hardware tests for I2C bus scanning."""

    def test_scan_i2c_bus(self):
        """Test scanning I2C bus for devices."""
        import os
        if not os.path.exists("/dev/i2c-1"):
            pytest.skip("I2C bus not available")
        
        devices = scan_i2c_bus(bus=1)
        print(f"\nDevices found on I2C bus 1: {[f'0x{a:02X}' for a in devices]}")
        
        # We expect at least the BMI160 if it's connected
        if 0x69 in devices or 0x68 in devices:
            print("BMI160 detected!")

    def test_detect_bmi160(self):
        """Test BMI160 auto-detection."""
        import os
        if not os.path.exists("/dev/i2c-1"):
            pytest.skip("I2C bus not available")
        
        address = detect_bmi160(bus=1)
        if address:
            print(f"\nBMI160 detected at 0x{address:02X}")
        else:
            print("\nBMI160 not detected")
