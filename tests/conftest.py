"""Pytest configuration and shared fixtures."""

import pytest
import sys
from pathlib import Path
from unittest.mock import Mock, MagicMock

# Add project root to path
project_root = Path(__file__).parent.parent
sys.path.insert(0, str(project_root))


def pytest_addoption(parser):
    """Add custom command-line options."""
    parser.addoption(
        "--hardware",
        action="store_true",
        default=False,
        help="Run tests that require physical hardware (LiDAR, IMU)",
    )


def pytest_configure(config):
    """Register custom markers."""
    config.addinivalue_line(
        "markers", "hardware: marks tests as requiring hardware (run with --hardware)"
    )
    config.addinivalue_line(
        "markers", "asyncio: marks tests as async"
    )


def pytest_collection_modifyitems(config, items):
    """Skip hardware tests unless --hardware flag is provided."""
    if config.getoption("--hardware"):
        # --hardware given: don't skip hardware tests
        return
    
    skip_hardware = pytest.mark.skip(reason="need --hardware option to run")
    for item in items:
        if "hardware" in item.keywords:
            item.add_marker(skip_hardware)


@pytest.fixture
def mock_serial():
    """Mock serial.Serial for LiDAR driver tests."""
    with pytest.importorskip("unittest.mock").patch('serial.Serial') as mock:
        mock_instance = MagicMock()
        mock_instance.is_open = True
        mock_instance.in_waiting = 0
        mock.return_value = mock_instance
        yield mock_instance


@pytest.fixture
def mock_smbus():
    """Mock smbus2.SMBus for IMU driver tests."""
    with pytest.importorskip("unittest.mock").patch('smbus2.SMBus') as mock:
        mock_instance = MagicMock()
        # Return BMI160 chip ID by default
        mock_instance.read_byte_data.return_value = 0xD1
        mock.return_value = mock_instance
        yield mock_instance


@pytest.fixture
def sample_config():
    """Create a sample CaptureConfig."""
    from capture.config import CaptureConfig
    return CaptureConfig()


@pytest.fixture
def sample_config_dict():
    """Sample configuration as dictionary."""
    return {
        "lidar": {
            "device": "/dev/rplidar",
            "baudrate": 1000000,
            "scan_mode": "DenseBoost",
            "scan_frequency_hz": 10.0,
            "enabled": True,
        },
        "imu": {
            "i2c_bus": 1,
            "i2c_address": 0x69,
            "sample_rate_hz": 100.0,
            "gyro_range": 250,
            "accel_range": 2,
            "enabled": True,
        },
        "server": {
            "host": "0.0.0.0",
            "http_port": 8080,
            "ws_port": 8081,
            "auth_token": None,
        },
        "session": {
            "session_dir": "/var/planar/sessions",
            "max_sessions": 100,
            "auto_cleanup": True,
        }
    }


@pytest.fixture
def mock_lidar_driver():
    """Create a mock LiDAR driver."""
    driver = Mock()
    driver.connect.return_value = True
    driver.disconnect.return_value = None
    driver.is_connected = True
    driver.is_scanning = False
    driver.status = "idle"
    driver.info = Mock(
        model="RPLIDAR S3",
        serial_number="ABC123",
        firmware_version="1.2.3",
    )
    return driver


@pytest.fixture
def mock_imu_driver():
    """Create a mock IMU driver."""
    driver = Mock()
    driver.connect.return_value = True
    driver.disconnect.return_value = None
    driver.is_connected = True
    driver.is_streaming = False
    driver.status = "idle"
    driver.chip_id = 0xD1
    driver.self_test.return_value = {
        "passed": True,
        "gyro": True,
        "accel": True,
    }
    return driver


@pytest.fixture
def sample_scan_points():
    """Generate sample scan points."""
    from capture.lidar_driver import ScanPoint
    import time
    
    now = time.time()
    return [
        ScanPoint(angle_deg=float(i * 10), distance_m=1.0 + i * 0.01, quality=50, timestamp=now)
        for i in range(36)
    ]


@pytest.fixture
def sample_scan_frame(sample_scan_points):
    """Generate a sample scan frame."""
    from capture.lidar_driver import ScanFrame
    import time
    
    return ScanFrame(
        frame_id=1,
        timestamp=time.time(),
        scan_rate_hz=10.0,
        points=sample_scan_points,
    )


@pytest.fixture
def sample_imu_sample():
    """Generate a sample IMU reading."""
    from capture.imu_driver import ImuSample
    import time
    
    return ImuSample(
        timestamp=time.time(),
        gyro_x=0.01,
        gyro_y=0.02,
        gyro_z=0.03,
        accel_x=0.0,
        accel_y=0.0,
        accel_z=9.81,
        temperature=25.0,
    )


# Hardware detection fixtures
@pytest.fixture
def hardware_available():
    """Check if hardware is available for testing."""
    import os
    
    lidar_available = os.path.exists("/dev/rplidar")
    
    imu_available = False
    try:
        import smbus2
        bus = smbus2.SMBus(1)
        chip_id = bus.read_byte_data(0x69, 0x00)
        imu_available = (chip_id == 0xD1)
        bus.close()
    except Exception:
        pass
    
    return {
        "lidar": lidar_available,
        "imu": imu_available,
        "all": lidar_available and imu_available,
    }


@pytest.fixture
def skip_without_hardware(hardware_available):
    """Skip test if hardware is not available."""
    if not hardware_available["all"]:
        pytest.skip("Hardware not available")


# ============================================================================
# Mock Driver Fixtures
# ============================================================================

@pytest.fixture
def mock_lidar():
    """Create a mock LiDAR driver with simulated room scanning."""
    from tests.mocks import MockLidarDriver
    driver = MockLidarDriver()
    driver.connect()
    yield driver
    driver.disconnect()


@pytest.fixture
def mock_imu():
    """Create a mock IMU driver with simulated sensor data."""
    from tests.mocks import MockIMUDriver
    driver = MockIMUDriver()
    driver.connect()
    yield driver
    driver.disconnect()


@pytest.fixture
def mock_lidar_with_rotation(mock_imu):
    """Mock IMU that simulates a rotation for yaw testing."""
    mock_imu.set_rotation_rate(10.0)  # 10 deg/s rotation
    return mock_imu


@pytest.fixture
def lidar_driver(request):
    """Get LiDAR driver - mock by default, real with --hardware."""
    use_hardware = request.config.getoption("--hardware")
    
    if use_hardware:
        from capture.lidar_driver import RPLidarDriver
        driver = RPLidarDriver()
        if not driver.connect():
            pytest.skip("Could not connect to LiDAR hardware")
        yield driver
        driver.disconnect()
    else:
        from tests.mocks import MockLidarDriver
        driver = MockLidarDriver()
        driver.connect()
        yield driver
        driver.disconnect()


@pytest.fixture
def imu_driver(request):
    """Get IMU driver - mock by default, real with --hardware."""
    use_hardware = request.config.getoption("--hardware")
    
    if use_hardware:
        from capture.imu_driver import BMI160Driver
        driver = BMI160Driver()
        if not driver.connect():
            pytest.skip("Could not connect to IMU hardware")
        yield driver
        driver.disconnect()
    else:
        from tests.mocks import MockIMUDriver
        driver = MockIMUDriver()
        driver.connect()
        yield driver
        driver.disconnect()
