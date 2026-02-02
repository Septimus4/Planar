"""Mock hardware adapters for testing without physical devices.

This module provides mock implementations of the LiDAR and IMU drivers
that simulate realistic hardware behavior for testing purposes.

Usage:
    # For unit tests (automatic)
    pytest tests/  # Uses mocks by default
    
    # For hardware tests
    pytest tests/ --hardware  # Uses real hardware
    
    # In code
    from tests.mocks import MockLidarDriver, MockIMUDriver
    lidar = MockLidarDriver()
    imu = MockIMUDriver()
"""

import time
import math
import random
import threading
from typing import Optional, Callable, List, Dict, Any, Union
from dataclasses import dataclass
from enum import Enum

# Import the real types for compatibility
from capture.lidar_driver import (
    LidarStatus, ScanPoint, ScanFrame, LidarInfo, RPLidarDriver
)
from capture.imu_driver import (
    ImuStatus, ImuSample, ImuInfo, BMI160Driver
)


# ============================================================================
# Realistic Mock Data
# ============================================================================

# Sample room geometry for LiDAR simulation (simple 5m x 4m room)
MOCK_ROOM_WALLS = [
    # (x1, y1, x2, y2) - wall segments
    (0.0, 0.0, 5.0, 0.0),   # Bottom wall
    (5.0, 0.0, 5.0, 4.0),   # Right wall
    (5.0, 4.0, 0.0, 4.0),   # Top wall
    (0.0, 4.0, 0.0, 0.0),   # Left wall
]

# Scanner position in room (center)
MOCK_SCANNER_POS = (2.5, 2.0)


def ray_segment_intersection(
    ray_origin: tuple, ray_angle_rad: float,
    seg_start: tuple, seg_end: tuple
) -> Optional[float]:
    """Calculate intersection of ray with line segment.
    
    Returns distance to intersection or None if no intersection.
    """
    ox, oy = ray_origin
    dx = math.cos(ray_angle_rad)
    dy = math.sin(ray_angle_rad)
    
    x1, y1 = seg_start
    x2, y2 = seg_end
    
    # Line segment direction
    sx = x2 - x1
    sy = y2 - y1
    
    denom = dx * sy - dy * sx
    if abs(denom) < 1e-10:
        return None  # Parallel
    
    t = ((x1 - ox) * sy - (y1 - oy) * sx) / denom
    u = ((x1 - ox) * dy - (y1 - oy) * dx) / denom
    
    if t > 0 and 0 <= u <= 1:
        return t
    return None


def simulate_lidar_scan(
    origin: tuple = MOCK_SCANNER_POS,
    walls: List[tuple] = MOCK_ROOM_WALLS,
    num_points: int = 360,
    noise_m: float = 0.005,
    max_range: float = 40.0,
) -> List[tuple]:
    """Simulate a LiDAR scan in a room.
    
    Returns list of (angle_deg, distance_m, quality) tuples.
    """
    points = []
    
    for i in range(num_points):
        angle_deg = i * 360.0 / num_points
        angle_rad = math.radians(angle_deg)
        
        # Find nearest wall intersection
        min_dist = max_range
        for wall in walls:
            seg_start = (wall[0], wall[1])
            seg_end = (wall[2], wall[3])
            
            dist = ray_segment_intersection(origin, angle_rad, seg_start, seg_end)
            if dist is not None and dist < min_dist:
                min_dist = dist
        
        # Add noise
        if min_dist < max_range:
            min_dist += random.gauss(0, noise_m)
            min_dist = max(0.15, min_dist)  # Minimum range
            quality = random.randint(40, 60)
        else:
            quality = 0
        
        points.append((angle_deg, min_dist, quality))
    
    return points


# ============================================================================
# Mock LiDAR Driver
# ============================================================================

class MockLidarDriver:
    """Mock RPLidar driver that simulates realistic behavior."""
    
    def __init__(
        self,
        port: str = "/dev/mock_lidar",
        baudrate: int = 1_000_000,
        timeout: float = 1.0,
        scan_rate_hz: float = 10.0,
        room_walls: Optional[List[tuple]] = None,
        scanner_pos: Optional[tuple] = None,
    ):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self._scan_rate_hz = scan_rate_hz
        
        # Room simulation
        self._room_walls = room_walls or MOCK_ROOM_WALLS
        self._scanner_pos = scanner_pos or MOCK_SCANNER_POS
        
        # State
        self._status = LidarStatus.DISCONNECTED
        self._info = LidarInfo(
            model="RPLIDAR S3 (MOCK)",
            serial_number="MOCK123456",
            firmware_version="1.0.0",
            hardware_revision="1",
            health_status="good",
        )
        self._connected = False
        self._scanning = False
        self._frame_id = 0
        self._current_frame: Optional[ScanFrame] = None
        
        # Threading
        self._scan_thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._callback: Optional[Callable[[ScanFrame], None]] = None
        self._frame_callbacks: List[Callable[[ScanFrame], None]] = []
    
    @property
    def status(self) -> LidarStatus:
        return self._status
    
    @property
    def info(self) -> LidarInfo:
        return self._info
    
    @property
    def is_connected(self) -> bool:
        return self._connected
    
    @property
    def is_scanning(self) -> bool:
        return self._scanning
    
    def connect(self) -> bool:
        """Simulate connection to device."""
        self._status = LidarStatus.CONNECTING
        time.sleep(0.1)  # Simulate connection delay
        
        self._connected = True
        self._status = LidarStatus.IDLE
        return True
    
    def disconnect(self):
        """Disconnect from device."""
        if self._scanning:
            self.stop_scan()
        self._connected = False
        self._status = LidarStatus.DISCONNECTED
    
    def get_info(self) -> LidarInfo:
        """Get device information."""
        return self._info
    
    def get_health(self) -> Dict[str, Any]:
        """Get device health status."""
        return {
            "status": "good",
            "error_code": 0,
        }
    
    def get_diagnostics(self) -> Dict[str, Any]:
        """Get full diagnostics."""
        return {
            "status": self._status.value,
            "connected": self._connected,
            "scanning": self._scanning,
            "info": {
                "model": self._info.model,
                "serial_number": self._info.serial_number,
                "firmware_version": self._info.firmware_version,
            },
            "health": self.get_health(),
        }
    
    def configure(
        self,
        scan_mode: str = "Standard",
        scan_frequency_hz: float = 10.0,
    ) -> bool:
        """Configure scan parameters."""
        self._scan_rate_hz = scan_frequency_hz
        return True
    
    def start_scan(self, callback: Optional[Callable[[ScanFrame], None]] = None) -> bool:
        """Start continuous scanning with optional callback."""
        if not self._connected:
            return False
        
        if self._scanning:
            return True
        
        self._callback = callback
        self._scanning = True
        self._status = LidarStatus.SCANNING
        self._stop_event.clear()
        
        self._scan_thread = threading.Thread(target=self._scan_loop, daemon=True)
        self._scan_thread.start()
        
        return True
    
    def stop_scan(self):
        """Stop scanning."""
        self._stop_event.set()
        self._scanning = False
        self._status = LidarStatus.IDLE
        
        if self._scan_thread and self._scan_thread.is_alive():
            self._scan_thread.join(timeout=1.0)
    
    def _scan_loop(self):
        """Background thread generating scan frames."""
        interval = 1.0 / self._scan_rate_hz
        
        while not self._stop_event.is_set():
            frame_start = time.time()
            
            # Generate simulated scan
            raw_points = simulate_lidar_scan(
                origin=self._scanner_pos,
                walls=self._room_walls,
            )
            
            # Convert to ScanPoints
            timestamp = time.time()
            points = [
                ScanPoint(
                    angle_deg=p[0],
                    distance_m=p[1],
                    quality=p[2],
                    timestamp=timestamp,
                )
                for p in raw_points
            ]
            
            self._frame_id += 1
            frame = ScanFrame(
                frame_id=self._frame_id,
                timestamp=timestamp,
                scan_rate_hz=self._scan_rate_hz,
                points=points,
            )
            
            # Store current frame
            self._current_frame = frame
            
            if self._callback:
                try:
                    self._callback(frame)
                except Exception as e:
                    pass  # Don't crash on callback errors
            
            # Call all registered frame callbacks
            for cb in self._frame_callbacks:
                try:
                    cb(frame)
                except Exception:
                    pass
            
            # Maintain scan rate
            elapsed = time.time() - frame_start
            if elapsed < interval:
                time.sleep(interval - elapsed)
    
    def get_current_frame(self) -> Optional[ScanFrame]:
        """Get the most recent scan frame."""
        return self._current_frame
    
    def add_frame_callback(self, callback: Callable[[ScanFrame], None]):
        """Register a callback for new frames."""
        if callback not in self._frame_callbacks:
            self._frame_callbacks.append(callback)
    
    def remove_frame_callback(self, callback: Callable[[ScanFrame], None]):
        """Unregister a frame callback."""
        if callback in self._frame_callbacks:
            self._frame_callbacks.remove(callback)


# ============================================================================
# Mock IMU Driver
# ============================================================================

class MockIMUDriver:
    """Mock BMI160 driver that simulates realistic behavior."""
    
    CHIP_ID_BMI160 = 0xD1
    
    def __init__(
        self,
        bus: int = 1,
        address: int = 0x69,
        gyro_range_dps: int = 250,
        accel_range_g: int = 2,
        sample_rate_hz: float = 100.0,
        # Simulation parameters
        gyro_noise_dps: float = 0.1,
        accel_noise_g: float = 0.01,
        gyro_bias_dps: float = 0.0,
    ):
        self.bus = bus
        self.address = address
        self._gyro_range_dps = gyro_range_dps
        self._accel_range_g = accel_range_g
        self._sample_rate_hz = sample_rate_hz
        
        # Simulation noise
        self._gyro_noise_dps = gyro_noise_dps
        self._accel_noise_g = accel_noise_g
        self._gyro_bias_dps = gyro_bias_dps
        
        # State
        self._status = ImuStatus.DISCONNECTED
        self._info = ImuInfo(
            chip_id=self.CHIP_ID_BMI160,
            bus=bus,
            address=address,
            connected=False,
        )
        self._connected = False
        self._streaming = False
        
        # Simulated orientation
        self._yaw_deg = 0.0
        self._rotation_rate_dps = 0.0  # Can be set for testing
        
        # Threading
        self._stream_thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._callback: Optional[Callable[[ImuSample], None]] = None
    
    @property
    def status(self) -> ImuStatus:
        return self._status
    
    @property
    def info(self) -> ImuInfo:
        return self._info
    
    @property
    def is_connected(self) -> bool:
        return self._connected
    
    @property
    def is_streaming(self) -> bool:
        return self._streaming
    
    def connect(self) -> bool:
        """Simulate connection to device."""
        self._status = ImuStatus.CONNECTING
        time.sleep(0.05)  # Simulate I2C init
        
        self._connected = True
        self._info.connected = True
        self._status = ImuStatus.IDLE
        return True
    
    def disconnect(self):
        """Disconnect from device."""
        if self._streaming:
            self.stop_streaming()
        self._connected = False
        self._info.connected = False
        self._status = ImuStatus.DISCONNECTED
    
    def get_chip_id(self) -> int:
        """Get the chip ID."""
        return self.CHIP_ID_BMI160
    
    def configure(
        self,
        gyro_range_dps: Optional[int] = None,
        accel_range_g: Optional[int] = None,
        sample_rate_hz: Optional[float] = None,
    ) -> bool:
        """Configure IMU parameters."""
        if gyro_range_dps is not None:
            self._gyro_range_dps = gyro_range_dps
        if accel_range_g is not None:
            self._accel_range_g = accel_range_g
        if sample_rate_hz is not None:
            self._sample_rate_hz = sample_rate_hz
        return True
    
    def get_diagnostics(self) -> Dict[str, Any]:
        """Get full diagnostics."""
        return {
            "status": self._status.value,
            "connected": self._connected,
            "streaming": self._streaming,
            "info": {
                "chip_id": hex(self._info.chip_id),
                "bus": self._info.bus,
                "address": hex(self._info.address),
            },
            "config": {
                "gyro_range_dps": self._gyro_range_dps,
                "accel_range_g": self._accel_range_g,
                "sample_rate_hz": self._sample_rate_hz,
            },
        }
    
    def self_test(self) -> Dict[str, Any]:
        """Run simulated self-test."""
        time.sleep(0.1)  # Simulate test time
        return {
            "passed": True,
            "gyro": True,
            "accel": True,
            "message": "Self-test passed (mock)",
        }
    
    def read_sample(self) -> ImuSample:
        """Read single sample with simulated values."""
        timestamp = time.time()
        
        # Simulate gyro (with noise and optional bias)
        gyro_x = random.gauss(0, math.radians(self._gyro_noise_dps))
        gyro_y = random.gauss(0, math.radians(self._gyro_noise_dps))
        gyro_z = math.radians(self._rotation_rate_dps + self._gyro_bias_dps)
        gyro_z += random.gauss(0, math.radians(self._gyro_noise_dps))
        
        # Simulate accel (gravity pointing down, with noise)
        accel_x = random.gauss(0, self._accel_noise_g * 9.81)
        accel_y = random.gauss(0, self._accel_noise_g * 9.81)
        accel_z = 9.81 + random.gauss(0, self._accel_noise_g * 9.81)
        
        # Temperature around room temp
        temperature = 25.0 + random.gauss(0, 0.5)
        
        return ImuSample(
            timestamp=timestamp,
            gyro_x=gyro_x,
            gyro_y=gyro_y,
            gyro_z=gyro_z,
            accel_x=accel_x,
            accel_y=accel_y,
            accel_z=accel_z,
            temperature=temperature,
        )
    
    def start_streaming(self, callback: Callable[[ImuSample], None]) -> bool:
        """Start continuous sampling with callback."""
        if not self._connected:
            return False
        
        if self._streaming:
            return True
        
        self._callback = callback
        self._streaming = True
        self._status = ImuStatus.STREAMING
        self._stop_event.clear()
        
        self._stream_thread = threading.Thread(target=self._stream_loop, daemon=True)
        self._stream_thread.start()
        
        return True
    
    def stop_streaming(self):
        """Stop streaming."""
        self._stop_event.set()
        self._streaming = False
        self._status = ImuStatus.IDLE
        
        if self._stream_thread and self._stream_thread.is_alive():
            self._stream_thread.join(timeout=1.0)
    
    def _stream_loop(self):
        """Background thread generating IMU samples."""
        interval = 1.0 / self._sample_rate_hz
        
        while not self._stop_event.is_set():
            sample_start = time.time()
            
            sample = self.read_sample()
            
            if self._callback:
                try:
                    self._callback(sample)
                except Exception:
                    pass  # Don't crash on callback errors
            
            # Maintain sample rate
            elapsed = time.time() - sample_start
            if elapsed < interval:
                time.sleep(interval - elapsed)
    
    # Methods to control simulation for testing
    def set_rotation_rate(self, rate_dps: float):
        """Set simulated rotation rate for testing yaw tracking."""
        self._rotation_rate_dps = rate_dps
    
    def set_gyro_bias(self, bias_dps: float):
        """Set simulated gyro bias for testing drift correction."""
        self._gyro_bias_dps = bias_dps


# ============================================================================
# Factory Functions
# ============================================================================

def create_lidar_driver(use_hardware: bool = False, **kwargs) -> Union[RPLidarDriver, "MockLidarDriver"]:
    """Factory function to create LiDAR driver.
    
    Args:
        use_hardware: If True, use real hardware driver. If False, use mock (default).
        **kwargs: Arguments passed to the driver constructor.
    
    Returns:
        Either RPLidarDriver or MockLidarDriver instance.
    """
    if use_hardware:
        return RPLidarDriver(**kwargs)
    else:
        return MockLidarDriver(**kwargs)


def create_imu_driver(use_hardware: bool = False, **kwargs) -> Union[BMI160Driver, "MockIMUDriver"]:
    """Factory function to create IMU driver.
    
    Args:
        use_hardware: If True, use real hardware driver. If False, use mock (default).
        **kwargs: Arguments passed to the driver constructor.
    
    Returns:
        Either BMI160Driver or MockIMUDriver instance.
    """
    if use_hardware:
        return BMI160Driver(**kwargs)
    else:
        return MockIMUDriver(**kwargs)
