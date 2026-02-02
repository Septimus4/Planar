"""BMI160 6-Axis IMU driver for Planar capture.

This module provides a Python interface to the Gravity BMI160 sensor
connected via I2C. The BMI160 contains a 3-axis accelerometer and
3-axis gyroscope.

Device address: 0x69 (default for Gravity module)
Chip ID register 0x00 should return 0xD1

Key registers:
- 0x00: CHIP_ID (should be 0xD1)
- 0x04-0x17: Sensor data
- 0x40-0x47: Configuration
- 0x7E: CMD register

For Planar, we primarily use the gyroscope Z-axis for yaw estimation.
"""

import time
import struct
import threading
import logging
from typing import Optional, Callable, List, Dict, Any, Tuple
from dataclasses import dataclass
from enum import Enum
import os

logger = logging.getLogger(__name__)


class ImuStatus(Enum):
    """IMU operational status."""
    DISCONNECTED = "disconnected"
    CONNECTING = "connecting"
    IDLE = "idle"
    STREAMING = "streaming"
    ERROR = "error"


@dataclass
class ImuSample:
    """Single IMU sample with all axes."""
    timestamp: float
    gyro_x: float  # rad/s
    gyro_y: float  # rad/s
    gyro_z: float  # rad/s
    accel_x: float  # m/s²
    accel_y: float  # m/s²
    accel_z: float  # m/s²
    temperature: float  # Celsius


@dataclass
class ImuInfo:
    """IMU device information."""
    chip_id: int = 0
    bus: int = 0
    address: int = 0
    connected: bool = False


class BMI160Driver:
    """Driver for BMI160 6-axis IMU sensor via I2C."""
    
    # BMI160 Register addresses
    REG_CHIP_ID = 0x00
    REG_ERR_REG = 0x02
    REG_PMU_STATUS = 0x03
    REG_DATA_0 = 0x04  # MAG data start
    REG_DATA_8 = 0x0C  # GYRO data start (X low)
    REG_DATA_14 = 0x12  # ACCEL data start (X low)
    REG_SENSORTIME_0 = 0x18
    REG_STATUS = 0x1B
    REG_TEMPERATURE_0 = 0x20
    REG_TEMPERATURE_1 = 0x21
    REG_ACC_CONF = 0x40
    REG_ACC_RANGE = 0x41
    REG_GYR_CONF = 0x42
    REG_GYR_RANGE = 0x43
    REG_INT_EN_0 = 0x50
    REG_INT_EN_1 = 0x51
    REG_INT_OUT_CTRL = 0x53
    REG_INT_MAP_0 = 0x55
    REG_INT_MAP_1 = 0x56
    REG_CMD = 0x7E
    
    # Commands
    CMD_SOFT_RESET = 0xB6
    CMD_ACC_NORMAL_MODE = 0x11
    CMD_GYR_NORMAL_MODE = 0x15
    CMD_ACC_SUSPEND = 0x10
    CMD_GYR_SUSPEND = 0x14
    
    # Chip ID for BMI160
    CHIP_ID_BMI160 = 0xD1
    
    # Gyroscope range settings (degrees per second)
    GYRO_RANGE_2000 = 0x00  # ±2000°/s
    GYRO_RANGE_1000 = 0x01  # ±1000°/s
    GYRO_RANGE_500 = 0x02   # ±500°/s
    GYRO_RANGE_250 = 0x03   # ±250°/s
    GYRO_RANGE_125 = 0x04   # ±125°/s
    
    GYRO_RANGE_MAP = {
        2000: (GYRO_RANGE_2000, 2000.0),
        1000: (GYRO_RANGE_1000, 1000.0),
        500: (GYRO_RANGE_500, 500.0),
        250: (GYRO_RANGE_250, 250.0),
        125: (GYRO_RANGE_125, 125.0),
    }
    
    # Accelerometer range settings (g)
    ACCEL_RANGE_2G = 0x03   # ±2g
    ACCEL_RANGE_4G = 0x05   # ±4g
    ACCEL_RANGE_8G = 0x08   # ±8g
    ACCEL_RANGE_16G = 0x0C  # ±16g
    
    ACCEL_RANGE_MAP = {
        2: (ACCEL_RANGE_2G, 2.0),
        4: (ACCEL_RANGE_4G, 4.0),
        8: (ACCEL_RANGE_8G, 8.0),
        16: (ACCEL_RANGE_16G, 16.0),
    }
    
    # Output data rates (Hz) - encoded values
    ODR_MAP = {
        25: 0x06,
        50: 0x07,
        100: 0x08,
        200: 0x09,
        400: 0x0A,
        800: 0x0B,
        1600: 0x0C,
    }
    
    def __init__(
        self,
        bus: int = 1,
        address: int = 0x69,
        gyro_range_dps: int = 250,
        accel_range_g: int = 2,
        sample_rate_hz: float = 100.0
    ):
        self.bus = bus
        self.address = address
        self.gyro_range_dps = gyro_range_dps
        self.accel_range_g = accel_range_g
        self.sample_rate_hz = sample_rate_hz
        
        self._i2c = None
        self._status = ImuStatus.DISCONNECTED
        self._info = ImuInfo()
        self._streaming = False
        self._stream_thread: Optional[threading.Thread] = None
        self._sample_callbacks: List[Callable[[ImuSample], None]] = []
        self._lock = threading.Lock()
        self._last_sample: Optional[ImuSample] = None
        self._sample_count = 0
        
        # Scale factors (set during configure)
        self._gyro_scale = 0.0  # LSB to rad/s
        self._accel_scale = 0.0  # LSB to m/s²
    
    @property
    def status(self) -> ImuStatus:
        """Get current IMU status."""
        return self._status
    
    @property
    def info(self) -> ImuInfo:
        """Get device information."""
        return self._info
    
    @property
    def is_connected(self) -> bool:
        """Check if device is connected."""
        return self._status not in (ImuStatus.DISCONNECTED, ImuStatus.ERROR)
    
    @property
    def is_streaming(self) -> bool:
        """Check if currently streaming data."""
        return self._streaming
    
    def _init_i2c(self) -> bool:
        """Initialize I2C bus."""
        try:
            import smbus2
            self._i2c = smbus2.SMBus(self.bus)
            return True
        except ImportError:
            logger.error("smbus2 not installed. Install with: pip install smbus2")
            return False
        except Exception as e:
            logger.error(f"Failed to open I2C bus {self.bus}: {e}")
            return False
    
    def _read_byte(self, reg: int) -> int:
        """Read single byte from register."""
        return self._i2c.read_byte_data(self.address, reg)
    
    def _write_byte(self, reg: int, value: int):
        """Write single byte to register."""
        self._i2c.write_byte_data(self.address, reg, value)
    
    def _read_bytes(self, reg: int, length: int) -> bytes:
        """Read multiple bytes starting from register."""
        return bytes(self._i2c.read_i2c_block_data(self.address, reg, length))
    
    def connect(self) -> bool:
        """Connect to the BMI160 sensor."""
        try:
            self._status = ImuStatus.CONNECTING
            logger.info(f"Connecting to BMI160 on I2C bus {self.bus}, address 0x{self.address:02X}")
            
            # Check if I2C device file exists
            i2c_dev = f"/dev/i2c-{self.bus}"
            if not os.path.exists(i2c_dev):
                logger.error(f"I2C device {i2c_dev} not found")
                self._status = ImuStatus.ERROR
                return False
            
            if not self._init_i2c():
                self._status = ImuStatus.ERROR
                return False
            
            # Read chip ID to verify device
            chip_id = self._read_byte(self.REG_CHIP_ID)
            self._info.chip_id = chip_id
            self._info.bus = self.bus
            self._info.address = self.address
            
            if chip_id != self.CHIP_ID_BMI160:
                logger.error(f"Unexpected chip ID: 0x{chip_id:02X}, expected 0x{self.CHIP_ID_BMI160:02X}")
                self._status = ImuStatus.ERROR
                return False
            
            logger.info(f"BMI160 chip ID verified: 0x{chip_id:02X}")
            
            # Soft reset
            self._write_byte(self.REG_CMD, self.CMD_SOFT_RESET)
            time.sleep(0.1)  # Wait for reset
            
            # Re-initialize I2C after reset
            if not self._init_i2c():
                self._status = ImuStatus.ERROR
                return False
            
            # Configure sensor
            if not self._configure_sensor():
                self._status = ImuStatus.ERROR
                return False
            
            self._info.connected = True
            self._status = ImuStatus.IDLE
            logger.info("BMI160 connected and configured")
            return True
            
        except Exception as e:
            logger.error(f"Failed to connect to BMI160: {e}")
            self._status = ImuStatus.ERROR
            return False
    
    def _configure_sensor(self) -> bool:
        """Configure accelerometer and gyroscope."""
        try:
            # Power on accelerometer and gyroscope
            self._write_byte(self.REG_CMD, self.CMD_ACC_NORMAL_MODE)
            time.sleep(0.01)
            self._write_byte(self.REG_CMD, self.CMD_GYR_NORMAL_MODE)
            time.sleep(0.1)  # Wait for gyro to power up
            
            # Check PMU status
            pmu_status = self._read_byte(self.REG_PMU_STATUS)
            acc_pmu = (pmu_status >> 4) & 0x03
            gyr_pmu = (pmu_status >> 2) & 0x03
            
            if acc_pmu != 0x01 or gyr_pmu != 0x01:
                logger.warning(f"PMU status: acc={acc_pmu}, gyr={gyr_pmu} (expected 1,1)")
            
            # Configure gyroscope range
            if self.gyro_range_dps in self.GYRO_RANGE_MAP:
                reg_val, full_scale = self.GYRO_RANGE_MAP[self.gyro_range_dps]
            else:
                reg_val, full_scale = self.GYRO_RANGE_MAP[250]
                logger.warning(f"Invalid gyro range {self.gyro_range_dps}, using 250°/s")
            
            self._write_byte(self.REG_GYR_RANGE, reg_val)
            # Scale: LSB to rad/s = (full_scale_dps / 32768) * (π/180)
            import math
            self._gyro_scale = (full_scale / 32768.0) * (math.pi / 180.0)
            
            # Configure accelerometer range
            if self.accel_range_g in self.ACCEL_RANGE_MAP:
                reg_val, full_scale = self.ACCEL_RANGE_MAP[self.accel_range_g]
            else:
                reg_val, full_scale = self.ACCEL_RANGE_MAP[2]
                logger.warning(f"Invalid accel range {self.accel_range_g}, using 2g")
            
            self._write_byte(self.REG_ACC_RANGE, reg_val)
            # Scale: LSB to m/s² = (full_scale_g / 32768) * 9.80665
            self._accel_scale = (full_scale / 32768.0) * 9.80665
            
            # Configure output data rate
            odr_nearest = min(self.ODR_MAP.keys(), key=lambda x: abs(x - self.sample_rate_hz))
            odr_val = self.ODR_MAP[odr_nearest]
            
            # Set ODR with bandwidth (normal mode, no oversampling)
            self._write_byte(self.REG_GYR_CONF, odr_val | 0x20)  # BWP = normal
            self._write_byte(self.REG_ACC_CONF, odr_val | 0x20)  # BWP = normal
            
            self.sample_rate_hz = odr_nearest
            logger.info(f"Configured: gyro ±{self.gyro_range_dps}°/s, accel ±{self.accel_range_g}g, ODR {odr_nearest}Hz")
            
            return True
            
        except Exception as e:
            logger.error(f"Failed to configure sensor: {e}")
            return False
    
    def disconnect(self):
        """Disconnect from the sensor."""
        if self._streaming:
            self.stop_streaming()
        
        if self._i2c:
            try:
                # Put sensor in suspend mode
                self._write_byte(self.REG_CMD, self.CMD_ACC_SUSPEND)
                self._write_byte(self.REG_CMD, self.CMD_GYR_SUSPEND)
                self._i2c.close()
            except Exception as e:
                logger.warning(f"Error during disconnect: {e}")
        
        self._i2c = None
        self._status = ImuStatus.DISCONNECTED
        self._info.connected = False
        logger.info("Disconnected from BMI160")
    
    def read_sample(self) -> Optional[ImuSample]:
        """Read a single IMU sample."""
        if not self.is_connected:
            return None
        
        try:
            timestamp = time.time()
            
            # Read gyro and accel data in one burst (12 bytes)
            # GYRO: X_L, X_H, Y_L, Y_H, Z_L, Z_H (0x0C-0x11)
            # ACCEL: X_L, X_H, Y_L, Y_H, Z_L, Z_H (0x12-0x17)
            data = self._read_bytes(self.REG_DATA_8, 12)
            
            # Parse gyroscope (signed 16-bit, little-endian)
            gx_raw = struct.unpack('<h', data[0:2])[0]
            gy_raw = struct.unpack('<h', data[2:4])[0]
            gz_raw = struct.unpack('<h', data[4:6])[0]
            
            # Parse accelerometer
            ax_raw = struct.unpack('<h', data[6:8])[0]
            ay_raw = struct.unpack('<h', data[8:10])[0]
            az_raw = struct.unpack('<h', data[10:12])[0]
            
            # Read temperature (optional)
            temp_data = self._read_bytes(self.REG_TEMPERATURE_0, 2)
            temp_raw = struct.unpack('<h', temp_data)[0]
            temperature = (temp_raw / 512.0) + 23.0
            
            sample = ImuSample(
                timestamp=timestamp,
                gyro_x=gx_raw * self._gyro_scale,
                gyro_y=gy_raw * self._gyro_scale,
                gyro_z=gz_raw * self._gyro_scale,
                accel_x=ax_raw * self._accel_scale,
                accel_y=ay_raw * self._accel_scale,
                accel_z=az_raw * self._accel_scale,
                temperature=temperature
            )
            
            with self._lock:
                self._last_sample = sample
            self._sample_count += 1
            
            return sample
            
        except Exception as e:
            logger.error(f"Failed to read sample: {e}")
            return None
    
    def start_streaming(self, callback: Optional[Callable[[ImuSample], None]] = None) -> bool:
        """Start continuous sampling."""
        if not self.is_connected:
            logger.error("Not connected")
            return False
        
        if self._streaming:
            logger.warning("Already streaming")
            return True
        
        if callback:
            self._sample_callbacks.append(callback)
        
        self._streaming = True
        self._status = ImuStatus.STREAMING
        self._stream_thread = threading.Thread(target=self._stream_loop, daemon=True)
        self._stream_thread.start()
        
        logger.info("IMU streaming started")
        return True
    
    def stop_streaming(self):
        """Stop continuous sampling."""
        self._streaming = False
        
        if self._stream_thread and self._stream_thread.is_alive():
            self._stream_thread.join(timeout=2.0)
        
        self._status = ImuStatus.IDLE if self.is_connected else ImuStatus.DISCONNECTED
        logger.info("IMU streaming stopped")
    
    def _stream_loop(self):
        """Internal streaming loop."""
        interval = 1.0 / self.sample_rate_hz
        next_time = time.time()
        
        while self._streaming:
            try:
                sample = self.read_sample()
                
                if sample:
                    for cb in self._sample_callbacks:
                        try:
                            cb(sample)
                        except Exception as e:
                            logger.error(f"Callback error: {e}")
                
                # Maintain consistent sample rate
                next_time += interval
                sleep_time = next_time - time.time()
                if sleep_time > 0:
                    time.sleep(sleep_time)
                else:
                    # Fell behind, reset timing
                    next_time = time.time()
                    
            except Exception as e:
                logger.error(f"Stream loop error: {e}")
                self._streaming = False
                self._status = ImuStatus.ERROR
                break
    
    def get_last_sample(self) -> Optional[ImuSample]:
        """Get the most recent sample."""
        with self._lock:
            return self._last_sample
    
    def add_sample_callback(self, callback: Callable[[ImuSample], None]):
        """Add a callback for new samples."""
        self._sample_callbacks.append(callback)
    
    def remove_sample_callback(self, callback: Callable[[ImuSample], None]):
        """Remove a sample callback."""
        if callback in self._sample_callbacks:
            self._sample_callbacks.remove(callback)
    
    def configure(
        self,
        gyro_range_dps: Optional[int] = None,
        accel_range_g: Optional[int] = None,
        sample_rate_hz: Optional[float] = None
    ) -> bool:
        """Reconfigure sensor parameters."""
        if gyro_range_dps is not None:
            self.gyro_range_dps = gyro_range_dps
        if accel_range_g is not None:
            self.accel_range_g = accel_range_g
        if sample_rate_hz is not None:
            self.sample_rate_hz = sample_rate_hz
        
        if self.is_connected:
            was_streaming = self._streaming
            if was_streaming:
                self.stop_streaming()
            
            success = self._configure_sensor()
            
            if was_streaming and success:
                self.start_streaming()
            
            return success
        return True
    
    def get_diagnostics(self) -> Dict[str, Any]:
        """Get diagnostic information."""
        return {
            "status": self._status.value,
            "connected": self.is_connected,
            "streaming": self.is_streaming,
            "bus": self.bus,
            "address": f"0x{self.address:02X}",
            "chip_id": f"0x{self._info.chip_id:02X}" if self._info.chip_id else "unknown",
            "config": {
                "gyro_range_dps": self.gyro_range_dps,
                "accel_range_g": self.accel_range_g,
                "sample_rate_hz": self.sample_rate_hz,
            },
            "stats": {
                "samples_read": self._sample_count,
            },
            "last_sample": {
                "gyro_z_rad_s": self._last_sample.gyro_z if self._last_sample else None,
                "accel_z_m_s2": self._last_sample.accel_z if self._last_sample else None,
                "temperature_c": self._last_sample.temperature if self._last_sample else None,
            } if self._last_sample else None
        }
    
    def self_test(self) -> Dict[str, Any]:
        """Perform self-test and return results."""
        results = {
            "passed": False,
            "chip_id_ok": False,
            "gyro_ok": False,
            "accel_ok": False,
            "errors": []
        }
        
        if not self.is_connected:
            results["errors"].append("Not connected")
            return results
        
        try:
            # Verify chip ID
            chip_id = self._read_byte(self.REG_CHIP_ID)
            results["chip_id_ok"] = (chip_id == self.CHIP_ID_BMI160)
            if not results["chip_id_ok"]:
                results["errors"].append(f"Wrong chip ID: 0x{chip_id:02X}")
            
            # Check error register
            err_reg = self._read_byte(self.REG_ERR_REG)
            if err_reg != 0:
                results["errors"].append(f"Error register: 0x{err_reg:02X}")
            
            # Read a sample and check values are reasonable
            sample = self.read_sample()
            if sample:
                # Gyro at rest should be near zero (< 0.1 rad/s = ~6 deg/s)
                gyro_magnitude = (sample.gyro_x**2 + sample.gyro_y**2 + sample.gyro_z**2)**0.5
                results["gyro_ok"] = gyro_magnitude < 0.2
                if not results["gyro_ok"]:
                    results["errors"].append(f"Gyro magnitude too high: {gyro_magnitude:.3f} rad/s")
                
                # Accel should be near 1g (9.8 m/s²) when stationary
                accel_magnitude = (sample.accel_x**2 + sample.accel_y**2 + sample.accel_z**2)**0.5
                results["accel_ok"] = 8.0 < accel_magnitude < 12.0
                if not results["accel_ok"]:
                    results["errors"].append(f"Accel magnitude unexpected: {accel_magnitude:.2f} m/s²")
            else:
                results["errors"].append("Failed to read sample")
            
            results["passed"] = results["chip_id_ok"] and results["gyro_ok"] and results["accel_ok"]
            
        except Exception as e:
            results["errors"].append(f"Self-test exception: {e}")
        
        return results


def scan_i2c_bus(bus: int = 1) -> List[int]:
    """Scan I2C bus and return list of responding addresses."""
    try:
        import smbus2
        addresses = []
        i2c = smbus2.SMBus(bus)
        
        for addr in range(0x03, 0x78):
            try:
                i2c.read_byte(addr)
                addresses.append(addr)
            except OSError:
                pass
        
        i2c.close()
        return addresses
        
    except ImportError:
        logger.error("smbus2 not installed")
        return []
    except Exception as e:
        logger.error(f"I2C scan failed: {e}")
        return []


def detect_bmi160(bus: int = 1) -> Optional[int]:
    """Detect BMI160 on I2C bus, return address if found."""
    # Common BMI160 addresses
    for addr in [0x69, 0x68]:
        try:
            import smbus2
            i2c = smbus2.SMBus(bus)
            chip_id = i2c.read_byte_data(addr, BMI160Driver.REG_CHIP_ID)
            i2c.close()
            
            if chip_id == BMI160Driver.CHIP_ID_BMI160:
                logger.info(f"BMI160 found at 0x{addr:02X}")
                return addr
        except Exception:
            pass
    
    return None
