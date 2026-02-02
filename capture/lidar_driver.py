"""RPLidar S3 driver for Planar capture.

This module provides a Python interface to the RPLidar S3 sensor.
It can operate in two modes:
1. Direct serial communication (standalone)
2. ROS2 bridge mode (via rplidar_ros topics)

The driver handles:
- Device detection and health checks
- Scan configuration (mode, frequency)
- Continuous scan streaming
- Error recovery
"""

import time
import struct
import threading
import logging
from typing import Optional, Callable, List, Tuple, Dict, Any
from dataclasses import dataclass
from enum import Enum
import os

logger = logging.getLogger(__name__)


class LidarStatus(Enum):
    """LiDAR operational status."""
    DISCONNECTED = "disconnected"
    CONNECTING = "connecting"
    IDLE = "idle"
    SCANNING = "scanning"
    ERROR = "error"


@dataclass
class ScanPoint:
    """Single LiDAR scan point."""
    angle_deg: float
    distance_m: float
    quality: int
    timestamp: float


@dataclass
class ScanFrame:
    """Complete 360° scan frame."""
    points: List[ScanPoint]
    timestamp: float
    scan_rate_hz: float
    frame_id: int


@dataclass
class LidarInfo:
    """LiDAR device information."""
    model: str = ""
    serial_number: str = ""
    firmware_version: str = ""
    hardware_revision: str = ""
    health_status: str = "unknown"


class RPLidarDriver:
    """Driver for RPLidar S3 sensor."""
    
    # RPLidar protocol constants
    SYNC_BYTE = 0xA5
    SYNC_BYTE2 = 0x5A
    
    # Commands
    CMD_STOP = 0x25
    CMD_RESET = 0x40
    CMD_SCAN = 0x20
    CMD_EXPRESS_SCAN = 0x82
    CMD_GET_INFO = 0x50
    CMD_GET_HEALTH = 0x52
    CMD_GET_SAMPLERATE = 0x59
    CMD_SET_MOTOR_PWM = 0xF0
    
    # Scan modes for S3
    SCAN_MODES = {
        "Standard": {"mode_id": 0, "sample_rate": 8000},
        "DenseBoost": {"mode_id": 3, "sample_rate": 32000},
    }
    
    def __init__(
        self,
        port: str = "/dev/rplidar",
        baudrate: int = 1_000_000,
        timeout: float = 1.0
    ):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        
        self._serial = None
        self._status = LidarStatus.DISCONNECTED
        self._info = LidarInfo()
        self._scanning = False
        self._scan_thread: Optional[threading.Thread] = None
        self._current_frame: Optional[ScanFrame] = None
        self._frame_callbacks: List[Callable[[ScanFrame], None]] = []
        self._lock = threading.Lock()
        self._frame_count = 0
        self._scan_mode = "DenseBoost"
        self._scan_frequency = 10.0
    
    @property
    def status(self) -> LidarStatus:
        """Get current LiDAR status."""
        return self._status
    
    @property
    def info(self) -> LidarInfo:
        """Get device information."""
        return self._info
    
    @property
    def is_connected(self) -> bool:
        """Check if device is connected."""
        return self._status not in (LidarStatus.DISCONNECTED, LidarStatus.ERROR)
    
    @property
    def is_scanning(self) -> bool:
        """Check if currently scanning."""
        return self._scanning
    
    def connect(self) -> bool:
        """Connect to the LiDAR device."""
        try:
            import serial
            
            self._status = LidarStatus.CONNECTING
            logger.info(f"Connecting to RPLidar on {self.port} at {self.baudrate} baud")
            
            # Check if device exists
            if not os.path.exists(self.port):
                logger.error(f"Device {self.port} not found")
                self._status = LidarStatus.ERROR
                return False
            
            self._serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS
            )
            
            # Small delay for device initialization
            time.sleep(0.1)
            
            # Stop any ongoing scan
            self._send_command(self.CMD_STOP)
            time.sleep(0.1)
            self._serial.flushInput()
            
            # Get device info
            if not self._get_device_info():
                logger.error("Failed to get device info")
                self.disconnect()
                return False
            
            # Check health
            if not self._check_health():
                logger.warning("Device health check failed")
            
            self._status = LidarStatus.IDLE
            logger.info(f"Connected to RPLidar S/N: {self._info.serial_number}")
            return True
            
        except ImportError:
            logger.error("pyserial not installed. Install with: pip install pyserial")
            self._status = LidarStatus.ERROR
            return False
        except Exception as e:
            logger.error(f"Failed to connect: {e}")
            self._status = LidarStatus.ERROR
            return False
    
    def disconnect(self):
        """Disconnect from the LiDAR device."""
        if self._scanning:
            self.stop_scan()
        
        if self._serial and self._serial.is_open:
            try:
                self._send_command(self.CMD_STOP)
                time.sleep(0.05)
                self._serial.close()
            except Exception as e:
                logger.warning(f"Error during disconnect: {e}")
        
        self._serial = None
        self._status = LidarStatus.DISCONNECTED
        logger.info("Disconnected from RPLidar")
    
    def _send_command(self, cmd: int, payload: bytes = b""):
        """Send command to LiDAR."""
        if not self._serial:
            return
        
        packet = bytes([self.SYNC_BYTE, cmd])
        if payload:
            packet += bytes([len(payload)]) + payload
            # Calculate checksum
            checksum = 0
            for b in packet:
                checksum ^= b
            packet += bytes([checksum])
        
        self._serial.write(packet)
    
    def _read_response_descriptor(self) -> Optional[Tuple[int, int, int]]:
        """Read response descriptor. Returns (data_len, send_mode, data_type) or None."""
        if not self._serial:
            return None
        
        descriptor = self._serial.read(7)
        if len(descriptor) < 7:
            return None
        
        if descriptor[0] != self.SYNC_BYTE or descriptor[1] != self.SYNC_BYTE2:
            return None
        
        data_len = descriptor[2] | (descriptor[3] << 8) | (descriptor[4] << 16) | ((descriptor[5] & 0x3F) << 24)
        send_mode = descriptor[5] >> 6
        data_type = descriptor[6]
        
        return (data_len, send_mode, data_type)
    
    def _get_device_info(self) -> bool:
        """Get device information."""
        try:
            self._send_command(self.CMD_GET_INFO)
            
            desc = self._read_response_descriptor()
            if not desc or desc[2] != 0x04:  # Info response type
                return False
            
            data = self._serial.read(20)
            if len(data) < 20:
                return False
            
            self._info.model = f"RPLidar Model {data[0]}"
            self._info.firmware_version = f"{data[2]}.{data[1]:02d}"
            self._info.hardware_revision = str(data[3])
            self._info.serial_number = data[4:20].hex().upper()
            
            return True
        except Exception as e:
            logger.error(f"Failed to get device info: {e}")
            return False
    
    def _check_health(self) -> bool:
        """Check device health status."""
        try:
            self._send_command(self.CMD_GET_HEALTH)
            
            desc = self._read_response_descriptor()
            if not desc or desc[2] != 0x06:  # Health response type
                return False
            
            data = self._serial.read(3)
            if len(data) < 3:
                return False
            
            status_code = data[0]
            error_code = data[1] | (data[2] << 8)
            
            status_names = {0: "OK", 1: "Warning", 2: "Error"}
            self._info.health_status = status_names.get(status_code, f"Unknown({status_code})")
            
            if status_code == 2:
                logger.error(f"LiDAR health error: code {error_code}")
                return False
            
            return True
        except Exception as e:
            logger.error(f"Failed to check health: {e}")
            return False
    
    def configure(
        self,
        scan_mode: str = "DenseBoost",
        scan_frequency_hz: float = 10.0
    ) -> bool:
        """Configure scan parameters."""
        if scan_mode not in self.SCAN_MODES:
            logger.error(f"Unknown scan mode: {scan_mode}")
            return False
        
        self._scan_mode = scan_mode
        self._scan_frequency = scan_frequency_hz
        logger.info(f"Configured: mode={scan_mode}, frequency={scan_frequency_hz}Hz")
        return True
    
    def start_scan(self, callback: Optional[Callable[[ScanFrame], None]] = None) -> bool:
        """Start scanning and optionally register a callback for frames."""
        if not self.is_connected:
            logger.error("Not connected")
            return False
        
        if self._scanning:
            logger.warning("Already scanning")
            return True
        
        if callback:
            self._frame_callbacks.append(callback)
        
        # Start motor via DTR pin (for USB connection)
        if self._serial:
            self._serial.dtr = False  # DTR low = motor on for RPLidar
            time.sleep(0.5)  # Wait for motor to spin up
        
        self._scanning = True
        self._status = LidarStatus.SCANNING
        self._scan_thread = threading.Thread(target=self._scan_loop, daemon=True)
        self._scan_thread.start()
        
        logger.info("Scan started")
        return True
    
    def stop_scan(self):
        """Stop scanning."""
        self._scanning = False
        
        if self._serial:
            self._send_command(self.CMD_STOP)
            time.sleep(0.1)
            # Stop motor via DTR pin
            self._serial.dtr = True  # DTR high = motor off
        
        if self._scan_thread and self._scan_thread.is_alive():
            self._scan_thread.join(timeout=2.0)
        
        self._status = LidarStatus.IDLE if self.is_connected else LidarStatus.DISCONNECTED
        logger.info("Scan stopped")
    
    def _scan_loop(self):
        """Internal scan loop running in separate thread."""
        try:
            # Flush any pending data
            if self._serial:
                self._serial.reset_input_buffer()
            
            # For DenseBoost/Express modes, use express scan command
            # For Standard mode, use normal scan command
            if self._scan_mode == "Standard":
                self._send_command(self.CMD_SCAN)
            else:
                mode_id = self.SCAN_MODES[self._scan_mode]["mode_id"]
                payload = bytes([mode_id, 0, 0, 0, 0])
                self._send_command(self.CMD_EXPRESS_SCAN, payload)
            
            # Wait a bit for response
            time.sleep(0.2)
            
            desc = self._read_response_descriptor()
            if not desc:
                logger.error("No response to scan command")
                self._scanning = False
                return
            
            logger.debug(f"Scan response: len={desc[0]}, mode={desc[1]}, type={hex(desc[2])}")
            
            # Determine scan data format based on response type
            is_express = (desc[2] == 0x82 or desc[2] == 0x84)  # Express scan types
            is_standard = (desc[2] == 0x81)  # Standard scan type
            
            # Collect points for one full rotation
            points = []
            last_angle = -1
            frame_start_time = time.time()
            
            while self._scanning:
                if is_standard:
                    # Standard scan: 5 bytes per point
                    packet = self._serial.read(5)
                    if len(packet) < 5:
                        continue
                    
                    new_points = self._parse_standard_scan_packet(packet)
                else:
                    # Express scan: 84 bytes per packet
                    packet = self._serial.read(84)
                    if len(packet) < 84:
                        continue
                    
                    new_points = self._parse_express_scan_packet(packet)
                
                for pt in new_points:
                    # Detect new frame (angle wraps around)
                    if last_angle > 300 and pt.angle_deg < 60:
                        # Complete frame
                        if len(points) > 100:  # Minimum valid frame
                            frame_time = time.time()
                            elapsed = frame_time - frame_start_time
                            scan_rate = 1.0 / elapsed if elapsed > 0 else 0
                            
                            frame = ScanFrame(
                                points=points.copy(),
                                timestamp=frame_time,
                                scan_rate_hz=scan_rate,
                                frame_id=self._frame_count
                            )
                            self._frame_count += 1
                            
                            with self._lock:
                                self._current_frame = frame
                            
                            # Notify callbacks
                            for cb in self._frame_callbacks:
                                try:
                                    cb(frame)
                                except Exception as e:
                                    logger.error(f"Callback error: {e}")
                        
                        points = []
                        frame_start_time = time.time()
                    
                    points.append(pt)
                    last_angle = pt.angle_deg
                    
        except Exception as e:
            logger.error(f"Scan loop error: {e}")
            self._scanning = False
            self._status = LidarStatus.ERROR
    
    def _parse_standard_scan_packet(self, packet: bytes) -> List[ScanPoint]:
        """Parse standard scan packet (5 bytes) into a point."""
        points = []
        timestamp = time.time()
        
        if len(packet) < 5:
            return points
        
        # Standard scan format:
        # Byte 0: quality (bits 7-2) | start_flag (bit 0) | inverted_start_flag (bit 1)
        # Byte 1-2: angle (Q6 format, little-endian) - bits 1-15 are angle, bit 0 is check bit
        # Byte 3-4: distance (Q2 format, little-endian) in mm
        
        quality = packet[0] >> 2
        start_flag = packet[0] & 0x01
        check_bit = (packet[1] & 0x01)
        
        # Validate packet (check bit should be 1, start flags should differ)
        if check_bit != 1:
            return points
        
        angle_q6 = ((packet[1] >> 1) | (packet[2] << 7))
        angle_deg = angle_q6 / 64.0
        
        distance_q2 = packet[3] | (packet[4] << 8)
        distance_mm = distance_q2 / 4.0
        
        if distance_mm > 0 and quality > 0:
            points.append(ScanPoint(
                angle_deg=angle_deg,
                distance_m=distance_mm / 1000.0,
                quality=quality,
                timestamp=timestamp
            ))
        
        return points

    def _parse_express_scan_packet(self, packet: bytes) -> List[ScanPoint]:
        """Parse express scan packet into points."""
        points = []
        timestamp = time.time()
        
        # Express scan packet format (simplified parsing)
        # Real implementation would need full protocol handling
        sync = packet[0] >> 4
        if sync != 0xA:
            return points
        
        start_angle = ((packet[1] >> 7) | (packet[2] << 1)) / 64.0
        
        # Parse cabins (16 cabins, each 5 bytes starting at offset 4)
        for i in range(16):
            offset = 4 + i * 5
            if offset + 5 > len(packet):
                break
            
            # Extract distance and angle delta (simplified)
            dist1 = (packet[offset] | ((packet[offset + 1] & 0x3F) << 8)) / 4.0
            dist2 = (packet[offset + 2] | ((packet[offset + 3] & 0x3F) << 8)) / 4.0
            
            angle1 = start_angle + i * (360.0 / 32.0)
            angle2 = start_angle + (i + 0.5) * (360.0 / 32.0)
            
            if angle1 >= 360:
                angle1 -= 360
            if angle2 >= 360:
                angle2 -= 360
            
            if dist1 > 0:
                points.append(ScanPoint(
                    angle_deg=angle1,
                    distance_m=dist1 / 1000.0,
                    quality=((packet[offset + 1] >> 6) & 0x03) * 20,
                    timestamp=timestamp
                ))
            
            if dist2 > 0:
                points.append(ScanPoint(
                    angle_deg=angle2,
                    distance_m=dist2 / 1000.0,
                    quality=((packet[offset + 3] >> 6) & 0x03) * 20,
                    timestamp=timestamp
                ))
        
        return points
    
    def get_current_frame(self) -> Optional[ScanFrame]:
        """Get the most recent complete scan frame."""
        with self._lock:
            return self._current_frame
    
    def add_frame_callback(self, callback: Callable[[ScanFrame], None]):
        """Add a callback for new scan frames."""
        self._frame_callbacks.append(callback)
    
    def remove_frame_callback(self, callback: Callable[[ScanFrame], None]):
        """Remove a frame callback."""
        if callback in self._frame_callbacks:
            self._frame_callbacks.remove(callback)
    
    def get_diagnostics(self) -> Dict[str, Any]:
        """Get diagnostic information."""
        return {
            "status": self._status.value,
            "connected": self.is_connected,
            "scanning": self.is_scanning,
            "port": self.port,
            "baudrate": self.baudrate,
            "info": {
                "model": self._info.model,
                "serial_number": self._info.serial_number,
                "firmware_version": self._info.firmware_version,
                "hardware_revision": self._info.hardware_revision,
                "health_status": self._info.health_status,
            },
            "config": {
                "scan_mode": self._scan_mode,
                "scan_frequency_hz": self._scan_frequency,
            },
            "stats": {
                "frames_captured": self._frame_count,
            }
        }


class RPLidarROS2Bridge:
    """Bridge to RPLidar via ROS2 rplidar_ros package.
    
    This provides an alternative interface that uses the ROS2 driver
    instead of direct serial communication, useful when running
    alongside other ROS2 nodes.
    """
    
    def __init__(self, topic: str = "/scan"):
        self.topic = topic
        self._node = None
        self._subscription = None
        self._current_frame: Optional[ScanFrame] = None
        self._frame_callbacks: List[Callable[[ScanFrame], None]] = []
        self._lock = threading.Lock()
        self._frame_count = 0
    
    def connect(self) -> bool:
        """Initialize ROS2 node and subscribe to scan topic."""
        try:
            import rclpy
            from rclpy.node import Node
            from sensor_msgs.msg import LaserScan
            
            if not rclpy.ok():
                rclpy.init()
            
            class ScanSubscriber(Node):
                def __init__(self, topic, callback):
                    super().__init__("planar_scan_subscriber")
                    self.subscription = self.create_subscription(
                        LaserScan, topic, callback, 10
                    )
            
            self._node = ScanSubscriber(self.topic, self._scan_callback)
            
            # Spin in background thread
            self._spin_thread = threading.Thread(
                target=lambda: rclpy.spin(self._node),
                daemon=True
            )
            self._spin_thread.start()
            
            logger.info(f"Connected to ROS2 topic: {self.topic}")
            return True
            
        except ImportError:
            logger.error("rclpy not available. Install ROS2 or use direct serial mode.")
            return False
        except Exception as e:
            logger.error(f"Failed to connect to ROS2: {e}")
            return False
    
    def _scan_callback(self, msg):
        """Handle incoming LaserScan message."""
        points = []
        timestamp = time.time()
        
        angle = msg.angle_min
        for i, distance in enumerate(msg.ranges):
            if msg.range_min < distance < msg.range_max:
                points.append(ScanPoint(
                    angle_deg=math.degrees(angle),
                    distance_m=distance,
                    quality=int(msg.intensities[i]) if msg.intensities else 50,
                    timestamp=timestamp
                ))
            angle += msg.angle_increment
        
        frame = ScanFrame(
            points=points,
            timestamp=timestamp,
            scan_rate_hz=1.0 / msg.scan_time if msg.scan_time > 0 else 10.0,
            frame_id=self._frame_count
        )
        self._frame_count += 1
        
        with self._lock:
            self._current_frame = frame
        
        for cb in self._frame_callbacks:
            try:
                cb(frame)
            except Exception as e:
                logger.error(f"Callback error: {e}")
    
    def disconnect(self):
        """Shutdown ROS2 node."""
        if self._node:
            self._node.destroy_node()
            self._node = None
    
    def get_current_frame(self) -> Optional[ScanFrame]:
        """Get most recent scan frame."""
        with self._lock:
            return self._current_frame
    
    def add_frame_callback(self, callback: Callable[[ScanFrame], None]):
        """Add callback for new frames."""
        self._frame_callbacks.append(callback)


# Import math for ROS2 bridge
import math
