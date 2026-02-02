"""Python client for connecting to Planar capture daemon.

This module provides a high-level API for desktop applications to:
- Connect to the Raspberry Pi capture daemon
- Monitor sensor status
- Configure sensors remotely
- Start/stop capture sessions
- Stream real-time sensor data

Example usage:
    from desktop.client import PlanarClient
    
    client = PlanarClient("192.168.1.100")
    
    # Check status
    status = client.get_status()
    print(f"LiDAR: {status['lidar']['status']}")
    print(f"IMU: {status['imu']['status']}")
    
    # Configure sensors
    client.configure_lidar(scan_mode="DenseBoost", scan_frequency_hz=10.0)
    client.configure_imu(sample_rate_hz=100)
    
    # Start streaming
    client.start_lidar()
    client.start_imu()
    
    # Start a capture session
    client.start_session("floor_scan_001")
    
    # Stream data with callbacks
    client.on_lidar_frame = lambda frame: print(f"Frame {frame['frame_id']}")
    client.on_imu_sample = lambda sample: print(f"Gyro Z: {sample['gyro']['z']}")
    client.connect_streams()
"""

import json
import time
import threading
import logging
from typing import Optional, Callable, Dict, Any, List
from dataclasses import dataclass
from urllib.parse import urljoin

logger = logging.getLogger(__name__)

# Try to import HTTP client
try:
    import requests
    REQUESTS_AVAILABLE = True
except ImportError:
    REQUESTS_AVAILABLE = False

# Try to import WebSocket client
try:
    import websocket
    WEBSOCKET_AVAILABLE = True
except ImportError:
    WEBSOCKET_AVAILABLE = False


@dataclass
class ConnectionConfig:
    """Configuration for connecting to capture daemon."""
    host: str = "localhost"
    http_port: int = 8080
    ws_port: int = 8081
    auth_token: Optional[str] = None
    timeout: float = 10.0
    reconnect_interval: float = 5.0
    
    @property
    def http_base_url(self) -> str:
        return f"http://{self.host}:{self.http_port}"
    
    @property
    def ws_base_url(self) -> str:
        return f"ws://{self.host}:{self.http_port}"


class PlanarClient:
    """Client for communicating with Planar capture daemon."""
    
    def __init__(
        self,
        host: str = "localhost",
        port: int = 8080,
        auth_token: Optional[str] = None,
        timeout: float = 10.0
    ):
        """Initialize client.
        
        Args:
            host: IP address or hostname of the Raspberry Pi
            port: HTTP port of the capture daemon
            auth_token: Optional authentication token
            timeout: Request timeout in seconds
        """
        self.config = ConnectionConfig(
            host=host,
            http_port=port,
            auth_token=auth_token,
            timeout=timeout
        )
        
        self._session = None
        self._connected = False
        
        # WebSocket connections
        self._ws_lidar: Optional[websocket.WebSocketApp] = None
        self._ws_imu: Optional[websocket.WebSocketApp] = None
        self._ws_events: Optional[websocket.WebSocketApp] = None
        self._ws_threads: Dict[str, threading.Thread] = {}
        
        # Callbacks
        self.on_lidar_frame: Optional[Callable[[Dict], None]] = None
        self.on_imu_sample: Optional[Callable[[Dict], None]] = None
        self.on_event: Optional[Callable[[Dict], None]] = None
        self.on_connection_lost: Optional[Callable[[str], None]] = None
        self.on_connection_restored: Optional[Callable[[str], None]] = None
        
        # Status cache
        self._last_status: Optional[Dict] = None
        self._last_status_time: float = 0
    
    def _get_headers(self) -> Dict[str, str]:
        """Get HTTP headers including auth if configured."""
        headers = {"Content-Type": "application/json"}
        if self.config.auth_token:
            headers["Authorization"] = f"Bearer {self.config.auth_token}"
        return headers
    
    def _request(self, method: str, endpoint: str, data: Optional[Dict] = None) -> Dict:
        """Make HTTP request to daemon."""
        if not REQUESTS_AVAILABLE:
            raise RuntimeError("requests library not installed. pip install requests")
        
        url = urljoin(self.config.http_base_url, endpoint)
        
        try:
            if method == "GET":
                resp = requests.get(
                    url,
                    headers=self._get_headers(),
                    timeout=self.config.timeout
                )
            elif method == "POST":
                resp = requests.post(
                    url,
                    headers=self._get_headers(),
                    json=data or {},
                    timeout=self.config.timeout
                )
            else:
                raise ValueError(f"Unsupported method: {method}")
            
            resp.raise_for_status()
            return resp.json()
            
        except requests.exceptions.ConnectionError as e:
            logger.error(f"Connection error: {e}")
            raise ConnectionError(f"Cannot connect to {self.config.host}:{self.config.http_port}")
        except requests.exceptions.Timeout as e:
            logger.error(f"Request timeout: {e}")
            raise TimeoutError(f"Request timed out after {self.config.timeout}s")
        except requests.exceptions.HTTPError as e:
            logger.error(f"HTTP error: {e}")
            try:
                error_data = e.response.json()
                raise RuntimeError(error_data.get("error", str(e)))
            except (ValueError, AttributeError):
                raise RuntimeError(str(e))
    
    # === Connection ===
    
    def ping(self) -> bool:
        """Check if daemon is reachable."""
        try:
            result = self._request("GET", "/health")
            return result.get("status") == "ok"
        except Exception:
            return False
    
    def connect(self) -> bool:
        """Verify connection to daemon."""
        if self.ping():
            self._connected = True
            logger.info(f"Connected to {self.config.host}:{self.config.http_port}")
            return True
        self._connected = False
        return False
    
    @property
    def is_connected(self) -> bool:
        """Check if connected."""
        return self._connected
    
    # === Status ===
    
    def get_status(self, cached: bool = False) -> Dict[str, Any]:
        """Get overall system status.
        
        Args:
            cached: If True, return cached status if available and recent
        """
        if cached and self._last_status and (time.time() - self._last_status_time < 1.0):
            return self._last_status
        
        status = self._request("GET", "/api/status")
        self._last_status = status
        self._last_status_time = time.time()
        return status
    
    def get_lidar_status(self) -> Dict[str, Any]:
        """Get LiDAR status and diagnostics."""
        return self._request("GET", "/api/lidar/status")
    
    def get_imu_status(self) -> Dict[str, Any]:
        """Get IMU status and diagnostics."""
        return self._request("GET", "/api/imu/status")
    
    def get_config(self) -> Dict[str, Any]:
        """Get current daemon configuration."""
        return self._request("GET", "/api/config")
    
    # === LiDAR Control ===
    
    def start_lidar(self) -> bool:
        """Start LiDAR scanning."""
        result = self._request("POST", "/api/lidar/start")
        return result.get("status") == "ok"
    
    def stop_lidar(self) -> bool:
        """Stop LiDAR scanning."""
        result = self._request("POST", "/api/lidar/stop")
        return result.get("status") == "ok"
    
    def configure_lidar(
        self,
        device: Optional[str] = None,
        baudrate: Optional[int] = None,
        scan_mode: Optional[str] = None,
        scan_frequency_hz: Optional[float] = None
    ) -> bool:
        """Configure LiDAR parameters.
        
        Args:
            device: Serial device path (e.g., /dev/rplidar)
            baudrate: Serial baudrate (e.g., 1000000)
            scan_mode: Scan mode (Standard, DenseBoost)
            scan_frequency_hz: Scan frequency in Hz
        """
        params = {}
        if device is not None:
            params["device"] = device
        if baudrate is not None:
            params["baudrate"] = baudrate
        if scan_mode is not None:
            params["scan_mode"] = scan_mode
        if scan_frequency_hz is not None:
            params["scan_frequency_hz"] = scan_frequency_hz
        
        result = self._request("POST", "/api/lidar/configure", params)
        return result.get("status") == "ok"
    
    # === IMU Control ===
    
    def start_imu(self) -> bool:
        """Start IMU streaming."""
        result = self._request("POST", "/api/imu/start")
        return result.get("status") == "ok"
    
    def stop_imu(self) -> bool:
        """Stop IMU streaming."""
        result = self._request("POST", "/api/imu/stop")
        return result.get("status") == "ok"
    
    def configure_imu(
        self,
        i2c_bus: Optional[int] = None,
        i2c_address: Optional[int] = None,
        gyro_range_dps: Optional[int] = None,
        accel_range_g: Optional[int] = None,
        sample_rate_hz: Optional[float] = None
    ) -> bool:
        """Configure IMU parameters.
        
        Args:
            i2c_bus: I2C bus number
            i2c_address: I2C address
            gyro_range_dps: Gyroscope range (125, 250, 500, 1000, 2000)
            accel_range_g: Accelerometer range (2, 4, 8, 16)
            sample_rate_hz: Sample rate in Hz
        """
        params = {}
        if i2c_bus is not None:
            params["i2c_bus"] = i2c_bus
        if i2c_address is not None:
            params["i2c_address"] = i2c_address
        if gyro_range_dps is not None:
            params["gyro_range_dps"] = gyro_range_dps
        if accel_range_g is not None:
            params["accel_range_g"] = accel_range_g
        if sample_rate_hz is not None:
            params["sample_rate_hz"] = sample_rate_hz
        
        result = self._request("POST", "/api/imu/configure", params)
        return result.get("status") == "ok"
    
    def imu_selftest(self) -> Dict[str, Any]:
        """Run IMU self-test."""
        return self._request("POST", "/api/imu/selftest")
    
    # === Session Management ===
    
    def list_sessions(self) -> List[Dict[str, Any]]:
        """List available capture sessions."""
        result = self._request("GET", "/api/session/list")
        return result.get("sessions", [])
    
    def start_session(self, name: Optional[str] = None) -> str:
        """Start a new capture session.
        
        Args:
            name: Session name (auto-generated if not provided)
            
        Returns:
            Session name
        """
        data = {"name": name} if name else {}
        result = self._request("POST", "/api/session/start", data)
        return result.get("name", "")
    
    def stop_session(self) -> Dict[str, Any]:
        """Stop the current capture session."""
        return self._request("POST", "/api/session/stop")
    
    # === Configuration ===
    
    def update_config(self, config: Dict[str, Any]) -> bool:
        """Update daemon configuration."""
        result = self._request("POST", "/api/config", config)
        return result.get("status") == "ok"
    
    # === WebSocket Streaming ===
    
    def connect_lidar_stream(self):
        """Connect to LiDAR WebSocket stream."""
        if not WEBSOCKET_AVAILABLE:
            raise RuntimeError("websocket-client not installed. pip install websocket-client")
        
        ws_url = f"{self.config.ws_base_url}/ws/lidar"
        
        def on_message(ws, message):
            try:
                data = json.loads(message)
                if data.get("type") == "lidar_frame" and self.on_lidar_frame:
                    self.on_lidar_frame(data.get("data", {}))
            except json.JSONDecodeError:
                pass
        
        def on_error(ws, error):
            logger.error(f"LiDAR WebSocket error: {error}")
            if self.on_connection_lost:
                self.on_connection_lost("lidar")
        
        def on_close(ws, close_status_code, close_msg):
            logger.info("LiDAR WebSocket closed")
        
        def on_open(ws):
            logger.info("LiDAR WebSocket connected")
            if self.on_connection_restored:
                self.on_connection_restored("lidar")
        
        self._ws_lidar = websocket.WebSocketApp(
            ws_url,
            on_message=on_message,
            on_error=on_error,
            on_close=on_close,
            on_open=on_open
        )
        
        thread = threading.Thread(
            target=self._ws_lidar.run_forever,
            daemon=True
        )
        thread.start()
        self._ws_threads["lidar"] = thread
    
    def connect_imu_stream(self):
        """Connect to IMU WebSocket stream."""
        if not WEBSOCKET_AVAILABLE:
            raise RuntimeError("websocket-client not installed. pip install websocket-client")
        
        ws_url = f"{self.config.ws_base_url}/ws/imu"
        
        def on_message(ws, message):
            try:
                data = json.loads(message)
                if data.get("type") == "imu_sample" and self.on_imu_sample:
                    self.on_imu_sample(data.get("data", {}))
            except json.JSONDecodeError:
                pass
        
        def on_error(ws, error):
            logger.error(f"IMU WebSocket error: {error}")
            if self.on_connection_lost:
                self.on_connection_lost("imu")
        
        def on_close(ws, close_status_code, close_msg):
            logger.info("IMU WebSocket closed")
        
        def on_open(ws):
            logger.info("IMU WebSocket connected")
            if self.on_connection_restored:
                self.on_connection_restored("imu")
        
        self._ws_imu = websocket.WebSocketApp(
            ws_url,
            on_message=on_message,
            on_error=on_error,
            on_close=on_close,
            on_open=on_open
        )
        
        thread = threading.Thread(
            target=self._ws_imu.run_forever,
            daemon=True
        )
        thread.start()
        self._ws_threads["imu"] = thread
    
    def connect_event_stream(self):
        """Connect to events WebSocket stream."""
        if not WEBSOCKET_AVAILABLE:
            raise RuntimeError("websocket-client not installed. pip install websocket-client")
        
        ws_url = f"{self.config.ws_base_url}/ws/events"
        
        def on_message(ws, message):
            try:
                data = json.loads(message)
                if self.on_event:
                    self.on_event(data)
            except json.JSONDecodeError:
                pass
        
        def on_error(ws, error):
            logger.error(f"Events WebSocket error: {error}")
        
        def on_close(ws, close_status_code, close_msg):
            logger.info("Events WebSocket closed")
        
        def on_open(ws):
            logger.info("Events WebSocket connected")
        
        self._ws_events = websocket.WebSocketApp(
            ws_url,
            on_message=on_message,
            on_error=on_error,
            on_close=on_close,
            on_open=on_open
        )
        
        thread = threading.Thread(
            target=self._ws_events.run_forever,
            daemon=True
        )
        thread.start()
        self._ws_threads["events"] = thread
    
    def connect_streams(self):
        """Connect all WebSocket streams."""
        self.connect_lidar_stream()
        self.connect_imu_stream()
        self.connect_event_stream()
    
    def disconnect_streams(self):
        """Disconnect all WebSocket streams."""
        if self._ws_lidar:
            self._ws_lidar.close()
            self._ws_lidar = None
        if self._ws_imu:
            self._ws_imu.close()
            self._ws_imu = None
        if self._ws_events:
            self._ws_events.close()
            self._ws_events = None
    
    def close(self):
        """Close all connections."""
        self.disconnect_streams()
        self._connected = False


def discover_devices(timeout: float = 3.0, ports: List[int] = None) -> List[str]:
    """Discover Planar capture daemons on the local network.
    
    This performs a simple scan of common local network ranges.
    For production use, consider mDNS/Bonjour discovery.
    
    Args:
        timeout: Timeout for each connection attempt
        ports: List of ports to scan (default: [8080])
        
    Returns:
        List of host:port strings for discovered daemons
    """
    import socket
    import concurrent.futures
    
    if ports is None:
        ports = [8080]
    
    discovered = []
    
    # Get local IP to determine network
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        local_ip = s.getsockname()[0]
        s.close()
    except Exception:
        local_ip = "192.168.1.1"
    
    # Scan local subnet
    subnet = ".".join(local_ip.split(".")[:3])
    
    def check_host(host_port):
        host, port = host_port
        try:
            client = PlanarClient(host, port, timeout=timeout)
            if client.ping():
                return f"{host}:{port}"
        except Exception:
            pass
        return None
    
    targets = [(f"{subnet}.{i}", port) for i in range(1, 255) for port in ports]
    
    with concurrent.futures.ThreadPoolExecutor(max_workers=50) as executor:
        results = executor.map(check_host, targets)
        for result in results:
            if result:
                discovered.append(result)
    
    return discovered
