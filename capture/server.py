"""HTTP and WebSocket server for remote control of Planar capture.

This module provides:
- REST API for configuration and control
- WebSocket streaming for real-time sensor data
- Device status monitoring
- Session management

API Endpoints:
  GET  /api/status         - Overall system status
  GET  /api/lidar/status   - LiDAR status and diagnostics
  POST /api/lidar/start    - Start LiDAR scanning
  POST /api/lidar/stop     - Stop LiDAR scanning
  POST /api/lidar/configure - Configure LiDAR parameters
  GET  /api/imu/status     - IMU status and diagnostics
  POST /api/imu/start      - Start IMU streaming
  POST /api/imu/stop       - Stop IMU streaming  
  POST /api/imu/configure  - Configure IMU parameters
  POST /api/imu/selftest   - Run IMU self-test
  GET  /api/session/list   - List sessions
  POST /api/session/start  - Start new capture session
  POST /api/session/stop   - Stop current session
  GET  /api/config         - Get current configuration
  POST /api/config         - Update configuration

WebSocket Endpoints:
  /ws/lidar   - Real-time LiDAR scan frames (JSON)
  /ws/imu     - Real-time IMU samples (JSON)
  /ws/events  - System events and status updates
"""

import asyncio
import json
import logging
import time
from typing import Optional, Dict, Any, Set, Callable
from dataclasses import dataclass, asdict
from enum import Enum
import threading

logger = logging.getLogger(__name__)

# Try to import web framework
try:
    from aiohttp import web
    import aiohttp
    AIOHTTP_AVAILABLE = True
except ImportError:
    AIOHTTP_AVAILABLE = False
    logger.warning("aiohttp not installed. Install with: pip install aiohttp")


class MessageType(Enum):
    """WebSocket message types."""
    LIDAR_FRAME = "lidar_frame"
    IMU_SAMPLE = "imu_sample"
    STATUS_UPDATE = "status_update"
    ERROR = "error"
    EVENT = "event"


@dataclass
class WSMessage:
    """WebSocket message format."""
    type: str
    data: Dict[str, Any]
    timestamp: float
    
    def to_json(self) -> str:
        return json.dumps({
            "type": self.type,
            "data": self.data,
            "timestamp": self.timestamp
        })


class CaptureServer:
    """HTTP and WebSocket server for remote capture control."""
    
    def __init__(
        self,
        host: str = "0.0.0.0",
        http_port: int = 8080,
        ws_port: int = 8081,
        auth_token: Optional[str] = None
    ):
        self.host = host
        self.http_port = http_port
        self.ws_port = ws_port
        self.auth_token = auth_token
        
        self._app: Optional[web.Application] = None
        self._runner: Optional[web.AppRunner] = None
        self._site: Optional[web.TCPSite] = None
        
        # Connected WebSocket clients
        self._lidar_clients: Set[web.WebSocketResponse] = set()
        self._imu_clients: Set[web.WebSocketResponse] = set()
        self._event_clients: Set[web.WebSocketResponse] = set()
        
        # Callbacks for actions (set by daemon)
        self.on_lidar_start: Optional[Callable[[], bool]] = None
        self.on_lidar_stop: Optional[Callable[[], None]] = None
        self.on_lidar_configure: Optional[Callable[[Dict], bool]] = None
        self.on_imu_start: Optional[Callable[[], bool]] = None
        self.on_imu_stop: Optional[Callable[[], None]] = None
        self.on_imu_configure: Optional[Callable[[Dict], bool]] = None
        self.on_imu_selftest: Optional[Callable[[], Dict]] = None
        self.on_session_start: Optional[Callable[[str], bool]] = None
        self.on_session_stop: Optional[Callable[[], Dict]] = None
        self.on_mark_station: Optional[Callable[[], Dict]] = None
        self.on_config_update: Optional[Callable[[Dict], bool]] = None
        
        # Status providers (set by daemon)
        self.get_status: Optional[Callable[[], Dict]] = None
        self.get_lidar_status: Optional[Callable[[], Dict]] = None
        self.get_imu_status: Optional[Callable[[], Dict]] = None
        self.get_sessions: Optional[Callable[[], list]] = None
        self.get_config: Optional[Callable[[], Dict]] = None
        
        self._running = False
    
    def _check_auth(self, request: web.Request) -> bool:
        """Check authentication token if configured."""
        if not self.auth_token:
            return True
        
        auth_header = request.headers.get("Authorization", "")
        if auth_header.startswith("Bearer "):
            token = auth_header[7:]
            return token == self.auth_token
        
        return False
    
    def _require_auth(self, handler):
        """Decorator to require authentication."""
        async def wrapper(request):
            if not self._check_auth(request):
                return web.json_response(
                    {"error": "Unauthorized"},
                    status=401
                )
            return await handler(request)
        return wrapper
    
    def _setup_routes(self, app: web.Application):
        """Configure HTTP routes."""
        # Status endpoints
        app.router.add_get("/api/status", self._handle_status)
        app.router.add_get("/api/lidar/status", self._handle_lidar_status)
        app.router.add_get("/api/imu/status", self._handle_imu_status)
        
        # LiDAR control
        app.router.add_post("/api/lidar/start", self._handle_lidar_start)
        app.router.add_post("/api/lidar/stop", self._handle_lidar_stop)
        app.router.add_post("/api/lidar/configure", self._handle_lidar_configure)
        
        # IMU control
        app.router.add_post("/api/imu/start", self._handle_imu_start)
        app.router.add_post("/api/imu/stop", self._handle_imu_stop)
        app.router.add_post("/api/imu/configure", self._handle_imu_configure)
        app.router.add_post("/api/imu/selftest", self._handle_imu_selftest)
        
        # Session management
        app.router.add_get("/api/session/list", self._handle_session_list)
        app.router.add_post("/api/session/start", self._handle_session_start)
        app.router.add_post("/api/session/stop", self._handle_session_stop)
        
        # Configuration
        app.router.add_get("/api/config", self._handle_config_get)
        app.router.add_post("/api/config", self._handle_config_set)
        
        # WebSocket endpoints
        app.router.add_get("/ws/lidar", self._handle_ws_lidar)
        app.router.add_get("/ws/imu", self._handle_ws_imu)
        app.router.add_get("/ws/events", self._handle_ws_events)
        app.router.add_get("/ws", self._handle_ws_unified)  # Unified endpoint for web UI
        
        # Session station marking
        app.router.add_post("/session/start", self._handle_session_start)
        app.router.add_post("/session/stop", self._handle_session_stop)
        app.router.add_post("/session/mark_station", self._handle_mark_station)
        
        # Simplified device control endpoints (for web UI)
        app.router.add_get("/status", self._handle_status)
        app.router.add_post("/lidar/start", self._handle_lidar_start)
        app.router.add_post("/lidar/stop", self._handle_lidar_stop)
        app.router.add_post("/imu/start", self._handle_imu_start)
        app.router.add_post("/imu/stop", self._handle_imu_stop)
        app.router.add_post("/imu/self_test", self._handle_imu_selftest)
        
        # Health check
        app.router.add_get("/health", self._handle_health)
    
    # === HTTP Handlers ===
    
    async def _handle_health(self, request: web.Request) -> web.Response:
        """Health check endpoint."""
        return web.json_response({"status": "ok", "timestamp": time.time()})
    
    async def _handle_status(self, request: web.Request) -> web.Response:
        """Get overall system status."""
        if not self._check_auth(request):
            return web.json_response({"error": "Unauthorized"}, status=401)
        
        if self.get_status:
            status = self.get_status()
        else:
            status = {"error": "Status provider not configured"}
        
        return web.json_response(status)
    
    async def _handle_lidar_status(self, request: web.Request) -> web.Response:
        """Get LiDAR status."""
        if not self._check_auth(request):
            return web.json_response({"error": "Unauthorized"}, status=401)
        
        if self.get_lidar_status:
            status = self.get_lidar_status()
        else:
            status = {"error": "LiDAR status provider not configured"}
        
        return web.json_response(status)
    
    async def _handle_imu_status(self, request: web.Request) -> web.Response:
        """Get IMU status."""
        if not self._check_auth(request):
            return web.json_response({"error": "Unauthorized"}, status=401)
        
        if self.get_imu_status:
            status = self.get_imu_status()
        else:
            status = {"error": "IMU status provider not configured"}
        
        return web.json_response(status)
    
    async def _handle_lidar_start(self, request: web.Request) -> web.Response:
        """Start LiDAR scanning."""
        if not self._check_auth(request):
            return web.json_response({"error": "Unauthorized"}, status=401)
        
        if self.on_lidar_start:
            success = self.on_lidar_start()
            if success:
                await self._broadcast_event("lidar_started", {})
                return web.json_response({"status": "ok"})
            else:
                return web.json_response({"error": "Failed to start LiDAR"}, status=500)
        
        return web.json_response({"error": "Handler not configured"}, status=501)
    
    async def _handle_lidar_stop(self, request: web.Request) -> web.Response:
        """Stop LiDAR scanning."""
        if not self._check_auth(request):
            return web.json_response({"error": "Unauthorized"}, status=401)
        
        if self.on_lidar_stop:
            self.on_lidar_stop()
            await self._broadcast_event("lidar_stopped", {})
            return web.json_response({"status": "ok"})
        
        return web.json_response({"error": "Handler not configured"}, status=501)
    
    async def _handle_lidar_configure(self, request: web.Request) -> web.Response:
        """Configure LiDAR parameters."""
        if not self._check_auth(request):
            return web.json_response({"error": "Unauthorized"}, status=401)
        
        try:
            data = await request.json()
        except json.JSONDecodeError:
            return web.json_response({"error": "Invalid JSON"}, status=400)
        
        if self.on_lidar_configure:
            success = self.on_lidar_configure(data)
            if success:
                await self._broadcast_event("lidar_configured", data)
                return web.json_response({"status": "ok"})
            else:
                return web.json_response({"error": "Configuration failed"}, status=400)
        
        return web.json_response({"error": "Handler not configured"}, status=501)
    
    async def _handle_imu_start(self, request: web.Request) -> web.Response:
        """Start IMU streaming."""
        if not self._check_auth(request):
            return web.json_response({"error": "Unauthorized"}, status=401)
        
        if self.on_imu_start:
            success = self.on_imu_start()
            if success:
                await self._broadcast_event("imu_started", {})
                return web.json_response({"status": "ok"})
            else:
                return web.json_response({"error": "Failed to start IMU"}, status=500)
        
        return web.json_response({"error": "Handler not configured"}, status=501)
    
    async def _handle_imu_stop(self, request: web.Request) -> web.Response:
        """Stop IMU streaming."""
        if not self._check_auth(request):
            return web.json_response({"error": "Unauthorized"}, status=401)
        
        if self.on_imu_stop:
            self.on_imu_stop()
            await self._broadcast_event("imu_stopped", {})
            return web.json_response({"status": "ok"})
        
        return web.json_response({"error": "Handler not configured"}, status=501)
    
    async def _handle_imu_configure(self, request: web.Request) -> web.Response:
        """Configure IMU parameters."""
        if not self._check_auth(request):
            return web.json_response({"error": "Unauthorized"}, status=401)
        
        try:
            data = await request.json()
        except json.JSONDecodeError:
            return web.json_response({"error": "Invalid JSON"}, status=400)
        
        if self.on_imu_configure:
            success = self.on_imu_configure(data)
            if success:
                await self._broadcast_event("imu_configured", data)
                return web.json_response({"status": "ok"})
            else:
                return web.json_response({"error": "Configuration failed"}, status=400)
        
        return web.json_response({"error": "Handler not configured"}, status=501)
    
    async def _handle_imu_selftest(self, request: web.Request) -> web.Response:
        """Run IMU self-test."""
        if not self._check_auth(request):
            return web.json_response({"error": "Unauthorized"}, status=401)
        
        if self.on_imu_selftest:
            results = self.on_imu_selftest()
            return web.json_response(results)
        
        return web.json_response({"error": "Handler not configured"}, status=501)
    
    async def _handle_session_list(self, request: web.Request) -> web.Response:
        """List capture sessions."""
        if not self._check_auth(request):
            return web.json_response({"error": "Unauthorized"}, status=401)
        
        if self.get_sessions:
            sessions = self.get_sessions()
            return web.json_response({"sessions": sessions})
        
        return web.json_response({"error": "Handler not configured"}, status=501)
    
    async def _handle_session_start(self, request: web.Request) -> web.Response:
        """Start a new capture session."""
        if not self._check_auth(request):
            return web.json_response({"error": "Unauthorized"}, status=401)
        
        try:
            data = await request.json()
            session_name = data.get("name", f"session_{int(time.time())}")
        except json.JSONDecodeError:
            session_name = f"session_{int(time.time())}"
        
        if self.on_session_start:
            success = self.on_session_start(session_name)
            if success:
                await self._broadcast_event("session_started", {"name": session_name})
                return web.json_response({"status": "ok", "name": session_name})
            else:
                return web.json_response({"error": "Failed to start session"}, status=500)
        
        return web.json_response({"error": "Handler not configured"}, status=501)
    
    async def _handle_session_stop(self, request: web.Request) -> web.Response:
        """Stop current capture session."""
        if not self._check_auth(request):
            return web.json_response({"error": "Unauthorized"}, status=401)
        
        if self.on_session_stop:
            result = self.on_session_stop()
            await self._broadcast_event("session_stopped", result)
            return web.json_response(result)
        
        return web.json_response({"error": "Handler not configured"}, status=501)
    
    async def _handle_config_get(self, request: web.Request) -> web.Response:
        """Get current configuration."""
        if not self._check_auth(request):
            return web.json_response({"error": "Unauthorized"}, status=401)
        
        if self.get_config:
            config = self.get_config()
            return web.json_response(config)
        
        return web.json_response({"error": "Handler not configured"}, status=501)
    
    async def _handle_config_set(self, request: web.Request) -> web.Response:
        """Update configuration."""
        if not self._check_auth(request):
            return web.json_response({"error": "Unauthorized"}, status=401)
        
        try:
            data = await request.json()
        except json.JSONDecodeError:
            return web.json_response({"error": "Invalid JSON"}, status=400)
        
        if self.on_config_update:
            success = self.on_config_update(data)
            if success:
                await self._broadcast_event("config_updated", data)
                return web.json_response({"status": "ok"})
            else:
                return web.json_response({"error": "Configuration update failed"}, status=400)
        
        return web.json_response({"error": "Handler not configured"}, status=501)
    
    async def _handle_mark_station(self, request: web.Request) -> web.Response:
        """Mark a station in the current session."""
        if not self._check_auth(request):
            return web.json_response({"error": "Unauthorized"}, status=401)
        
        if self.on_mark_station:
            result = self.on_mark_station()
            if result.get("status") == "ok":
                await self._broadcast_event("station_captured", result)
                return web.json_response(result)
            else:
                return web.json_response(result, status=400)
        
        return web.json_response({"error": "Handler not configured"}, status=501)
    
    # === WebSocket Handlers ===
    
    async def _handle_ws_lidar(self, request: web.Request) -> web.WebSocketResponse:
        """WebSocket endpoint for LiDAR data streaming."""
        ws = web.WebSocketResponse()
        await ws.prepare(request)
        
        self._lidar_clients.add(ws)
        logger.info(f"LiDAR WebSocket client connected ({len(self._lidar_clients)} total)")
        
        try:
            async for msg in ws:
                if msg.type == aiohttp.WSMsgType.TEXT:
                    # Handle any commands from client
                    try:
                        data = json.loads(msg.data)
                        if data.get("command") == "ping":
                            await ws.send_json({"type": "pong", "timestamp": time.time()})
                    except json.JSONDecodeError:
                        pass
                elif msg.type == aiohttp.WSMsgType.ERROR:
                    logger.error(f"WebSocket error: {ws.exception()}")
        finally:
            self._lidar_clients.discard(ws)
            logger.info(f"LiDAR WebSocket client disconnected ({len(self._lidar_clients)} remaining)")
        
        return ws
    
    async def _handle_ws_imu(self, request: web.Request) -> web.WebSocketResponse:
        """WebSocket endpoint for IMU data streaming."""
        ws = web.WebSocketResponse()
        await ws.prepare(request)
        
        self._imu_clients.add(ws)
        logger.info(f"IMU WebSocket client connected ({len(self._imu_clients)} total)")
        
        try:
            async for msg in ws:
                if msg.type == aiohttp.WSMsgType.TEXT:
                    try:
                        data = json.loads(msg.data)
                        if data.get("command") == "ping":
                            await ws.send_json({"type": "pong", "timestamp": time.time()})
                    except json.JSONDecodeError:
                        pass
                elif msg.type == aiohttp.WSMsgType.ERROR:
                    logger.error(f"WebSocket error: {ws.exception()}")
        finally:
            self._imu_clients.discard(ws)
            logger.info(f"IMU WebSocket client disconnected ({len(self._imu_clients)} remaining)")
        
        return ws
    
    async def _handle_ws_events(self, request: web.Request) -> web.WebSocketResponse:
        """WebSocket endpoint for system events."""
        ws = web.WebSocketResponse()
        await ws.prepare(request)
        
        self._event_clients.add(ws)
        logger.info(f"Events WebSocket client connected ({len(self._event_clients)} total)")
        
        # Send initial status
        if self.get_status:
            status = self.get_status()
            await ws.send_json({
                "type": "status_update",
                "data": status,
                "timestamp": time.time()
            })
        
        try:
            async for msg in ws:
                if msg.type == aiohttp.WSMsgType.TEXT:
                    try:
                        data = json.loads(msg.data)
                        if data.get("command") == "ping":
                            await ws.send_json({"type": "pong", "timestamp": time.time()})
                    except json.JSONDecodeError:
                        pass
                elif msg.type == aiohttp.WSMsgType.ERROR:
                    logger.error(f"WebSocket error: {ws.exception()}")
        finally:
            self._event_clients.discard(ws)
            logger.info(f"Events WebSocket client disconnected ({len(self._event_clients)} remaining)")
        
        return ws
    
    async def _handle_ws_unified(self, request: web.Request) -> web.WebSocketResponse:
        """Unified WebSocket endpoint for web UI - receives both LiDAR and IMU data."""
        ws = web.WebSocketResponse()
        await ws.prepare(request)
        
        # Add to all client sets for unified streaming
        self._lidar_clients.add(ws)
        self._imu_clients.add(ws)
        self._event_clients.add(ws)
        logger.info(f"Unified WebSocket client connected")
        
        # Send initial status
        if self.get_status:
            status = self.get_status()
            await ws.send_json({
                "type": "status",
                "lidar": status.get("lidar", {}),
                "imu": status.get("imu", {}),
                "session": status.get("session", {}),
                "timestamp": time.time()
            })
        
        try:
            async for msg in ws:
                if msg.type == aiohttp.WSMsgType.TEXT:
                    try:
                        data = json.loads(msg.data)
                        if data.get("command") == "ping":
                            await ws.send_json({"type": "pong", "timestamp": time.time()})
                    except json.JSONDecodeError:
                        pass
                elif msg.type == aiohttp.WSMsgType.ERROR:
                    logger.error(f"WebSocket error: {ws.exception()}")
        finally:
            self._lidar_clients.discard(ws)
            self._imu_clients.discard(ws)
            self._event_clients.discard(ws)
            logger.info(f"Unified WebSocket client disconnected")
        
        return ws
    
    # === Broadcasting ===
    
    async def _broadcast_event(self, event_type: str, data: Dict[str, Any]):
        """Broadcast event to all event clients."""
        msg = {
            "type": "event",
            "event": event_type,
            "data": data,
            "timestamp": time.time()
        }
        
        dead_clients = set()
        for ws in self._event_clients:
            try:
                await ws.send_json(msg)
            except Exception:
                dead_clients.add(ws)
        
        self._event_clients -= dead_clients
    
    def broadcast_lidar_frame_sync(self, frame_data: Dict[str, Any]):
        """Broadcast LiDAR frame (called from sync context, schedules async)."""
        if not self._lidar_clients:
            return
        
        # Convert to format expected by web UI
        # frame_data contains: points as list of (angle, distance, quality)
        msg = {
            "type": "lidar_frame",
            "points": frame_data.get("points", []),
            "timestamp": time.time()
        }
        
        asyncio.run_coroutine_threadsafe(
            self._broadcast_to_clients(self._lidar_clients, msg),
            asyncio.get_event_loop()
        )
    
    def broadcast_imu_sample_sync(self, sample_data: Dict[str, Any]):
        """Broadcast IMU sample (called from sync context, schedules async)."""
        if not self._imu_clients:
            return
        
        # Convert to format expected by web UI
        msg = {
            "type": "imu_sample",
            "gyro_z": sample_data.get("gyro_z", 0),
            "accel_z": sample_data.get("accel_z", 0),
            "accel_x": sample_data.get("accel_x", 0),
            "accel_y": sample_data.get("accel_y", 0),
            "temperature": sample_data.get("temperature", 0),
            "timestamp": time.time()
        }
        
        asyncio.run_coroutine_threadsafe(
            self._broadcast_to_clients(self._imu_clients, msg),
            asyncio.get_event_loop()
        )
    
    async def _broadcast_to_clients(self, clients: Set[web.WebSocketResponse], msg: Dict):
        """Broadcast message to set of clients."""
        dead_clients = set()
        for ws in clients:
            try:
                await ws.send_json(msg)
            except Exception:
                dead_clients.add(ws)
        
        clients -= dead_clients
    
    # === Server Lifecycle ===
    
    async def start(self):
        """Start the HTTP/WebSocket server."""
        if not AIOHTTP_AVAILABLE:
            raise RuntimeError("aiohttp not installed")
        
        self._app = web.Application()
        self._setup_routes(self._app)
        
        self._runner = web.AppRunner(self._app)
        await self._runner.setup()
        
        self._site = web.TCPSite(self._runner, self.host, self.http_port)
        await self._site.start()
        
        self._running = True
        logger.info(f"Capture server started on http://{self.host}:{self.http_port}")
    
    async def stop(self):
        """Stop the server."""
        self._running = False
        
        # Close all WebSocket connections
        for ws in list(self._lidar_clients):
            await ws.close()
        for ws in list(self._imu_clients):
            await ws.close()
        for ws in list(self._event_clients):
            await ws.close()
        
        if self._site:
            await self._site.stop()
        if self._runner:
            await self._runner.cleanup()
        
        logger.info("Capture server stopped")
    
    @property
    def is_running(self) -> bool:
        """Check if server is running."""
        return self._running
    
    @property
    def client_count(self) -> Dict[str, int]:
        """Get count of connected clients by type."""
        return {
            "lidar": len(self._lidar_clients),
            "imu": len(self._imu_clients),
            "events": len(self._event_clients),
        }
