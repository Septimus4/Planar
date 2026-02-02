"""Tests for capture server API."""

import json
import pytest
from unittest.mock import Mock, MagicMock, AsyncMock, patch
import asyncio

# Check if aiohttp is available
pytest.importorskip("aiohttp")

from capture.server import CaptureServer, MessageType, WSMessage


class TestWSMessage:
    """Tests for WebSocket message format."""

    def test_to_json(self):
        msg = WSMessage(
            type="lidar_frame",
            data={"frame_id": 1, "points": []},
            timestamp=1000.0,
        )
        
        json_str = msg.to_json()
        parsed = json.loads(json_str)
        
        assert parsed["type"] == "lidar_frame"
        assert parsed["data"]["frame_id"] == 1
        assert parsed["timestamp"] == 1000.0


class TestCaptureServerUnit:
    """Unit tests for CaptureServer."""

    def test_initial_state(self):
        server = CaptureServer(host="127.0.0.1", http_port=8080)
        assert server.host == "127.0.0.1"
        assert server.http_port == 8080
        assert server.is_running is False

    def test_with_auth_token(self):
        server = CaptureServer(auth_token="secret123")
        assert server.auth_token == "secret123"

    def test_client_count_initially_zero(self):
        server = CaptureServer()
        counts = server.client_count
        assert counts["lidar"] == 0
        assert counts["imu"] == 0
        assert counts["events"] == 0

    def test_callbacks_initially_none(self):
        server = CaptureServer()
        assert server.on_lidar_start is None
        assert server.on_lidar_stop is None
        assert server.on_imu_start is None
        assert server.on_imu_stop is None
        assert server.get_status is None

    def test_set_callbacks(self):
        server = CaptureServer()
        
        mock_start = Mock(return_value=True)
        mock_stop = Mock()
        mock_status = Mock(return_value={"status": "ok"})
        
        server.on_lidar_start = mock_start
        server.on_lidar_stop = mock_stop
        server.get_status = mock_status
        
        assert server.on_lidar_start is mock_start
        assert server.on_lidar_stop is mock_stop
        assert server.get_status is mock_status


@pytest.mark.asyncio
class TestCaptureServerAsync:
    """Async tests for CaptureServer."""

    @pytest.fixture
    def server(self):
        """Create a test server instance."""
        return CaptureServer(host="127.0.0.1", http_port=0)  # Port 0 = auto-assign

    async def test_server_lifecycle(self, server):
        """Test starting and stopping server."""
        await server.start()
        assert server.is_running is True
        
        await server.stop()
        assert server.is_running is False


@pytest.mark.asyncio
class TestCaptureServerAPI:
    """Tests for HTTP API endpoints using aiohttp test client."""

    @pytest.fixture
    async def client(self):
        """Create test client for server."""
        from aiohttp import web
        from aiohttp.test_utils import TestClient, TestServer
        
        server = CaptureServer()
        
        # Set up mock handlers
        server.get_status = Mock(return_value={
            "daemon": {"running": True},
            "lidar": {"status": "idle"},
            "imu": {"status": "idle"},
        })
        server.get_lidar_status = Mock(return_value={
            "status": "idle",
            "connected": True,
        })
        server.get_imu_status = Mock(return_value={
            "status": "idle",
            "connected": True,
        })
        server.on_lidar_start = Mock(return_value=True)
        server.on_lidar_stop = Mock()
        server.on_imu_start = Mock(return_value=True)
        server.on_imu_stop = Mock()
        server.on_imu_selftest = Mock(return_value={"passed": True})
        server.get_sessions = Mock(return_value=[])
        server.on_session_start = Mock(return_value=True)
        server.on_session_stop = Mock(return_value={"name": "test"})
        server.get_config = Mock(return_value={"lidar": {}, "imu": {}})
        server.on_config_update = Mock(return_value=True)
        server.on_lidar_configure = Mock(return_value=True)
        server.on_imu_configure = Mock(return_value=True)
        
        # Create app and set up routes
        app = web.Application()
        server._setup_routes(app)
        server._app = app
        
        async with TestClient(TestServer(app)) as client:
            # Store server reference for handlers
            client.server_instance = server
            yield client

    async def test_health_endpoint(self, client):
        """Test /health endpoint."""
        resp = await client.get("/health")
        assert resp.status == 200
        
        data = await resp.json()
        assert data["status"] == "ok"
        assert "timestamp" in data

    async def test_status_endpoint(self, client):
        """Test /api/status endpoint."""
        resp = await client.get("/api/status")
        assert resp.status == 200
        
        data = await resp.json()
        assert "daemon" in data
        assert "lidar" in data
        assert "imu" in data

    async def test_lidar_status_endpoint(self, client):
        """Test /api/lidar/status endpoint."""
        resp = await client.get("/api/lidar/status")
        assert resp.status == 200
        
        data = await resp.json()
        assert "status" in data
        assert "connected" in data

    async def test_lidar_start_endpoint(self, client):
        """Test POST /api/lidar/start endpoint."""
        resp = await client.post("/api/lidar/start")
        assert resp.status == 200
        
        data = await resp.json()
        assert data["status"] == "ok"
        client.server_instance.on_lidar_start.assert_called_once()

    async def test_lidar_stop_endpoint(self, client):
        """Test POST /api/lidar/stop endpoint."""
        resp = await client.post("/api/lidar/stop")
        assert resp.status == 200
        
        data = await resp.json()
        assert data["status"] == "ok"
        client.server_instance.on_lidar_stop.assert_called_once()

    async def test_lidar_configure_endpoint(self, client):
        """Test POST /api/lidar/configure endpoint."""
        resp = await client.post(
            "/api/lidar/configure",
            json={"scan_mode": "DenseBoost", "scan_frequency_hz": 10.0}
        )
        assert resp.status == 200
        
        data = await resp.json()
        assert data["status"] == "ok"

    async def test_imu_status_endpoint(self, client):
        """Test /api/imu/status endpoint."""
        resp = await client.get("/api/imu/status")
        assert resp.status == 200
        
        data = await resp.json()
        assert "status" in data

    async def test_imu_start_endpoint(self, client):
        """Test POST /api/imu/start endpoint."""
        resp = await client.post("/api/imu/start")
        assert resp.status == 200
        
        data = await resp.json()
        assert data["status"] == "ok"

    async def test_imu_selftest_endpoint(self, client):
        """Test POST /api/imu/selftest endpoint."""
        resp = await client.post("/api/imu/selftest")
        assert resp.status == 200
        
        data = await resp.json()
        assert "passed" in data

    async def test_session_list_endpoint(self, client):
        """Test GET /api/session/list endpoint."""
        resp = await client.get("/api/session/list")
        assert resp.status == 200
        
        data = await resp.json()
        assert "sessions" in data

    async def test_session_start_endpoint(self, client):
        """Test POST /api/session/start endpoint."""
        resp = await client.post(
            "/api/session/start",
            json={"name": "test_session"}
        )
        assert resp.status == 200
        
        data = await resp.json()
        assert data["status"] == "ok"

    async def test_session_stop_endpoint(self, client):
        """Test POST /api/session/stop endpoint."""
        resp = await client.post("/api/session/stop")
        assert resp.status == 200
        
        data = await resp.json()
        assert "name" in data

    async def test_config_get_endpoint(self, client):
        """Test GET /api/config endpoint."""
        resp = await client.get("/api/config")
        assert resp.status == 200
        
        data = await resp.json()
        assert "lidar" in data
        assert "imu" in data

    async def test_config_set_endpoint(self, client):
        """Test POST /api/config endpoint."""
        resp = await client.post(
            "/api/config",
            json={"lidar": {"scan_mode": "Standard"}}
        )
        assert resp.status == 200

    async def test_invalid_json_returns_400(self, client):
        """Test that invalid JSON returns 400."""
        resp = await client.post(
            "/api/lidar/configure",
            data="not json",
            headers={"Content-Type": "application/json"}
        )
        assert resp.status == 400


class TestAuthenticationUnit:
    """Tests for authentication handling."""

    def test_check_auth_no_token_configured(self):
        server = CaptureServer(auth_token=None)
        
        mock_request = Mock()
        mock_request.headers = {}
        
        assert server._check_auth(mock_request) is True

    def test_check_auth_valid_token(self):
        server = CaptureServer(auth_token="secret123")
        
        mock_request = Mock()
        mock_request.headers = {"Authorization": "Bearer secret123"}
        
        assert server._check_auth(mock_request) is True

    def test_check_auth_invalid_token(self):
        server = CaptureServer(auth_token="secret123")
        
        mock_request = Mock()
        mock_request.headers = {"Authorization": "Bearer wrongtoken"}
        
        assert server._check_auth(mock_request) is False

    def test_check_auth_missing_header(self):
        server = CaptureServer(auth_token="secret123")
        
        mock_request = Mock()
        mock_request.headers = {}
        
        assert server._check_auth(mock_request) is False

    def test_check_auth_wrong_scheme(self):
        server = CaptureServer(auth_token="secret123")
        
        mock_request = Mock()
        mock_request.headers = {"Authorization": "Basic secret123"}
        
        assert server._check_auth(mock_request) is False
