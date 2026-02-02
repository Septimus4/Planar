"""Tests for desktop controller module."""

import sys

import pytest

from desktop.client import PlanarClient
from desktop.controller import CaptureController, main


class TestDesktopController:
    """Tests for desktop controller CLI."""

    def test_imports(self):
        """Verify all desktop modules can be imported."""
        from desktop import client, controller
        assert hasattr(controller, 'main')
        assert hasattr(controller, 'CaptureController')
        assert hasattr(client, 'PlanarClient')

    def test_entry_point_help(self, monkeypatch):
        """Test that the main entry point can display help."""
        # Mock sys.argv to simulate --help
        monkeypatch.setattr(sys, 'argv', ['planar-desktop', '--help'])

        # The --help flag causes argparse to call sys.exit(0)
        with pytest.raises(SystemExit) as exc_info:
            main()

        assert exc_info.value.code == 0

    def test_controller_creation(self):
        """Test that a controller can be instantiated."""
        client = PlanarClient(host="localhost", port=8080)
        controller = CaptureController(client)
        assert controller.client == client
        assert controller._monitoring is False

    def test_client_config(self):
        """Test client configuration."""
        client = PlanarClient(host="192.168.1.100", port=8080, timeout=5.0)
        assert client.config.host == "192.168.1.100"
        assert client.config.http_port == 8080
        assert client.config.timeout == 5.0
        assert client.config.http_base_url == "http://192.168.1.100:8080"
        assert client.config.ws_base_url == "ws://192.168.1.100:8080"


class TestPyInstallerCompatibility:
    """Tests to ensure PyInstaller compatibility."""

    def test_no_dynamic_imports(self):
        """Verify that desktop modules don't use problematic dynamic imports."""
        import desktop.client
        import desktop.controller

        # Check that the main modules are importable
        assert desktop.controller.__file__
        assert desktop.client.__file__

    def test_dependencies_available(self):
        """Verify that required dependencies are available."""
        # These should be available in the desktop extras
        try:
            import requests
            assert requests.__version__
        except ImportError:
            pytest.skip("requests not installed (optional dependency)")

        try:
            import websocket
            assert websocket.__version__
        except ImportError:
            pytest.skip("websocket-client not installed (optional dependency)")

    def test_excluded_modules_not_required(self):
        """Verify that excluded modules (capture, processing) aren't imported."""
        # Import desktop modules to ensure they don't pull in capture/processing
        import desktop.client  # noqa: F401
        import desktop.controller  # noqa: F401

        # Desktop modules should not import capture-specific modules
        # We check that 'capture' is either not loaded, or if it is loaded,
        # it doesn't have the daemon module (which is Raspberry Pi specific)
        capture_loaded = 'capture' in sys.modules
        if capture_loaded:
            capture_module = sys.modules.get('capture')
            assert not hasattr(capture_module, 'daemon'), \
                "Desktop module should not import capture.daemon"
