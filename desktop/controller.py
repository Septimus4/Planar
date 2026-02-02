"""Desktop controller CLI and GUI for Planar capture.

This module provides:
- Command-line interface for remote capture control
- Status monitoring and diagnostics
- Sensor troubleshooting tools

Usage:
    # CLI mode
    python -m desktop.controller --host 192.168.1.100 status
    python -m desktop.controller --host 192.168.1.100 lidar start
    python -m desktop.controller --host 192.168.1.100 imu selftest
    python -m desktop.controller --host 192.168.1.100 session start my_scan
    
    # Interactive mode
    python -m desktop.controller --host 192.168.1.100 interactive
"""

import argparse
import json
import sys
import time
import threading
from typing import Optional, Dict, Any

from .client import PlanarClient, discover_devices


class CaptureController:
    """High-level controller for desktop capture management."""
    
    def __init__(self, client: PlanarClient):
        self.client = client
        self._monitoring = False
        self._monitor_thread: Optional[threading.Thread] = None
    
    def check_connection(self) -> bool:
        """Verify connection to Pi."""
        if self.client.ping():
            print(f"✓ Connected to {self.client.config.host}:{self.client.config.http_port}")
            return True
        else:
            print(f"✗ Cannot connect to {self.client.config.host}:{self.client.config.http_port}")
            return False
    
    def print_status(self):
        """Print comprehensive system status."""
        try:
            status = self.client.get_status()
        except Exception as e:
            print(f"Error getting status: {e}")
            return
        
        print("\n" + "=" * 60)
        print("PLANAR CAPTURE STATUS")
        print("=" * 60)
        
        # Daemon info
        daemon = status.get("daemon", {})
        print(f"\nDaemon:")
        print(f"  Running: {daemon.get('running', False)}")
        uptime = daemon.get("uptime", 0)
        print(f"  Uptime:  {int(uptime // 60)}m {int(uptime % 60)}s")
        
        # LiDAR status
        lidar = status.get("lidar", {})
        lidar_status = lidar.get("status", "unknown")
        lidar_icon = "✓" if lidar.get("connected") else "✗"
        print(f"\nLiDAR: {lidar_icon}")
        print(f"  Status:   {lidar_status}")
        print(f"  Scanning: {lidar.get('scanning', False)}")
        
        # IMU status
        imu = status.get("imu", {})
        imu_status = imu.get("status", "unknown")
        imu_icon = "✓" if imu.get("connected") else "✗"
        print(f"\nIMU: {imu_icon}")
        print(f"  Status:    {imu_status}")
        print(f"  Streaming: {imu.get('streaming', False)}")
        
        # Session info
        session = status.get("session", {})
        if session.get("active"):
            print(f"\nActive Session: {session.get('name')}")
            duration = session.get("duration", 0)
            print(f"  Duration:     {int(duration // 60)}m {int(duration % 60)}s")
            print(f"  Stations:     {session.get('station_count', 0)}")
            print(f"  LiDAR Points: {session.get('lidar_points', 0):,}")
            print(f"  IMU Samples:  {session.get('imu_samples', 0):,}")
        else:
            print(f"\nNo active session")
        
        # Server clients
        server = status.get("server", {})
        clients = server.get("clients", {})
        total_clients = sum(clients.values())
        if total_clients > 0:
            print(f"\nConnected clients: {total_clients}")
            for stream, count in clients.items():
                if count > 0:
                    print(f"  {stream}: {count}")
        
        print("\n" + "=" * 60)
    
    def print_lidar_details(self):
        """Print detailed LiDAR diagnostics."""
        try:
            status = self.client.get_lidar_status()
        except Exception as e:
            print(f"Error getting LiDAR status: {e}")
            return
        
        print("\n" + "-" * 40)
        print("LiDAR DIAGNOSTICS")
        print("-" * 40)
        
        print(f"Status:    {status.get('status', 'unknown')}")
        print(f"Connected: {status.get('connected', False)}")
        print(f"Scanning:  {status.get('scanning', False)}")
        print(f"Port:      {status.get('port', 'N/A')}")
        print(f"Baudrate:  {status.get('baudrate', 'N/A')}")
        
        info = status.get("info", {})
        if info:
            print(f"\nDevice Info:")
            print(f"  Model:     {info.get('model', 'N/A')}")
            print(f"  S/N:       {info.get('serial_number', 'N/A')}")
            print(f"  Firmware:  {info.get('firmware_version', 'N/A')}")
            print(f"  Hardware:  {info.get('hardware_revision', 'N/A')}")
            print(f"  Health:    {info.get('health_status', 'N/A')}")
        
        config = status.get("config", {})
        if config:
            print(f"\nConfiguration:")
            print(f"  Scan Mode:      {config.get('scan_mode', 'N/A')}")
            print(f"  Scan Frequency: {config.get('scan_frequency_hz', 'N/A')} Hz")
        
        stats = status.get("stats", {})
        if stats:
            print(f"\nStatistics:")
            print(f"  Frames Captured: {stats.get('frames_captured', 0):,}")
    
    def print_imu_details(self):
        """Print detailed IMU diagnostics."""
        try:
            status = self.client.get_imu_status()
        except Exception as e:
            print(f"Error getting IMU status: {e}")
            return
        
        print("\n" + "-" * 40)
        print("IMU DIAGNOSTICS")
        print("-" * 40)
        
        print(f"Status:    {status.get('status', 'unknown')}")
        print(f"Connected: {status.get('connected', False)}")
        print(f"Streaming: {status.get('streaming', False)}")
        print(f"I2C Bus:   {status.get('bus', 'N/A')}")
        print(f"Address:   {status.get('address', 'N/A')}")
        print(f"Chip ID:   {status.get('chip_id', 'N/A')}")
        
        config = status.get("config", {})
        if config:
            print(f"\nConfiguration:")
            print(f"  Gyro Range:   ±{config.get('gyro_range_dps', 'N/A')}°/s")
            print(f"  Accel Range:  ±{config.get('accel_range_g', 'N/A')}g")
            print(f"  Sample Rate:  {config.get('sample_rate_hz', 'N/A')} Hz")
        
        last_sample = status.get("last_sample")
        if last_sample:
            print(f"\nLast Sample:")
            print(f"  Gyro Z:      {last_sample.get('gyro_z_rad_s', 'N/A'):.6f} rad/s")
            print(f"  Accel Z:     {last_sample.get('accel_z_m_s2', 'N/A'):.3f} m/s²")
            print(f"  Temperature: {last_sample.get('temperature_c', 'N/A'):.1f}°C")
        
        stats = status.get("stats", {})
        if stats:
            print(f"\nStatistics:")
            print(f"  Samples Read: {stats.get('samples_read', 0):,}")
    
    def run_imu_selftest(self) -> bool:
        """Run IMU self-test and display results."""
        print("\nRunning IMU self-test...")
        
        try:
            results = self.client.imu_selftest()
        except Exception as e:
            print(f"Self-test failed: {e}")
            return False
        
        print("\n" + "-" * 40)
        print("IMU SELF-TEST RESULTS")
        print("-" * 40)
        
        passed = results.get("passed", False)
        icon = "✓ PASSED" if passed else "✗ FAILED"
        print(f"\nOverall: {icon}")
        
        print(f"\nChecks:")
        print(f"  Chip ID:       {'✓' if results.get('chip_id_ok') else '✗'}")
        print(f"  Gyroscope:     {'✓' if results.get('gyro_ok') else '✗'}")
        print(f"  Accelerometer: {'✓' if results.get('accel_ok') else '✗'}")
        
        errors = results.get("errors", [])
        if errors:
            print(f"\nErrors:")
            for err in errors:
                print(f"  - {err}")
        
        return passed
    
    def start_live_monitor(self, interval: float = 1.0):
        """Start live status monitoring."""
        self._monitoring = True
        
        def monitor_loop():
            while self._monitoring:
                try:
                    # Clear screen
                    print("\033[H\033[J", end="")
                    self.print_status()
                    print(f"\n[Press Ctrl+C to stop monitoring]")
                except Exception as e:
                    print(f"Monitor error: {e}")
                time.sleep(interval)
        
        self._monitor_thread = threading.Thread(target=monitor_loop, daemon=True)
        self._monitor_thread.start()
        
        try:
            while self._monitoring:
                time.sleep(0.1)
        except KeyboardInterrupt:
            self._monitoring = False
            print("\nMonitoring stopped.")
    
    def interactive_mode(self):
        """Run interactive command mode."""
        print("\n" + "=" * 60)
        print("PLANAR CAPTURE CONTROLLER - Interactive Mode")
        print("=" * 60)
        print("Commands: status, lidar, imu, session, config, help, quit")
        print()
        
        while True:
            try:
                cmd = input("planar> ").strip().lower().split()
                if not cmd:
                    continue
                
                if cmd[0] == "quit" or cmd[0] == "exit":
                    break
                elif cmd[0] == "help":
                    self._print_help()
                elif cmd[0] == "status":
                    self.print_status()
                elif cmd[0] == "lidar":
                    self._handle_lidar_cmd(cmd[1:])
                elif cmd[0] == "imu":
                    self._handle_imu_cmd(cmd[1:])
                elif cmd[0] == "session":
                    self._handle_session_cmd(cmd[1:])
                elif cmd[0] == "config":
                    self._handle_config_cmd(cmd[1:])
                elif cmd[0] == "monitor":
                    self.start_live_monitor()
                else:
                    print(f"Unknown command: {cmd[0]}")
                    
            except KeyboardInterrupt:
                print()
                continue
            except EOFError:
                break
            except Exception as e:
                print(f"Error: {e}")
        
        print("Goodbye!")
    
    def _print_help(self):
        """Print help message."""
        print("""
Commands:
  status              Show system status
  monitor             Live status monitoring
  
  lidar status        Show LiDAR details
  lidar start         Start LiDAR scanning
  lidar stop          Stop LiDAR scanning
  lidar config        Show LiDAR configuration
  
  imu status          Show IMU details
  imu start           Start IMU streaming
  imu stop            Stop IMU streaming
  imu selftest        Run IMU self-test
  imu config          Show IMU configuration
  
  session list        List capture sessions
  session start NAME  Start new session
  session stop        Stop current session
  
  config show         Show all configuration
  
  help                Show this help
  quit                Exit
""")
    
    def _handle_lidar_cmd(self, args):
        """Handle LiDAR subcommands."""
        if not args:
            self.print_lidar_details()
            return
        
        subcmd = args[0]
        if subcmd == "status":
            self.print_lidar_details()
        elif subcmd == "start":
            print("Starting LiDAR...")
            if self.client.start_lidar():
                print("✓ LiDAR started")
            else:
                print("✗ Failed to start LiDAR")
        elif subcmd == "stop":
            print("Stopping LiDAR...")
            if self.client.stop_lidar():
                print("✓ LiDAR stopped")
            else:
                print("✗ Failed to stop LiDAR")
        elif subcmd == "config":
            config = self.client.get_config()
            print(json.dumps(config.get("lidar", {}), indent=2))
        else:
            print(f"Unknown lidar command: {subcmd}")
    
    def _handle_imu_cmd(self, args):
        """Handle IMU subcommands."""
        if not args:
            self.print_imu_details()
            return
        
        subcmd = args[0]
        if subcmd == "status":
            self.print_imu_details()
        elif subcmd == "start":
            print("Starting IMU...")
            if self.client.start_imu():
                print("✓ IMU started")
            else:
                print("✗ Failed to start IMU")
        elif subcmd == "stop":
            print("Stopping IMU...")
            if self.client.stop_imu():
                print("✓ IMU stopped")
            else:
                print("✗ Failed to stop IMU")
        elif subcmd == "selftest":
            self.run_imu_selftest()
        elif subcmd == "config":
            config = self.client.get_config()
            print(json.dumps(config.get("imu", {}), indent=2))
        else:
            print(f"Unknown imu command: {subcmd}")
    
    def _handle_session_cmd(self, args):
        """Handle session subcommands."""
        if not args:
            sessions = self.client.list_sessions()
            if sessions:
                print("\nCapture Sessions:")
                for s in sessions:
                    print(f"  - {s['name']}")
            else:
                print("No sessions found")
            return
        
        subcmd = args[0]
        if subcmd == "list":
            sessions = self.client.list_sessions()
            if sessions:
                print("\nCapture Sessions:")
                for s in sessions:
                    print(f"  - {s['name']}")
                    if s.get("created"):
                        print(f"    Created: {s['created']}")
                    if s.get("stations"):
                        print(f"    Stations: {s['stations']}")
            else:
                print("No sessions found")
        elif subcmd == "start":
            name = args[1] if len(args) > 1 else None
            print(f"Starting session{' ' + name if name else ''}...")
            session_name = self.client.start_session(name)
            if session_name:
                print(f"✓ Session started: {session_name}")
            else:
                print("✗ Failed to start session")
        elif subcmd == "stop":
            print("Stopping session...")
            result = self.client.stop_session()
            if "error" not in result:
                print(f"✓ Session stopped: {result.get('name')}")
                print(f"  Duration: {result.get('duration', 0):.1f}s")
                print(f"  Points: {result.get('lidar_points', 0):,}")
                print(f"  Samples: {result.get('imu_samples', 0):,}")
            else:
                print(f"✗ {result.get('error')}")
        else:
            print(f"Unknown session command: {subcmd}")
    
    def _handle_config_cmd(self, args):
        """Handle config subcommands."""
        config = self.client.get_config()
        print(json.dumps(config, indent=2))


def main():
    """CLI entry point."""
    parser = argparse.ArgumentParser(
        description="Planar capture controller",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s --host 192.168.1.100 status
  %(prog)s --host 192.168.1.100 lidar start
  %(prog)s --host 192.168.1.100 imu selftest
  %(prog)s --host 192.168.1.100 session start my_scan
  %(prog)s --host 192.168.1.100 interactive
  %(prog)s discover
"""
    )
    
    parser.add_argument("--host", "-H", default="localhost",
                        help="Capture daemon host (default: localhost)")
    parser.add_argument("--port", "-p", type=int, default=8080,
                        help="Capture daemon port (default: 8080)")
    parser.add_argument("--token", "-t", help="Authentication token")
    parser.add_argument("--timeout", type=float, default=10.0,
                        help="Request timeout in seconds")
    
    subparsers = parser.add_subparsers(dest="command", help="Command")
    
    # Discover command
    discover_parser = subparsers.add_parser("discover", help="Discover devices on network")
    discover_parser.add_argument("--timeout", type=float, default=2.0)
    
    # Status command
    subparsers.add_parser("status", help="Show system status")
    
    # Monitor command
    monitor_parser = subparsers.add_parser("monitor", help="Live status monitoring")
    monitor_parser.add_argument("--interval", type=float, default=1.0)
    
    # LiDAR commands
    lidar_parser = subparsers.add_parser("lidar", help="LiDAR control")
    lidar_sub = lidar_parser.add_subparsers(dest="lidar_cmd")
    lidar_sub.add_parser("status", help="Show LiDAR status")
    lidar_sub.add_parser("start", help="Start LiDAR")
    lidar_sub.add_parser("stop", help="Stop LiDAR")
    lidar_config = lidar_sub.add_parser("configure", help="Configure LiDAR")
    lidar_config.add_argument("--mode", choices=["Standard", "DenseBoost"])
    lidar_config.add_argument("--frequency", type=float)
    
    # IMU commands
    imu_parser = subparsers.add_parser("imu", help="IMU control")
    imu_sub = imu_parser.add_subparsers(dest="imu_cmd")
    imu_sub.add_parser("status", help="Show IMU status")
    imu_sub.add_parser("start", help="Start IMU")
    imu_sub.add_parser("stop", help="Stop IMU")
    imu_sub.add_parser("selftest", help="Run IMU self-test")
    imu_config = imu_sub.add_parser("configure", help="Configure IMU")
    imu_config.add_argument("--gyro-range", type=int, choices=[125, 250, 500, 1000, 2000])
    imu_config.add_argument("--accel-range", type=int, choices=[2, 4, 8, 16])
    imu_config.add_argument("--sample-rate", type=float)
    
    # Session commands
    session_parser = subparsers.add_parser("session", help="Session management")
    session_sub = session_parser.add_subparsers(dest="session_cmd")
    session_sub.add_parser("list", help="List sessions")
    session_start = session_sub.add_parser("start", help="Start session")
    session_start.add_argument("name", nargs="?", help="Session name")
    session_sub.add_parser("stop", help="Stop session")
    
    # Config command
    subparsers.add_parser("config", help="Show configuration")
    
    # Interactive command
    subparsers.add_parser("interactive", help="Interactive mode")
    
    args = parser.parse_args()
    
    # Handle discover command
    if args.command == "discover":
        print("Discovering Planar devices on network...")
        devices = discover_devices(timeout=args.timeout)
        if devices:
            print(f"\nFound {len(devices)} device(s):")
            for d in devices:
                print(f"  - {d}")
        else:
            print("No devices found")
        return 0
    
    # Create client
    client = PlanarClient(
        host=args.host,
        port=args.port,
        auth_token=args.token,
        timeout=args.timeout
    )
    
    controller = CaptureController(client)
    
    # Check connection
    if not controller.check_connection():
        return 1
    
    # Handle commands
    if args.command == "status":
        controller.print_status()
    
    elif args.command == "monitor":
        controller.start_live_monitor(interval=args.interval)
    
    elif args.command == "lidar":
        if args.lidar_cmd == "status":
            controller.print_lidar_details()
        elif args.lidar_cmd == "start":
            if client.start_lidar():
                print("✓ LiDAR started")
            else:
                print("✗ Failed to start LiDAR")
        elif args.lidar_cmd == "stop":
            if client.stop_lidar():
                print("✓ LiDAR stopped")
            else:
                print("✗ Failed to stop LiDAR")
        elif args.lidar_cmd == "configure":
            params = {}
            if hasattr(args, 'mode') and args.mode:
                params["scan_mode"] = args.mode
            if hasattr(args, 'frequency') and args.frequency:
                params["scan_frequency_hz"] = args.frequency
            if params:
                if client.configure_lidar(**params):
                    print("✓ LiDAR configured")
                else:
                    print("✗ Configuration failed")
        else:
            controller.print_lidar_details()
    
    elif args.command == "imu":
        if args.imu_cmd == "status":
            controller.print_imu_details()
        elif args.imu_cmd == "start":
            if client.start_imu():
                print("✓ IMU started")
            else:
                print("✗ Failed to start IMU")
        elif args.imu_cmd == "stop":
            if client.stop_imu():
                print("✓ IMU stopped")
            else:
                print("✗ Failed to stop IMU")
        elif args.imu_cmd == "selftest":
            controller.run_imu_selftest()
        elif args.imu_cmd == "configure":
            params = {}
            if hasattr(args, 'gyro_range') and args.gyro_range:
                params["gyro_range_dps"] = args.gyro_range
            if hasattr(args, 'accel_range') and args.accel_range:
                params["accel_range_g"] = args.accel_range
            if hasattr(args, 'sample_rate') and args.sample_rate:
                params["sample_rate_hz"] = args.sample_rate
            if params:
                if client.configure_imu(**params):
                    print("✓ IMU configured")
                else:
                    print("✗ Configuration failed")
        else:
            controller.print_imu_details()
    
    elif args.command == "session":
        if args.session_cmd == "list":
            sessions = client.list_sessions()
            if sessions:
                print("\nCapture Sessions:")
                for s in sessions:
                    print(f"  - {s['name']}")
            else:
                print("No sessions found")
        elif args.session_cmd == "start":
            name = getattr(args, 'name', None)
            session_name = client.start_session(name)
            if session_name:
                print(f"✓ Session started: {session_name}")
            else:
                print("✗ Failed to start session")
        elif args.session_cmd == "stop":
            result = client.stop_session()
            if "error" not in result:
                print(f"✓ Session stopped: {result.get('name')}")
            else:
                print(f"✗ {result.get('error')}")
        else:
            sessions = client.list_sessions()
            if sessions:
                for s in sessions:
                    print(f"  - {s['name']}")
    
    elif args.command == "config":
        config = client.get_config()
        print(json.dumps(config, indent=2))
    
    elif args.command == "interactive":
        controller.interactive_mode()
    
    else:
        controller.print_status()
    
    return 0


if __name__ == "__main__":
    sys.exit(main())
