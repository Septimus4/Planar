#!/usr/bin/env python3
"""Hardware diagnostic script for Planar sensors.

This script checks the LiDAR and IMU hardware and provides troubleshooting
guidance. Run this locally on the Raspberry Pi to verify sensor connectivity.

Usage:
    python3 diagnose_hardware.py
    python3 diagnose_hardware.py --lidar-only
    python3 diagnose_hardware.py --imu-only
"""

import argparse
import os
import sys
import time


def check_lidar(device: str = "/dev/rplidar", baudrate: int = 1_000_000) -> dict:
    """Check RPLidar connectivity and basic communication."""
    results = {
        "device_exists": False,
        "device_readable": False,
        "serial_open": False,
        "device_responds": False,
        "info": {},
        "errors": [],
        "warnings": [],
    }
    
    print("\n" + "=" * 50)
    print("LIDAR DIAGNOSTICS")
    print("=" * 50)
    
    # Check device exists
    print(f"\nChecking device: {device}")
    if os.path.exists(device):
        results["device_exists"] = True
        print(f"  ✓ Device exists")
        
        # Check symlink target if it's a symlink
        if os.path.islink(device):
            target = os.path.realpath(device)
            print(f"  ✓ Symlink points to: {target}")
    else:
        print(f"  ✗ Device not found")
        results["errors"].append(f"Device {device} not found")
        
        # Check for common alternatives
        alternatives = ["/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyACM0"]
        found = [d for d in alternatives if os.path.exists(d)]
        if found:
            print(f"  ! Found alternatives: {', '.join(found)}")
            results["warnings"].append(f"Try using: {found[0]}")
        else:
            print("  ! No serial devices found. Is the LiDAR connected?")
            results["errors"].append("Check USB connection and udev rules")
        return results
    
    # Check permissions
    if os.access(device, os.R_OK | os.W_OK):
        results["device_readable"] = True
        print(f"  ✓ Device is readable/writable")
    else:
        print(f"  ✗ Permission denied")
        results["errors"].append("Add user to dialout group: sudo usermod -aG dialout $USER")
        return results
    
    # Try to open serial port
    try:
        import serial
        print(f"\nOpening serial port at {baudrate} baud...")
        
        ser = serial.Serial(
            port=device,
            baudrate=baudrate,
            timeout=2.0,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS
        )
        results["serial_open"] = True
        print(f"  ✓ Serial port opened")
        
        # Send stop command first
        ser.write(bytes([0xA5, 0x25]))
        time.sleep(0.1)
        ser.flushInput()
        
        # Send get_info command
        print("\nRequesting device info...")
        ser.write(bytes([0xA5, 0x50]))
        time.sleep(0.5)
        
        # Read response descriptor (7 bytes)
        desc = ser.read(7)
        if len(desc) >= 7 and desc[0] == 0xA5 and desc[1] == 0x5A:
            print(f"  ✓ Got response descriptor")
            
            # Read device info (20 bytes)
            info_data = ser.read(20)
            if len(info_data) >= 20:
                results["device_responds"] = True
                results["info"] = {
                    "model": f"Model {info_data[0]}",
                    "firmware": f"{info_data[2]}.{info_data[1]:02d}",
                    "hardware": str(info_data[3]),
                    "serial": info_data[4:20].hex().upper(),
                }
                print(f"  ✓ Device info received")
                print(f"    Model:    {results['info']['model']}")
                print(f"    Firmware: {results['info']['firmware']}")
                print(f"    Hardware: Rev {results['info']['hardware']}")
                print(f"    S/N:      {results['info']['serial']}")
            else:
                print(f"  ✗ Incomplete info response")
                results["warnings"].append("Device may need reset")
        else:
            print(f"  ✗ No valid response (got {len(desc)} bytes)")
            results["errors"].append("Device not responding. Try power cycling.")
        
        # Check health
        print("\nChecking device health...")
        ser.write(bytes([0xA5, 0x52]))
        time.sleep(0.3)
        
        desc = ser.read(7)
        if len(desc) >= 7:
            health_data = ser.read(3)
            if len(health_data) >= 3:
                status = health_data[0]
                status_names = {0: "Good", 1: "Warning", 2: "Error"}
                print(f"  Health: {status_names.get(status, f'Unknown({status})')}")
                if status == 2:
                    error_code = health_data[1] | (health_data[2] << 8)
                    results["errors"].append(f"Health error code: {error_code}")
        
        ser.close()
        
    except ImportError:
        print("  ✗ pyserial not installed")
        results["errors"].append("Install pyserial: pip install pyserial")
    except serial.SerialException as e:
        print(f"  ✗ Serial error: {e}")
        results["errors"].append(str(e))
    except Exception as e:
        print(f"  ✗ Error: {e}")
        results["errors"].append(str(e))
    
    return results


def check_imu(bus: int = 1, address: int = 0x69) -> dict:
    """Check BMI160 IMU connectivity."""
    results = {
        "bus_exists": False,
        "device_found": False,
        "chip_id_valid": False,
        "can_read_data": False,
        "info": {},
        "errors": [],
        "warnings": [],
    }
    
    print("\n" + "=" * 50)
    print("IMU DIAGNOSTICS")
    print("=" * 50)
    
    # Check I2C bus exists
    i2c_dev = f"/dev/i2c-{bus}"
    print(f"\nChecking I2C bus: {i2c_dev}")
    
    if os.path.exists(i2c_dev):
        results["bus_exists"] = True
        print(f"  ✓ I2C bus exists")
    else:
        print(f"  ✗ I2C bus not found")
        results["errors"].append("Enable I2C: sudo raspi-config -> Interface Options -> I2C")
        
        # Check available buses
        buses = [f for f in os.listdir("/dev") if f.startswith("i2c-")]
        if buses:
            print(f"  ! Available buses: {', '.join(buses)}")
        return results
    
    # Check permissions
    if not os.access(i2c_dev, os.R_OK | os.W_OK):
        print(f"  ✗ Permission denied")
        results["errors"].append("Add user to i2c group: sudo usermod -aG i2c $USER")
        return results
    
    # Try to communicate with device
    try:
        import smbus2
        print(f"\nScanning I2C bus for devices...")
        
        i2c = smbus2.SMBus(bus)
        
        # Scan for devices
        found_devices = []
        for addr in range(0x03, 0x78):
            try:
                i2c.read_byte(addr)
                found_devices.append(addr)
            except OSError:
                pass
        
        if found_devices:
            print(f"  Found devices at: {', '.join(f'0x{a:02X}' for a in found_devices)}")
        else:
            print(f"  ✗ No I2C devices found")
            results["errors"].append("Check IMU wiring and power")
            i2c.close()
            return results
        
        # Check for BMI160 at expected address
        print(f"\nChecking for BMI160 at 0x{address:02X}...")
        
        if address in found_devices:
            results["device_found"] = True
            print(f"  ✓ Device found at 0x{address:02X}")
            
            # Read chip ID
            chip_id = i2c.read_byte_data(address, 0x00)
            results["info"]["chip_id"] = f"0x{chip_id:02X}"
            
            if chip_id == 0xD1:
                results["chip_id_valid"] = True
                print(f"  ✓ Chip ID: 0x{chip_id:02X} (BMI160 confirmed)")
            else:
                print(f"  ✗ Unexpected chip ID: 0x{chip_id:02X} (expected 0xD1)")
                results["warnings"].append(f"Unknown device at 0x{address:02X}")
            
            # Try to read error register
            err_reg = i2c.read_byte_data(address, 0x02)
            if err_reg != 0:
                print(f"  ! Error register: 0x{err_reg:02X}")
                results["warnings"].append(f"Device has error flags: 0x{err_reg:02X}")
            
            # Read PMU status
            pmu_status = i2c.read_byte_data(address, 0x03)
            acc_pmu = (pmu_status >> 4) & 0x03
            gyr_pmu = (pmu_status >> 2) & 0x03
            pmu_modes = {0: "suspend", 1: "normal", 2: "low-power", 3: "fast-startup"}
            print(f"  Power mode - Accel: {pmu_modes.get(acc_pmu, '?')}, Gyro: {pmu_modes.get(gyr_pmu, '?')}")
            
            # Try to read sensor data
            print("\nReading sensor data...")
            
            # Power on sensors if suspended
            if acc_pmu == 0:
                i2c.write_byte_data(address, 0x7E, 0x11)  # ACC normal mode
                time.sleep(0.01)
            if gyr_pmu == 0:
                i2c.write_byte_data(address, 0x7E, 0x15)  # GYR normal mode
                time.sleep(0.1)
            
            # Read gyro and accel data
            import struct
            data = bytes(i2c.read_i2c_block_data(address, 0x0C, 12))
            
            gx = struct.unpack('<h', data[0:2])[0]
            gy = struct.unpack('<h', data[2:4])[0]
            gz = struct.unpack('<h', data[4:6])[0]
            ax = struct.unpack('<h', data[6:8])[0]
            ay = struct.unpack('<h', data[8:10])[0]
            az = struct.unpack('<h', data[10:12])[0]
            
            # Default scales (±250°/s, ±2g)
            gyro_scale = 250.0 / 32768.0 * 3.14159 / 180.0  # rad/s
            accel_scale = 2.0 / 32768.0 * 9.80665  # m/s²
            
            results["info"]["gyro"] = {
                "x": gx * gyro_scale,
                "y": gy * gyro_scale,
                "z": gz * gyro_scale,
            }
            results["info"]["accel"] = {
                "x": ax * accel_scale,
                "y": ay * accel_scale,
                "z": az * accel_scale,
            }
            
            print(f"  Gyro (rad/s):  X={gx*gyro_scale:+.4f}, Y={gy*gyro_scale:+.4f}, Z={gz*gyro_scale:+.4f}")
            print(f"  Accel (m/s²): X={ax*accel_scale:+.3f}, Y={ay*accel_scale:+.3f}, Z={az*accel_scale:+.3f}")
            
            # Validate readings
            gyro_mag = (results["info"]["gyro"]["x"]**2 + 
                       results["info"]["gyro"]["y"]**2 + 
                       results["info"]["gyro"]["z"]**2) ** 0.5
            accel_mag = (results["info"]["accel"]["x"]**2 + 
                        results["info"]["accel"]["y"]**2 + 
                        results["info"]["accel"]["z"]**2) ** 0.5
            
            if gyro_mag < 0.5:  # Should be near zero when stationary
                print(f"  ✓ Gyro readings look reasonable (magnitude: {gyro_mag:.4f} rad/s)")
            else:
                print(f"  ! Gyro magnitude high: {gyro_mag:.4f} rad/s")
                results["warnings"].append("High gyro reading - device may be moving or faulty")
            
            if 8.0 < accel_mag < 12.0:  # Should be near 9.8 m/s²
                print(f"  ✓ Accel readings look reasonable (magnitude: {accel_mag:.2f} m/s²)")
                results["can_read_data"] = True
            else:
                print(f"  ! Accel magnitude unexpected: {accel_mag:.2f} m/s² (expected ~9.8)")
                results["warnings"].append("Accel reading unusual - check orientation or calibration")
            
            # Read temperature
            temp_data = bytes(i2c.read_i2c_block_data(address, 0x20, 2))
            temp_raw = struct.unpack('<h', temp_data)[0]
            temp_c = (temp_raw / 512.0) + 23.0
            results["info"]["temperature"] = temp_c
            print(f"  Temperature: {temp_c:.1f}°C")
            
        else:
            print(f"  ✗ No device at 0x{address:02X}")
            
            # Check alternate addresses
            if 0x68 in found_devices:
                print(f"  ! Found device at 0x68 (alternate BMI160 address)")
                results["warnings"].append("Try address 0x68 instead of 0x69")
            else:
                results["errors"].append("BMI160 not found. Check wiring.")
        
        i2c.close()
        
    except ImportError:
        print("  ✗ smbus2 not installed")
        results["errors"].append("Install smbus2: pip install smbus2")
    except Exception as e:
        print(f"  ✗ Error: {e}")
        results["errors"].append(str(e))
    
    return results


def print_summary(lidar_results: dict, imu_results: dict):
    """Print summary and recommendations."""
    print("\n" + "=" * 50)
    print("SUMMARY")
    print("=" * 50)
    
    all_ok = True
    
    # LiDAR summary
    if lidar_results:
        lidar_ok = lidar_results.get("device_responds", False)
        icon = "✓" if lidar_ok else "✗"
        print(f"\nLiDAR: {icon}")
        if lidar_ok:
            info = lidar_results.get("info", {})
            print(f"  Model: {info.get('model', 'N/A')}")
            print(f"  S/N:   {info.get('serial', 'N/A')}")
        else:
            all_ok = False
            for err in lidar_results.get("errors", []):
                print(f"  Error: {err}")
    
    # IMU summary
    if imu_results:
        imu_ok = imu_results.get("chip_id_valid", False) and imu_results.get("can_read_data", False)
        icon = "✓" if imu_ok else "✗"
        print(f"\nIMU: {icon}")
        if imu_ok:
            info = imu_results.get("info", {})
            print(f"  Chip ID: {info.get('chip_id', 'N/A')}")
            print(f"  Temperature: {info.get('temperature', 'N/A'):.1f}°C")
        else:
            all_ok = False
            for err in imu_results.get("errors", []):
                print(f"  Error: {err}")
    
    # Overall status
    print("\n" + "-" * 50)
    if all_ok:
        print("All sensors OK! Ready to start capture daemon.")
        print("\nStart with: sudo systemctl start planar-capture")
    else:
        print("Some issues found. Please address the errors above.")
        
        # Collect all errors and warnings
        all_errors = []
        all_warnings = []
        if lidar_results:
            all_errors.extend(lidar_results.get("errors", []))
            all_warnings.extend(lidar_results.get("warnings", []))
        if imu_results:
            all_errors.extend(imu_results.get("errors", []))
            all_warnings.extend(imu_results.get("warnings", []))
        
        if all_warnings:
            print("\nWarnings:")
            for w in all_warnings:
                print(f"  ! {w}")
        
        if all_errors:
            print("\nRequired fixes:")
            for i, e in enumerate(all_errors, 1):
                print(f"  {i}. {e}")


def main():
    parser = argparse.ArgumentParser(description="Diagnose Planar hardware")
    parser.add_argument("--lidar-only", action="store_true", help="Check LiDAR only")
    parser.add_argument("--imu-only", action="store_true", help="Check IMU only")
    parser.add_argument("--lidar-device", default="/dev/rplidar", help="LiDAR device path")
    parser.add_argument("--lidar-baudrate", type=int, default=1000000, help="LiDAR baudrate")
    parser.add_argument("--imu-bus", type=int, default=1, help="I2C bus number")
    parser.add_argument("--imu-address", type=lambda x: int(x, 0), default=0x69, help="IMU I2C address")
    args = parser.parse_args()
    
    print("Planar Hardware Diagnostics")
    print("===========================")
    
    lidar_results = None
    imu_results = None
    
    if not args.imu_only:
        lidar_results = check_lidar(args.lidar_device, args.lidar_baudrate)
    
    if not args.lidar_only:
        imu_results = check_imu(args.imu_bus, args.imu_address)
    
    print_summary(lidar_results, imu_results)
    
    # Return exit code based on results
    all_ok = True
    if lidar_results and not lidar_results.get("device_responds"):
        all_ok = False
    if imu_results and not imu_results.get("can_read_data"):
        all_ok = False
    
    return 0 if all_ok else 1


if __name__ == "__main__":
    sys.exit(main())
