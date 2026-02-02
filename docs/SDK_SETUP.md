# Hardware Setup Guide````markdown

# rplidar_sdk build notes

This guide covers setting up both the RPLidar SDK and the BMI160 IMU for use with the Planar capture system.

This document explains how to prepare a host (or Raspberry Pi) to build the `rplidar_sdk` C++ SDK that this project stages under `third_party/` and how to run the SDK demos to validate a connected SLAMTEC LIDAR.

## RPLidar SDK Setup

1) Install build tools and SDK dependencies (Debian/Ubuntu / Raspberry Pi OS):

The Planar project includes the SLAMTEC RPLidar SDK under `third_party/rplidar_sdk/`. While the Planar Python drivers can communicate directly with the device, the SDK provides useful diagnostic tools.

```bash

### Prerequisitessudo apt update

sudo apt install -y build-essential git cmake pkg-config libusb-1.0-0-dev python3-pip python3-venv

```bash```

sudo apt update

sudo apt install -y build-essential git cmake pkg-config libusb-1.0-0-devOn Raspberry Pi OS you may also need additional cross-arch packages; consult your distro docs.

```

2) Build `rplidar_sdk` (after installing cmake etc):

### Building the SDK

```bash

```bashcd /path/to/Planar/third_party/rplidar_sdk

cd /path/to/Planar/third_party/rplidar_sdkmkdir -p build && cd build

cmake ..

# Create build directorymake -j$(nproc)

mkdir -p build && cd build# result: libraries and examples in build/

```

# Build with CMake

cmake ..If you want, I can attempt to install `cmake` and build `rplidar_sdk` in this environment; however, installing system packages requires `sudo` and may be disallowed. If you give permission, I can run the apt commands and build the SDK now.

make -j$(nproc)

## SDK LIDAR smoke-test (how to run the bundled demos)

# Binaries are in build/ or output/Linux/Release/

```Use the SDK demos to verify the LIDAR and USB-to-UART bridge are working. The SDK provides two small Linux-friendly utilities under `third_party/rplidar_sdk/output/Linux/Release/`: `ultra_simple` and `simple_grabber`.



### SDK UtilitiesNotes:

- Some SLAMTEC models use non-standard baud rates (for example S2/S3 devices use 1000000). Check your model's recommended baud in the SDK help or datasheet. The SDK's help lists common baud rates (A1=115200, A2M7=256000, S2/S3=1000000, etc.).

The SDK includes diagnostic utilities:- The repo includes a udev rule at `provisioning/udev/99-rplidar.rules` you can install to create a stable device node (for example `/dev/rplidar`). Installing that rule will make the examples more convenient to run.



#### ultra_simpleExample: run `ultra_simple` (prints usage when run without args):



Basic scan test with angle/distance output:```bash

# run with serial channel; use the device path your system exposes or an /dev/rplidar symlink

```bashsudo ./third_party/rplidar_sdk/output/Linux/Release/ultra_simple --channel --serial /dev/rplidar 1000000

sudo ./output/Linux/Release/ultra_simple --channel --serial /dev/rplidar 1000000```

```

Sample partial output (live scan lines):

Sample output:

``````

theta: 309.15 Dist: 00000.00 Q: 0   theta: 309.15 Dist: 00000.00 Q: 0

theta: 311.89 Dist: 01418.00 Q: 47   theta: 309.30 Dist: 00000.00 Q: 0

theta: 312.03 Dist: 01418.00 Q: 47   theta: 311.89 Dist: 01418.00 Q: 47

```   theta: 312.03 Dist: 01418.00 Q: 47

```

#### simple_grabber

Example: run `simple_grabber` — this prints LIDAR info and a simple ASCII visualization stream:

ASCII visualization of the scan:

```bash

```bashsudo ./third_party/rplidar_sdk/output/Linux/Release/simple_grabber --channel --serial /dev/rplidar 1000000

sudo ./output/Linux/Release/simple_grabber --channel --serial /dev/rplidar 1000000```

```

Sample partial output:

Sample output:

``````

SLAMTEC LIDAR S/N: 62F1E18BC7E598C2C4E29BF49124497CSLAMTEC LIDAR S/N: 62F1E18BC7E598C2C4E29BF49124497C

Version:  SL_LIDAR_SDK_VERSIONVersion:  SL_LIDAR_SDK_VERSION

Firmware Ver: 1.02Firmware Ver: 1.02

Hardware Rev: 18Hardware Rev: 18

Lidar health status : OK. (errorcode: 0)Lidar health status : OK. (errorcode: 0)

waiting for data...waiting for data...

      *                                                                       *                                                                 

   * ***** *                                      *************           * ***** *                                      *************        

******************************************************************************************************************************************************

``````



### Common Baud RatesTroubleshooting

- If you see "Error, cannot bind to the specified serial port /dev/ttyUSB0" or similar, check:

| Model | Baud Rate |  - The device node exists (eg. `ls -l /dev/ttyUSB0` or `/dev/rplidar`).

|-------|-----------|  - Your user is in the `dialout` group or run the binary with `sudo`.

| A1 | 115200 |  - Kernel `dmesg` for `cp210x` errors (eg. `failed set request 0x12 status: -110`) which indicate USB control-transfer timeouts — try a different USB cable, port, or a powered hub and replug the device.

| A2M7 | 256000 |- If your device requires a high baud (e.g. `1000000`) pass that as the last argument to the examples.

| S2/S3 | 1000000 |- To create a stable device node, install the project's udev rule:



### udev Rules```bash

sudo cp provisioning/udev/99-rplidar.rules /etc/udev/rules.d/

Install the provided udev rule for a stable device symlink:sudo udevadm control --reload

sudo udevadm trigger

```bash# then unplug/replug the LIDAR and use /dev/rplidar (if the rule provides that name)

sudo cp provisioning/udev/99-rplidar.rules /etc/udev/rules.d/```

sudo udevadm control --reload-rules

sudo udevadm triggerIf you'd like, I can add a small wrapper script to the repo that tries common baud rates (115200, 256000, 1000000) and starts the first-working example automatically — say the word and I'll add it.

# Unplug/replug the LiDAR, then use /dev/rplidar

```````



### Troubleshooting## I2C IMU: Gravity BMI160 (how to detect & smoke-test)



**"Error, cannot bind to the specified serial port"**We use the Gravity BMI160 6-axis IMU on the I2C bus. On Raspberry Pi / similar SBCs this commonly appears as `/dev/i2c-1`.

- Check device exists: `ls -l /dev/ttyUSB0` or `/dev/rplidar`

- Add user to dialout: `sudo usermod -aG dialout $USER`Quick checks (example outputs from this host):

- Run with sudo if needed

```bash

**USB timeout errors (`status: -110`)**$ ls -l /dev/i2c-1

- Try a different USB cablecrw-rw---- 1 root i2c 89, 1 Jan 27 15:32 /dev/i2c-1

- Use a different USB port (prefer USB 2.0)

- Use a powered USB hub$ i2cdetect -y 1

- Replug the device    0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f

00:                         -- -- -- -- -- -- -- -- 

---10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 

20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 

## BMI160 IMU Setup30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 

40: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 

The Planar capture system uses the Bosch BMI160 6-axis IMU via I2C.50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 

60: -- -- -- -- -- -- -- -- -- 69 -- -- -- -- -- -- 

### Hardware Specifications70: -- -- -- -- -- -- -- --

```

| Parameter | Value |

|-----------|-------|In the example above the IMU (Gravity BMI160) is at address `0x69` on I2C bus 1. Note that some IMU breakout boards let you select `0x68`/`0x69` via a pull pin (check the module docs).

| Interface | I2C |

| Default Address | 0x69 (Gravity board) |Install tools

| Alternate Address | 0x68 (selectable) |

| Chip ID | 0xD1 |```bash

| Gyroscope Range | ±125 to ±2000 °/s |sudo apt update

| Accelerometer Range | ±2g to ±16g |sudo apt install -y i2c-tools python3-smbus python3-pip

| Data Rate | Up to 400 Hz |# (optional) pip3 install smbus2

```

### Prerequisites

Permissions and access

```bash- The device node `/dev/i2c-1` is owned by `root:i2c`. To access it as a normal user add the user to the `i2c` group:

sudo apt update

sudo apt install -y i2c-tools python3-smbus```bash

```sudo usermod -aG i2c $USER

# then log out/in or run: newgrp i2c

For Python development:```

```bash

pip install smbus2Safe probe & simple read

```- Use `i2cdetect` to find the address (you already ran this). To inspect registers safely, use `i2cdump` or a short Python test with `smbus2`.



### Enable I2C (Raspberry Pi)Example: dump readable registers (read-only probe):



```bash```bash

sudo raspi-configsudo i2cdump -y 1 0x69

# Navigate: Interface Options -> I2C -> Enable```

# Reboot if prompted

```Example: quick Python read using smbus2 (reads one byte from register 0x00 — adjust according to the sensor datasheet if you want a specific WHO_AM_I/register):



### Verify I2C Device```python

from smbus2 import SMBus

```bashbus = SMBus(1)

# List I2C busesaddr = 0x69

ls -l /dev/i2c-*reg = 0x00

# Expected: /dev/i2c-1try:

   v = bus.read_byte_data(addr, reg)

# Scan for devices   print(hex(v))

i2cdetect -y 1finally:

```   bus.close()

```

Expected output:

```Notes & troubleshooting

     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f- If `i2cdetect` shows no device, ensure I2C is enabled (Raspberry Pi: `sudo raspi-config` -> Interface Options -> I2C) and replug the sensor.

00:                         -- -- -- -- -- -- -- -- - If reads fail or return garbage, verify wiring (SDA/SCL/GND/3.3V), check the module VCC (use 3.3V, not 5V unless board is 5V tolerant), and validate the SA0/AD0 pin that selects 0x68 vs 0x69.

10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- - If you need a persistent device mapping or helper, consider adding a udev rule or documenting the I2C address in your project's metadata so agents can find the bus/address automatically.

20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 

30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
40: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
60: -- -- -- -- -- -- -- -- -- 69 -- -- -- -- -- -- 
70: -- -- -- -- -- -- -- --
```

The `69` shows the BMI160 is at address 0x69.

### Set Permissions

Add your user to the `i2c` group:

```bash
sudo usermod -aG i2c $USER
# Log out and back in, or run:
newgrp i2c
```

### Verify with Planar

```bash
cd /path/to/Planar
source .venv/bin/activate

# Run hardware tests
python -m pytest tests/test_imu_driver.py -v -m hardware
```

Expected output:
```
tests/test_imu_driver.py::TestBMI160Hardware::test_hardware_connect PASSED
tests/test_imu_driver.py::TestBMI160Hardware::test_hardware_chip_id PASSED
tests/test_imu_driver.py::TestBMI160Hardware::test_hardware_self_test PASSED
...
```

### Python API

```python
from capture.imu_driver import BMI160Driver

# Create driver
driver = BMI160Driver(bus=1, address=0x69)

# Connect
if driver.connect():
    print(f"Chip ID: 0x{driver.info.chip_id:02X}")
    
    # Run self-test
    result = driver.self_test()
    print(f"Self-test passed: {result['passed']}")
    
    # Start streaming
    def on_sample(sample):
        print(f"Gyro Z: {sample.gyro_z:.4f} rad/s")
    
    driver.start_streaming(callback=on_sample)
    
    # ... do work ...
    
    driver.stop_streaming()
    driver.disconnect()
```

### Configuration

The IMU configuration is defined in `capture/config.py`:

```python
@dataclass
class ImuConfig:
    i2c_bus: int = 1
    i2c_address: int = 0x69
    sample_rate_hz: int = 100
    gyro_range: int = 500   # ±500 °/s
    accel_range: int = 4    # ±4g
    enabled: bool = True
```

### Register Dump (Debugging)

```bash
# Dump all readable registers
sudo i2cdump -y 1 0x69
```

Quick Python read:
```python
from smbus2 import SMBus

bus = SMBus(1)
chip_id = bus.read_byte_data(0x69, 0x00)
print(f"Chip ID: 0x{chip_id:02X}")  # Should print 0xD1
bus.close()
```

### Troubleshooting

**Device not detected in i2cdetect**
- Ensure I2C is enabled: `sudo raspi-config` -> Interface Options -> I2C
- Check wiring: SDA (GPIO 2), SCL (GPIO 3), GND, 3.3V
- Verify module VCC is 3.3V (not 5V unless board has level shifter)
- Some boards have AD0/SA0 pin for address selection (0x68 vs 0x69)

**Permission denied**
```bash
sudo usermod -aG i2c $USER
# Then log out and back in
```

**Reads return garbage**
- Check wiring connections
- Verify I2C bus speed (default 100kHz should work)
- Ensure stable power supply

**Self-test fails**
- Allow device to stabilize after power-on
- Ensure device is stationary during self-test
- Check for mechanical damage

---

## Full System Test

After setting up both devices, run the complete hardware test suite:

```bash
cd /path/to/Planar
source .venv/bin/activate

# Run all hardware tests
python -m pytest tests/ -v -m hardware
```

Expected results:
```
tests/test_imu_driver.py::TestBMI160Hardware::test_hardware_connect PASSED
tests/test_imu_driver.py::TestBMI160Hardware::test_hardware_chip_id PASSED
tests/test_imu_driver.py::TestBMI160Hardware::test_hardware_self_test PASSED
...
tests/test_lidar_driver.py::TestRPLidarHardware::test_hardware_connect_and_get_info PASSED
tests/test_lidar_driver.py::TestRPLidarHardware::test_hardware_scan_frames PASSED
tests/test_lidar_driver.py::TestRPLidarHardware::test_hardware_scan_data_validity PASSED
...
```

## Starting the Capture Daemon

Once hardware is verified:

```bash
planar-capture --port 8080 \
    --lidar-device /dev/rplidar \
    --imu-bus 1 \
    --imu-address 0x69 \
    --debug
```

Or as a systemd service:
```bash
sudo cp provisioning/systemd/planar-capture.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable planar-capture
sudo systemctl start planar-capture
```

## See Also

- [Architecture Overview](ARCHITECTURE.md) - System architecture
- [Connecting the RPLidar](CONNECT_LIDAR.md) - Detailed LiDAR setup
- [ROS2 Setup](ROS2_SETUP.md) - Optional ROS2 integration
