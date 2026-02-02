# ROS2 Integration (Optional)# rplidar_sdk build notes



This guide covers optional ROS2 integration for the Planar capture system. ROS2 is **not required** for basic operation - Planar has native Python drivers for both the RPLidar and BMI160. However, ROS2 can be useful for visualization, bag recording, and integration with other robotics tools.This document explains how to prepare a host (or Raspberry Pi) to build the `rplidar_sdk` C++ SDK that this project stages under `third_party/` and how to run the SDK demos to validate a connected SLAMTEC LIDAR.



## Overview1) Install build tools and SDK dependencies (Debian/Ubuntu / Raspberry Pi OS):



Planar's native approach vs ROS2:```bash

sudo apt update

| Feature | Native Planar | ROS2 |sudo apt install -y build-essential git cmake pkg-config libusb-1.0-0-dev python3-pip python3-venv

|---------|---------------|------|```

| LiDAR Driver | `capture/lidar_driver.py` | `sllidar_ros2` package |

| IMU Driver | `capture/imu_driver.py` | `imu_tools` package |On Raspberry Pi OS you may also need additional cross-arch packages; consult your distro docs.

| Data Format | CSV/JSON sessions | ROS bags |

| Remote Control | HTTP/WebSocket API | ROS services/topics |2) Build `rplidar_sdk` (after installing cmake etc):

| Processing | `processing/pipeline.py` | Various ROS packages |

| Dependencies | Pure Python | Full ROS2 installation |```bash

cd /path/to/Planar/third_party/rplidar_sdk

**Recommendation**: Use native Planar for production capture. Use ROS2 for development visualization and debugging.mkdir -p build && cd build

cmake ..

## Installing ROS2make -j$(nproc)

# result: libraries and examples in build/

### Humble (Ubuntu 22.04)```



```bashIf you want, I can attempt to install `cmake` and build `rplidar_sdk` in this environment; however, installing system packages requires `sudo` and may be disallowed. If you give permission, I can run the apt commands and build the SDK now.

# Set locale

sudo apt update && sudo apt install locales## SDK LIDAR smoke-test (how to run the bundled demos)

sudo locale-gen en_US en_US.UTF-8

sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8Use the SDK demos to verify the LIDAR and USB-to-UART bridge are working. The SDK provides two small Linux-friendly utilities under `third_party/rplidar_sdk/output/Linux/Release/`: `ultra_simple` and `simple_grabber`.

export LANG=en_US.UTF-8

Notes:

# Add ROS2 apt repository- Some SLAMTEC models use non-standard baud rates (for example S2/S3 devices use 1000000). Check your model's recommended baud in the SDK help or datasheet. The SDK's help lists common baud rates (A1=115200, A2M7=256000, S2/S3=1000000, etc.).

sudo apt install software-properties-common- The repo includes a udev rule at `provisioning/udev/99-rplidar.rules` you can install to create a stable device node (for example `/dev/rplidar`). Installing that rule will make the examples more convenient to run.

sudo add-apt-repository universe

sudo apt update && sudo apt install curl -yExample: run `ultra_simple` (prints usage when run without args):

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null```bash

# run with serial channel; use the device path your system exposes or an /dev/rplidar symlink

# Install ROS2 Humblesudo ./third_party/rplidar_sdk/output/Linux/Release/ultra_simple --channel --serial /dev/rplidar 1000000

sudo apt update```

sudo apt install ros-humble-desktop

Sample partial output (live scan lines):

# Source ROS2

echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc```

source ~/.bashrc   theta: 309.15 Dist: 00000.00 Q: 0

```   theta: 309.30 Dist: 00000.00 Q: 0

   theta: 311.89 Dist: 01418.00 Q: 47

### Raspberry Pi (Ubuntu 22.04)   theta: 312.03 Dist: 01418.00 Q: 47

```

```bash

# Same as above, but use ros-humble-ros-base for minimal installExample: run `simple_grabber` — this prints LIDAR info and a simple ASCII visualization stream:

sudo apt install ros-humble-ros-base

``````bash

sudo ./third_party/rplidar_sdk/output/Linux/Release/simple_grabber --channel --serial /dev/rplidar 1000000

## RPLidar ROS2 Package```



### InstallationSample partial output:



```bash```

# Create workspaceSLAMTEC LIDAR S/N: 62F1E18BC7E598C2C4E29BF49124497C

mkdir -p ~/ros2_ws/srcVersion:  SL_LIDAR_SDK_VERSION

cd ~/ros2_ws/srcFirmware Ver: 1.02

Hardware Rev: 18

# Clone sllidar_ros2 (official SLAMTEC package)Lidar health status : OK. (errorcode: 0)

git clone https://github.com/Slamtec/sllidar_ros2.gitwaiting for data...

      *                                                                 

# Build   * ***** *                                      *************        

cd ~/ros2_ws***************************************************************************

colcon build --symlink-install```



# Source workspaceTroubleshooting

source install/setup.bash- If you see "Error, cannot bind to the specified serial port /dev/ttyUSB0" or similar, check:

```   - The device node exists (eg. `ls -l /dev/ttyUSB0` or `/dev/rplidar`).

   - Your user is in the `dialout` group or run the binary with `sudo`.

### Running   - Kernel `dmesg` for `cp210x` errors (eg. `failed set request 0x12 status: -110`) which indicate USB control-transfer timeouts — try a different USB cable, port, or a powered hub and replug the device.

- If your device requires a high baud (e.g. `1000000`) pass that as the last argument to the examples.

```bash- To create a stable device node, install the project's udev rule:

# Launch RPLidar node (adjust device and baudrate)

ros2 launch sllidar_ros2 sllidar_s3_launch.py serial_port:=/dev/rplidar serial_baudrate:=1000000```bash

```sudo cp provisioning/udev/99-rplidar.rules /etc/udev/rules.d/

sudo udevadm control --reload

### Visualizationsudo udevadm trigger

# then unplug/replug the LIDAR and use /dev/rplidar (if the rule provides that name)

```bash```

# In another terminal

ros2 run rviz2 rviz2If you'd like, I can add a small wrapper script to the repo that tries common baud rates (115200, 256000, 1000000) and starts the first-working example automatically — say the word and I'll add it.


# Add LaserScan display, set topic to /scan
```

## BMI160 IMU ROS2

### Using imu_tools

```bash
cd ~/ros2_ws/src
git clone https://github.com/CCNYRoboticsLab/imu_tools.git -b humble

cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### Custom BMI160 Node

Create a simple ROS2 node that publishes IMU data:

```python
#!/usr/bin/env python3
"""ROS2 node for BMI160 IMU."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from capture.imu_driver import BMI160Driver

class BMI160Node(Node):
    def __init__(self):
        super().__init__('bmi160_imu')
        
        # Parameters
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('i2c_address', 0x69)
        self.declare_parameter('frame_id', 'imu_link')
        
        bus = self.get_parameter('i2c_bus').value
        address = self.get_parameter('i2c_address').value
        self.frame_id = self.get_parameter('frame_id').value
        
        # Publisher
        self.pub = self.create_publisher(Imu, 'imu/data_raw', 10)
        
        # Driver
        self.driver = BMI160Driver(bus=bus, address=address)
        if not self.driver.connect():
            self.get_logger().error('Failed to connect to BMI160')
            return
        
        self.driver.start_streaming(callback=self.on_sample)
        self.get_logger().info(f'BMI160 IMU node started on bus {bus}, address 0x{address:02X}')
    
    def on_sample(self, sample):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        
        msg.angular_velocity.x = sample.gyro_x
        msg.angular_velocity.y = sample.gyro_y
        msg.angular_velocity.z = sample.gyro_z
        
        msg.linear_acceleration.x = sample.accel_x
        msg.linear_acceleration.y = sample.accel_y
        msg.linear_acceleration.z = sample.accel_z
        
        self.pub.publish(msg)
    
    def destroy_node(self):
        self.driver.stop_streaming()
        self.driver.disconnect()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = BMI160Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Recording ROS Bags

```bash
# Record LiDAR and IMU topics
ros2 bag record /scan /imu/data_raw -o capture_session

# Play back
ros2 bag play capture_session
```

## Converting ROS Bags to Planar Format

Script to convert ROS bag to Planar session format:

```python
#!/usr/bin/env python3
"""Convert ROS2 bag to Planar session format."""

import csv
import json
from pathlib import Path
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan, Imu

def convert_bag_to_session(bag_path: str, output_dir: str):
    """Convert ROS2 bag to Planar session."""
    output_path = Path(output_dir)
    output_path.mkdir(parents=True, exist_ok=True)
    
    # Setup bag reader
    reader = SequentialReader()
    storage_options = StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = ConverterOptions('', '')
    reader.open(storage_options, converter_options)
    
    # Open output files
    lidar_file = open(output_path / 'lidar_station_0.csv', 'w', newline='')
    imu_file = open(output_path / 'imu_log.csv', 'w', newline='')
    
    lidar_writer = csv.writer(lidar_file)
    lidar_writer.writerow(['timestamp', 'angle_deg', 'distance_m', 'quality'])
    
    imu_writer = csv.writer(imu_file)
    imu_writer.writerow(['timestamp', 'gyro_x', 'gyro_y', 'gyro_z', 
                         'accel_x', 'accel_y', 'accel_z', 'temperature'])
    
    events = []
    
    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        ts = timestamp / 1e9  # Convert to seconds
        
        if topic == '/scan':
            msg = deserialize_message(data, LaserScan)
            angle = msg.angle_min
            for r in msg.ranges:
                if r > msg.range_min and r < msg.range_max:
                    lidar_writer.writerow([ts, math.degrees(angle), r, 50])
                angle += msg.angle_increment
        
        elif topic == '/imu/data_raw':
            msg = deserialize_message(data, Imu)
            imu_writer.writerow([
                ts,
                msg.angular_velocity.x,
                msg.angular_velocity.y,
                msg.angular_velocity.z,
                msg.linear_acceleration.x,
                msg.linear_acceleration.y,
                msg.linear_acceleration.z,
                0.0  # Temperature not in standard Imu msg
            ])
    
    lidar_file.close()
    imu_file.close()
    
    # Write metadata
    metadata = {
        "project": "Planar",
        "name": Path(bag_path).stem,
        "source": "ros2_bag",
        "stations": 1
    }
    with open(output_path / 'metadata.json', 'w') as f:
        json.dump(metadata, f, indent=2)
    
    # Write events
    with open(output_path / 'events.json', 'w') as f:
        json.dump(events, f, indent=2)
    
    print(f"Converted {bag_path} to {output_dir}")

if __name__ == '__main__':
    import sys
    if len(sys.argv) != 3:
        print("Usage: convert_bag.py <bag_path> <output_dir>")
        sys.exit(1)
    convert_bag_to_session(sys.argv[1], sys.argv[2])
```

## Docker-based ROS2 Setup

For isolation, run ROS2 in Docker:

```bash
# Pull ROS2 Humble image
docker pull ros:humble

# Run with device access
docker run -it --rm \
    --privileged \
    --device=/dev/rplidar \
    --device=/dev/i2c-1 \
    -v $(pwd):/workspace \
    ros:humble \
    bash
```

## Hybrid Workflow

Recommended workflow combining native Planar and ROS2:

1. **Capture**: Use native Planar daemon for reliable data collection
2. **Visualization**: Use ROS2 + RViz during development/debugging
3. **Processing**: Use native Planar pipeline for floor plan generation
4. **Integration**: Convert between formats as needed

```bash
# Terminal 1: Planar capture daemon
planar-capture --port 8080

# Terminal 2: Optional ROS2 visualization
ros2 launch sllidar_ros2 sllidar_s3_launch.py &
ros2 run rviz2 rviz2
```

## See Also

- [Hardware Setup](SDK_SETUP.md) - Setting up devices
- [Architecture Overview](ARCHITECTURE.md) - System design
- [sllidar_ros2](https://github.com/Slamtec/sllidar_ros2) - Official SLAMTEC ROS2 driver
- [ROS2 Documentation](https://docs.ros.org/en/humble/) - ROS2 Humble docs
