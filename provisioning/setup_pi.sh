#!/bin/bash
# Planar capture daemon setup script for Raspberry Pi
# Run as root: sudo ./setup_pi.sh

set -e

echo "==================================="
echo "Planar Capture Setup for Raspberry Pi"
echo "==================================="

# Check if running as root
if [ "$EUID" -ne 0 ]; then
    echo "Please run as root: sudo $0"
    exit 1
fi

PLANAR_DIR="/home/pi/planar"
CONFIG_DIR="/etc/planar"
SESSION_DIR="/var/planar/sessions"
USER="pi"

echo ""
echo "1. Installing system dependencies..."
apt-get update
apt-get install -y python3 python3-pip python3-venv i2c-tools

echo ""
echo "2. Enabling I2C interface..."
if ! grep -q "^dtparam=i2c_arm=on" /boot/config.txt 2>/dev/null; then
    echo "dtparam=i2c_arm=on" >> /boot/config.txt
    echo "   I2C enabled in /boot/config.txt (reboot required)"
fi

# Also check firmware config
if ! grep -q "^dtparam=i2c_arm=on" /boot/firmware/config.txt 2>/dev/null; then
    echo "dtparam=i2c_arm=on" >> /boot/firmware/config.txt 2>/dev/null || true
fi

echo ""
echo "3. Setting up udev rules for RPLidar..."
cp -v provisioning/udev/99-rplidar.rules /etc/udev/rules.d/
udevadm control --reload-rules
udevadm trigger

echo ""
echo "4. Creating directories..."
mkdir -p "$CONFIG_DIR"
mkdir -p "$SESSION_DIR"
chown -R "$USER:$USER" "$SESSION_DIR"

echo ""
echo "5. Installing configuration..."
cp -v provisioning/capture_config.json "$CONFIG_DIR/capture.json"
chown "$USER:$USER" "$CONFIG_DIR/capture.json"

echo ""
echo "6. Installing Python dependencies..."
sudo -u "$USER" pip3 install --user pyserial smbus2 aiohttp

echo ""
echo "7. Installing systemd service..."
cp -v provisioning/systemd/planar-capture.service /etc/systemd/system/
systemctl daemon-reload
systemctl enable planar-capture.service

echo ""
echo "8. Adding user to required groups..."
usermod -aG dialout "$USER"
usermod -aG i2c "$USER"

echo ""
echo "==================================="
echo "Setup complete!"
echo "==================================="
echo ""
echo "Next steps:"
echo "  1. Reboot to enable I2C: sudo reboot"
echo "  2. After reboot, verify devices:"
echo "     - LiDAR: ls -l /dev/rplidar"
echo "     - IMU:   sudo i2cdetect -y 1"
echo "  3. Start the service: sudo systemctl start planar-capture"
echo "  4. Check status: sudo systemctl status planar-capture"
echo "  5. View logs: journalctl -u planar-capture -f"
echo ""
echo "Connect from desktop with:"
echo "  python -m desktop.controller --host <PI_IP> status"
echo ""
