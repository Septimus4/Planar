#!/usr/bin/env bash
# Probe common serial device names and baud rates and start simple_grabber on first success.
# Usage: sudo ./scripts/probe_rplidar.sh [device]    # device optional, e.g. /dev/rplidar

set -eu

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
SDK_BIN="$SCRIPT_DIR/../third_party/rplidar_sdk/output/Linux/Release/simple_grabber"

if [ ! -x "$SDK_BIN" ]; then
  echo "simple_grabber binary not found or not executable: $SDK_BIN" >&2
  exit 2
fi

DEVICE_ARG=${1:-}
DEVICES=()
if [ -n "$DEVICE_ARG" ]; then
  DEVICES+=("$DEVICE_ARG")
else
  DEVICES+=(/dev/rplidar /dev/ttyUSB0 /dev/ttyUSB1 /dev/ttyACM0)
fi

BAUDS=(115200 256000 1000000)

try_one() {
  local dev=$1
  local baud=$2
  if [ ! -e "$dev" ]; then
    return 1
  fi
  echo "Trying $dev @ $baud..."
  # run the program for a short time to see if it can bind and returns data
  timeout 6s "$SDK_BIN" --channel --serial "$dev" "$baud" >/tmp/probe_rplidar.out 2>&1 || true
  # look for evidence of success in output
  if grep -q -E "Lidar health status|theta:|SLAMTEC LIDAR S/N" /tmp/probe_rplidar.out; then
    echo "SUCCESS: $dev @ $baud"
    cat /tmp/probe_rplidar.out
    echo "Launching interactive simple_grabber (CTRL-C to stop)"
    exec "$SDK_BIN" --channel --serial "$dev" "$baud"
  fi
  return 1
}

for d in "${DEVICES[@]}"; do
  for b in "${BAUDS[@]}"; do
    if try_one "$d" "$b"; then
      exit 0
    fi
  done
done

echo "No working device found on tried paths and baud rates." >&2
echo "Tried devices: ${DEVICES[*]}" >&2
echo "Tried baud rates: ${BAUDS[*]}" >&2
exit 1
