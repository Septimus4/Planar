#!/usr/bin/env bash
# Install helper for Slamtec rplidar_sdk
# - clones into third_party/rplidar_sdk
# - attempts to build SDK (if cmake available)

set -euo pipefail
WORKDIR=$(cd "$(dirname "$0")/.." && pwd)
TP_DIR="$WORKDIR/third_party"
SDK_DIR="$TP_DIR/rplidar_sdk"

mkdir -p "$TP_DIR"

if [ -d "$SDK_DIR" ]; then
  echo "rplidar_sdk already exists at $SDK_DIR"
  exit 0
fi

if ! command -v git >/dev/null 2>&1; then
  echo "git not found. Please install git and retry." >&2
  exit 1
fi

echo "Cloning rplidar_sdk into $SDK_DIR..."
git clone https://github.com/slamtec/rplidar_sdk.git "$SDK_DIR"

# try to build the SDK (optional)
if command -v cmake >/dev/null 2>&1 && command -v make >/dev/null 2>&1; then
  echo "Building rplidar_sdk (this may take a moment)..."
  mkdir -p "$SDK_DIR/build"
  cd "$SDK_DIR/build"
  cmake ..
  make -j$(nproc || echo 1)
  echo "Built rplidar_sdk in $SDK_DIR/build"
else
  echo "cmake or make not available — skipped build. Install build-essential and cmake to build the SDK."
fi

echo "rplidar_sdk is staged at: $SDK_DIR"
exit 0
