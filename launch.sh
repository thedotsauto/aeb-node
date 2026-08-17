#!/usr/bin/env bash
# -----------------------------------------------------------------------------
# launch.sh – AEB node full launcher
#
# Runs on the Raspberry Pi Zero W.  Handles everything in one shot:
#   1. Bring up the MCP2515 SocketCAN interface
#   2. Pull the latest code from the remote
#   3. CMake configure + build the C++ lidar node
#   4. Install Python dependencies for the ToF node
#   5. Start aeb_node (lidar + CAN) and tof_node.py in parallel
#   6. Wait; Ctrl-C cleanly stops both and brings CAN down
#
# Usage:
#   chmod +x launch.sh
#   ./launch.sh
#
# Override defaults via environment variables before running:
#   RPLIDAR_SDK_DIR=/path/to/rplidar_sdk ./launch.sh
#   CAN_INTERFACE=can1 CAN_BITRATE=250000 ./launch.sh
# -----------------------------------------------------------------------------

set -euo pipefail

# ---------------------------------------------------------------------------
# Configuration – override with environment variables if needed
# ---------------------------------------------------------------------------
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$SCRIPT_DIR"
BUILD_DIR="$REPO_DIR/build"
RPLIDAR_SDK_DIR="${RPLIDAR_SDK_DIR:-$HOME/rplidar_sdk}"
CAN_INTERFACE="${CAN_INTERFACE:-can0}"
CAN_BITRATE="${CAN_BITRATE:-500000}"
LIDAR_DEVICE="${LIDAR_DEVICE:-/dev/ttyUSB0}"

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
log()  { echo "[launch] $*"; }
err()  { echo "[launch] ERROR: $*" >&2; }
die()  { err "$*"; exit 1; }

# ---------------------------------------------------------------------------
# 1. SocketCAN interface setup
# ---------------------------------------------------------------------------
log "Bringing up $CAN_INTERFACE at $CAN_BITRATE bps..."

# Silently take the interface down first (it may already be up with wrong config)
sudo ip link set "$CAN_INTERFACE" down 2>/dev/null || true
sudo ip link set "$CAN_INTERFACE" type can bitrate "$CAN_BITRATE" \
    || die "Could not configure $CAN_INTERFACE. Is the MCP2515 HAT present and the driver loaded?"
sudo ip link set "$CAN_INTERFACE" up \
    || die "Could not bring up $CAN_INTERFACE"

log "$CAN_INTERFACE is up at $CAN_BITRATE bps"

# ---------------------------------------------------------------------------
# 2. Pull latest code
# ---------------------------------------------------------------------------
log "Pulling latest code..."
git -C "$REPO_DIR" pull --ff-only || log "git pull failed – continuing with local code"

# ---------------------------------------------------------------------------
# 3. CMake configure + build (C++ lidar + CAN node)
# ---------------------------------------------------------------------------
if [[ ! -d "$RPLIDAR_SDK_DIR" ]]; then
    die "RPLidar SDK not found at $RPLIDAR_SDK_DIR. Set RPLIDAR_SDK_DIR and re-run."
fi

log "Configuring CMake build..."
cmake -B "$BUILD_DIR" \
      -S "$REPO_DIR/aeb_node" \
      -DCMAKE_BUILD_TYPE=RelWithDebInfo \
      -DAEB_RPLIDAR_SDK_DIR="$RPLIDAR_SDK_DIR" \
      -DAEB_WARNINGS_AS_ERRORS=OFF \
      --log-level=WARNING

log "Building aeb_node (this may take a few minutes on Pi Zero W)..."
cmake --build "$BUILD_DIR" -j"$(nproc)"

AEB_BIN="$BUILD_DIR/bin/aeb_node"
[[ -x "$AEB_BIN" ]] || die "Build succeeded but binary not found at $AEB_BIN"
log "aeb_node built: $AEB_BIN"

# ---------------------------------------------------------------------------
# 4. Python dependencies for ToF node
# ---------------------------------------------------------------------------
log "Installing ToF Python dependencies..."
pip3 install -q -r "$REPO_DIR/tof/requirements.txt" \
    || die "pip3 install failed. Check network and pip configuration."

# ---------------------------------------------------------------------------
# 5. Pre-flight checks
# ---------------------------------------------------------------------------
if [[ ! -e "/dev/i2c-1" ]]; then
    err "I2C device /dev/i2c-1 not found."
    err "Enable I2C: sudo raspi-config → Interface Options → I2C → Enable"
    err "Then reboot and re-run this script."
    die "I2C not enabled"
fi

if [[ ! -e "$LIDAR_DEVICE" ]]; then
    err "Lidar serial device $LIDAR_DEVICE not found."
    err "Connect the RPLidar USB adapter, or set LIDAR_DEVICE=/dev/ttyACM0 etc."
    die "Lidar device not found"
fi

# ---------------------------------------------------------------------------
# 6. Launch both nodes
# ---------------------------------------------------------------------------
log "Starting aeb_node (lidar → perception → CAN)..."
"$AEB_BIN" \
    --device "$LIDAR_DEVICE" \
    --can-interface "$CAN_INTERFACE" \
    --health-interval 5 &
AEB_PID=$!

log "Starting tof_node.py (VL53L5CX → CAN)..."
python3 "$REPO_DIR/tof/tof_node.py" \
    --can-interface "$CAN_INTERFACE" &
TOF_PID=$!

log "Both nodes running (aeb_node PID=$AEB_PID, tof_node PID=$TOF_PID)"
log "Press Ctrl-C to stop."

# ---------------------------------------------------------------------------
# 7. Shutdown handler
# ---------------------------------------------------------------------------
cleanup() {
    log "Shutting down..."
    kill "$AEB_PID" "$TOF_PID" 2>/dev/null || true
    wait "$AEB_PID" "$TOF_PID" 2>/dev/null || true
    sudo ip link set "$CAN_INTERFACE" down 2>/dev/null || true
    log "Done."
}
trap cleanup INT TERM

# Wait for either process to exit (crash or signal)
wait -n "$AEB_PID" "$TOF_PID" 2>/dev/null || true

# If one exited (e.g. crashed), kill the other
cleanup
