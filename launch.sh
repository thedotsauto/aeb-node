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
#   ./launch.sh                     # full build on every run
#   ./launch.sh --no-build          # skip cmake + pip (after first build)
#   ./launch.sh --no-can            # viewer-only, no MCP2515 HAT needed
#   ./launch.sh --no-build --no-can # fast viewer-only start
#
# Override defaults via environment variables before running:
#   RPLIDAR_SDK_DIR=/path/to/rplidar_sdk ./launch.sh
#   CAN_INTERFACE=can1 CAN_BITRATE=250000 ./launch.sh
# -----------------------------------------------------------------------------

set -euo pipefail

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
log()  { echo "[launch] $*"; }
err()  { echo "[launch] ERROR: $*" >&2; }
die()  { err "$*"; exit 1; }

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

# Parse flags
SKIP_BUILD=0
SKIP_TOF=0
SKIP_CAN=0
for arg in "$@"; do
    case "$arg" in
        --no-build) SKIP_BUILD=1 ;;
        --no-tof)   SKIP_TOF=1 ;;
        --no-can)   SKIP_CAN=1 ;;
        *) die "Unknown argument: $arg" ;;
    esac
done

# ---------------------------------------------------------------------------
# 1. SocketCAN interface setup
# ---------------------------------------------------------------------------
if [[ "$SKIP_CAN" -eq 1 ]]; then
    log "CAN disabled (--no-can), skipping interface setup."
else
    log "Bringing up $CAN_INTERFACE at $CAN_BITRATE bps..."

    # Silently take the interface down first (it may already be up with wrong config)
    sudo ip link set "$CAN_INTERFACE" down 2>/dev/null || true
    sudo ip link set "$CAN_INTERFACE" type can bitrate "$CAN_BITRATE" \
        || die "Could not configure $CAN_INTERFACE. Is the MCP2515 HAT present and the driver loaded?"
    sudo ip link set "$CAN_INTERFACE" up \
        || die "Could not bring up $CAN_INTERFACE"

    log "$CAN_INTERFACE is up at $CAN_BITRATE bps"
fi

# ---------------------------------------------------------------------------
# 2. Pull latest code
# ---------------------------------------------------------------------------
log "Pulling latest code..."
git -C "$REPO_DIR" pull --ff-only || log "git pull failed – continuing with local code"

# ---------------------------------------------------------------------------
# 3. CMake configure + build (C++ lidar + CAN node)
# ---------------------------------------------------------------------------
AEB_BIN="$BUILD_DIR/bin/aeb_node"

if [[ "$SKIP_BUILD" -eq 1 ]]; then
    log "Skipping build (--no-build)."
    [[ -x "$AEB_BIN" ]] || die "--no-build specified but binary not found at $AEB_BIN. Run without --no-build first."
else
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

    [[ -x "$AEB_BIN" ]] || die "Build succeeded but binary not found at $AEB_BIN"
    log "aeb_node built: $AEB_BIN"

    if [[ "$SKIP_TOF" -eq 0 ]]; then
        log "Installing ToF Python dependencies..."
        pip3 install -q -r "$REPO_DIR/tof/requirements.txt" \
            || die "pip3 install failed. Check network connection."
        log "ToF dependencies installed."
    fi
fi

TOF_PYTHON="python3"

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
# Kill any stale instance that may still hold the TCP port.
if pgrep -x aeb_node > /dev/null 2>&1; then
    log "Stopping existing aeb_node process..."
    pkill -x aeb_node || true
    sleep 1
fi

log "Starting aeb_node (lidar → perception → CAN)..."
AEB_ARGS=(--device "$LIDAR_DEVICE" --health-interval 5)
if [[ "$SKIP_CAN" -eq 1 ]]; then
    AEB_ARGS+=(--no-can)
else
    AEB_ARGS+=(--can-interface "$CAN_INTERFACE")
fi
"$AEB_BIN" "${AEB_ARGS[@]}" &
AEB_PID=$!

TOF_PID=""
if [[ "$SKIP_TOF" -eq 0 ]]; then
    log "Starting tof_node.py (VL53L5CX → CAN)..."
    "$TOF_PYTHON" "$REPO_DIR/tof/tof_node.py" \
        --can-interface "$CAN_INTERFACE" &
    TOF_PID=$!
    log "Both nodes running (aeb_node PID=$AEB_PID, tof_node PID=$TOF_PID)"
else
    log "ToF disabled (--no-tof). Only aeb_node running (PID=$AEB_PID)"
fi
log "Press Ctrl-C to stop."

# ---------------------------------------------------------------------------
# 7. Shutdown handler
# ---------------------------------------------------------------------------
cleanup() {
    log "Shutting down..."
    kill "$AEB_PID" ${TOF_PID:+"$TOF_PID"} 2>/dev/null || true
    wait "$AEB_PID" ${TOF_PID:+"$TOF_PID"} 2>/dev/null || true
    if [[ "$SKIP_CAN" -eq 0 ]]; then
        sudo ip link set "$CAN_INTERFACE" down 2>/dev/null || true
    fi
    log "Done."
}
trap cleanup INT TERM

# Wait for either process to exit (crash or signal)
# shellcheck disable=SC2086
wait -n "$AEB_PID" ${TOF_PID:+"$TOF_PID"} 2>/dev/null || true

# If one exited (e.g. crashed), kill the other
cleanup
