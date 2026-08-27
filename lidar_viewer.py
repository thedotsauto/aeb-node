"""Real-time 360° LiDAR viewer for the AEB node.

Connects to the aeb-node TCP stream (default port 7000), decodes AEB1 binary
frames (see aeb_node/common/Protocol.hpp) and renders a live polar plot.

Usage
-----
    # aeb-node running on the same machine
    python lidar_viewer.py

    # aeb-node running on the Pi (replace IP)
    python lidar_viewer.py --host 10.57.224.211

    # custom port
    python lidar_viewer.py --host 10.57.224.211 --port 7000

Requirements
------------
    pip install matplotlib numpy
"""

from __future__ import annotations

import argparse
import math
import socket
import struct
import sys
import threading
import time
from collections import deque
from typing import Deque, List, Tuple

import matplotlib.pyplot as plt
import numpy as np

# ── AEB1 protocol constants (matches Protocol.hpp) ────────────────────────────

MAGIC          = 0x42454131        # "AEB1" little-endian
VERSION        = 1
HEADER_SIZE    = 32
POINT_STRIDE   = 10               # bytes per point in v1
FLAG_CRC32     = 0x00000001
MAX_POINTS     = 65536

# ── Types ─────────────────────────────────────────────────────────────────────

ScanPoint = Tuple[float, float]   # (angle_rad, distance_m)


# ── Protocol decoder ──────────────────────────────────────────────────────────

class Decoder:
    """Streaming AEB1 decoder — mirrors ProtocolDecoder in Protocol.hpp."""

    def __init__(self) -> None:
        self._buf = bytearray()

    def feed(self, data: bytes) -> None:
        self._buf.extend(data)

    def decode_all(self) -> list[list[ScanPoint]]:
        """Return every complete frame currently in the buffer."""
        frames: list[list[ScanPoint]] = []
        while True:
            frame = self._try_decode_one()
            if frame is None:
                break
            frames.append(frame)
        return frames

    def _try_decode_one(self) -> list[ScanPoint] | None:
        buf = self._buf

        if len(buf) < HEADER_SIZE:
            return None

        # magic
        magic = struct.unpack_from("<I", buf, 0)[0]
        if magic != MAGIC:
            self._resync()
            return None

        version, header_size = struct.unpack_from("<HH", buf, 4)
        if version != VERSION or header_size < HEADER_SIZE:
            self._resync()
            return None

        flags, sequence = struct.unpack_from("<II", buf, 8)
        timestamp_us    = struct.unpack_from("<Q", buf, 16)[0]
        point_count, point_stride = struct.unpack_from("<IH", buf, 24)

        if point_count > MAX_POINTS or point_stride < POINT_STRIDE:
            self._resync()
            return None

        trailer = 4 if (flags & FLAG_CRC32) else 0
        packet_size = header_size + point_count * point_stride + trailer

        if len(buf) < packet_size:
            return None  # need more data

        points: list[ScanPoint] = []
        offset = header_size
        for _ in range(point_count):
            angle_deg, dist_mm = struct.unpack_from("<ff", buf, offset)
            quality = buf[offset + 8]
            offset += point_stride

            if dist_mm > 0.0 and quality > 0:
                angle_rad = math.radians(angle_deg)
                points.append((angle_rad, dist_mm / 1000.0))  # → metres

        del self._buf[:packet_size]
        return points

    def _resync(self) -> None:
        """Drop bytes until the next MAGIC candidate."""
        buf = self._buf
        for i in range(1, max(1, len(buf) - 3)):
            if struct.unpack_from("<I", buf, i)[0] == MAGIC:
                del self._buf[:i]
                return
        if len(buf) > 3:
            del self._buf[:len(buf) - 3]


# ── Receiver thread ───────────────────────────────────────────────────────────

class Receiver(threading.Thread):
    """Reads from the TCP socket and decodes frames in the background."""

    def __init__(self, host: str, port: int) -> None:
        super().__init__(daemon=True, name="LidarReceiver")
        self.host = host
        self.port = port
        self._lock = threading.Lock()
        self._latest: list[ScanPoint] = []
        self._frame_count = 0
        self._connected = False
        self._error = ""

    @property
    def connected(self) -> bool:
        return self._connected

    @property
    def error(self) -> str:
        return self._error

    def latest_frame(self) -> tuple[list[ScanPoint], int]:
        with self._lock:
            return list(self._latest), self._frame_count

    def run(self) -> None:
        while True:
            self._error = ""
            try:
                self._run_session()
            except Exception as exc:
                self._connected = False
                self._error = str(exc)
                time.sleep(1.0)

    def _run_session(self) -> None:
        decoder = Decoder()
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
            sock.settimeout(5.0)
            sock.connect((self.host, self.port))
            sock.settimeout(2.0)
            self._connected = True

            while True:
                chunk = sock.recv(4096)
                if not chunk:
                    raise ConnectionError("Server closed connection")
                decoder.feed(chunk)
                for frame in decoder.decode_all():
                    with self._lock:
                        self._latest = frame
                        self._frame_count += 1


# ── Viewer ────────────────────────────────────────────────────────────────────

def run_viewer(host: str, port: int, max_range_m: float) -> None:
    rx = Receiver(host, port)
    rx.start()

    print(f"Connecting to {host}:{port} …")

    plt.style.use("dark_background")
    fig = plt.figure(figsize=(8, 8))
    fig.canvas.manager.set_window_title("AEB LiDAR — 360° View")
    ax = fig.add_subplot(111, projection="polar")

    ax.set_facecolor("#0a0a0a")
    ax.set_ylim(0, max_range_m)
    ax.set_theta_zero_location("N")   # 0° at the top (forward)
    ax.set_theta_direction(-1)         # clockwise, matching RPLidar convention
    ax.set_rlabel_position(90)
    ax.yaxis.label.set_color("grey")
    ax.tick_params(colors="grey")
    ax.grid(color="#333333", linestyle="--", linewidth=0.5)

    # range rings labels
    for r in np.arange(0.5, max_range_m + 0.5, 0.5):
        ax.text(0, r, f"{r:.1f}m", color="#888888", fontsize=6,
                ha="left", va="bottom")

    scatter = ax.scatter([], [], s=2, c=[], cmap="plasma",
                         vmin=0, vmax=max_range_m, alpha=0.85)

    status_text = ax.text(0.5, -0.06, "Connecting…",
                          transform=ax.transAxes, ha="center",
                          color="#aaaaaa", fontsize=9)

    zoom_range = [max_range_m]   # mutable so closures can write it

    def set_zoom(new_range: float) -> None:
        new_range = max(0.5, min(new_range, 50.0))
        zoom_range[0] = new_range
        ax.set_ylim(0, new_range)
        scatter.set_clim(0, new_range)
        fig.canvas.draw_idle()

    def on_scroll(event):
        factor = 0.85 if event.button == "up" else 1.0 / 0.85
        set_zoom(zoom_range[0] * factor)

    def on_key(event):
        if event.key in ("+", "="):
            set_zoom(zoom_range[0] * 0.75)
        elif event.key == "-":
            set_zoom(zoom_range[0] / 0.75)
        elif event.key == "r":
            set_zoom(max_range_m)

    fig.canvas.mpl_connect("scroll_event", on_scroll)
    fig.canvas.mpl_connect("key_press_event", on_key)

    last_frame_idx = -1

    def update(_frame):
        nonlocal last_frame_idx

        points, count = rx.latest_frame()

        if not rx.connected:
            status_text.set_text(f"Reconnecting… {rx.error}")
            status_text.set_color("#ff6666")
        elif count == last_frame_idx or not points:
            return
        else:
            last_frame_idx = count
            angles = np.array([p[0] for p in points])
            dists  = np.array([p[1] for p in points])

            scatter.set_offsets(np.c_[angles, dists])
            scatter.set_array(dists)

            status_text.set_text(
                f"Frame #{count}   {len(points)} pts   "
                f"max {dists.max():.2f} m   min {dists.min():.2f} m   "
                f"view {zoom_range[0]:.1f} m  (scroll or +/- to zoom, r=reset)"
            )
            status_text.set_color("#aaaaaa")

    from matplotlib.animation import FuncAnimation
    ani = FuncAnimation(fig, update, interval=80, cache_frame_data=False)

    plt.tight_layout()
    plt.show()


# ── Entry point ───────────────────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(
        description="Real-time 360° LiDAR polar viewer for the AEB node."
    )
    parser.add_argument("--host", default="localhost",
                        help="aeb-node IP address (default: localhost)")
    parser.add_argument("--port", type=int, default=7000,
                        help="TCP port (default: 7000)")
    parser.add_argument("--max-range", type=float, default=6.0,
                        help="Polar plot radius in metres (default: 6.0)")
    args = parser.parse_args()

    try:
        run_viewer(args.host, args.port, args.max_range)
    except KeyboardInterrupt:
        print("\nViewer closed.")


if __name__ == "__main__":
    main()
