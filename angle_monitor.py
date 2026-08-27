"""angle_monitor.py — Runs on the Pi Zero W alongside the AEB node.

Connects to the AEB node's TCP stream (localhost:7000), reads every scan
frame and:

  1. Extracts the distance at two fixed angles (default 85° and 265°).
  2. Prints a compact live readout to the terminal.
  3. Re-broadcasts the raw AEB1 byte stream on a second port (default 7001)
     so the laptop lidar_viewer can connect without competing for port 7000.

Usage
-----
    # defaults: monitor angles 85° and 265°, relay on port 7001
    python3 angle_monitor.py

    # custom angles / ports
    python3 angle_monitor.py --angle-a 90 --angle-b 270 --relay-port 7001

    # Then on your laptop:
    python lidar_viewer.py --host <pi-ip> --port 7001

Architecture
------------
    AEB node :7000 ──► angle_monitor (this script)
                           ├── prints 85° / 265° every frame
                           └── re-serves raw stream on :7001 ──► laptop viewer

No Python dependencies beyond the standard library.
"""

from __future__ import annotations

import argparse
import math
import socket
import struct
import threading
import time
from typing import Optional

# ── AEB1 protocol (mirrors Protocol.hpp) ──────────────────────────────────────

MAGIC       = 0x42454131
VERSION     = 1
HEADER_SIZE = 32
POINT_STRIDE = 10
FLAG_CRC32  = 0x00000001
MAX_POINTS  = 65536


def _decode_frames(buf: bytearray) -> tuple[list[list[tuple[float, float, int]]], bytearray]:
    """Decode all complete AEB1 frames from *buf*.

    Returns (frames, remaining_bytes).
    Each frame is a list of (angle_deg, distance_m, quality) tuples.
    """
    frames: list[list[tuple[float, float, int]]] = []

    while len(buf) >= HEADER_SIZE:
        magic = struct.unpack_from("<I", buf, 0)[0]
        if magic != MAGIC:
            # resync: skip until next magic candidate
            for i in range(1, max(1, len(buf) - 3)):
                if struct.unpack_from("<I", buf, i)[0] == MAGIC:
                    buf = buf[i:]
                    break
            else:
                buf = buf[max(0, len(buf) - 3):]
            continue

        version, header_size = struct.unpack_from("<HH", buf, 4)
        if version != VERSION or header_size < HEADER_SIZE:
            buf = buf[1:]
            continue

        flags       = struct.unpack_from("<I", buf, 8)[0]
        point_count = struct.unpack_from("<I", buf, 24)[0]
        point_stride = struct.unpack_from("<H", buf, 28)[0]

        if point_count > MAX_POINTS or point_stride < POINT_STRIDE:
            buf = buf[1:]
            continue

        trailer = 4 if (flags & FLAG_CRC32) else 0
        packet_size = header_size + point_count * point_stride + trailer

        if len(buf) < packet_size:
            break  # need more data

        points: list[tuple[float, float, int]] = []
        offset = header_size
        for _ in range(point_count):
            angle_deg, dist_mm = struct.unpack_from("<ff", buf, offset)
            quality = buf[offset + 8]
            offset += point_stride
            if dist_mm > 0.0 and quality > 0:
                points.append((angle_deg, dist_mm / 1000.0, quality))

        buf = buf[packet_size:]
        frames.append(points)

    return frames, buf


def _nearest_point(points: list[tuple[float, float, int]], target_deg: float,
                   tolerance_deg: float = 3.0) -> Optional[tuple[float, float, int]]:
    """Return the point closest in angle to *target_deg* within *tolerance_deg*."""
    best: Optional[tuple[float, float, int]] = None
    best_diff = float("inf")
    for angle, dist, qual in points:
        diff = abs((angle - target_deg + 180) % 360 - 180)
        if diff < best_diff and diff <= tolerance_deg:
            best_diff = diff
            best = (angle, dist, qual)
    return best


# ── Relay server (re-broadcasts raw bytes to laptop viewer) ───────────────────

class RelayServer:
    """Listens on a TCP port and forwards every byte it receives to all clients."""

    def __init__(self, port: int) -> None:
        self._port = port
        self._clients: list[socket.socket] = []
        self._lock = threading.Lock()
        self._srv: Optional[socket.socket] = None

    def start(self) -> None:
        self._srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._srv.bind(("0.0.0.0", self._port))
        self._srv.listen(4)
        t = threading.Thread(target=self._accept_loop, daemon=True, name="RelayAccept")
        t.start()
        print(f"[relay] Listening on port {self._port} for viewer connections")

    def _accept_loop(self) -> None:
        while True:
            try:
                conn, addr = self._srv.accept()
                conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                with self._lock:
                    self._clients.append(conn)
                print(f"[relay] Viewer connected from {addr[0]}:{addr[1]}")
            except Exception:
                break

    def broadcast(self, data: bytes) -> None:
        """Forward *data* to every connected viewer; drop dead clients."""
        if not data:
            return
        dead: list[socket.socket] = []
        with self._lock:
            clients = list(self._clients)
        for c in clients:
            try:
                c.sendall(data)
            except OSError:
                dead.append(c)
        if dead:
            with self._lock:
                for d in dead:
                    self._clients.remove(d)
                    d.close()


# ── Main monitor loop ─────────────────────────────────────────────────────────

def run(aeb_host: str, aeb_port: int, relay_port: int,
        angle_a: float, angle_b: float, tolerance: float) -> None:

    relay = RelayServer(relay_port)
    relay.start()

    print(f"[monitor] Watching  A={angle_a}°  B={angle_b}°  (±{tolerance}°)")
    print(f"[monitor] Connecting to AEB node at {aeb_host}:{aeb_port} …")

    while True:
        try:
            _run_session(aeb_host, aeb_port, relay, angle_a, angle_b, tolerance)
        except Exception as exc:
            print(f"[monitor] Disconnected ({exc}), retrying in 2 s …")
            time.sleep(2.0)


def _run_session(aeb_host: str, aeb_port: int, relay: RelayServer,
                 angle_a: float, angle_b: float, tolerance: float) -> None:
    buf = bytearray()

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.settimeout(5.0)
        sock.connect((aeb_host, aeb_port))
        sock.settimeout(2.0)
        print(f"[monitor] Connected to AEB node")

        while True:
            chunk = sock.recv(8192)
            if not chunk:
                raise ConnectionError("AEB node closed connection")

            relay.broadcast(chunk)          # pass-through to laptop viewer

            buf.extend(chunk)
            frames, buf = _decode_frames(buf)

            for points in frames:
                pa = _nearest_point(points, angle_a, tolerance)
                pb = _nearest_point(points, angle_b, tolerance)

                da = f"{pa[1]*100:5.1f} cm  (actual {pa[0]:.1f}°)" if pa else "  ---  (no return)"
                db = f"{pb[1]*100:5.1f} cm  (actual {pb[0]:.1f}°)" if pb else "  ---  (no return)"

                print(f"\r  {angle_a:5.1f}°: {da}    {angle_b:5.1f}°: {db}   pts={len(points):<4}",
                      end="", flush=True)


# ── Entry point ───────────────────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(
        description="Monitor two lidar angles and relay stream to laptop viewer."
    )
    parser.add_argument("--aeb-host",    default="localhost",
                        help="AEB node host (default: localhost)")
    parser.add_argument("--aeb-port",    type=int, default=7000,
                        help="AEB node TCP port (default: 7000)")
    parser.add_argument("--relay-port",  type=int, default=7001,
                        help="Port to re-broadcast stream for laptop viewer (default: 7001)")
    parser.add_argument("--angle-a",     type=float, default=85.0,
                        help="First angle to monitor in degrees (default: 85)")
    parser.add_argument("--angle-b",     type=float, default=265.0,
                        help="Second angle to monitor in degrees (default: 265)")
    parser.add_argument("--tolerance",   type=float, default=3.0,
                        help="Angular tolerance in degrees (default: 3.0)")
    args = parser.parse_args()

    try:
        run(args.aeb_host, args.aeb_port, args.relay_port,
            args.angle_a, args.angle_b, args.tolerance)
    except KeyboardInterrupt:
        print("\n[monitor] Stopped.")


if __name__ == "__main__":
    main()
