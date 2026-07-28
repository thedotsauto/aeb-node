#!/usr/bin/env python3
"""Mac-side visualiser for the AEB node scan stream.

Connects to the node over TCP, decodes the binary protocol with
:mod:`protocol` and renders each revolution on a polar plot.

This is a development tool. It has no influence on the vehicle: the node
discards frames whenever this viewer is absent or slow, so nothing here can
affect acquisition timing or the safety path.

Usage:
    python3 viewer.py --host raspberrypi.local --port 7000

Requires: matplotlib, numpy (see requirements.txt)
"""

from __future__ import annotations

import argparse
import socket
import sys
import threading
from typing import List, Optional, Tuple

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation

from protocol import ProtocolDecoder, ProtocolError, ScanFrame

RECEIVE_CHUNK_BYTES = 65536


class ScanClient:
    """Background TCP reader that always exposes the most recent frame.

    Owns the socket, the decoder and the reader thread. The renderer only ever
    takes a snapshot, so it never waits on the network: a stalled link freezes
    the picture rather than the user interface.

    Reconnection is automatic and indefinite, which matches the node's own
    behaviour - either end may restart without the other needing attention.
    """

    def __init__(self, host: str, port: int, reconnect_delay_s: float = 1.0) -> None:
        self._host = host
        self._port = port
        self._reconnect_delay_s = reconnect_delay_s
        self._decoder = ProtocolDecoder()
        self._lock = threading.Lock()
        self._latest: Optional[ScanFrame] = None
        self._connected = False
        self._frames_received = 0
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="scan-client", daemon=True)

    def start(self) -> None:
        """Begin connecting and reading in the background."""
        self._thread.start()

    def stop(self) -> None:
        """Stop reading and join the reader thread."""
        self._stop.set()
        self._thread.join(timeout=2.0)

    def snapshot(self) -> Tuple[Optional[ScanFrame], bool, int]:
        """Return the newest frame, the link state and the total frame count."""
        with self._lock:
            return self._latest, self._connected, self._frames_received

    def _run(self) -> None:
        """Reader thread: connect, consume, recover, repeat."""
        while not self._stop.is_set():
            try:
                self._session()
            except OSError as exc:
                print(f"viewer: connection error: {exc}", file=sys.stderr)
            except ProtocolError as exc:
                # The stream is unrecoverable at this position; drop the
                # connection rather than guessing at a resynchronisation point.
                print(f"viewer: protocol error: {exc}", file=sys.stderr)

            with self._lock:
                self._connected = False
            self._decoder.reset()
            self._stop.wait(self._reconnect_delay_s)

    def _session(self) -> None:
        """Run one connection until it closes or fails."""
        with socket.create_connection((self._host, self._port), timeout=5.0) as sock:
            sock.settimeout(1.0)
            print(f"viewer: connected to {self._host}:{self._port}")
            with self._lock:
                self._connected = True

            while not self._stop.is_set():
                try:
                    chunk = sock.recv(RECEIVE_CHUNK_BYTES)
                except socket.timeout:
                    continue  # Idle sensor; the session is still healthy.
                if not chunk:
                    print("viewer: node closed the connection")
                    return

                self._decoder.feed(chunk)
                for frame in self._decoder.frames():
                    with self._lock:
                        self._latest = frame
                        self._frames_received += 1


class ScanRenderer:
    """Polar rendering of the latest frame.

    Single responsibility: presentation. It pulls from :class:`ScanClient` at
    its own fixed rate, completely decoupled from network timing, and simply
    redraws whatever the most recent frame happens to be.
    """

    def __init__(self, client: ScanClient, max_range_mm: float, min_quality: int) -> None:
        self._client = client
        self._max_range_mm = max_range_mm
        self._min_quality = min_quality
        self._last_sequence = -1
        self._animation: Optional[FuncAnimation] = None

        self._figure, self._axes = plt.subplots(subplot_kw={"projection": "polar"})
        self._figure.canvas.manager.set_window_title("AEB scan viewer")

        # The RPLidar reports bearings clockwise from its zero mark, so the plot
        # is oriented to match the physical sensor rather than the mathematical
        # convention. Getting this wrong mirrors the world left to right.
        self._axes.set_theta_zero_location("N")
        self._axes.set_theta_direction(-1)
        self._axes.set_ylim(0.0, max_range_mm)
        self._axes.set_title("AEB scan viewer - starting", fontsize=10)
        self._axes.grid(True, alpha=0.3)

        self._scatter = self._axes.scatter([], [], s=2.0, c=[], cmap="viridis",
                                           vmin=0, vmax=255)

    def run(self, interval_ms: int) -> None:
        """Show the window and refresh until the user closes it."""
        # FuncAnimation must be kept referenced or the GUI loop discards it.
        self._animation = FuncAnimation(self._figure, self._update, interval=interval_ms,
                                        blit=False, cache_frame_data=False)
        plt.show()

    def _update(self, _tick: int):
        """Redraw from the newest available frame."""
        frame, connected, total = self._client.snapshot()

        if frame is None:
            state = "connected, no frames yet" if connected else "disconnected"
            self._axes.set_title(f"AEB scan viewer - {state}", fontsize=10)
            return (self._scatter,)

        keep = (frame.distances_mm > 0.0) & (frame.qualities >= self._min_quality)
        angles_rad = np.deg2rad(frame.angles_deg[keep])
        distances = frame.distances_mm[keep]

        self._scatter.set_offsets(np.column_stack((angles_rad, distances)))
        self._scatter.set_array(frame.qualities[keep])
        self._axes.set_title(self._status(frame, connected, total, int(distances.size)),
                             fontsize=10)
        return (self._scatter,)

    def _status(self, frame: ScanFrame, connected: bool, total: int, shown: int) -> str:
        """Build the title line, including any detected sequence gap."""
        gap = ""
        if 0 <= self._last_sequence < frame.sequence - 1:
            # Frames are dropped by the node whenever this viewer cannot keep
            # up. Surfacing it makes that trade-off visible rather than silent.
            gap = f"  gap:{frame.sequence - self._last_sequence - 1}"
        self._last_sequence = frame.sequence

        link = "live" if connected else "stale"
        return (f"AEB scan viewer - {link}  seq:{frame.sequence}  "
                f"points:{shown}/{len(frame)}  rx:{total}{gap}")


def parse_arguments(argv: Optional[List[str]] = None) -> argparse.Namespace:
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(description="AEB node scan viewer")
    parser.add_argument("--host", default="raspberrypi.local", help="node hostname or IP")
    parser.add_argument("--port", type=int, default=7000, help="node TCP port")
    parser.add_argument("--max-range", type=float, default=12000.0,
                        help="plot radius in millimetres")
    parser.add_argument("--min-quality", type=int, default=1,
                        help="discard points below this reported quality")
    parser.add_argument("--fps", type=float, default=20.0, help="render refresh rate")
    return parser.parse_args(argv)


def main(argv: Optional[List[str]] = None) -> int:
    """Entry point."""
    args = parse_arguments(argv)
    if args.fps <= 0.0:
        print("error: --fps must be positive", file=sys.stderr)
        return 2
    if args.max_range <= 0.0:
        print("error: --max-range must be positive", file=sys.stderr)
        return 2

    client = ScanClient(args.host, args.port)
    client.start()
    try:
        ScanRenderer(client, args.max_range, args.min_quality).run(int(1000.0 / args.fps))
    except KeyboardInterrupt:
        pass
    finally:
        client.stop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
