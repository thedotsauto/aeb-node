#!/usr/bin/env python3
"""can_bridge.py — UNO Q mode only (./launch.sh --unoq).

Forwards CAN frames written by aeb_node to /tmp/aeb_can_fifo on to the
STM32 MCU via the Arduino UNO Q Bridge (Arduino_RouterBridge / App Lab).

This script does exactly one thing:

    /tmp/aeb_can_fifo -> parse "<id> <dlc> <byte0> ... <byteN-1>" -> Bridge.notify

It contains no AEB logic, perception, LiDAR logic, braking logic, MCP2515
code or CAN bus hardware code. The frame itself (ID, DLC, payload) is
generated entirely by the existing C++ AEB logic; this script only relays
what CanBus already wrote to the FIFO (see aeb_node/canbus/CanBus.cpp).

Bridge API
----------
Uses `arduino.app_utils.Bridge`, the App Lab Python runtime's client for the
Arduino UNO Q Bridge (a MessagePack-RPC connection to the on-device router,
which forwards to the STM32 MCU sketch). This module is only importable on
the Arduino UNO Q's Linux MPU, inside the App Lab Python runtime where the
'arduino-router' service is running.

    Bridge.notify(method_name, *params)

sends a fire-and-forget RPC notification to whatever the MCU sketch has
registered with `Bridge.provide("can_frame", ...)`. `notify` (rather than
`call`) is used because no response is expected back from the MCU for a
print-only frame.

Wire call: Bridge.notify("can_frame", can_id, dlc, b0, b1, ..., b7)
    - Always exactly 10 parameters (fixed arity), zero-padded up to 8 data
      bytes, so the MCU-side handler can bind a fixed C++ signature.

Usage
-----
    python3 can_bridge.py --fifo /tmp/aeb_can_fifo
"""

from __future__ import annotations

import argparse
import os
import sys

try:
    from arduino.app_utils import Bridge
except ImportError as exc:  # pragma: no cover - depends on target hardware
    print(
        "[CAN BRIDGE] ERROR: could not import 'arduino.app_utils' "
        f"({exc}). This script must run on the Arduino UNO Q's Linux MPU, "
        "inside the App Lab Python runtime, where the 'arduino-router' "
        "bridge service and this package are provided.",
        file=sys.stderr,
    )
    sys.exit(1)


def _forward(line: str) -> None:
    """Parse one '<id> <dlc> <byte0> ... <byteN-1>' line and relay it."""
    parts = line.split()
    if len(parts) < 2:
        return

    try:
        can_id = int(parts[0])
        dlc = int(parts[1])
        raw_bytes = [int(x) for x in parts[2:2 + dlc]]
    except ValueError:
        print(f"[CAN BRIDGE] malformed line: {line.strip()}")
        return

    data_str = " ".join(f"{b:02X}" for b in raw_bytes)
    print(f"[CAN BRIDGE] ID=0x{can_id:X} DLC={dlc} DATA={data_str}")

    padded = (raw_bytes + [0] * 8)[:8]
    try:
        Bridge.notify("can_frame", can_id, dlc, *padded)
    except Exception as exc:
        print(f"[CAN BRIDGE] failed to forward frame to STM32: {exc}")


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Forward AEB CAN frames from a FIFO to the Arduino UNO Q Bridge."
    )
    parser.add_argument("--fifo", default="/tmp/aeb_can_fifo",
                        help="FIFO written by aeb_node in --unoq mode (default: /tmp/aeb_can_fifo)")
    args = parser.parse_args()

    if not os.path.exists(args.fifo):
        os.mkfifo(args.fifo)

    print("[CAN BRIDGE] connecting to Arduino UNO Q router...")
    Bridge.notify("aeb_can_bridge_ready")
    print(f"[CAN BRIDGE] connected. watching {args.fifo}")

    try:
        while True:
            # Opening a FIFO for reading blocks until a writer (aeb_node)
            # opens it, and reopening in this loop lets the bridge survive
            # aeb_node restarting.
            with open(args.fifo, "r") as fifo:
                for line in fifo:
                    _forward(line)
    except KeyboardInterrupt:
        pass

    print("[CAN BRIDGE] stopped")
    return 0


if __name__ == "__main__":
    sys.exit(main())
