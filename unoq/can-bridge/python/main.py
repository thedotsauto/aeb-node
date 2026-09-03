# App Lab Python sketch for the "can-bridge" application (Linux MPU side).
#
# Deploy target on the Arduino UNO Q board:
#   ~/ArduinoApps/can-bridge/python/main.py
#
# Reads AEB CAN frames written by aeb_node (./launch.sh --unoq) to
# /tmp/aeb_can_fifo and forwards each one to the STM32 MCU via the Arduino
# UNO Q Bridge. Linux is the SENDER here: sketch/sketch.ino (unchanged)
# registers Bridge.provide("can_frame", printCanFrame) on the MCU side, so
# this file only calls Bridge.notify(...), never Bridge.provide("can_frame").
#
# FIFO line format (written by aeb_node's CanBus, unchanged --
# see aeb_node/canbus/CanBus.cpp): "<CAN_ID> <DLC> <BYTE0> ... <BYTE(DLC-1)>"
#
#   291 8 17 34 51 68 85 102 119 136
#     CAN ID = 291 decimal = 0x123
#     DLC    = 8
#     data   = 11 22 33 44 55 66 77 88 (hex)
#
# The STM32 handler (sketch.ino, unchanged) has a fixed 10-parameter
# signature (id, dlc, b0..b7), so the Bridge.notify call below always sends
# exactly 8 data-byte parameters, zero-padded when DLC < 8. This padding is
# only on the outbound Bridge call; it does not change the FIFO wire format,
# CanMessage, DLC, or any AEB CAN generation logic.

import os
import time

from arduino.app_utils import App, Bridge

FIFO_PATH = "/tmp/aeb_can_fifo"
MAX_DATA_BYTES = 8
_RETRY_DELAY_S = 0.05  # Avoid busy-spinning while the FIFO/writer isn't ready.

_fifo = None  # Open file handle for FIFO_PATH, re-opened whenever it is None.


def _parse_frame(line: str):
    """Parse one FIFO line into (can_id, dlc, [byte, ...]); None if invalid."""
    parts = line.split()
    if len(parts) < 2:
        return None

    try:
        can_id = int(parts[0])
        dlc = int(parts[1])
    except ValueError:
        return None

    if not (0 <= dlc <= MAX_DATA_BYTES):
        return None

    byte_tokens = parts[2:2 + dlc]
    if len(byte_tokens) != dlc:
        return None  # Line is truncated: fewer bytes than DLC declares.

    try:
        data = [int(b) for b in byte_tokens]
    except ValueError:
        return None

    if any(b < 0 or b > 255 for b in data):
        return None

    return can_id, dlc, data


def _forward(line: str) -> None:
    frame = _parse_frame(line)
    if frame is None:
        print(f"[CAN BRIDGE] malformed line: {line.strip()}")
        return

    can_id, dlc, data = frame
    data_str = " ".join(f"{b:02X}" for b in data)
    print(f"[CAN BRIDGE] ID=0x{can_id:X} DLC={dlc} DATA={data_str}")

    padded = (data + [0] * MAX_DATA_BYTES)[:MAX_DATA_BYTES]
    try:
        Bridge.notify("can_frame", can_id, dlc, *padded)
    except Exception as exc:
        print(f"[CAN BRIDGE] failed to forward frame to STM32: {exc}")


def _ensure_fifo_open() -> bool:
    """Try to open FIFO_PATH for non-blocking reads.

    launch.sh (--unoq) is responsible for creating the FIFO; this never
    calls os.mkfifo(). Returns True once _fifo is open. If the FIFO does
    not exist yet, returns False so the caller retries on a later loop
    iteration instead of blocking or crashing.
    """
    global _fifo
    try:
        fd = os.open(FIFO_PATH, os.O_RDONLY | os.O_NONBLOCK)
    except OSError:
        return False  # FIFO not created yet; retry later.
    _fifo = os.fdopen(fd, "r")
    return True


def main_loop() -> None:
    """One App Lab loop iteration: read and forward a single CAN frame."""
    global _fifo

    if _fifo is None:
        if not _ensure_fifo_open():
            time.sleep(_RETRY_DELAY_S)
            return

    try:
        line = _fifo.readline()
    except OSError:
        # No writer connected right now, or a transient read error; not
        # fatal. Retry on a later loop iteration.
        time.sleep(_RETRY_DELAY_S)
        return

    if not line:
        # Nothing available yet (no writer connected, or writer idle).
        # Keep the handle open and retry later.
        time.sleep(_RETRY_DELAY_S)
        return

    _forward(line)


if __name__ == "__main__":
    print(f"[CAN BRIDGE] watching {FIFO_PATH}")
    App.run(user_loop=main_loop)
