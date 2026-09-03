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

from arduino.app_utils import App, Bridge

FIFO_PATH = "/tmp/aeb_can_fifo"
MAX_DATA_BYTES = 8

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


def _ensure_fifo_open():
    """(Re)open FIFO_PATH for reading, blocking until aeb_node attaches."""
    global _fifo
    if not os.path.exists(FIFO_PATH):
        os.mkfifo(FIFO_PATH)
    # A blocking open() for read on a FIFO waits for a writer (aeb_node) to
    # attach; this is the expected, non-crashing way to handle "no writer
    # connected yet".
    _fifo = open(FIFO_PATH, "r")


def main_loop() -> None:
    """One App Lab loop iteration: read and forward a single CAN frame."""
    global _fifo

    if _fifo is None:
        _ensure_fifo_open()

    line = _fifo.readline()
    if line == "":
        # Writer (aeb_node) closed the FIFO; reopen and wait for the next one.
        _fifo.close()
        _fifo = None
        return

    _forward(line)


if __name__ == "__main__":
    print(f"[CAN BRIDGE] watching {FIFO_PATH}")
    App.run(user_loop=main_loop)
