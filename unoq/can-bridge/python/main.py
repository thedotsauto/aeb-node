# App Lab Python sketch for the "can-bridge" application (Linux MPU side).
#
# Deploy target on the Arduino UNO Q board:
#   ~/ArduinoApps/can-bridge/python/main.py
#
# Runs a TCP server on 127.0.0.1:39001 that aeb_node (./launch.sh --unoq)
# connects to, and forwards each received CAN frame to the STM32 MCU via
# the Arduino UNO Q Bridge. Linux is the SENDER here: sketch/sketch.ino
# (unchanged) registers Bridge.provide("can_frame", printCanFrame) on the
# MCU side, so this file only calls Bridge.notify(...), never
# Bridge.provide("can_frame").
#
# TCP payload format (written by aeb_node's CanBus, unchanged --
# see aeb_node/canbus/CanBus.cpp): "<CAN_ID> <DLC> <BYTE0> ... <BYTE(DLC-1)>\n"
#
#   291 8 17 34 51 68 85 102 119 136
#     CAN ID = 291 decimal = 0x123
#     DLC    = 8
#     data   = 11 22 33 44 55 66 77 88 (hex)
#
# The STM32 handler (sketch.ino, unchanged) has a fixed 10-parameter
# signature (id, dlc, b0..b7), so the Bridge.notify call below always sends
# exactly 8 data-byte parameters, zero-padded when DLC < 8. This padding is
# only on the outbound Bridge call; it does not change the wire format,
# CanMessage, DLC, or any AEB CAN generation logic.
#
# The TCP server runs in its own dedicated thread (not via
# App.run(user_loop=...) polling): accept()/recv() block naturally, so
# there is no FIFO, no os.mkfifo(), and no polling loop for IPC.

import socket
import threading

from arduino.app_utils import App, Bridge

TCP_HOST = "127.0.0.1"
TCP_PORT = 39001
MAX_DATA_BYTES = 8


def _parse_frame(line: str):
    """Parse one received line into (can_id, dlc, [byte, ...]); None if invalid."""
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


def _handle_connection(conn: socket.socket, addr) -> None:
    print(f"[CAN BRIDGE] aeb_node connected from {addr[0]}:{addr[1]}")
    buffer = ""
    with conn:
        while True:
            try:
                chunk = conn.recv(4096)
            except OSError as exc:
                print(f"[CAN BRIDGE] TCP recv error: {exc}")
                return

            if not chunk:
                print("[CAN BRIDGE] aeb_node disconnected")
                return

            buffer += chunk.decode("utf-8", errors="replace")
            while "\n" in buffer:
                line, buffer = buffer.split("\n", 1)
                print(f"[CAN BRIDGE] TCP RX: {line}")
                _forward(line)


def _serve_forever() -> None:
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((TCP_HOST, TCP_PORT))
    server.listen(1)
    print(f"[CAN BRIDGE] listening on {TCP_HOST}:{TCP_PORT}")

    while True:
        conn, addr = server.accept()
        _handle_connection(conn, addr)


def _start_server_thread() -> None:
    thread = threading.Thread(target=_serve_forever, name="CanBridgeTcpServer", daemon=True)
    thread.start()


if __name__ == "__main__":
    _start_server_thread()
    App.run()
