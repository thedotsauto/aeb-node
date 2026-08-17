#!/usr/bin/env python3
"""
VL53L5CX Time-of-Flight AEB node.

Polls the Smartelex VL53L5CX 8x8 ToF imager over I2C.
If any zone detects a surface at or closer than MAX_DISTANCE_MM,
a brake-engage CAN frame is sent; otherwise a brake-disengage frame is sent.
Both share CAN arbitration ID 0x080 with the lidar node.

NOTE ON CAN ARBITRATION:
  Both this node and aeb_node transmit on 0x080.  When they agree (both BRAKE
  or both CLEAR) the bus carries a clean frame.  When they disagree, a bit
  error occurs and the losing node retransmits.  For a prototype this is
  acceptable; in production each node should use a unique sender ID and the
  ECU should OR the two decisions.

------------------------------------------------------------------------------
VL53L5CX WIRING TO RASPBERRY PI ZERO W (40-pin GPIO header)
------------------------------------------------------------------------------

  VL53L5CX breakout    Pi Zero W GPIO header
  ─────────────────    ─────────────────────────────────────
  VIN / 3V3            Pin  1  (3.3 V)
  GND                  Pin  6  (GND)
  SDA                  Pin  3  (GPIO 2 – I2C1 SDA)
  SCL                  Pin  5  (GPIO 3 – I2C1 SCL)
  LPn *                Pin  1  (3.3 V)   – tie HIGH to keep sensor enabled
  INT                  not connected      – polling mode, interrupt unused

  * Some Smartelex breakouts pull LPn HIGH by default; check your board.
    If the sensor is not found, try adding a 10 kΩ pull-up from LPn to 3.3 V.

Prerequisite – enable I2C on the Pi:
  sudo raspi-config  →  Interface Options  →  I2C  →  Enable
  (or add  dtparam=i2c_arm=on  to /boot/config.txt)

MCP2515 CAN HAT uses SPI0 (GPIO 8-11) and GPIO 25 – no pin conflict.
------------------------------------------------------------------------------
"""

from __future__ import annotations

import argparse
import sys
import time

import can
from vl53l5cx import VL53L5CX

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

BRAKE_CAN_ID    = 0x080
BRAKE_ENGAGE    = 0x02   # object detected within MAX_DISTANCE_MM
BRAKE_DISENGAGE = 0x01   # no object within MAX_DISTANCE_MM
MAX_DISTANCE_MM = 2000   # 200 cm
POLL_SLEEP_S    = 0.005  # 5 ms back-off when no data is ready yet


def main() -> int:
    parser = argparse.ArgumentParser(description="VL53L5CX ToF AEB node")
    parser.add_argument("--can-interface", default="can0",
                        help="SocketCAN interface name (default: can0)")
    args = parser.parse_args()

    # --- CAN bus setup ------------------------------------------------------
    try:
        bus = can.interface.Bus(channel=args.can_interface, bustype="socketcan")
    except OSError as exc:
        print(f"tof_node: cannot open CAN interface {args.can_interface}: {exc}",
              file=sys.stderr)
        return 1

    # --- VL53L5CX setup -----------------------------------------------------
    try:
        sensor = VL53L5CX()
    except Exception as exc:
        print(f"tof_node: cannot initialise VL53L5CX: {exc}", file=sys.stderr)
        bus.shutdown()
        return 1

    sensor.start_ranging()
    print(f"tof_node: ranging, CAN on {args.can_interface}, "
          f"brake threshold {MAX_DISTANCE_MM} mm")

    try:
        while True:
            if not sensor.data_ready():
                time.sleep(POLL_SLEEP_S)
                continue

            data = sensor.get_data()

            # Any zone with a positive, in-range reading triggers the brake.
            obstacle = any(
                0 < d <= MAX_DISTANCE_MM
                for d in data.distance_mm
            )

            payload = BRAKE_ENGAGE if obstacle else BRAKE_DISENGAGE
            msg = can.Message(
                arbitration_id=BRAKE_CAN_ID,
                data=[payload],
                is_extended_id=False,
            )
            try:
                bus.send(msg)
            except can.CanError as exc:
                print(f"tof_node: CAN send error: {exc}", file=sys.stderr)

            print("BRAKE" if obstacle else "clear",
                  f" zones={sum(1 for d in data.distance_mm if 0 < d <= MAX_DISTANCE_MM)}"
                  f"/{len(data.distance_mm)}")

    except KeyboardInterrupt:
        pass
    finally:
        sensor.stop_ranging()
        bus.shutdown()
        print("tof_node: stopped")

    return 0


if __name__ == "__main__":
    sys.exit(main())
