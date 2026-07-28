# aeb-node

Automotive-grade prototype of an **Autonomous Emergency Braking (AEB) sensing node**, built around a Slamtec RPLidar C1 on a Raspberry Pi Zero W.

The software is written as if it will eventually run on a real vehicle: modular, deterministic, RAII throughout, no global state, no singletons, and a strict architectural boundary between the safety path and development tooling.

> **Status:** Phase 1 complete (acquisition → TCP streaming → visualisation). Not safety-certified and not for use on a public road.

---

## Architecture

```
Lidar ──► ScanFrame ──┬──► [ perception / braking ]   safety path      (Phase 3)
                      └──► TcpServer ──► Mac viewer   development only
```

The arrow direction is the whole point: **obstacle detection and emergency braking must never depend on the networking layer.** If the TCP client disconnects, stalls or crashes, lidar acquisition and (from Phase 3) obstacle detection continue at full rate.

This is enforced in three places:

| Level | Mechanism |
|---|---|
| Source | `common/Types.hpp` has zero dependencies; the safety path never includes `network/` |
| Build | `aeb_lidar` does **not** link `aeb_network` — a violation is a link error |
| Runtime | `--no-network` creates no socket at all; the pipeline is unchanged |

### Layers

| Directory | Responsibility |
|---|---|
| `common/` | Domain types (`ScanPoint`, `ScanFrame`) and the binary wire protocol |
| `lidar/` | Acquisition. `RpLidarDevice` is the only consumer of the Slamtec SDK |
| `network/` | Development streaming server (single client, non-blocking, drop-oldest) |
| `app/` | Composition root: configuration, signals, supervision |
| `viewer/` | Mac-side polar visualiser (Python) |

Key design properties:

- **Producer isolation.** `TcpServer::publish()` performs no syscalls and cannot fail. Overflow drops the *oldest* frame — back-pressure never reaches the lidar thread.
- **Non-blocking I/O throughout.** One `poll` loop, self-pipe wakeup, explicit partial-write handling.
- **Deterministic shutdown.** Termination signals are blocked and consumed synchronously via `sigtimedwait`, so teardown runs on a known thread in a known order — no async signal handlers, no global flags.
- **Allocation-free steady state.** Buffers are sized once; frames are recycled through a pool.
- **SDK containment.** Vendor headers appear in exactly one `.cpp`.

---

## Wire protocol

Binary only. No JSON, no WebSockets. All fields little-endian; no packed structs are sent — serialisation is explicit, byte by byte, so the format is identical on ARM32 and x86-64.

```
Offset  Size  Field           Notes
 0       4    magic           0x42454131 ("AEB1")
 4       2    version         1
 6       2    header_size     32
 8       4    flags           bit0 = CRC32 trailer present
12       4    sequence
16       8    timestamp_us    monotonic
24       4    point_count
28       2    point_stride    10
30       2    reserved
32     N*S    points          angle f32 | distance f32 | quality u8 | reserved u8
              crc32           present only if flags bit0 (not emitted in v1)
```

**Forward compatible by construction.** A newer writer may grow the header (`header_size`), grow each point record (`point_stride`), or add the CRC trailer (`flags`) — existing readers skip what they don't understand. Current decoders already account for the CRC trailer in their framing, so enabling CRC32 later will not break deployed viewers. Unknown flags are *rejected*, because silently mis-parsing sensor data is worse than refusing it.

---

## Build (Raspberry Pi)

Requires the [Slamtec RPLidar SDK](https://github.com/Slamtec/rplidar_sdk), built separately.

```bash
git clone https://github.com/Slamtec/rplidar_sdk.git ~/rplidar_sdk
make -C ~/rplidar_sdk

cmake -S aeb_node -B build -DAEB_RPLIDAR_SDK_DIR=$HOME/rplidar_sdk
cmake --build build -j2
```

Run:

```bash
./build/bin/aeb_node --device /dev/ttyUSB0 --port 7000
```

| Option | Default | Purpose |
|---|---|---|
| `--device PATH` | `/dev/ttyUSB0` | Serial port (prefer a `/dev/serial/by-id/...` path) |
| `--baud RATE` | `460800` | C1 requires 460800 |
| `--bind ADDR` / `--port N` | `0.0.0.0` / `7000` | Stream endpoint |
| `--queue DEPTH` | `16` | Outbound frame queue depth |
| `--no-network` | off | Disable streaming entirely (vehicle configuration) |
| `--no-reconnect` | off | Exit acquisition on device error instead of recovering |
| `--health-interval SEC` | `5` | Health reporting period; `0` disables |

Build options: `-DAEB_WARNINGS_AS_ERRORS=OFF` (default ON), `-DAEB_BUILD_TESTS=ON` (Phase 2).

## Viewer (macOS)

```bash
python3 -m venv .venv && source .venv/bin/activate
pip install -r aeb_node/viewer/requirements.txt
python3 aeb_node/viewer/viewer.py --host raspberrypi.local --port 7000
```

The viewer reconnects automatically and reports sequence gaps, so frames dropped by the node are visible rather than silent.

---

## Roadmap

- [x] **Phase 1** — continuous acquisition, TCP streaming, Mac visualisation
- [ ] **Phase 2** — record to disk, replay, unit tests
- [ ] **Phase 3** — obstacle detection, time-to-collision, braking logic
- [ ] **Phase 4** — CAN bus integration, brake requests to an independent ECU

Non-goals: ROS, browser visualisation, WebSockets, JSON protocols.

## Conventions

- C++17, CMake, POSIX sockets, `std::thread`, RAII
- No global variables, no singletons, no blocking inside business logic
- One responsibility per class; implementation files stay under 250 lines
- Full Doxygen comments; no placeholder code

## Hardware

Raspberry Pi Zero W · Raspberry Pi OS Lite (Bookworm) · Slamtec RPLidar C1 via CP2102 USB-UART @ 460800 baud

## License

MIT — see [LICENSE](LICENSE).
