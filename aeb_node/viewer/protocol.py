"""Python mirror of the AEB binary scan protocol.

This module is the Python counterpart of ``common/Protocol.hpp`` and must be
kept byte-for-byte consistent with it. It performs no I/O, so it can be tested
against a byte string with no socket and no hardware involved.

Wire format (version 1, little-endian, no padding):

    header  magic u32 | version u16 | header_size u16 | flags u32
            sequence u32 | timestamp_us u64 | point_count u32
            point_stride u16 | reserved u16                        (32 bytes)
    point   angle_deg f32 | distance_mm f32 | quality u8 | reserved u8

Forward compatibility mirrors the C++ decoder exactly: a larger ``header_size``
or ``point_stride`` is skipped over, and the CRC32 trailer slot is accounted for
in the framing even though version 1 never emits it.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass, field
from typing import Iterator, Optional

import numpy as np

MAGIC = 0x42454131
VERSION = 1
HEADER_SIZE_V1 = 32
POINT_STRIDE_V1 = 10
FLAG_CRC32 = 0x00000001
KNOWN_FLAGS = FLAG_CRC32
CRC_TRAILER_SIZE = 4
MAX_POINTS_PER_FRAME = 65536

_HEADER = struct.Struct("<IHHIIQIHH")
_POINT = struct.Struct("<ffBB")

_POINT_DTYPE = np.dtype([
    ("angle_deg", "<f4"),
    ("distance_mm", "<f4"),
    ("quality", "u1"),
    ("reserved", "u1"),
])

assert _HEADER.size == HEADER_SIZE_V1
assert _POINT.size == POINT_STRIDE_V1


class ProtocolError(Exception):
    """Raised when the byte stream cannot be interpreted as a valid packet."""


@dataclass
class ScanFrame:
    """One decoded revolution, stored column-wise for efficient plotting."""

    sequence: int = 0
    timestamp_us: int = 0
    angles_deg: np.ndarray = field(default_factory=lambda: np.empty(0, dtype=np.float32))
    distances_mm: np.ndarray = field(default_factory=lambda: np.empty(0, dtype=np.float32))
    qualities: np.ndarray = field(default_factory=lambda: np.empty(0, dtype=np.uint8))

    def __len__(self) -> int:
        return int(self.angles_deg.size)


class ProtocolDecoder:
    """Reassembles :class:`ScanFrame` objects from a TCP byte stream.

    Single responsibility: framing and parsing. TCP delivers a stream, not
    messages, so the decoder buffers partial packets and is driven in a
    feed/drain loop.
    """

    def __init__(self) -> None:
        self._buffer = bytearray()

    def feed(self, data: bytes) -> None:
        """Append freshly received bytes to the reassembly buffer."""
        self._buffer.extend(data)

    def reset(self) -> None:
        """Discard buffered bytes, e.g. after reconnecting."""
        self._buffer.clear()

    @property
    def buffered(self) -> int:
        """Number of bytes held pending reassembly."""
        return len(self._buffer)

    def frames(self) -> Iterator[ScanFrame]:
        """Yield every complete frame currently buffered.

        Raises:
            ProtocolError: if the stream is corrupt or uses a format this
                decoder cannot safely interpret.
        """
        while True:
            frame = self._decode_one()
            if frame is None:
                return
            yield frame

    def _decode_one(self) -> Optional[ScanFrame]:
        """Decode and consume one packet, or return None if incomplete."""
        if len(self._buffer) < HEADER_SIZE_V1:
            return None

        (magic, version, header_size, flags, sequence, timestamp_us,
         point_count, point_stride, _reserved) = _HEADER.unpack_from(self._buffer, 0)

        if magic != MAGIC:
            raise ProtocolError(f"bad magic 0x{magic:08X}; stream is not synchronised")
        if version != VERSION:
            raise ProtocolError(f"unsupported protocol version {version}")
        if header_size < HEADER_SIZE_V1 or point_stride < POINT_STRIDE_V1:
            raise ProtocolError("header or point record smaller than the version 1 baseline")
        if flags & ~KNOWN_FLAGS:
            # An unknown flag may imply an unknown framing change; refusing is
            # safer than silently mis-parsing sensor data.
            raise ProtocolError(f"unknown protocol flags 0x{flags:08X}")
        if point_count > MAX_POINTS_PER_FRAME:
            raise ProtocolError(f"implausible point count {point_count}")

        payload_size = point_count * point_stride
        trailer_size = CRC_TRAILER_SIZE if (flags & FLAG_CRC32) else 0
        packet_size = header_size + payload_size + trailer_size
        if len(self._buffer) < packet_size:
            return None

        frame = self._decode_points(header_size, point_count, point_stride)
        frame.sequence = sequence
        frame.timestamp_us = timestamp_us

        del self._buffer[:packet_size]
        return frame

    def _decode_points(self, header_size: int, count: int, stride: int) -> ScanFrame:
        """Extract the point array, skipping fields a newer writer may have added."""
        payload = memoryview(self._buffer)[header_size:header_size + count * stride]

        if stride == POINT_STRIDE_V1:
            # Fast path: one vectorised read instead of a per-point loop.
            raw = np.frombuffer(payload, dtype=_POINT_DTYPE, count=count)
            return ScanFrame(
                angles_deg=np.ascontiguousarray(raw["angle_deg"]),
                distances_mm=np.ascontiguousarray(raw["distance_mm"]),
                qualities=np.ascontiguousarray(raw["quality"]),
            )

        angles = np.empty(count, dtype=np.float32)
        distances = np.empty(count, dtype=np.float32)
        qualities = np.empty(count, dtype=np.uint8)
        for i in range(count):
            angle, distance, quality, _pad = _POINT.unpack_from(payload, i * stride)
            angles[i] = angle
            distances[i] = distance
            qualities[i] = quality
        return ScanFrame(angles_deg=angles, distances_mm=distances, qualities=qualities)
