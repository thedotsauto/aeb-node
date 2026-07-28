/**
 * @file Protocol.hpp
 * @brief Binary wire format for streaming @ref aeb::ScanFrame over a byte stream.
 *
 * This header defines the AEB scan streaming protocol and the two classes that
 * implement it:
 *
 *  - @ref aeb::proto::ProtocolEncoder : ScanFrame  -> bytes
 *  - @ref aeb::proto::ProtocolDecoder : bytes      -> ScanFrame
 *
 * The protocol is **development and visualisation only**. Nothing in the
 * perception or braking path may depend on this header.
 *
 * @section wire Wire format (protocol version 1)
 *
 * All multi-byte fields are **little-endian**. Floats are IEEE-754 binary32
 * transmitted as their little-endian bit pattern. There are no packed structs
 * and no @c memcpy of host structures: every field is read and written byte by
 * byte, so the format is identical on any compiler, ABI or architecture.
 *
 * @code
 * Offset  Size  Field          Description
 * ------  ----  -------------  ----------------------------------------------
 *  0       4    magic          0x42454131 ("AEB1"), little-endian
 *  4       2    version        Protocol version, currently 1
 *  6       2    header_size    Total header bytes, currently 32
 *  8       4    flags          Bit flags, see kFlag* constants
 * 12       4    sequence       Frame sequence number
 * 16       8    timestamp_us   Monotonic acquisition timestamp, microseconds
 * 24       4    point_count    Number of points that follow
 * 28       2    point_stride   Bytes per point, currently 10
 * 30       2    reserved       Must be written as 0, ignored on read
 * ------  ----  -------------  ----------------------------------------------
 * 32      N*S   points         point_count records of point_stride bytes
 * 32+N*S   0/4  crc32          Present only if kFlagCrc32 is set (see below)
 * @endcode
 *
 * Point record (point_stride = 10):
 * @code
 * Offset  Size  Field          Description
 *  0       4    angle_deg      IEEE-754 binary32
 *  4       4    distance_mm    IEEE-754 binary32
 *  8       1    quality        0..255
 *  9       1    reserved       Must be written as 0, ignored on read
 * @endcode
 *
 * @section compat Forward compatibility
 *
 * Three mechanisms keep old readers working against newer writers:
 *
 *  1. @c header_size lets a writer append new header fields. A decoder that
 *     does not know them skips @c (header_size - 32) bytes.
 *  2. @c point_stride lets a writer append new per-point fields (e.g. intensity
 *     or a validity mask). A decoder skips @c (point_stride - 10) trailing
 *     bytes of each record.
 *  3. @c flags announces optional trailers. @ref kFlagCrc32 reserves the CRC32
 *     trailer slot. Version 1 writers never set it; version 1 readers already
 *     understand the framing implication and skip the 4 trailer bytes, so CRC
 *     can be switched on later without breaking deployed viewers.
 *
 * Flags that a decoder does not recognise are **rejected**, because an unknown
 * flag may imply an unknown framing change and silently mis-parsing a safety
 * data stream is worse than refusing it.
 */

#ifndef AEB_COMMON_PROTOCOL_HPP
#define AEB_COMMON_PROTOCOL_HPP

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <vector>

#include "Types.hpp"

namespace aeb::proto {

/** @brief Magic number identifying an AEB scan packet ("AEB1"). */
inline constexpr std::uint32_t kMagic = 0x42454131U;

/** @brief Protocol version implemented by this translation unit. */
inline constexpr std::uint16_t kVersion = 1U;

/** @brief Size in bytes of the version 1 packet header. */
inline constexpr std::uint16_t kHeaderSizeV1 = 32U;

/** @brief Size in bytes of a version 1 point record. */
inline constexpr std::uint16_t kPointStrideV1 = 10U;

/**
 * @brief Flag: a 4-byte little-endian CRC32 trailer follows the point array.
 *
 * Not emitted by version 1 encoders. Decoders in this version parse and skip
 * the trailer so that enabling CRC later is a compatible change.
 */
inline constexpr std::uint32_t kFlagCrc32 = 0x00000001U;

/** @brief Mask of every flag this implementation understands. */
inline constexpr std::uint32_t kKnownFlagsMask = kFlagCrc32;

/** @brief Size in bytes of the CRC32 trailer when @ref kFlagCrc32 is set. */
inline constexpr std::size_t kCrcTrailerSize = 4U;

/**
 * @brief Hard upper bound on points per frame accepted by the decoder.
 *
 * The RPLidar C1 emits roughly 1000 points per revolution at 10 Hz. This bound
 * exists purely to stop a corrupt or hostile @c point_count from triggering a
 * multi-gigabyte allocation on a 512 MB Pi Zero.
 */
inline constexpr std::uint32_t kMaxPointsPerFrame = 65536U;

/** @brief Hard upper bound on @c point_stride accepted by the decoder. */
inline constexpr std::uint16_t kMaxPointStride = 256U;

/** @brief Hard upper bound on @c header_size accepted by the decoder. */
inline constexpr std::uint16_t kMaxHeaderSize = 1024U;

/**
 * @brief Outcome of a decode attempt.
 */
enum class DecodeStatus {
    Ok,             ///< A complete, well-formed frame was produced.
    NeedMoreData,   ///< Buffer holds a valid but incomplete packet prefix.
    BadMagic,       ///< Stream is not synchronised to a packet boundary.
    BadVersion,     ///< Packet version is not supported by this decoder.
    BadHeader,      ///< header_size, point_stride or flags are unusable.
    TooLarge,       ///< point_count exceeds @ref kMaxPointsPerFrame.
    CrcMismatch     ///< Reserved for when CRC validation is enabled.
};

/**
 * @brief Human-readable name of a @ref DecodeStatus, for logging.
 * @param status Status to describe.
 * @return Static, never-null, NUL-terminated string.
 */
[[nodiscard]] inline const char* toString(DecodeStatus status) noexcept
{
    switch (status) {
        case DecodeStatus::Ok:           return "Ok";
        case DecodeStatus::NeedMoreData: return "NeedMoreData";
        case DecodeStatus::BadMagic:     return "BadMagic";
        case DecodeStatus::BadVersion:   return "BadVersion";
        case DecodeStatus::BadHeader:    return "BadHeader";
        case DecodeStatus::TooLarge:     return "TooLarge";
        case DecodeStatus::CrcMismatch:  return "CrcMismatch";
    }
    return "Unknown";
}

/**
 * @brief Endian-explicit primitive write helpers.
 *
 * These append to a caller-owned buffer, so the encoder never owns memory it
 * did not receive and callers can reuse one buffer across frames.
 */
namespace detail {

/**
 * @brief Append an unsigned integer as @p Bytes little-endian octets.
 * @tparam Bytes Number of octets to emit (1, 2, 4 or 8).
 * @param out   Destination buffer, appended to.
 * @param value Value to encode; bits above @p Bytes*8 are discarded.
 */
template <std::size_t Bytes>
inline void putUint(std::vector<std::uint8_t>& out, std::uint64_t value)
{
    static_assert(Bytes == 1U || Bytes == 2U || Bytes == 4U || Bytes == 8U,
                  "unsupported integer width");
    for (std::size_t i = 0U; i < Bytes; ++i) {
        out.push_back(static_cast<std::uint8_t>((value >> (8U * i)) & 0xFFU));
    }
}

/**
 * @brief Append an IEEE-754 binary32 value in little-endian bit order.
 * @param out   Destination buffer, appended to.
 * @param value Value to encode.
 */
inline void putFloat32(std::vector<std::uint8_t>& out, float value)
{
    std::uint32_t bits = 0U;
    std::memcpy(&bits, &value, sizeof(bits));
    putUint<4U>(out, bits);
}

/**
 * @brief Read @p Bytes little-endian octets as an unsigned integer.
 * @tparam Bytes Number of octets to consume (1, 2, 4 or 8).
 * @param data Pointer to at least @p Bytes readable octets.
 * @return Decoded value, zero-extended to 64 bits.
 */
template <std::size_t Bytes>
[[nodiscard]] inline std::uint64_t getUint(const std::uint8_t* data) noexcept
{
    static_assert(Bytes == 1U || Bytes == 2U || Bytes == 4U || Bytes == 8U,
                  "unsupported integer width");
    std::uint64_t value = 0U;
    for (std::size_t i = 0U; i < Bytes; ++i) {
        value |= static_cast<std::uint64_t>(data[i]) << (8U * i);
    }
    return value;
}

/**
 * @brief Read an IEEE-754 binary32 value from little-endian octets.
 * @param data Pointer to at least 4 readable octets.
 * @return Decoded value.
 */
[[nodiscard]] inline float getFloat32(const std::uint8_t* data) noexcept
{
    const std::uint32_t bits = static_cast<std::uint32_t>(getUint<4U>(data));
    float value = 0.0F;
    std::memcpy(&value, &bits, sizeof(value));
    return value;
}

}  // namespace detail

/**
 * @brief Serialises a @ref aeb::ScanFrame into the version 1 wire format.
 *
 * Single responsibility: byte layout. The encoder performs no I/O, owns no
 * resources, holds no state between calls and never blocks, which makes it
 * trivially unit testable and safe to call from any thread.
 */
class ProtocolEncoder {
public:
    /**
     * @brief Encode @p frame, replacing the contents of @p out.
     *
     * @param frame Frame to serialise. Points beyond @ref kMaxPointsPerFrame
     *              are not emitted; the encoded @c point_count reflects what
     *              was actually written, so the packet stays self-consistent.
     * @param out   Destination buffer. Cleared first, then filled. Reusing the
     *              same buffer across frames avoids a per-frame allocation on
     *              the Pi Zero.
     *
     * @note Never throws except for @c std::bad_alloc from @p out.
     */
    static void encode(const ScanFrame& frame, std::vector<std::uint8_t>& out)
    {
        const std::uint32_t count = pointCount(frame);

        out.clear();
        out.reserve(static_cast<std::size_t>(kHeaderSizeV1) +
                    static_cast<std::size_t>(count) * kPointStrideV1);

        detail::putUint<4U>(out, kMagic);
        detail::putUint<2U>(out, kVersion);
        detail::putUint<2U>(out, kHeaderSizeV1);
        detail::putUint<4U>(out, 0U);  // flags: no CRC trailer in version 1
        detail::putUint<4U>(out, frame.sequence);
        detail::putUint<8U>(out, frame.timestamp_us);
        detail::putUint<4U>(out, count);
        detail::putUint<2U>(out, kPointStrideV1);
        detail::putUint<2U>(out, 0U);  // reserved

        for (std::uint32_t i = 0U; i < count; ++i) {
            const ScanPoint& p = frame.points[i];
            detail::putFloat32(out, p.angle_deg);
            detail::putFloat32(out, p.distance_mm);
            detail::putUint<1U>(out, p.quality);
            detail::putUint<1U>(out, 0U);  // reserved
        }
    }

    /**
     * @brief Exact number of bytes @ref encode will produce for @p frame.
     * @param frame Frame to measure.
     * @return Packet size in bytes.
     */
    [[nodiscard]] static std::size_t encodedSize(const ScanFrame& frame) noexcept
    {
        return static_cast<std::size_t>(kHeaderSizeV1) +
               static_cast<std::size_t>(pointCount(frame)) * kPointStrideV1;
    }

private:
    /**
     * @brief Number of points that will actually be emitted for @p frame.
     * @param frame Frame to inspect.
     * @return @c min(frame.size(), kMaxPointsPerFrame).
     */
    [[nodiscard]] static std::uint32_t pointCount(const ScanFrame& frame) noexcept
    {
        const std::size_t n = frame.points.size();
        return (n > kMaxPointsPerFrame) ? kMaxPointsPerFrame
                                        : static_cast<std::uint32_t>(n);
    }
};

/**
 * @brief Reassembles @ref aeb::ScanFrame objects from an arbitrary byte stream.
 *
 * TCP delivers a stream, not messages: a read may yield half a header or three
 * packets. This decoder therefore keeps an internal accumulation buffer and is
 * driven in a feed/drain loop:
 *
 * @code
 * decoder.feed(buf, n);
 * aeb::ScanFrame frame;
 * for (;;) {
 *     const auto st = decoder.decode(frame);
 *     if (st == DecodeStatus::NeedMoreData) break;
 *     if (st != DecodeStatus::Ok) { handleError(st); break; }
 *     consume(frame);
 * }
 * @endcode
 *
 * Single responsibility: framing and parsing. It performs no I/O and never
 * blocks. Instances are **not** thread-safe; give each connection its own.
 */
class ProtocolDecoder {
public:
    ProtocolDecoder() = default;

    /**
     * @brief Append freshly received bytes to the internal buffer.
     * @param data Pointer to @p size readable octets. Ignored if null.
     * @param size Number of octets to append.
     */
    void feed(const std::uint8_t* data, std::size_t size)
    {
        if (data == nullptr || size == 0U) {
            return;
        }
        buffer_.insert(buffer_.end(), data, data + size);
    }

    /**
     * @brief Attempt to extract one complete frame from the buffer.
     *
     * On @ref DecodeStatus::Ok the consumed bytes are removed from the buffer.
     * On @ref DecodeStatus::NeedMoreData nothing is consumed. On any other
     * status the stream is unrecoverable at this position; the caller must
     * either drop the connection or call @ref resynchronise.
     *
     * @param[out] out Receives the decoded frame on success; untouched
     *                 otherwise.
     * @return Decode status.
     */
    [[nodiscard]] DecodeStatus decode(ScanFrame& out)
    {
        if (buffer_.size() < kHeaderSizeV1) {
            return DecodeStatus::NeedMoreData;
        }

        const std::uint8_t* const h = buffer_.data();

        if (detail::getUint<4U>(h) != kMagic) {
            return DecodeStatus::BadMagic;
        }

        const auto version = static_cast<std::uint16_t>(detail::getUint<2U>(h + 4));
        if (version != kVersion) {
            return DecodeStatus::BadVersion;
        }

        const auto header_size  = static_cast<std::uint16_t>(detail::getUint<2U>(h + 6));
        const auto flags        = static_cast<std::uint32_t>(detail::getUint<4U>(h + 8));
        const auto sequence     = static_cast<std::uint32_t>(detail::getUint<4U>(h + 12));
        const auto timestamp_us = detail::getUint<8U>(h + 16);
        const auto point_count  = static_cast<std::uint32_t>(detail::getUint<4U>(h + 24));
        const auto point_stride = static_cast<std::uint16_t>(detail::getUint<2U>(h + 28));

        // Forward compatibility: a newer writer may grow the header or the
        // point record, but never shrink them below the version 1 baseline.
        if (header_size < kHeaderSizeV1 || header_size > kMaxHeaderSize ||
            point_stride < kPointStrideV1 || point_stride > kMaxPointStride) {
            return DecodeStatus::BadHeader;
        }
        if ((flags & ~kKnownFlagsMask) != 0U) {
            return DecodeStatus::BadHeader;
        }
        if (point_count > kMaxPointsPerFrame) {
            return DecodeStatus::TooLarge;
        }

        const std::size_t payload_size =
            static_cast<std::size_t>(point_count) * point_stride;
        const std::size_t trailer_size =
            ((flags & kFlagCrc32) != 0U) ? kCrcTrailerSize : 0U;
        const std::size_t packet_size =
            static_cast<std::size_t>(header_size) + payload_size + trailer_size;

        if (buffer_.size() < packet_size) {
            return DecodeStatus::NeedMoreData;
        }

        out.sequence     = sequence;
        out.timestamp_us = timestamp_us;
        out.points.clear();
        out.points.resize(point_count);

        const std::uint8_t* p = buffer_.data() + header_size;
        for (std::uint32_t i = 0U; i < point_count; ++i) {
            out.points[i].angle_deg   = detail::getFloat32(p);
            out.points[i].distance_mm = detail::getFloat32(p + 4);
            out.points[i].quality     = static_cast<std::uint8_t>(*(p + 8));
            p += point_stride;  // skips any fields added by a newer writer
        }

        // The CRC trailer is parsed for framing purposes only. Version 1
        // encoders never set kFlagCrc32, so no packet can currently reach this
        // path; validation is added together with the emitting encoder.
        buffer_.erase(buffer_.begin(),
                      buffer_.begin() + static_cast<std::ptrdiff_t>(packet_size));
        return DecodeStatus::Ok;
    }

    /**
     * @brief Discard bytes until the next possible packet start.
     *
     * Scans for the next occurrence of @ref kMagic and drops everything before
     * it. Used to recover from a corrupted stream without tearing down the
     * connection.
     *
     * @return @c true if a candidate packet start was found and retained.
     */
    bool resynchronise()
    {
        if (buffer_.size() < sizeof(kMagic)) {
            buffer_.clear();
            return false;
        }
        const std::size_t limit = buffer_.size() - sizeof(kMagic);
        for (std::size_t i = 1U; i <= limit; ++i) {
            if (detail::getUint<4U>(buffer_.data() + i) == kMagic) {
                buffer_.erase(buffer_.begin(),
                              buffer_.begin() + static_cast<std::ptrdiff_t>(i));
                return true;
            }
        }
        // Keep the last 3 bytes: a magic value may straddle the next read.
        buffer_.erase(buffer_.begin(),
                      buffer_.begin() +
                          static_cast<std::ptrdiff_t>(buffer_.size() - 3U));
        return false;
    }

    /**
     * @brief Drop all buffered bytes, e.g. when a client reconnects.
     */
    void reset() noexcept { buffer_.clear(); }

    /**
     * @brief Number of bytes currently held pending reassembly.
     */
    [[nodiscard]] std::size_t buffered() const noexcept { return buffer_.size(); }

private:
    /**
     * @brief Reassembly buffer holding one partial packet at most, plus any
     *        complete packets not yet drained by the caller.
     */
    std::vector<std::uint8_t> buffer_{};
};

}  // namespace aeb::proto

#endif  // AEB_COMMON_PROTOCOL_HPP
