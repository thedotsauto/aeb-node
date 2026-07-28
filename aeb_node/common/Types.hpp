/**
 * @file Types.hpp
 * @brief Core domain value types shared by every layer of the AEB node.
 *
 * This header defines the *domain model* of the system: the data that the
 * sensing, perception and actuation layers agree upon. It is deliberately free
 * of any dependency on the Slamtec SDK, on POSIX sockets, or on the wire
 * protocol, so that:
 *
 *  - the perception / braking logic can be unit tested without hardware,
 *  - the networking layer can be removed entirely without touching safety code,
 *  - a replay source can produce the exact same types as the live sensor.
 *
 * @note Nothing in this header allocates outside of @ref aeb::ScanFrame's
 *       std::vector, and no type here owns an OS resource.
 */

#ifndef AEB_COMMON_TYPES_HPP
#define AEB_COMMON_TYPES_HPP

#include <cstdint>
#include <vector>

namespace aeb {

/**
 * @brief Monotonic timestamp in microseconds.
 *
 * Always sourced from a steady (monotonic) clock, never from wall time, so that
 * time-to-collision math cannot be corrupted by NTP steps or DST changes.
 */
using TimestampUs = std::uint64_t;

/**
 * @brief Monotonically increasing frame counter.
 *
 * Wraps at 2^32. Consumers must compare sequence numbers using modular
 * arithmetic (i.e. @c static_cast<std::int32_t>(a - b)) rather than @c < .
 */
using SequenceNumber = std::uint32_t;

/**
 * @brief A single lidar measurement in the sensor's polar frame.
 *
 * Layout is intentionally simple and trivially copyable. This struct is a
 * *domain* type; it is never written to a socket or to disk directly. See
 * ProtocolEncoder for the explicit, endian-defined wire representation.
 */
struct ScanPoint {
    /**
     * @brief Bearing of the measurement in degrees, range [0.0, 360.0).
     *
     * Measured clockwise from the sensor's zero mark, as reported by the
     * RPLidar C1.
     */
    float angle_deg{0.0F};

    /**
     * @brief Radial distance to the reflecting surface in millimetres.
     *
     * A value of @c 0.0F means "no return" (invalid measurement) and must be
     * discarded by consumers rather than treated as an obstacle at range zero.
     * @see ScanPoint::isValid
     */
    float distance_mm{0.0F};

    /**
     * @brief Vendor-reported reflectivity/confidence, range [0, 255].
     *
     * Higher is better. A quality of 0 accompanies invalid measurements.
     */
    std::uint8_t quality{0U};

    /**
     * @brief Test whether this measurement carries a usable range reading.
     * @return @c true if the point has a non-zero distance and non-zero quality.
     */
    [[nodiscard]] constexpr bool isValid() const noexcept
    {
        return distance_mm > 0.0F && quality > 0U;
    }
};

static_assert(sizeof(float) == 4, "AEB node requires 32-bit IEEE-754 float");

/**
 * @brief One complete 360-degree revolution of lidar measurements.
 *
 * A ScanFrame is the unit of work that flows through the whole pipeline:
 * @c Lidar produces it, the obstacle detector consumes it, and (for development
 * only) the TcpServer serialises it. It owns its points; copies are deep and
 * moves are cheap.
 *
 * @note Frames are self-describing: a consumer never needs out-of-band context
 *       to interpret one. This is what makes record/replay (Phase 2) lossless.
 */
struct ScanFrame {
    /**
     * @brief Sequence number assigned by the producer, incremented per frame.
     *
     * Gaps indicate dropped frames and are the primary health signal for the
     * acquisition thread.
     */
    SequenceNumber sequence{0U};

    /**
     * @brief Monotonic acquisition timestamp of the *end* of the revolution.
     *
     * Using a single, consistently chosen instant per frame keeps the velocity
     * estimate used by time-to-collision unbiased.
     */
    TimestampUs timestamp_us{0U};

    /**
     * @brief The measurements of this revolution, in acquisition order.
     *
     * May contain invalid points; filtering is a perception concern, not an
     * acquisition concern, so the raw data is preserved for recording.
     */
    std::vector<ScanPoint> points{};

    /**
     * @brief Number of measurements in this frame.
     */
    [[nodiscard]] std::size_t size() const noexcept { return points.size(); }

    /**
     * @brief Test whether the frame carries no measurements at all.
     */
    [[nodiscard]] bool empty() const noexcept { return points.empty(); }
};

}  // namespace aeb

#endif  // AEB_COMMON_TYPES_HPP
