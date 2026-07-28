/**
 * @file RpLidarDevice.hpp
 * @brief RAII ownership of the Slamtec driver and its serial channel.
 *
 * This class is the hardware boundary. It answers exactly one question - "how
 * do I obtain one revolution of measurements from an RPLidar C1?" - and knows
 * nothing about threads, retry policy, sequence numbers or timestamps. Those
 * belong to @ref aeb::Lidar.
 *
 * The separation matters beyond file size:
 *
 *  - the SDK's raw-pointer ownership model and its teardown ordering rules are
 *    contained in one small, auditable unit;
 *  - the supervision policy in @ref aeb::Lidar can be reviewed without reading
 *    any vendor code;
 *  - a replay or simulation device implementing the same three operations can
 *    be substituted in Phase 2.
 *
 * Internal header of the lidar layer; application code uses @ref aeb::Lidar.
 */

#ifndef AEB_LIDAR_RPLIDARDEVICE_HPP
#define AEB_LIDAR_RPLIDARDEVICE_HPP

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include "common/Types.hpp"
#include "lidar/Lidar.hpp"

// Forward declarations keep the Slamtec headers out of this header; only
// RpLidarDevice.cpp includes them.
namespace sl {
class ILidarDriver;
class IChannel;
}  // namespace sl

namespace aeb::hw {

/**
 * @brief Opaque holder for the SDK measurement buffer.
 *
 * Defined in RpLidarDevice.cpp. Using an opaque type rather than naming the
 * SDK's packed node structure here guarantees that no translation unit can see
 * a differently attributed declaration of it.
 */
struct NodeBuffer;

/**
 * @brief Owns one connected RPLidar C1 and yields complete revolutions.
 *
 * Lifetime is RAII: @ref open acquires the channel and driver, @ref close and
 * the destructor release them in the reverse order the SDK requires. Every
 * failure path inside @ref open releases whatever it had already acquired, so
 * the object is always either fully open or fully closed - never in between.
 *
 * Non-copyable and non-movable: it owns raw SDK pointers whose lifetime must
 * not be duplicated or relocated.
 *
 * Not thread-safe. @ref aeb::Lidar provides the necessary serialisation.
 */
class RpLidarDevice {
public:
    /**
     * @brief Construct a closed device and size the measurement buffer.
     * @param config Serial port, baud rate, timeout and buffer sizing.
     */
    explicit RpLidarDevice(const LidarConfig& config);

    /** @brief Closes the device if it is open. */
    ~RpLidarDevice();

    RpLidarDevice(const RpLidarDevice&) = delete;
    RpLidarDevice& operator=(const RpLidarDevice&) = delete;
    RpLidarDevice(RpLidarDevice&&) = delete;
    RpLidarDevice& operator=(RpLidarDevice&&) = delete;

    /**
     * @brief Open the serial channel, connect, verify and start scanning.
     *
     * Performs the full bring-up sequence: create channel, create driver,
     * connect, read device info (which proves the link is genuinely alive
     * rather than merely open), start the motor and start the scan.
     *
     * @param[out] error Receives a diagnostic message on failure.
     * @return @c true if the device is scanning.
     */
    [[nodiscard]] bool open(std::string& error);

    /**
     * @brief Stop the motor and release the driver and channel.
     *
     * Best effort and @c noexcept: the device may already be physically
     * unplugged, in which case the stop commands fail harmlessly and teardown
     * continues regardless. Safe to call when already closed.
     */
    void close() noexcept;

    /** @brief Whether the device is currently open. */
    [[nodiscard]] bool isOpen() const noexcept { return driver_ != nullptr; }

    /**
     * @brief Block until one complete revolution is available, then convert it.
     *
     * Measurements are sorted by bearing and converted from the SDK's
     * fixed-point representation into @ref aeb::ScanPoint. Points reporting no
     * return are discarded here rather than forwarded as range-zero readings.
     *
     * @param[out] points Receives the revolution. Cleared first; its existing
     *                    capacity is reused, so a caller that reuses one vector
     *                    performs no steady-state allocation.
     * @param[out] error  Receives a diagnostic message on failure.
     * @return @c true on success; @c false on timeout or driver error, after
     *         which the caller should @ref close and retry.
     */
    [[nodiscard]] bool grabScan(std::vector<ScanPoint>& points, std::string& error);

private:
    /** @brief Connection and buffer sizing parameters. */
    LidarConfig config_;

    /** @brief SDK driver instance; null while closed. */
    sl::ILidarDriver* driver_{nullptr};

    /** @brief SDK serial channel; null while closed. Outlived by the driver. */
    sl::IChannel* channel_{nullptr};

    /** @brief Reusable SDK node buffer, allocated once at construction. */
    std::unique_ptr<NodeBuffer> nodes_;
};

}  // namespace aeb::hw

#endif  // AEB_LIDAR_RPLIDARDEVICE_HPP
