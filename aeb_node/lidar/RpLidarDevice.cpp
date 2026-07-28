/**
 * @file RpLidarDevice.cpp
 * @brief Slamtec SDK integration: the only file that includes vendor headers.
 */

#include "lidar/RpLidarDevice.hpp"

#include <utility>

#include "sl_lidar.h"
#include "sl_lidar_driver.h"

namespace aeb::hw {
namespace {

/**
 * @brief Convert a Q14 fixed-point angle from the SDK to degrees.
 * @param angle_z_q14 Angle in Q14 format, where 16384 corresponds to 90 degrees.
 * @return Bearing in degrees.
 */
[[nodiscard]] float toDegrees(std::uint16_t angle_z_q14) noexcept
{
    return static_cast<float>(angle_z_q14) * 90.0F / 16384.0F;
}

/**
 * @brief Convert a Q2 fixed-point distance from the SDK to millimetres.
 * @param dist_mm_q2 Distance in quarter-millimetre units.
 * @return Distance in millimetres; 0.0F denotes no return.
 */
[[nodiscard]] float toMillimetres(std::uint32_t dist_mm_q2) noexcept
{
    return static_cast<float>(dist_mm_q2) / 4.0F;
}

/**
 * @brief Render a Slamtec result code as a short diagnostic string.
 * @param result SDK result code.
 * @return Human-readable description.
 */
[[nodiscard]] const char* describe(sl_result result) noexcept
{
    switch (result) {
        case SL_RESULT_OK:                    return "ok";
        case SL_RESULT_ALREADY_DONE:          return "already done";
        case SL_RESULT_INVALID_DATA:          return "invalid data";
        case SL_RESULT_OPERATION_FAIL:        return "operation failed";
        case SL_RESULT_OPERATION_TIMEOUT:     return "operation timed out";
        case SL_RESULT_OPERATION_STOP:        return "operation stopped";
        case SL_RESULT_OPERATION_NOT_SUPPORT: return "operation not supported";
        case SL_RESULT_FORMAT_NOT_SUPPORT:    return "format not supported";
        case SL_RESULT_INSUFFICIENT_MEMORY:   return "insufficient memory";
        default:                              return "unknown error";
    }
}

}  // namespace

/**
 * @brief Concrete definition of the opaque measurement buffer.
 */
struct NodeBuffer {
    /** @brief Storage for one revolution of raw SDK measurements. */
    std::vector<sl_lidar_response_measurement_node_hq_t> nodes;
};

RpLidarDevice::RpLidarDevice(const LidarConfig& config)
    : config_{config}, nodes_{std::make_unique<NodeBuffer>()}
{
    nodes_->nodes.resize(config_.max_nodes_per_scan);
}

RpLidarDevice::~RpLidarDevice()
{
    close();
}

bool RpLidarDevice::open(std::string& error)
{
    close();

    auto channel_result =
        sl::createSerialPortChannel(config_.serial_port, static_cast<int>(config_.baudrate));
    if (!channel_result) {
        error = "failed to create serial channel on " + config_.serial_port;
        return false;
    }
    channel_ = *channel_result;

    auto driver_result = sl::createLidarDriver();
    if (!driver_result) {
        error = "failed to create lidar driver";
        close();
        return false;
    }
    driver_ = *driver_result;

    sl_result result = driver_->connect(channel_);
    if (!SL_IS_OK(result)) {
        error = "connect to " + config_.serial_port + " failed: " + describe(result);
        close();
        return false;
    }

    // Reading device info proves the link is genuinely alive, not merely open:
    // a powered-off sensor on a live USB adapter opens successfully but never
    // answers.
    sl_lidar_response_device_info_t info{};
    result = driver_->getDeviceInfo(info);
    if (!SL_IS_OK(result)) {
        error = std::string{"getDeviceInfo failed: "} + describe(result);
        close();
        return false;
    }

    // Some C1 firmware has no separately controlled motor; that is not a fault.
    result = driver_->setMotorSpeed();
    if (!SL_IS_OK(result) && result != SL_RESULT_OPERATION_NOT_SUPPORT) {
        error = std::string{"setMotorSpeed failed: "} + describe(result);
        close();
        return false;
    }

    result = driver_->startScan(false /*force*/, true /*use typical scan mode*/);
    if (!SL_IS_OK(result)) {
        error = std::string{"startScan failed: "} + describe(result);
        close();
        return false;
    }

    error.clear();
    return true;
}

void RpLidarDevice::close() noexcept
{
    if (driver_ != nullptr) {
        (void)driver_->stop();
        (void)driver_->setMotorSpeed(0);
        delete driver_;
        driver_ = nullptr;
    }
    if (channel_ != nullptr) {
        // Destroyed after the driver, which holds a reference to it.
        delete channel_;
        channel_ = nullptr;
    }
}

bool RpLidarDevice::grabScan(std::vector<ScanPoint>& points, std::string& error)
{
    if (driver_ == nullptr) {
        error = "grabScan called on a closed device";
        return false;
    }

    std::size_t count = nodes_->nodes.size();
    const sl_result result =
        driver_->grabScanDataHq(nodes_->nodes.data(), count, config_.scan_timeout_ms);
    if (!SL_IS_OK(result)) {
        error = std::string{"grabScanDataHq failed: "} + describe(result);
        return false;
    }

    // Sorting by bearing is cheap here and gives every downstream consumer a
    // canonical ordering, which simplifies both rendering and sector-based
    // obstacle detection.
    (void)driver_->ascendScanData(nodes_->nodes.data(), count);

    points.clear();
    for (std::size_t i = 0U; i < count; ++i) {
        const auto& node = nodes_->nodes[i];
        const float distance_mm = toMillimetres(node.dist_mm_q2);
        if (distance_mm <= 0.0F) {
            continue;  // "No return" marker: never forward as a range of zero.
        }
        ScanPoint point;
        point.angle_deg = toDegrees(node.angle_z_q14);
        point.distance_mm = distance_mm;
        point.quality = node.quality;
        points.push_back(point);
    }

    error.clear();
    return true;
}

}  // namespace aeb::hw
