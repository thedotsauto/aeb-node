#pragma once

#include <cstdint>

namespace aeb {

/**
 * @brief Output of one @ref Perception::process call.
 *
 * First version reports only the valid point count; obstacle detection
 * fields are reserved for a later phase and are not yet populated.
 */
struct DetectionResult {
    /** @brief True if an obstacle is detected in the monitored zone. */
    bool obstacle_detected = false;

    /** @brief Distance to the nearest obstacle in millimetres, or -1.0f if none. */
    float nearest_distance_mm = -1.0f;

    /** @brief Angle to the nearest obstacle in degrees, or 0.0f if none. */
    float nearest_angle_deg = 0.0f;

    /** @brief Total raw points in the source ScanFrame. */
    std::uint32_t total_points = 0;

    /** @brief Points remaining after quality and angle filtering. */
    std::uint32_t angle_filtered_points = 0;

    /** @brief Points remaining after quality, angle, and distance filtering. */
    std::uint32_t distance_filtered_points = 0;

    /** @brief Timestamp copied from the source @c ScanFrame. */
    std::uint64_t timestamp = 0;
};

}  // namespace aeb