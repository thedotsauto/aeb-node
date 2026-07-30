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
    /** @brief Reserved for a future obstacle-detection phase; not yet set. */
    bool obstacle_detected = false;

    /** @brief Reserved for a future obstacle-detection phase; not yet set. */
    float nearest_distance_mm = -1.0f;

    /** @brief Reserved for a future obstacle-detection phase; not yet set. */
    float nearest_angle_deg = 0.0f;

    /** @brief Number of scan points counted by @ref Perception::process. */
    std::uint32_t valid_points = 0;

    /** @brief Timestamp copied from the source @c ScanFrame. */
    std::uint64_t timestamp = 0;
};

}  // namespace aeb