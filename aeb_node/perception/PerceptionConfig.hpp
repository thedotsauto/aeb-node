#pragma once

#include <cstdint>

namespace aeb {

/**
 * @brief Tunable thresholds for @ref Perception.
 */
struct PerceptionConfig {
    /** @brief Lower bound of the field of view considered, in degrees. */
    float min_angle_deg = -80.0f;

    /** @brief Upper bound of the field of view considered, in degrees. */
    float max_angle_deg = 80.0f;

    /** @brief Points beyond this range are ignored, in millimetres. */
    float max_distance_mm = 4000.0f;

    /** @brief Points reported below this quality are ignored. */
    std::uint8_t minimum_quality = 15;
};

}  // namespace aeb