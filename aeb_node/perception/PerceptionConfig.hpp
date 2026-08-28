#pragma once

#include <cstdint>

namespace aeb {

/**
 * @brief Tunable thresholds for @ref Perception.
 */
struct PerceptionConfig {
    /** @brief Lower bound of the field of view considered, in degrees. */
    float min_angle_deg = -45.0f;

    /** @brief Upper bound of the field of view considered, in degrees. */
    float max_angle_deg = 45.0f;

    /** @brief Points beyond this range are ignored, in millimetres. */
    float max_distance_mm = 2000.0f;

    /** @brief Points reported below this quality are ignored. */
    std::uint8_t minimum_quality = 15;

    /**
     * @brief Minimum number of quality-passing points that must fall inside a
     * sector before it is declared as containing an obstacle.
     *
     * Raising this above 1 suppresses single-point false positives caused by
     * vehicle body reflections or random noise returns.
     */
    std::uint32_t min_hits_per_sector = 3U;
};

}  // namespace aeb