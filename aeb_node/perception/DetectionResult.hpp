#pragma once

#include <cstdint>

namespace aeb {

/**
 * @brief Output of one @ref Perception::process call.
 *
 * First version reports only the valid point count; obstacle detection
 * fields are reserved for a later phase and are not yet populated.
 */
/**
 * @brief Obstacle detection details for a single field-of-view sector.
 */
struct SectorResult {
    /** @brief True if an obstacle is detected in this sector. */
    bool obstacle_detected = false;

    /** @brief Distance to the nearest obstacle in millimetres, or -1.0f if none. */
    float nearest_distance_mm = -1.0f;

    /** @brief Angle to the nearest obstacle in degrees, or 0.0f if none. */
    float nearest_angle_deg = 0.0f;
};

/**
 * @brief Output of one @ref Perception::process call, describing the environment.
 */
struct DetectionResult {
    /** @brief Left sector result (1st fifth of FOV). */
    SectorResult left;

    /** @brief Left-Center sector result (2nd fifth of FOV). */
    SectorResult left_center;

    /** @brief Center sector result (3rd fifth of FOV). */
    SectorResult center;

    /** @brief Right-Center sector result (4th fifth of FOV). */
    SectorResult right_center;

    /** @brief Right sector result (5th fifth of FOV). */
    SectorResult right;

    /** @brief Total raw points in the source ScanFrame. */
    std::uint32_t total_points = 0;

    /** @brief Points remaining after angle filtering. */
    std::uint32_t angle_filtered_points = 0;

    /** @brief Points remaining after angle and distance filtering. */
    std::uint32_t distance_filtered_points = 0;

    /** @brief Points remaining after angle, distance, and quality filtering. */
    std::uint32_t quality_filtered_points = 0;

    /** @brief Timestamp copied from the source @c ScanFrame. */
    std::uint64_t timestamp = 0;
};

}  // namespace aeb