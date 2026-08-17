#include "perception/Perception.hpp"

namespace aeb {

Perception::Perception() : Perception(PerceptionConfig{}) {}

Perception::Perception(const PerceptionConfig& config) : config_(config) {}

DetectionResult Perception::process(const ScanFrame& frame)
{
    DetectionResult result;
    result.timestamp = frame.timestamp_us;
    result.total_points = static_cast<std::uint32_t>(frame.points.size());

    // Compute sector width dynamically based on FOV configuration
    const float fov_span = config_.max_angle_deg - config_.min_angle_deg;
    const float sector_width = fov_span / 5.0F;

    // Direct pointers to result sectors for clean indexed mapping
    SectorResult* sectors[5] = {
        &result.left,
        &result.left_center,
        &result.center,
        &result.right_center,
        &result.right
    };

    for (const auto& point : frame.points) {
        // Discard points with zero/negative distance as they are "no return" (invalid)
        if (point.distance_mm <= 0.0F) {
            continue;
        }

        // 1. Angle Filter: normalize angle to [-180, 180) first
        float norm_angle = point.angle_deg;
        if (norm_angle > 180.0F) {
            norm_angle -= 360.0F;
        }

        if (norm_angle < config_.min_angle_deg || norm_angle > config_.max_angle_deg) {
            continue;
        }
        result.angle_filtered_points++;

        // 2. Distance Filter
        if (point.distance_mm > config_.max_distance_mm) {
            continue;
        }
        result.distance_filtered_points++;

        // 3. Quality Filter
        if (point.quality < config_.minimum_quality) {
            continue;
        }
        result.quality_filtered_points++;

        // 4. Sectorization
        float relative_angle = norm_angle - config_.min_angle_deg;
        int sector_idx = static_cast<int>(relative_angle / sector_width);
        if (sector_idx < 0) {
            sector_idx = 0;
        } else if (sector_idx >= 5) {
            sector_idx = 4;
        }

        // Track nearest obstacle in this sector
        SectorResult& sector = *sectors[sector_idx];
        if (!sector.obstacle_detected || point.distance_mm < sector.nearest_distance_mm) {
            sector.obstacle_detected = true;
            sector.nearest_distance_mm = point.distance_mm;
            sector.nearest_angle_deg = norm_angle;
        }
    }

    return result;
}

}  // namespace aeb