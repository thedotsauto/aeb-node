#include "perception/Perception.hpp"

namespace aeb {

Perception::Perception() : Perception(PerceptionConfig{}) {}

Perception::Perception(const PerceptionConfig& config) : config_(config) {}

DetectionResult Perception::process(const ScanFrame& frame)
{
    DetectionResult result;
    result.timestamp = frame.timestamp_us;
    result.total_points = static_cast<std::uint32_t>(frame.points.size());

    float min_distance = -1.0f;
    float min_angle = 0.0f;
    bool found_any = false;

    for (const auto& point : frame.points) {
        // 1. Quality and basic validity check
        if (point.distance_mm <= 0.0F || point.quality < config_.minimum_quality) {
            continue;
        }

        // 2. Angle filter: normalize angle to [-180, 180) first
        float norm_angle = point.angle_deg;
        if (norm_angle > 180.0F) {
            norm_angle -= 360.0F;
        }

        if (norm_angle < config_.min_angle_deg || norm_angle > config_.max_angle_deg) {
            continue;
        }
        result.angle_filtered_points++;

        // 3. Distance filter
        if (point.distance_mm > config_.max_distance_mm) {
            continue;
        }
        result.distance_filtered_points++;

        // 4. Find nearest point
        if (!found_any || point.distance_mm < min_distance) {
            min_distance = point.distance_mm;
            min_angle = norm_angle;
            found_any = true;
        }
    }

    if (found_any) {
        result.obstacle_detected = true;
        result.nearest_distance_mm = min_distance;
        result.nearest_angle_deg = min_angle;
    } else {
        result.obstacle_detected = false;
        result.nearest_distance_mm = -1.0F;
        result.nearest_angle_deg = 0.0F;
    }

    return result;
}

}  // namespace aeb