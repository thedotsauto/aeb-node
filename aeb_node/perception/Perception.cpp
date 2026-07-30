#include "perception/Perception.hpp"

namespace aeb {

Perception::Perception() : Perception(PerceptionConfig{}) {}

Perception::Perception(const PerceptionConfig& config) : config_(config) {}

DetectionResult Perception::process(const ScanFrame& frame)
{
    DetectionResult result;

    result.timestamp = frame.timestamp_us;
    result.valid_points = static_cast<std::uint32_t>(frame.points.size());

    return result;
}

}  // namespace aeb