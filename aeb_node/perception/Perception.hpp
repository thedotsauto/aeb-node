#pragma once

#include "common/Types.hpp"
#include "perception/DetectionResult.hpp"
#include "perception/PerceptionConfig.hpp"

namespace aeb {

/**
 * @brief Runs on every acquired @ref ScanFrame, ahead of network publish.
 *
 * First version only counts valid points; filtering by angle, range and
 * quality using @ref PerceptionConfig is not yet implemented.
 */
class Perception {
public:
    /** @brief Construct with default thresholds. */
    Perception();

    /**
     * @brief Construct with explicit thresholds.
     * @param config Tunable thresholds, copied into the instance.
     */
    explicit Perception(const PerceptionConfig& config);

    /**
     * @brief Process one frame.
     * @param frame Frame to inspect. Not modified.
     * @return Detection result for this frame.
     */
    DetectionResult process(const ScanFrame& frame);

private:
    /** @brief Thresholds for future filtering; unused in this version. */
    PerceptionConfig config_;
};

}  // namespace aeb