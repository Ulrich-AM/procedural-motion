#pragma once

#include "analysis/proc_analysis.h"

#include <cstdint>
#include <vector>

namespace proc {

struct ContactObservation {
  std::vector<double> confidence;
  std::vector<std::uint8_t> contact;
};

std::vector<std::uint8_t> applyHysteresis(const std::vector<double>& confidence, double enter, double exit);

ContactObservation detectMarkerContact(const std::vector<double>& time, const std::vector<double>& height_above_ground,
                                       const std::vector<double>& horizontal_speed, const std::vector<double>& vertical_speed,
                                       const AnalysisSettings& settings);

}  // namespace proc
