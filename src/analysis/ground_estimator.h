#pragma once

#include "analysis/proc_analysis.h"

namespace proc {

GroundEstimate estimateGround(const std::vector<double>& height_m, const std::vector<double>& speed_mps,
                              const AnalysisSettings& settings);

}  // namespace proc
