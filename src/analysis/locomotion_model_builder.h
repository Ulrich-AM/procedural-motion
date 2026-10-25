#pragma once

#include "analysis/proc_analysis.h"
#include "locomotion/locomotion_model.h"

#include <vector>

namespace proc {

ProcLocomotionModel buildLocomotionModel(const std::vector<ProcAnalysis>& analyses, const ModelBuildSettings& settings,
                                         std::vector<std::string>& warnings);

}  // namespace proc
