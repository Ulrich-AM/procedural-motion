#pragma once

#include "analysis/proc_analysis.h"
#include "body/proc_body.h"
#include "locomotion/locomotion_model.h"

#include <string>
#include <vector>

namespace proc {

struct SyntheticClipSpec {
  std::string regime = "walk";
  double speed_mps = 1.2;
  double period_s = 1.0;
  double stance_fraction = 0.6;
  std::string subject = "synthetic";
};

// Repository-owned demonstration cycles. Knees flex on the negative side of each
// hinge axis. Arms hang down and swing opposite the same-side leg.
ProcAnalysis makeSyntheticLocomotionAnalysis(const ProcBody& body, const SyntheticClipSpec& spec);

std::vector<ProcAnalysis> makeSyntheticLocomotionSet(const ProcBody& body);

ProcLocomotionModel buildSyntheticLocomotionModel(const ProcBody& body, std::vector<std::string>& warnings);

}  // namespace proc
