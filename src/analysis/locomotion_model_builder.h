#pragma once

#include "analysis/proc_analysis.h"
#include "locomotion/locomotion_model.h"

#include <optional>
#include <string>
#include <vector>

namespace proc {

// Authoring input for one analyzed cycle. Empty regime and speed use the analysis values.
// The analysis object is not modified.
struct ReferenceCycleInput {
  const ProcAnalysis* analysis = nullptr;
  int cycle_index = -1;
  std::string regime;
  std::optional<double> speed_override_mps;
};

struct ReferenceFidelity {
  bool available = false;
  int cycles = 0;
  double hip_rms = 0.0;
  double knee_rms = 0.0;
  double ankle_rms = 0.0;
  double pelvis_rms = 0.0;
  double shoulder_rms = 0.0;
  double elbow_rms = 0.0;
  double stance_difference = 0.0;
  double flight_difference = 0.0;
  double toe_off_phase_difference = 0.0;
  double initial_contact_phase_difference = 0.0;
};

ProcLocomotionModel buildLocomotionModel(const std::vector<ProcAnalysis>& analyses, const ModelBuildSettings& settings,
                                         std::vector<std::string>& warnings);

ProcLocomotionModel buildLocomotionModel(const std::vector<ReferenceCycleInput>& inputs, const ModelBuildSettings& settings,
                                         std::vector<std::string>& warnings);

ReferenceFidelity measureReferenceFidelity(const ProcLocomotionModel& model, const std::vector<ReferenceCycleInput>& inputs,
                                           const ModelBuildSettings& settings);

}  // namespace proc
