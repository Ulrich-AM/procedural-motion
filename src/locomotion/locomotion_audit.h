#pragma once

#include "body/proc_body.h"
#include "locomotion/locomotion_model.h"

#include <string>

namespace proc {

struct ConstraintAudit {
  int poses = 0;
  int violating_poses = 0;
  int violations = 0;
  int walk_poses = 0;
  int run_poses = 0;
  int sprint_poses = 0;
  int walk_violations = 0;
  int run_violations = 0;
  int sprint_violations = 0;
  std::string worst_joint;
  double worst_excess_deg = 0.0;
  std::string worst_message;
  std::string worst_regime;
  int worst_knot = -1;
  double worst_phase = 0.0;
  bool ok = true;
};

// Samples every regime, speed knot, and phase. Does not modify the model or the body.
ConstraintAudit auditLocomotionModel(const ProcBody& body, const ProcLocomotionModel& model);

std::string formatConstraintAudit(const ConstraintAudit& audit, const std::string& body_name);

}  // namespace proc
