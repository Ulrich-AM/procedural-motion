#include "locomotion/locomotion_audit.h"

#include "locomotion/locomotion_generator.h"

#include <sstream>

namespace proc {
namespace {

void auditRegime(const ProcBody& body, const ProcLocomotionModel& model, LocomotionRegime regime, const RegimeModel& data,
                 ConstraintAudit& audit) {
  if (!data.present) return;
  for (int knot = 0; knot < static_cast<int>(data.knots.size()); ++knot) {
    for (int sample = 0; sample < kLocoPhaseSamples; ++sample) {
      LocomotionState state;
      state.regime = regime;
      state.fade = 1.0;
      state.phase = static_cast<double>(sample) / static_cast<double>(kLocoPhaseSamples);
      state.dimensionless_speed = data.knots[static_cast<std::size_t>(knot)].dimensionless_speed;
      state.actual_speed_mps = 1.0;
      const LocomotionSample generated = generateLocomotionPose(body, model, state);
      const LimitResult limits = applyJointLimits(body, generated.pose, LimitMode::Warn);
      ++audit.poses;
      if (regime == LocomotionRegime::Walk) ++audit.walk_poses;
      else if (regime == LocomotionRegime::Run) ++audit.run_poses;
      else if (regime == LocomotionRegime::Sprint) ++audit.sprint_poses;
      if (!limits.violations.empty()) {
        ++audit.violating_poses;
        audit.ok = false;
        if (regime == LocomotionRegime::Walk) ++audit.walk_violations;
        else if (regime == LocomotionRegime::Run) ++audit.run_violations;
        else if (regime == LocomotionRegime::Sprint) ++audit.sprint_violations;
      }
      for (const JointViolation& violation : limits.violations) {
        ++audit.violations;
        if (violation.excess_deg >= audit.worst_excess_deg) {
          audit.worst_excess_deg = violation.excess_deg;
          audit.worst_joint = violation.joint;
          audit.worst_message = violation.message;
          audit.worst_regime = regimeName(regime);
          audit.worst_knot = knot;
          audit.worst_phase = state.phase;
        }
      }
    }
  }
}

}  // namespace

ConstraintAudit auditLocomotionModel(const ProcBody& body, const ProcLocomotionModel& model) {
  ConstraintAudit audit;
  auditRegime(body, model, LocomotionRegime::Walk, model.walk, audit);
  auditRegime(body, model, LocomotionRegime::Run, model.run, audit);
  auditRegime(body, model, LocomotionRegime::Sprint, model.sprint, audit);
  if (audit.worst_joint.empty()) audit.worst_joint = "none";
  return audit;
}

std::string formatConstraintAudit(const ConstraintAudit& audit, const std::string& body_name) {
  std::ostringstream text;
  text.setf(std::ios::fixed);
  text.precision(2);
  text << "MODEL CONSTRAINT AUDIT\nbody: " << body_name << "\n";
  text << "walk: " << audit.walk_poses << " samples\nviolating samples: " << audit.walk_violations << " / " << audit.walk_poses << "\n";
  text << "run: " << audit.run_poses << " samples\nviolating samples: " << audit.run_violations << " / " << audit.run_poses << "\n";
  text << "sprint: " << audit.sprint_poses << " samples\nviolating samples: " << audit.sprint_violations << " / " << audit.sprint_poses << "\n";
  text << "poses " << audit.poses << "\nposes with violations " << audit.violating_poses << "\n";
  text << "total joint violations " << audit.violations << "\n";
  text << "worst joint: " << (audit.worst_joint.empty() ? "none" : audit.worst_joint) << "\n";
  if (audit.violations > 0) {
    text << "worst excess " << audit.worst_excess_deg << " deg\nregime " << audit.worst_regime << " knot " << audit.worst_knot
         << " phase " << audit.worst_phase << "\n" << audit.worst_message << "\n";
  }
  text << (audit.ok ? "PASS\n" : "WARN\n");
  return text.str();
}

}  // namespace proc
