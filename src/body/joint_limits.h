#pragma once

#include "body/proc_body.h"
#include "motion/proc_motion.h"

#include <string>
#include <vector>

namespace proc {

enum class LimitMode { Warn, Clamp };

struct JointViolation {
  std::string joint;
  std::string message;
  double requested_deg = 0.0;
  double excess_deg = 0.0;
};

// Signed hinge angle about the constraint's own axis. Positive and negative
// follow that axis. They are not a second, side-specific sign convention.
double hingeAngleDegrees(const glm::dquat& rotation, const glm::dvec3& axis);

struct LimitResult {
  Pose pose;
  std::vector<JointViolation> violations;
  bool clamped = false;
};

// WARN reports violations and leaves the requested rotation untouched.
// CLAMP rewrites only the runtime pose. The motion asset is not modified.
LimitResult applyJointLimits(const ProcBody& body, const Pose& pose, LimitMode mode);

}  // namespace proc
