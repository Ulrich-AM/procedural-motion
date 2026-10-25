#pragma once

#include "body/proc_body.h"
#include "motion/proc_motion.h"

#include <array>

namespace proc {

struct WorldJoint {
  glm::dvec3 position{0.0, 0.0, 0.0};
  glm::dquat rotation{1.0, 0.0, 0.0, 0.0};
};

struct WorldPose {
  std::array<WorldJoint, kJointCount> joints{};
};

WorldPose forwardKinematics(const ProcBody& body, const Pose& pose);

}  // namespace proc
