#pragma once

#include "render/renderer.h"
#include "rig/forward_kinematics.h"
#include "motion/proc_motion.h"

#include <vector>

namespace proc {

struct SkeletonDrawOptions {
  const ContactSample* contacts = nullptr;
  const std::vector<glm::vec3>* trajectory = nullptr;
  bool draw_facing_marker = true;
};

void drawGroundGrid(Renderer& renderer);
void drawSkeleton(Renderer& renderer, const ProcBody& body, const WorldPose& world, const SkeletonDrawOptions& options);

}  // namespace proc
