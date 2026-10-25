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
  float shade = 1.0f;
};

void drawGroundGrid(Renderer& renderer);
void drawSkeleton(Renderer& renderer, const ProcBody& body, const WorldPose& world, const SkeletonDrawOptions& options);
void drawSourceChain(Renderer& renderer, const std::vector<glm::dvec3>& positions, const std::vector<int>& parents,
                     const glm::vec3& color, float radius);

}  // namespace proc
