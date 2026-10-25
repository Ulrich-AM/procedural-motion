#include "rig/forward_kinematics.h"

#include <glm/gtc/quaternion.hpp>

namespace proc {

WorldPose forwardKinematics(const ProcBody& body, const Pose& pose) {
  WorldPose world;
  world.joints[kRootJoint].position = body.joints[kRootJoint].rest_position + pose.root_translation;
  world.joints[kRootJoint].rotation = glm::normalize(pose.root_rotation);

  for (int index = 0; index < kJointCount; ++index) {
    const int parent = body.joints[index].parent;
    if (parent < 0) continue;
    const int bone_index = body.joints[index].incoming_bone;
    const glm::dvec3 rest = bone_index >= 0 ? body.bones[bone_index].rest_vector : glm::dvec3(0.0);
    const glm::dquat parent_rotation = world.joints[parent].rotation;
    world.joints[index].position = world.joints[parent].position + parent_rotation * rest;
    world.joints[index].rotation = glm::normalize(parent_rotation * pose.local_rotation[index]);
  }
  return world;
}

}  // namespace proc
