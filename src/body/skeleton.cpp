#include "body/skeleton.h"

namespace proc {

std::optional<int> findJoint(std::string_view name) {
  for (int index = 0; index < kJointCount; ++index) {
    if (name == kJointNames[index]) return index;
  }
  return std::nullopt;
}

std::optional<int> findBone(std::string_view name) {
  for (int index = 0; index < kBoneCount; ++index) {
    if (name == kCanonicalBones[index].name) return index;
  }
  return std::nullopt;
}

bool jointIsEndpoint(int index) {
  for (const CanonicalBone& bone : kCanonicalBones) {
    if (bone.parent == index) return false;
  }
  return true;
}

}  // namespace proc
