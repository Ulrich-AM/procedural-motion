#pragma once

#include "body/skeleton.h"

#include <array>

namespace proc {

// Source joint indices for the roles analysis measures.
// Built from a retarget mapping. -1 means that role was not mapped.
struct SourceSemanticMap {
  int root = -1;
  int left_hip = -1;
  int left_knee = -1;
  int left_foot = -1;
  int left_toe = -1;
  int right_hip = -1;
  int right_knee = -1;
  int right_foot = -1;
  int right_toe = -1;
};

inline int mappedSource(const std::array<int, kJointCount>& mapping, JointId joint) {
  const int index = jointIndex(joint);
  if (index < 0 || index >= kJointCount) return -1;
  return mapping[static_cast<std::size_t>(index)];
}

inline SourceSemanticMap semanticsFromMapping(const std::array<int, kJointCount>& mapping) {
  SourceSemanticMap semantics;
  semantics.root = mappedSource(mapping, JointId::LegRoot);
  semantics.left_hip = mappedSource(mapping, JointId::LeftHip);
  semantics.left_knee = mappedSource(mapping, JointId::LeftKnee);
  semantics.left_foot = mappedSource(mapping, JointId::LeftFoot);
  semantics.left_toe = mappedSource(mapping, JointId::LeftToe);
  semantics.right_hip = mappedSource(mapping, JointId::RightHip);
  semantics.right_knee = mappedSource(mapping, JointId::RightKnee);
  semantics.right_foot = mappedSource(mapping, JointId::RightFoot);
  semantics.right_toe = mappedSource(mapping, JointId::RightToe);
  return semantics;
}

}  // namespace proc
