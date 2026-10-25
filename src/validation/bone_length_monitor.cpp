#include "validation/bone_length_monitor.h"

#include "validation/tolerances.h"

#include <glm/geometric.hpp>

#include <cmath>

namespace proc {

BoneLengthReport measureBoneLengths(const ProcBody& body, const WorldPose& world, double tolerance_m) {
  if (tolerance_m <= 0.0) tolerance_m = kRuntimeBoneToleranceM;
  BoneLengthReport report;
  report.samples.reserve(kBoneCount);
  for (int index = 0; index < kBoneCount; ++index) {
    const BoneGeom& bone = body.bones[index];
    BoneLengthSample sample;
    sample.bone = bone.name;
    sample.expected_m = bone.length_m;
    if (bone.parent_joint >= 0 && bone.child_joint >= 0) {
      sample.actual_m = glm::length(world.joints[bone.child_joint].position - world.joints[bone.parent_joint].position);
    }
    sample.error_m = std::abs(sample.actual_m - sample.expected_m);
    if (sample.error_m > report.max_error_m) {
      report.max_error_m = sample.error_m;
      report.worst_index = index;
    }
    report.samples.push_back(sample);
  }
  report.ok = report.max_error_m <= tolerance_m;
  return report;
}

}  // namespace proc
