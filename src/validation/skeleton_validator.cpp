#include "validation/skeleton_validator.h"

#include "validation/tolerances.h"

#include <glm/geometric.hpp>

#include <cmath>
#include <sstream>
#include <string>
#include <vector>

namespace proc {
namespace {

std::string quoteName(std::string_view text) { return "\"" + std::string(text) + "\""; }

bool finite3(const glm::dvec3& value) {
  return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

}  // namespace

void validateProcBody(const ProcBody& body, Status& status) {
  if (body.skeleton != kSkeletonId) {
    status.error("skeleton is " + quoteName(body.skeleton) + ", expected " + quoteName(kSkeletonId));
  }

  for (int index = 0; index < kJointCount; ++index) {
    const JointGeom& joint = body.joints[index];
    if (joint.name != kJointNames[index]) {
      status.error("missing joint " + quoteName(kJointNames[index]));
      continue;
    }
    if (!finite3(joint.rest_position)) {
      status.error("joint " + quoteName(joint.name) + " rest position is not finite");
    }
  }

  for (int index = 0; index < kBoneCount; ++index) {
    const BoneGeom& bone = body.bones[index];
    const CanonicalBone& expected = kCanonicalBones[index];
    if (bone.name != expected.name) {
      status.error("missing bone " + quoteName(expected.name));
      continue;
    }
    if (bone.parent_joint != expected.parent || bone.child_joint != expected.child) {
      status.error("invalid hierarchy: bone " + quoteName(bone.name) + " connects the wrong joints");
      continue;
    }
    if (!bone.locked) {
      status.error("bone " + quoteName(bone.name) + " must be locked; motion cannot resize it");
    }
    if (!std::isfinite(bone.length_m) || bone.length_m <= 1e-6) {
      status.error("bone " + quoteName(bone.name) + " has a zero or negative length");
      continue;
    }
    if (!finite3(bone.rest_vector) || !finite3(bone.rest_direction)) {
      status.error("bone " + quoteName(bone.name) + " rest vector is not finite");
      continue;
    }
    const double magnitude = glm::length(bone.rest_vector);
    if (std::abs(magnitude - bone.length_m) > kAuthoringLengthToleranceM) {
      std::ostringstream stream;
      stream << "bone " << quoteName(bone.name) << " length_m " << bone.length_m
             << " does not match rest_vector_m magnitude " << magnitude;
      status.error(stream.str());
    }
    const double direction_length = glm::length(bone.rest_direction);
    if (std::abs(direction_length - 1.0) > kAuthoringDirectionTolerance) {
      status.error("bone " + quoteName(bone.name) + " rest_direction is not normalized");
    }
    if (magnitude > 1e-8) {
      const glm::dvec3 unit = bone.rest_vector / magnitude;
      if (glm::length(unit - bone.rest_direction) > kAuthoringDirectionTolerance) {
        status.error("bone " + quoteName(bone.name) +
                     " rest_direction does not match normalize(rest_vector_m)");
      }
    }
    const glm::dvec3 expected_position =
        body.joints[bone.parent_joint].rest_position + bone.rest_vector;
    const double position_error =
        glm::length(expected_position - body.joints[bone.child_joint].rest_position);
    if (position_error > kAuthoringPositionToleranceM) {
      status.error("bone " + quoteName(bone.name) +
                   " rest vector does not meet the child rest position");
    }
  }

  std::vector<int> parent(kJointCount, -2);
  parent[kRootJoint] = -1;
  for (const BoneGeom& bone : body.bones) {
    if (bone.child_joint < 0 || bone.child_joint >= kJointCount) continue;
    if (bone.parent_joint < 0 || bone.parent_joint >= kJointCount) continue;
    if (parent[bone.child_joint] != -2) {
      status.error("joint " + quoteName(kJointNames[bone.child_joint]) + " has more than one parent");
    }
    parent[bone.child_joint] = bone.parent_joint;
  }
  for (int index = 0; index < kJointCount; ++index) {
    if (parent[index] == -2) {
      status.error("joint " + quoteName(kJointNames[index]) + " is not connected to legrootj");
    }
  }
  std::vector<int> color(kJointCount, 0);
  for (int start = 0; start < kJointCount; ++start) {
    int cursor = start;
    std::vector<int> trail;
    while (cursor >= 0 && color[cursor] == 0) {
      color[cursor] = 1;
      trail.push_back(cursor);
      cursor = parent[cursor];
    }
    if (cursor >= 0 && color[cursor] == 1) {
      status.error("invalid hierarchy: cycle detected at " + quoteName(kJointNames[start]));
    }
    for (int node : trail) color[node] = 2;
  }
  if (body.joints[kRootJoint].name == "legrootj" && body.joints[kRootJoint].parent != -1) {
    status.error("root must be legrootj");
  }

  for (int index = 0; index < kJointCount; ++index) {
    const JointConstraint& constraint = body.constraints[index];
    const bool endpoint = jointIsEndpoint(index);
    if (index == kRootJoint && constraint.type != JointType::Root) {
      status.error("legrootj constraint type must be root");
    }
    if (endpoint && constraint.type != JointType::Endpoint) {
      status.error("joint " + quoteName(kJointNames[index]) + " is an endpoint and cannot rotate a child");
    }
    if (!endpoint && index != kRootJoint && constraint.type == JointType::Endpoint) {
      status.error("joint " + quoteName(kJointNames[index]) + " has a child bone and cannot be an endpoint");
    }
    if (constraint.type == JointType::Hinge) {
      if (!std::isfinite(constraint.min_deg) || !std::isfinite(constraint.max_deg) ||
          !(constraint.min_deg < constraint.max_deg)) {
        status.error("hinge " + quoteName(kJointNames[index]) + " needs a finite min_deg < max_deg");
      }
      if (!finite3(constraint.axis) || glm::length(constraint.axis) < 1e-8) {
        status.error("hinge " + quoteName(kJointNames[index]) + " needs a finite axis");
      }
    }
    if (constraint.type == JointType::Ball) {
      if (!std::isfinite(constraint.max_swing_deg) || constraint.max_swing_deg <= 0.0 ||
          constraint.max_swing_deg > 180.0) {
        status.error("ball " + quoteName(kJointNames[index]) + " needs max_swing_deg in (0, 180]");
      }
      if (!std::isfinite(constraint.max_twist_deg) || constraint.max_twist_deg < 0.0 ||
          constraint.max_twist_deg > 180.0) {
        status.error("ball " + quoteName(kJointNames[index]) + " needs max_twist_deg in [0, 180]");
      }
      if (!finite3(constraint.axis) || glm::length(constraint.axis) < 1e-8) {
        status.error("ball " + quoteName(kJointNames[index]) + " needs a finite twist_axis");
      }
    }
    if (constraint.authority.empty()) {
      status.warn("constraint for " + quoteName(kJointNames[index]) +
                  " does not say whether its limits are placeholders");
    }
  }
}

}  // namespace proc
