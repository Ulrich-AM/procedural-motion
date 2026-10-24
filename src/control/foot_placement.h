#pragma once

#include "control/leg_ik.h"
#include "math_types.h"
#include "physics/body_state.h"
#include "physics/physical_metrics.h"

#include <algorithm>
#include <cmath>
#include <string>

namespace pm {

struct PlannedFoot {
  Vec3 landing;
  Vec3 ankle;
  double distance = 0;
  bool reachable = true;
};

// Pulls a floor target back to the horizontal reach of this leg. The vertical
// span is the live hip height, not a fraction of leg length that can be shorter
// than the crouch and zero the reach.
inline PlannedFoot clampFootTarget(Vec3 pelvis, Vec3 desiredFloor, Vec3 forward, Vec3 right, Vec3 hip,
                                   const LegLengths& lengths, double sign, double ankleHeight,
                                   const BodyKinematics* otherFoot, const PhysicalMetrics& metrics) {
  desiredFloor.z = 0;
  const double leg = lengths.thigh + lengths.shin;
  const double vertical = std::max(0.20, hip.z - ankleHeight);
  const double reach = 0.96 * leg;
  double reachable = std::sqrt(std::max(0.03, reach * reach - vertical * vertical));
  double placeSag = dot(desiredFloor - pelvis, forward);
  double placeLat = dot(desiredFloor - pelvis, right);
  placeSag = std::clamp(placeSag, -std::min(0.34, reachable), std::min(0.36, reachable));
  if (otherFoot) {
    const double otherLat = dot(otherFoot->position - pelvis, right);
    if (sign < 0) placeLat = std::min(placeLat, otherLat - metrics.stanceWidth);
    else placeLat = std::max(placeLat, otherLat + metrics.stanceWidth);
  }
  const double latLimit = std::min(0.32, std::max(0.08, reachable));
  placeLat = std::clamp(placeLat, sign < 0 ? -latLimit : 0.05, sign < 0 ? -0.05 : latLimit);
  const double horiz = std::hypot(placeSag, placeLat);
  if (horiz > reachable && horiz > 1e-4) {
    placeSag *= reachable / horiz;
    placeLat *= reachable / horiz;
  }
  PlannedFoot planned;
  planned.reachable = horiz <= reachable + 1e-3;
  planned.landing = pelvis + forward * placeSag + right * placeLat;
  planned.landing.z = 0;
  planned.ankle = planned.landing + Vec3{0, 0, ankleHeight};
  planned.distance = horiz;
  return planned;
}

inline const JointState* findJointAngle(const BodyState& state, const std::string& name) {
  for (const JointState& joint : state.joints) {
    if (joint.name == name) return &joint;
  }
  return nullptr;
}

inline void calibrateHipBias(LegLengths& lengths, const BodyState& state, const std::string& side, Vec3 hip,
                             Vec3 foot, Vec3 forward, Vec3 right, Vec3 up) {
  const LegPose echo = solveLeg(hip, foot, forward, right, up, lengths);
  if (const JointState* hinge = findJointAngle(state, side + "_hip_flex")) lengths.hipFlexBias = hinge->angle - echo.hipFlex;
  if (const JointState* hinge = findJointAngle(state, side + "_hip_abd")) lengths.hipAbdBias = hinge->angle - echo.hipAbd;
}

}  // namespace pm
