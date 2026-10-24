#pragma once

#include "control/control_frame.h"
#include "control/leg_ik.h"
#include "physics/body_state.h"

#include <algorithm>
#include <string>

namespace pm {

inline const JointState* findHinge(const BodyState& state, const std::string& name) {
  for (const JointState& joint : state.joints) {
    if (joint.name == name) return &joint;
  }
  return nullptr;
}

inline const BodyKinematics* findBody(const BodyState& state, const std::string& name) {
  for (const BodyKinematics& body : state.bodies) {
    if (body.name == name) return &body;
  }
  return nullptr;
}

inline double smoothstep01(double phase) {
  const double s = std::clamp(phase, 0.0, 1.0);
  return s * s * (3.0 - 2.0 * s);
}

// Procedural swing. Nothing is stored. clearance is the extra height at mid-swing.
inline Vec3 swingAnkle(Vec3 start, Vec3 target, double phase, double clearance) {
  const double s = smoothstep01(std::min(1.0, phase));
  Vec3 ankle = start + (target - start) * s;
  ankle.z = start.z + (target.z - start.z) * s + clearance * 4.0 * s * (1.0 - s);
  return ankle;
}

// Softer than the standing hip spring. The stiff standing abduction gain buzzes
// and never reaches a swing target at these timesteps.
inline void commandLegAngles(ControlFrame& frame, const BodyState& state, const std::string& side,
                             const double targets[5], double weight) {
  const char* suffix[5] = {"_hip_flex", "_hip_abd", "_knee", "_ankle_pitch", "_ankle_roll"};
  const double kp[5] = {340.0, 400.0, 300.0, 60.0, 60.0};
  const double kd[5] = {26.0, 36.0, 20.0, 8.0, 8.0};
  const double limit[5] = {100.0, 120.0, 110.0, 70.0, 70.0};
  for (int i = 0; i < 5; ++i) {
    const JointState* hinge = findHinge(state, side + suffix[i]);
    if (!hinge) continue;
    JointCommand command;
    command.target = std::clamp(targets[i], hinge->rangeMin + 0.02, hinge->rangeMax - 0.02);
    command.kp = kp[i];
    command.kd = kd[i];
    command.limit = limit[i];
    frame.setSwing(hinge->name, command, weight);
  }
}

inline void commandLegPose(ControlFrame& frame, const BodyState& state, const std::string& side, const LegPose& pose,
                           double weight) {
  const double targets[5] = {pose.hipFlex, pose.hipAbd, pose.knee, pose.anklePitch, pose.ankleRoll};
  commandLegAngles(frame, state, side, targets, weight);
}

}  // namespace pm
