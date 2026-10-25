#pragma once

#include "body/skeleton.h"
#include "import/source_motion.h"
#include "math/quat.h"
#include "motion/proc_motion.h"

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

namespace proc {
namespace test {

enum class SyntheticKind { Walk, Run, NonGait, Turning };

struct SyntheticGait {
  SourceClip source;
  ProcMotion motion;
  double speed = 0.0;
  double period = 0.0;
  double stance = 0.0;
  double clearance = 0.0;
  double leg_length = 0.0;
  std::vector<double> left_ic;
  std::vector<double> left_to;
  std::vector<double> right_ic;
  std::vector<double> right_to;
};

inline SyntheticGait makeSyntheticGait(SyntheticKind kind) {
  SyntheticGait gait;
  double fps = 50.0;
  double duration = 3.0;
  double speed = 1.2;
  double period = 1.0;
  double stance = 0.60;
  double right_phase = 0.50;
  double lead = 0.25;
  double clearance = 0.08;
  double turn_rate = 0.0;
  const bool nongait = kind == SyntheticKind::NonGait;
  if (kind == SyntheticKind::Run) {
    duration = 2.4;
    speed = 3.0;
    period = 0.80;
    stance = 0.30;
    right_phase = 0.40;
    clearance = 0.10;
  } else if (kind == SyntheticKind::Turning) {
    turn_rate = 3.14159265358979323846 * 0.5;
  }
  gait.speed = speed;
  gait.period = period;
  gait.stance = stance;
  gait.clearance = clearance;
  gait.leg_length = 0.90;

  auto addJoint = [&](const char* name, int parent, const glm::dvec3& rest) {
    SourceJoint joint;
    joint.name = name;
    joint.parent = parent;
    joint.rest_global_position = rest;
    joint.rest_global_rotation = quatIdentity();
    joint.rest_local_rotation = quatIdentity();
    gait.source.joints.push_back(joint);
  };
  addJoint("Hips", -1, {0.0, 0.0, 0.95});
  addJoint("LeftUpLeg", 0, {-0.10, 0.0, 0.90});
  addJoint("LeftLeg", 1, {-0.10, 0.0, 0.45});
  addJoint("LeftFoot", 2, {-0.10, 0.0, 0.0});
  addJoint("LeftToe", 3, {-0.10, 0.18, 0.0});
  addJoint("RightUpLeg", 0, {0.10, 0.0, 0.90});
  addJoint("RightLeg", 5, {0.10, 0.0, 0.45});
  addJoint("RightFoot", 6, {0.10, 0.0, 0.0});
  addJoint("RightToe", 7, {0.10, 0.18, 0.0});
  gait.source.name = kind == SyntheticKind::Run ? "synthetic_run" : kind == SyntheticKind::NonGait ? "synthetic_non_gait" : kind == SyntheticKind::Turning ? "synthetic_turn" : "synthetic_walk";
  gait.source.format = "synthetic";
  gait.source.nominal_fps = fps;
  gait.source.duration = duration;
  gait.source.root = 0;

  const int frame_count = static_cast<int>(std::round(duration * fps)) + 1;
  auto footAt = [&](double time_s, double phase0, double lateral, glm::dvec3& ankle, glm::dvec3& toe) {
    const double local = time_s - phase0;
    bool planted = false;
    double along = lead;
    double height = clearance;
    if (!nongait && local >= -1e-9) {
      const double wrapped = std::fmod(local, period);
      planted = wrapped <= stance + 1e-9;
      const double ic = time_s - wrapped;
      const double plant = speed * ic + lead;
      if (planted) {
        along = plant;
        height = 0.0;
      } else {
        const double swing = std::clamp((wrapped - stance) / std::max(1e-6, period - stance), 0.0, 1.0);
        const double shaped = swing * swing * (3.0 - 2.0 * swing);
        along = plant + speed * period * shaped;
        height = clearance * std::sin(3.14159265358979323846 * swing);
      }
    } else if (!nongait) {
      along = speed * phase0 + lead;
      height = clearance * 0.4;
    }
    ankle = glm::dvec3(lateral, along, height);
    toe = ankle + glm::dvec3(0.0, 0.18, 0.0);
  };

  struct Tap {
    double start;
    double end;
    bool left;
    double along;
  };
  const std::vector<Tap> taps = {{0.15, 0.22, true, 0.10}, {0.40, 0.48, true, 0.00}, {0.55, 0.62, false, -0.05},
                                 {1.10, 1.18, true, 0.12}, {1.45, 1.70, false, -0.08}, {2.20, 2.28, true, 0.04},
                                 {2.55, 2.62, false, 0.15}};

  for (int index = 0; index < frame_count; ++index) {
    const double time_s = static_cast<double>(index) / fps;
    SourceFrame frame;
    frame.time = time_s;
    frame.global_position.assign(gait.source.joints.size(), glm::dvec3(0.0));
    frame.global_rotation.assign(gait.source.joints.size(), quatIdentity());
    glm::dvec3 hips(0.0, speed * time_s, 0.95 + 0.015 * std::sin(4.0 * 3.14159265358979323846 * time_s / period));
    if (turn_rate > 0.0) {
      const double angle = turn_rate * time_s;
      const double radius = speed / turn_rate;
      hips.x = radius * std::sin(angle);
      hips.y = radius * (1.0 - std::cos(angle));
    }
    if (nongait) {
      hips.x = 0.03 * std::sin(3.0 * time_s);
      hips.y = 0.04 * std::sin(5.0 * time_s);
      hips.z = 0.95;
    }
    frame.global_position[0] = hips;
    glm::dvec3 left_ankle;
    glm::dvec3 left_toe;
    glm::dvec3 right_ankle;
    glm::dvec3 right_toe;
    if (!nongait) {
      footAt(time_s, 0.0, -0.10, left_ankle, left_toe);
      footAt(time_s, right_phase, 0.10, right_ankle, right_toe);
    } else {
      left_ankle = glm::dvec3(-0.10, 0.0, 0.12);
      right_ankle = glm::dvec3(0.10, 0.0, 0.12);
      for (const Tap& tap : taps) {
        if (time_s + 1e-9 >= tap.start && time_s <= tap.end + 1e-9) {
          if (tap.left) left_ankle = glm::dvec3(-0.10, tap.along, 0.0);
          else right_ankle = glm::dvec3(0.10, tap.along, 0.0);
        }
      }
      left_toe = left_ankle + glm::dvec3(0.0, 0.18, 0.0);
      right_toe = right_ankle + glm::dvec3(0.0, 0.18, 0.0);
    }
    frame.global_position[1] = hips + glm::dvec3(-0.10, 0.0, -0.05);
    frame.global_position[2] = frame.global_position[1] + glm::dvec3(0.0, 0.0, -0.45);
    frame.global_position[3] = left_ankle;
    frame.global_position[4] = left_toe;
    frame.global_position[5] = hips + glm::dvec3(0.10, 0.0, -0.05);
    frame.global_position[6] = frame.global_position[5] + glm::dvec3(0.0, 0.0, -0.45);
    frame.global_position[7] = right_ankle;
    frame.global_position[8] = right_toe;
    gait.source.frames.push_back(frame);

    Pose pose;
    pose.time = time_s;
    pose.root_translation = hips - gait.source.joints[0].rest_global_position;
    auto flex = [&](double phase0) {
      const double local = time_s - phase0;
      if (nongait || local < 0.0) return 0.0;
      const double wrapped = std::fmod(local, period);
      if (wrapped <= stance) return 0.0;
      const double swing = (wrapped - stance) / std::max(1e-6, period - stance);
      return std::sin(3.14159265358979323846 * std::clamp(swing, 0.0, 1.0));
    };
    const double left_flex = flex(0.0);
    const double right_flex = flex(right_phase);
    pose.local_rotation[6] = quatAxisAngle(glm::dvec3(1.0, 0.0, 0.0), left_flex * 0.35);
    pose.local_rotation[7] = quatAxisAngle(glm::dvec3(1.0, 0.0, 0.0), left_flex * 0.90);
    pose.local_rotation[8] = quatAxisAngle(glm::dvec3(1.0, 0.0, 0.0), -left_flex * 0.25);
    pose.local_rotation[10] = quatAxisAngle(glm::dvec3(1.0, 0.0, 0.0), right_flex * 0.35);
    pose.local_rotation[11] = quatAxisAngle(glm::dvec3(1.0, 0.0, 0.0), right_flex * 0.90);
    pose.local_rotation[12] = quatAxisAngle(glm::dvec3(1.0, 0.0, 0.0), -right_flex * 0.25);
    const double arm = std::sin(2.0 * 3.14159265358979323846 * time_s / period);
    pose.local_rotation[14] = quatAxisAngle(glm::dvec3(1.0, 0.0, 0.0), arm * 0.40);
    pose.local_rotation[18] = quatAxisAngle(glm::dvec3(1.0, 0.0, 0.0), -arm * 0.40);
    gait.motion.frames.push_back(pose);
  }
  gait.motion.skeleton = kSkeletonId;
  gait.motion.source.type = "synthetic";
  gait.motion.source.clip = gait.source.name;
  gait.motion.fps = fps;
  gait.motion.frame_count = frame_count;
  gait.motion.duration_s = duration;
  if (!nongait) {
    for (double time_s = 0.0; time_s < duration - 1e-3; time_s += period) gait.left_ic.push_back(time_s);
    for (double time_s = stance; time_s < duration - 1e-3; time_s += period) gait.left_to.push_back(time_s);
    for (double time_s = right_phase; time_s < duration - 1e-3; time_s += period) gait.right_ic.push_back(time_s);
    for (double time_s = right_phase + stance; time_s < duration - 1e-3; time_s += period) gait.right_to.push_back(time_s);
  }
  return gait;
}

}  // namespace test
}  // namespace proc
