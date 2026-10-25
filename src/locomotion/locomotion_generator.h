#pragma once

#include "body/proc_body.h"
#include "locomotion/locomotion_model.h"
#include "motion/proc_motion.h"

#include <string>

namespace proc {

struct LocomotionCommand {
  double desired_speed_mps = 0.0;
  double heading = 0.0;
  RegimeForce force = RegimeForce::Auto;
  double max_accel_mps2 = 2.5;
};

struct LocomotionState {
  double phase = 0.0;
  double desired_speed_mps = 0.0;
  double actual_speed_mps = 0.0;
  double heading = 0.0;
  glm::dvec3 root_position{0.0, 0.0, 0.0};
  LocomotionRegime regime = LocomotionRegime::Idle;
  double blend = 0.0;
  LocomotionRegime blend_regime = LocomotionRegime::Walk;
  bool clamped = false;
  bool inferred_transition = false;
  int knot_a = -1;
  int knot_b = -1;
  double knot_weight = 0.0;
  double cadence_spm = 0.0;
  double dimensionless_speed = 0.0;
  double reference_stride_m = 0.0;
  double implied_stride_m = 0.0;
  double fade = 1.0;
  double contact_left = 0.0;
  double contact_right = 0.0;
  std::string note;
};

struct LocomotionSample {
  Pose pose;
  LocomotionState state;
};

const RegimeModel* regimeData(const ProcLocomotionModel& model, LocomotionRegime regime);

void advanceLocomotion(LocomotionState& state, const ProcLocomotionModel& model, const ProcBody& body, const LocomotionCommand& command,
                       double dt);

LocomotionSample generateLocomotionPose(const ProcBody& body, const ProcLocomotionModel& model, const LocomotionState& state);

ProcMotion bakeLocomotion(const ProcBody& body, const ProcLocomotionModel& model, LocomotionCommand command, double duration_s, double fps);

}  // namespace proc
