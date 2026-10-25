#include "locomotion/locomotion_generator.h"

#include "locomotion/body_scale.h"

#include <algorithm>
#include <cmath>

namespace proc {
namespace {

constexpr double kIdleEnter = 0.12;
constexpr double kIdleExit = 0.28;
constexpr double kFadeSpeed = 0.35;
constexpr double kHysteresis = 0.04;

struct KnotLerp {
  const SpeedKnot* a = nullptr;
  const SpeedKnot* b = nullptr;
  int index_a = -1;
  int index_b = -1;
  double weight = 0.0;
  bool clamped = false;
};

bool supportContains(const RegimeModel& regime, double speed, double margin) {
  if (!regime.present || regime.knots.empty()) return false;
  return speed >= regime.support_min - margin && speed <= regime.support_max + margin;
}

KnotLerp sampleKnots(const RegimeModel& regime, double speed) {
  KnotLerp sample;
  if (regime.knots.empty()) return sample;
  if (speed <= regime.knots.front().dimensionless_speed || regime.knots.size() == 1) {
    sample.a = sample.b = &regime.knots.front();
    sample.index_a = sample.index_b = 0;
    sample.clamped = speed < regime.knots.front().dimensionless_speed - 1e-9;
    return sample;
  }
  if (speed >= regime.knots.back().dimensionless_speed) {
    const int last = static_cast<int>(regime.knots.size()) - 1;
    sample.a = sample.b = &regime.knots.back();
    sample.index_a = sample.index_b = last;
    sample.clamped = true;
    return sample;
  }
  for (int index = 0; index + 1 < static_cast<int>(regime.knots.size()); ++index) {
    const SpeedKnot& left = regime.knots[static_cast<std::size_t>(index)];
    const SpeedKnot& right = regime.knots[static_cast<std::size_t>(index + 1)];
    if (speed >= left.dimensionless_speed && speed <= right.dimensionless_speed) {
      const double span = right.dimensionless_speed - left.dimensionless_speed;
      sample.a = &left;
      sample.b = &right;
      sample.index_a = index;
      sample.index_b = index + 1;
      sample.weight = span > 1e-9 ? (speed - left.dimensionless_speed) / span : 0.0;
      return sample;
    }
  }
  sample.a = sample.b = &regime.knots.back();
  sample.index_a = sample.index_b = static_cast<int>(regime.knots.size()) - 1;
  sample.clamped = true;
  return sample;
}

double lerpScalar(double a, double b, double weight) { return a * (1.0 - weight) + b * weight; }

glm::dvec3 lerpLog(const glm::dvec3& a, const glm::dvec3& b, double weight) { return a * (1.0 - weight) + b * weight; }

struct GaitSample {
  std::array<glm::dvec3, kLocoChannels> rotation{};
  double pelvis_vertical = 0.0;
  double cadence_spm = 0.0;
  double normalized_stride = 0.0;
  double contact_left_heel = 0.0;
  double contact_left_toe = 0.0;
  double contact_right_heel = 0.0;
  double contact_right_toe = 0.0;
  KnotLerp knots;
  bool clamped = false;
};

GaitSample sampleRegime(const RegimeModel& regime, double speed, double phase) {
  GaitSample gait;
  gait.knots = sampleKnots(regime, speed);
  gait.clamped = gait.knots.clamped;
  if (gait.knots.a == nullptr) return gait;
  const SpeedKnot& a = *gait.knots.a;
  const SpeedKnot& b = *gait.knots.b;
  const double weight = gait.knots.weight;
  for (int channel = 0; channel < kLocoChannels; ++channel) {
    const glm::dvec3 left = sampleLog(a.rotation[static_cast<std::size_t>(channel)], phase);
    const glm::dvec3 right = sampleLog(b.rotation[static_cast<std::size_t>(channel)], phase);
    gait.rotation[static_cast<std::size_t>(channel)] = lerpLog(left, right, weight);
  }
  gait.pelvis_vertical = lerpScalar(samplePeriodic(a.pelvis_vertical, phase), samplePeriodic(b.pelvis_vertical, phase), weight);
  gait.cadence_spm = lerpScalar(a.cadence_spm, b.cadence_spm, weight);
  gait.normalized_stride = lerpScalar(a.normalized_stride_length, b.normalized_stride_length, weight);
  gait.contact_left_heel = lerpScalar(samplePeriodic(a.contact_left_heel, phase), samplePeriodic(b.contact_left_heel, phase), weight);
  gait.contact_left_toe = lerpScalar(samplePeriodic(a.contact_left_toe, phase), samplePeriodic(b.contact_left_toe, phase), weight);
  gait.contact_right_heel = lerpScalar(samplePeriodic(a.contact_right_heel, phase), samplePeriodic(b.contact_right_heel, phase), weight);
  gait.contact_right_toe = lerpScalar(samplePeriodic(a.contact_right_toe, phase), samplePeriodic(b.contact_right_toe, phase), weight);
  return gait;
}

GaitSample blendGaits(const GaitSample& lower, const GaitSample& upper, double weight) {
  if (upper.knots.a == nullptr) return lower;
  if (lower.knots.a == nullptr) return upper;
  GaitSample mixed = lower;
  for (int channel = 0; channel < kLocoChannels; ++channel) {
    mixed.rotation[static_cast<std::size_t>(channel)] =
        lerpLog(lower.rotation[static_cast<std::size_t>(channel)], upper.rotation[static_cast<std::size_t>(channel)], weight);
  }
  mixed.pelvis_vertical = lerpScalar(lower.pelvis_vertical, upper.pelvis_vertical, weight);
  mixed.cadence_spm = lerpScalar(lower.cadence_spm, upper.cadence_spm, weight);
  mixed.normalized_stride = lerpScalar(lower.normalized_stride, upper.normalized_stride, weight);
  mixed.contact_left_heel = lerpScalar(lower.contact_left_heel, upper.contact_left_heel, weight);
  mixed.contact_left_toe = lerpScalar(lower.contact_left_toe, upper.contact_left_toe, weight);
  mixed.contact_right_heel = lerpScalar(lower.contact_right_heel, upper.contact_right_heel, weight);
  mixed.contact_right_toe = lerpScalar(lower.contact_right_toe, upper.contact_right_toe, weight);
  mixed.clamped = lower.clamped || upper.clamped;
  return mixed;
}

LocomotionRegime forcedRegime(RegimeForce force) {
  switch (force) {
    case RegimeForce::Walk: return LocomotionRegime::Walk;
    case RegimeForce::Run: return LocomotionRegime::Run;
    case RegimeForce::Sprint: return LocomotionRegime::Sprint;
    case RegimeForce::Auto: return LocomotionRegime::Walk;
  }
  return LocomotionRegime::Walk;
}

LocomotionRegime nearestRegime(const ProcLocomotionModel& model, double speed) {
  LocomotionRegime best = LocomotionRegime::Walk;
  double best_distance = 1e9;
  bool found = false;
  for (LocomotionRegime regime : {LocomotionRegime::Walk, LocomotionRegime::Run, LocomotionRegime::Sprint}) {
    const RegimeModel* data = regimeData(model, regime);
    if (data == nullptr || !data->present) continue;
    const double center = 0.5 * (data->support_min + data->support_max);
    const double distance = std::abs(speed - center);
    if (!found || distance < best_distance) {
      best = regime;
      best_distance = distance;
      found = true;
    }
  }
  return best;
}

void chooseRegime(LocomotionState& state, const ProcLocomotionModel& model, RegimeForce force) {
  state.blend = 0.0;
  state.inferred_transition = false;
  state.blend_regime = LocomotionRegime::Walk;
  if (force != RegimeForce::Auto) {
    state.regime = forcedRegime(force);
    return;
  }
  if (state.regime == LocomotionRegime::Idle) {
    if (state.actual_speed_mps < kIdleExit) return;
    state.regime = nearestRegime(model, state.dimensionless_speed);
  } else if (state.actual_speed_mps < kIdleEnter) {
    state.regime = LocomotionRegime::Idle;
    return;
  }
  const RegimeModel* current = regimeData(model, state.regime);
  if (current == nullptr || !current->present || !supportContains(*current, state.dimensionless_speed, kHysteresis)) {
    state.regime = nearestRegime(model, state.dimensionless_speed);
  }
  const RegimeModel* walk = regimeData(model, LocomotionRegime::Walk);
  const RegimeModel* run = regimeData(model, LocomotionRegime::Run);
  const RegimeModel* sprint = regimeData(model, LocomotionRegime::Sprint);
  auto consider = [&](const RegimeModel* lower, const RegimeModel* upper, LocomotionRegime lower_id, LocomotionRegime upper_id) {
    if (lower == nullptr || upper == nullptr || !lower->present || !upper->present) return;
    const double overlap_lo = std::max(lower->support_min, upper->support_min);
    const double overlap_hi = std::min(lower->support_max, upper->support_max);
    const double speed = state.dimensionless_speed;
    if (overlap_hi > overlap_lo + 1e-6 && speed >= overlap_lo && speed <= overlap_hi) {
      state.blend = (speed - overlap_lo) / (overlap_hi - overlap_lo);
      if (state.regime != lower_id && state.regime != upper_id) state.regime = state.blend < 0.5 ? lower_id : upper_id;
      state.blend_regime = state.regime == lower_id ? upper_id : lower_id;
      return;
    }
    if (lower->support_max < upper->support_min && speed > lower->support_max && speed < upper->support_min) {
      state.blend = (speed - lower->support_max) / (upper->support_min - lower->support_max);
      state.inferred_transition = true;
      if (state.regime != lower_id && state.regime != upper_id) state.regime = state.blend < 0.5 ? lower_id : upper_id;
      state.blend_regime = state.regime == lower_id ? upper_id : lower_id;
    }
  };
  consider(walk, run, LocomotionRegime::Walk, LocomotionRegime::Run);
  if (state.blend <= 0.0) consider(run, sprint, LocomotionRegime::Run, LocomotionRegime::Sprint);
}

GaitSample sampleState(const ProcLocomotionModel& model, const LocomotionState& state) {
  if (state.regime == LocomotionRegime::Idle && state.blend <= 0.0) return {};
  const RegimeModel* primary = regimeData(model, state.regime);
  GaitSample gait = primary != nullptr ? sampleRegime(*primary, state.dimensionless_speed, state.phase) : GaitSample{};
  if (state.blend > 0.0) {
    const RegimeModel* other = regimeData(model, state.blend_regime);
    if (other != nullptr) {
      const GaitSample extra = sampleRegime(*other, state.dimensionless_speed, state.phase);
      const bool primary_is_lower = static_cast<int>(state.regime) < static_cast<int>(state.blend_regime);
      gait = primary_is_lower ? blendGaits(gait, extra, state.blend) : blendGaits(extra, gait, state.blend);
    }
  }
  return gait;
}

}  // namespace

const RegimeModel* regimeData(const ProcLocomotionModel& model, LocomotionRegime regime) {
  switch (regime) {
    case LocomotionRegime::Walk: return &model.walk;
    case LocomotionRegime::Run: return &model.run;
    case LocomotionRegime::Sprint: return &model.sprint;
    case LocomotionRegime::Idle: return nullptr;
  }
  return nullptr;
}

void advanceLocomotion(LocomotionState& state, const ProcLocomotionModel& model, const ProcBody& body, const LocomotionCommand& command,
                       double dt) {
  if (dt < 0.0) dt = 0.0;
  state.desired_speed_mps = std::max(0.0, command.desired_speed_mps);
  state.heading = command.heading;
  const double limit = std::max(0.0, command.max_accel_mps2) * dt;
  const double delta = std::clamp(state.desired_speed_mps - state.actual_speed_mps, -limit, limit);
  state.actual_speed_mps = std::max(0.0, state.actual_speed_mps + delta);
  const double leg = targetLegLength(body);
  state.dimensionless_speed = dimensionlessSpeed(state.actual_speed_mps, leg, model.gravity_mps2);
  chooseRegime(state, model, command.force);
  state.fade = std::clamp(state.actual_speed_mps / kFadeSpeed, 0.0, 1.0);
  const GaitSample gait = sampleState(model, state);
  state.cadence_spm = gait.cadence_spm;
  state.clamped = gait.clamped;
  state.knot_a = gait.knots.index_a;
  state.knot_b = gait.knots.index_b;
  state.knot_weight = gait.knots.weight;
  state.reference_stride_m = gait.normalized_stride * leg;
  state.implied_stride_m = state.cadence_spm > 1e-6 ? state.actual_speed_mps * 120.0 / state.cadence_spm : 0.0;
  state.contact_left = std::max(gait.contact_left_heel, gait.contact_left_toe);
  state.contact_right = std::max(gait.contact_right_heel, gait.contact_right_toe);
  state.note.clear();
  if (state.clamped) state.note = "speed clamped to the regime support";
  if (state.inferred_transition) state.note = "regime transition is inferred between supports";
  if (state.regime != LocomotionRegime::Idle) {
    state.phase = wrapPhase(state.phase + dt * (state.cadence_spm / 120.0) * state.fade);
  }
  const glm::dvec3 forward(std::sin(state.heading), std::cos(state.heading), 0.0);
  state.root_position += forward * state.actual_speed_mps * dt;
}

LocomotionSample generateLocomotionPose(const ProcBody& body, const ProcLocomotionModel& model, const LocomotionState& state) {
  LocomotionSample generated;
  generated.state = state;
  const double leg = targetLegLength(body);
  const GaitSample gait = sampleState(model, state);
  const double fade = state.fade;
  Pose pose;
  for (int channel = 0; channel < kLocoChannels; ++channel) {
    const glm::dvec3 log_vector = gait.rotation[static_cast<std::size_t>(channel)] * fade;
    const glm::dquat rotation = quatExp(log_vector);
    if (channel == static_cast<int>(LocoChannel::Pelvis)) continue;
    pose.local_rotation[jointIndex(locoJoint(channel))] = rotation;
  }
  const glm::dquat heading = quatAxisAngle(glm::dvec3(0.0, 0.0, 1.0), -state.heading);
  const glm::dquat residual = quatExp(gait.rotation[static_cast<int>(LocoChannel::Pelvis)] * fade);
  pose.root_rotation = quatUnit(heading * residual);
  const glm::dvec3 rest = body.joints[jointIndex(JointId::LegRoot)].rest_position;
  const double bob = gait.pelvis_vertical * leg * fade;
  pose.root_translation = glm::dvec3(state.root_position.x - rest.x, state.root_position.y - rest.y, bob);
  pose.contacts.block_present = gait.knots.a != nullptr;
  pose.contacts.left_heel = gait.contact_left_heel >= 0.5;
  pose.contacts.left_toe = gait.contact_left_toe >= 0.5;
  pose.contacts.right_heel = gait.contact_right_heel >= 0.5;
  pose.contacts.right_toe = gait.contact_right_toe >= 0.5;
  generated.pose = pose;
  generated.state.reference_stride_m = gait.normalized_stride * leg;
  generated.state.cadence_spm = gait.cadence_spm;
  generated.state.implied_stride_m = gait.cadence_spm > 1e-6 ? state.actual_speed_mps * 120.0 / gait.cadence_spm : 0.0;
  generated.state.contact_left = std::max(gait.contact_left_heel, gait.contact_left_toe);
  generated.state.contact_right = std::max(gait.contact_right_heel, gait.contact_right_toe);
  generated.state.knot_a = gait.knots.index_a;
  generated.state.knot_b = gait.knots.index_b;
  generated.state.knot_weight = gait.knots.weight;
  generated.state.clamped = gait.clamped;
  return generated;
}

ProcMotion bakeLocomotion(const ProcBody& body, const ProcLocomotionModel& model, LocomotionCommand command, double duration_s, double fps) {
  ProcMotion motion;
  motion.skeleton = body.skeleton.empty() ? kSkeletonId : body.skeleton;
  motion.source.type = "procedural";
  motion.source.clip = "generated";
  motion.fps = fps > 1e-6 ? fps : 30.0;
  duration_s = std::max(0.0, duration_s);
  const int frame_count = std::max(1, static_cast<int>(std::round(duration_s * motion.fps)) + 1);
  motion.frame_count = frame_count;
  motion.duration_s = (frame_count - 1) / motion.fps;
  LocomotionState state;
  state.actual_speed_mps = std::max(0.0, command.desired_speed_mps);
  state.desired_speed_mps = state.actual_speed_mps;
  const double dt = 1.0 / motion.fps;
  for (int frame = 0; frame < frame_count; ++frame) {
    if (frame > 0) advanceLocomotion(state, model, body, command, dt);
    else {
      state.heading = command.heading;
      state.dimensionless_speed = dimensionlessSpeed(state.actual_speed_mps, targetLegLength(body), model.gravity_mps2);
      chooseRegime(state, model, command.force);
      state.fade = std::clamp(state.actual_speed_mps / kFadeSpeed, 0.0, 1.0);
    }
    LocomotionSample sample = generateLocomotionPose(body, model, state);
    sample.pose.time = frame * dt;
    motion.frames.push_back(sample.pose);
  }
  return motion;
}

}  // namespace proc
