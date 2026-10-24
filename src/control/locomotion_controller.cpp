#include "control/locomotion_controller.h"

#include "control/foot_placement.h"
#include "control/swing_command.h"

#include <algorithm>
#include <cmath>

namespace pm {
namespace {

constexpr double kGravity = 9.81;

// duration = clamp(0.62 - 0.12 * speed, 0.42, 0.70)
// 0.2 m/s -> 0.60 s, 0.8 m/s -> 0.52 s, 1.4 m/s -> 0.45 s.
double stepDuration(double speed) { return std::clamp(0.62 - 0.12 * speed, 0.42, 0.70); }

double horizontalSpeed(Vec3 velocity) {
  velocity.z = 0;
  return length(velocity);
}

}  // namespace

const char* locoStateLabel(LocoState state) {
  switch (state) {
    case LocoState::Standing:
      return "Standing";
    case LocoState::Starting:
      return "Starting";
    case LocoState::LeftSwing:
      return "LeftSwing";
    case LocoState::RightSwing:
      return "RightSwing";
    case LocoState::Landing:
      return "Landing";
    case LocoState::Stopping:
      return "Stopping";
    case LocoState::Fallen:
      return "Fallen";
  }
  return "Standing";
}

void LocomotionController::reset() {
  state_ = LocoState::Standing;
  swing_.clear();
  lastSwing_.clear();
  stateTime_ = 0;
  duration_ = 0.6;
  swingWeight_ = 0;
  swingStart_ = {};
  ankleTarget_ = {};
  landing_ = {};
  stepDistance_ = 0;
  haveHold_ = false;
  completedSteps_ = 0;
  sameSideSteps_ = 0;
  usedLeft_ = false;
  usedRight_ = false;
  braking_ = false;
  armBlend_ = 0;
  readyTime_ = 0;
  capture_ = {};
  captureError_ = 0;
  leftLeg_.calibrated = false;
  rightLeg_.calibrated = false;
}

void LocomotionController::capture(const SkeletonDefinition& skeleton, const BodyState& state) {
  reset();
  forward_ = skeleton.forward;
  right_ = skeleton.right;
  up_ = skeleton.up;
  if (length(forward_) < 0.5) forward_ = {0, -1, 0};
  if (length(right_) < 0.5) right_ = {-1, 0, 0};
  if (length(up_) < 0.5) up_ = {0, 0, 1};
  auto measure = [&](Side& side, const char* name, double sign) {
    side.name = name;
    side.sign = sign;
    const BodyKinematics* hip = findBody(state, side.name + "_upper_leg");
    const BodyKinematics* knee = findBody(state, side.name + "_lower_leg");
    const BodyKinematics* foot = findBody(state, side.name + "_foot");
    if (hip && knee) side.lengths.thigh = std::max(0.2, length(knee->position - hip->position));
    if (knee && foot) side.lengths.shin = std::max(0.2, length(foot->position - knee->position));
    if (const JointState* hinge = findHinge(state, side.name + "_knee")) {
      side.lengths.kneeSign = std::abs(hinge->rangeMin) >= std::abs(hinge->rangeMax) ? -1.0 : 1.0;
    }
  };
  measure(leftLeg_, "left", -1);
  measure(rightLeg_, "right", 1);
  captured_ = true;
}

Vec3 LocomotionController::axisOf(const BodyKinematics& body, Vec3 local) const {
  const double* r = body.rotation;
  return {r[0] * local.x + r[1] * local.y + r[2] * local.z, r[3] * local.x + r[4] * local.y + r[5] * local.z,
          r[6] * local.x + r[7] * local.y + r[8] * local.z};
}

void LocomotionController::holdContactAngles(const BodyState& state) {
  const char* suffix[5] = {"_hip_flex", "_hip_abd", "_knee", "_ankle_pitch", "_ankle_roll"};
  for (int i = 0; i < 5; ++i) {
    const JointState* hinge = findHinge(state, swing_ + suffix[i]);
    hold_[i] = hinge ? hinge->angle : 0;
  }
  haveHold_ = true;
}

void LocomotionController::commandHold(const BodyState& state, ControlFrame& frame, double weight) const {
  if (!haveHold_ || swing_.empty()) return;
  commandLegAngles(frame, state, swing_, hold_, weight);
}

void LocomotionController::lowerArms(const BodyState& state, ControlFrame& frame, double phase, bool swinging) const {
  // Shoulder abduction about the character forward axis drops the T-pose arms.
  // Positive abduction swings a limb toward character-left, so the two arms
  // use opposite signs. The target eases in so the first motion does not throw
  // the torso. The swing is a small opposite-phase flex, not a clip.
  const double wave = swinging ? 0.12 * armBlend_ * std::sin(std::clamp(phase, 0.0, 1.0) * 3.141592653589793) : 0;
  const double flexSign = swing_ == "right" ? 1.0 : -1.0;
  auto set = [&](const std::string& name, double target) {
    const JointState* hinge = findHinge(state, name);
    if (!hinge) return;
    JointCommand command;
    command.target = std::clamp(target, hinge->rangeMin + 0.05, hinge->rangeMax - 0.05);
    command.kp = 8;
    command.kd = 3;
    command.limit = 6;
    frame.setSwing(name, command, armBlend_);
  };
  const double drop = 0.55 * armBlend_;
  set("left_shoulder_abd", -drop);
  set("right_shoulder_abd", drop);
  set("left_shoulder_flex", flexSign * wave);
  set("right_shoulder_flex", -flexSign * wave);
}

LocomotionReport LocomotionController::update(const BodyState& state, StandingController& standing, ControlFrame& frame,
                                              const FootContactTracker& feet, const PhysicalMetrics& metrics,
                                              Vec3 desiredWorld, double dt, bool enabled) {
  LocomotionReport report;
  report.enabled = enabled;
  if (!captured_) return report;
  dt = std::clamp(dt, 1e-4, 0.05);
  if (enabled) {
    armBlend_ = std::min(1.0, armBlend_ + dt / 1.4);
    readyTime_ += dt;
  }
  desiredWorld.z = 0;
  if (!std::isfinite(desiredWorld.x) || !std::isfinite(desiredWorld.y)) desiredWorld = {};
  const double requested = std::min(1.6, horizontalSpeed(desiredWorld));
  if (requested > 1e-6 && requested < horizontalSpeed(desiredWorld)) {
    desiredWorld = normalize(desiredWorld) * requested;
  }
  report.desired = desiredWorld;

  const BodyKinematics* pelvis = findBody(state, "pelvis");
  Vec3 forward = forward_;
  Vec3 right = right_;
  Vec3 up = up_;
  double pelvisUp = 1;
  if (pelvis) {
    forward = axisOf(*pelvis, forward_);
    right = axisOf(*pelvis, right_);
    up = axisOf(*pelvis, up_);
    forward.z = 0;
    right.z = 0;
    forward = normalize(forward);
    right = normalize(right);
    pelvisUp = up.z;
  }
  if (length(forward) < 0.5) forward = forward_;
  if (length(right) < 0.5) right = right_;

  const bool fallen = !pelvis || pelvis->position.z < 0.42 || pelvisUp < 0.3;
  if (!enabled || fallen) {
    if (fallen) state_ = LocoState::Fallen;
    else reset();
    standing.setSwingFoot("");
    report.state = enabled ? state_ : LocoState::Standing;
    return report;
  }

  const double height = std::clamp(state.centerOfMass.z, 0.45, 1.15);
  const double omega = std::sqrt(kGravity / height);
  Vec3 velocity = state.centerOfMassVelocity;
  velocity.z = 0;
  capture_ = state.centerOfMass + velocity * (1.0 / omega);
  capture_.z = 0;
  const Vec3 support = pelvis ? Vec3{pelvis->position.x, pelvis->position.y, 0} : Vec3{};
  captureError_ = length(capture_ - support);
  report.capture = capture_;
  report.captureError = captureError_;
  report.support = support;
  report.supportValid = pelvis != nullptr;

  const double comSpeed = horizontalSpeed(velocity);
  const bool wantMove = requested > 0.06;
  braking_ = !wantMove && comSpeed > 0.16 && state_ != LocoState::Standing;
  const double timingSpeed = wantMove ? requested : comSpeed;

  auto swingState = [&](const std::string& side) {
    return side == "left" ? LocoState::LeftSwing : LocoState::RightSwing;
  };

  auto place = [&](const std::string& side) {
    const Side& spec = side == "left" ? leftLeg_ : rightLeg_;
    const BodyKinematics* hip = findBody(state, side + "_upper_leg");
    const BodyKinematics* foot = findBody(state, side + "_foot");
    const BodyKinematics* other = findBody(state, side == "left" ? "right_foot" : "left_foot");
    if (!hip || !foot || !pelvis) return;
    // target = capture + desired * velocityBias + velocityError * correction + lateral offset.
    // When desired velocity is zero the bias term drops out and the step lands
    // ahead of the moving COM, which is the braking step.
    const Vec3 velocityError = desiredWorld - velocity;
    const double bias = wantMove ? 0.28 : 0.0;
    Vec3 along = desiredWorld * bias + velocityError * (wantMove ? 0.10 : 0.16);
    if (!wantMove) along = velocity * 0.18;
    const double emergency = std::clamp((captureError_ - 0.08) / 0.12, 0.0, 1.0);
    Vec3 desired = capture_ * (0.45 + 0.40 * emergency) + (support + along) * (0.55 - 0.40 * emergency);
    desired = desired + right * (spec.sign * metrics.stanceWidth);
    desired.z = 0;
    Vec3 aim = wantMove ? desiredWorld : velocity;
    aim.z = 0;
    if (length(aim) > 0.05) {
      const double along = dot(desired - foot->position, normalize(aim));
      const double want = std::clamp(0.14 + 0.16 * timingSpeed, 0.16, 0.32);
      if (along < want) desired = desired + normalize(aim) * (want - along);
    }
    const PlannedFoot planned =
        clampFootTarget(pelvis->position, desired, forward, right, hip->position, spec.lengths, spec.sign,
                        metrics.ankleHeight, other, metrics);
    landing_ = planned.landing;
    ankleTarget_ = planned.ankle;
    stepDistance_ = length(Vec3{planned.ankle.x - foot->position.x, planned.ankle.y - foot->position.y, 0});
    duration_ = stepDuration(timingSpeed);
    if (emergency > 0.6) duration_ = std::min(duration_, 0.42);
  };

  auto chooseSwing = [&]() {
    const double lat = dot(capture_ - support, right);
    const bool emergency = captureError_ > 0.14 && std::abs(lat) > 0.07;
    if (emergency) return lat < 0 ? std::string("left") : std::string("right");
    if (lastSwing_ == "left") return std::string("right");
    if (lastSwing_ == "right") return std::string("left");
    const double side = dot(desiredWorld, right);
    if (side > 0.12) return std::string("right");
    if (side < -0.12) return std::string("left");
    return std::string("right");
  };

  auto beginSwing = [&]() {
    const BodyKinematics* foot = findBody(state, swing_ + "_foot");
    if (!foot) return;
    swingStart_ = foot->position;
    place(swing_);
    state_ = swingState(swing_);
    stateTime_ = 0;
    swingWeight_ = 1;
    haveHold_ = false;
    Side& spec = swing_ == "left" ? leftLeg_ : rightLeg_;
    const BodyKinematics* hip = findBody(state, swing_ + "_upper_leg");
    if (hip && !spec.calibrated) {
      calibrateHipBias(spec.lengths, state, swing_, hip->position, foot->position, forward, right, up_);
      spec.calibrated = true;
    }
  };

  const FootContact& swingContact = feet.of(swing_.empty() ? "left" : swing_);
  const std::string stanceSide = swing_ == "left" ? "right" : "left";
  const FootContact& stanceContact = feet.of(swing_.empty() ? "right" : stanceSide);

  if (state_ == LocoState::Standing) {
    standing.setSwingFoot("");
    const bool needsBrake = !wantMove && comSpeed > 0.75 && readyTime_ > 2.0;
    if (readyTime_ > 0.8 && (wantMove || needsBrake) && pelvisUp > 0.7 && pelvis->position.z > 0.62) {
      swing_ = chooseSwing();
      state_ = LocoState::Starting;
      stateTime_ = 0;
      braking_ = needsBrake && !wantMove;
    }
  }

  if (state_ == LocoState::Starting) {
    stateTime_ += dt;
    standing.setSwingFoot(swing_);
    const bool stanceReady = stanceContact.mode == FootContactMode::Planted || stanceContact.load > 40.0;
    const bool unloaded = swingContact.load < std::max(30.0, stanceContact.load * 0.75);
    if (stateTime_ > 0.22 || (stateTime_ > 0.10 && stanceReady && unloaded)) beginSwing();
  } else if (state_ == LocoState::LeftSwing || state_ == LocoState::RightSwing || state_ == LocoState::Stopping) {
    stateTime_ += dt;
    standing.setSwingFoot(swing_);
    const double phase = duration_ > 1e-4 ? stateTime_ / duration_ : 1;
    const BodyKinematics* hip = findBody(state, swing_ + "_upper_leg");
    if (hip) {
      const Side& spec = swing_ == "left" ? leftLeg_ : rightLeg_;
      const double clearance = 0.07 + 0.025 * std::min(1.4, timingSpeed);
      const Vec3 ankle = swingAnkle(swingStart_, ankleTarget_, phase, clearance);
      LegPose pose = solveLeg(hip->position, ankle, forward, right, up_, spec.lengths);
      pose.hipFlex += -1.4 * dot(ankle - swingStart_, forward);
      pose.hipAbd += -0.8 * dot(ankle - swingStart_, right);
      commandLegPose(frame, state, swing_, pose, 1);
    }
    const bool planted = (swingContact.mode == FootContactMode::Planted && phase > 0.72) ||
                         (phase > 0.95 && swingContact.load > 30.0);
    if (planted || phase >= 1.30) {
      holdContactAngles(state);
      if (const BodyKinematics* foot = findBody(state, swing_ + "_foot")) {
        landing_ = {foot->position.x, foot->position.y, 0};
      }
      state_ = LocoState::Landing;
      stateTime_ = 0;
      swingWeight_ = 1;
    }
    report.phase = std::clamp(phase, 0.0, 1.0);
  } else if (state_ == LocoState::Landing) {
    stateTime_ += dt;
    // Physics owns the foot. The swing target is the joint pose at contact,
    // faded out. It is not a world-space ankle chase.
    standing.setSwingFoot("");
    swingWeight_ = 1.0 - std::clamp(stateTime_ / 0.12, 0.0, 1.0);
    commandHold(state, frame, swingWeight_);
    report.phase = 1;
    if (swingWeight_ <= 0.02) {
      const char* suffix[5] = {"_hip_flex", "_hip_abd", "_knee", "_ankle_pitch", "_ankle_roll"};
      for (const char* name : suffix) {
        if (const JointState* hinge = findHinge(state, swing_ + name)) {
          standing.shiftReference(hinge->name, hinge->angle);
          standing.noteAnkle(hinge->name, hinge->angle);
        }
      }
      if (!lastSwing_.empty() && lastSwing_ == swing_) ++sameSideSteps_;
      lastSwing_ = swing_;
      ++completedSteps_;
      if (swing_ == "left") usedLeft_ = true;
      if (swing_ == "right") usedRight_ = true;
      swing_.clear();
      haveHold_ = false;
      standing.setSwingFoot("");
      const bool stillMoving = wantMove || comSpeed > 0.16;
      if (pelvisUp < 0.55 || pelvis->position.z < 0.5) {
        state_ = LocoState::Fallen;
      } else if (stillMoving && completedSteps_ < 24) {
        swing_ = chooseSwing();
        state_ = LocoState::Starting;
        stateTime_ = 0;
        braking_ = !wantMove;
      } else {
        state_ = LocoState::Standing;
        stateTime_ = 0;
        braking_ = false;
      }
    }
  }

  const bool swinging = state_ == LocoState::LeftSwing || state_ == LocoState::RightSwing || state_ == LocoState::Stopping;
  if (swinging || state_ == LocoState::Starting || state_ == LocoState::Landing || wantMove) {
    lowerArms(state, frame, report.phase, swinging);
  }

  report.state = state_;
  if (braking_ && (state_ == LocoState::LeftSwing || state_ == LocoState::RightSwing || state_ == LocoState::Starting)) {
    report.state = LocoState::Stopping;
  }
  report.swingFoot = swing_;
  report.landing = landing_;
  report.landingValid = state_ != LocoState::Standing && state_ != LocoState::Fallen;
  report.stepDistance = stepDistance_;
  report.duration = duration_;
  report.completedSteps = completedSteps_;
  report.sameSideSteps = sameSideSteps_;
  report.usedLeft = usedLeft_;
  report.usedRight = usedRight_;
  if (state_ == LocoState::LeftSwing || state_ == LocoState::RightSwing || state_ == LocoState::Stopping ||
      state_ == LocoState::Landing) {
    const double clearance = 0.07 + 0.025 * std::min(1.4, timingSpeed);
    for (int i = 0; i < 8; ++i) {
      report.path[i] = swingAnkle(swingStart_, ankleTarget_, static_cast<double>(i) / 7.0, clearance);
    }
    report.pathCount = 8;
  }
  return report;
}

}  // namespace pm
