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
    case LocoState::Transfer:
      return "Transfer";
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
  stepCounted_ = false;
  support_ = {};
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

LocoBalance LocomotionController::balanceHint(Vec3 desiredWorld) const {
  LocoBalance hint;
  desiredWorld.z = 0;
  const bool moving = horizontalSpeed(desiredWorld) > 0.06 || braking_;
  const bool gait = state_ == LocoState::Transfer || state_ == LocoState::LeftSwing || state_ == LocoState::RightSwing ||
                    state_ == LocoState::Landing || state_ == LocoState::Stopping;
  if (!gait && !moving) return hint;

  const bool swinging = state_ == LocoState::LeftSwing || state_ == LocoState::RightSwing;
  const bool landing = state_ == LocoState::Landing;
  std::string stance = swing_ == "left" ? "right" : "left";
  if (swing_.empty()) stance.clear();
  if (landing && !swing_.empty()) stance = swing_;
  hint.stanceFocus = stance;
  hint.swingFoot = swinging ? swing_ : "";
  // Transfer only shifts the target toward the stance sole. Sagittal correction
  // stays strong until the foot is actually in the air, then forward falling is allowed.
  // One ankle saturates near a 10 cm capture error. Single support has to stay
  // well under that, or dorsiflexion pivots the sole onto the heel and the body
  // falls backward. Lateral stays a little firmer than sagittal.
  hint.sagittalScale = swinging ? 0.40 : 1.0;
  hint.lateralScale = swinging ? 0.55 : 1.05;
  hint.stanceBias = swinging ? 1.0 : (landing ? 0.72 : 0.58);
  hint.kneeExtra = 0;
  hint.pelvisLean = 0;
  hint.heightAssist = false;
  if (state_ == LocoState::Standing) {
    hint.stanceFocus.clear();
    hint.swingFoot.clear();
    hint.sagittalScale = 1;
    hint.lateralScale = 1;
    hint.kneeExtra = 0;
    hint.pelvisLean = 0;
    hint.heightAssist = false;
    hint.stanceBias = 0;
  }
  return hint;
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
    report.comHeight = pelvis ? pelvis->position.z : 0;
    report.completedSteps = completedSteps_;
    return report;
  }

  const double height = std::clamp(state.centerOfMass.z, 0.45, 1.15);
  const double omega = std::sqrt(kGravity / height);
  Vec3 velocity = state.centerOfMassVelocity;
  velocity.z = 0;
  capture_ = state.centerOfMass + velocity * (1.0 / omega);
  capture_.z = 0;
  support_ = measureSupport(state, feet, metrics, forward, right, capture_);
  const Vec3 supportPoint = support_.valid ? support_.center : Vec3{pelvis->position.x, pelvis->position.y, 0};
  captureError_ = support_.valid ? support_.captureOutside : length(capture_ - supportPoint);
  report.capture = capture_;
  report.captureError = captureError_;
  report.support = supportPoint;
  report.supportValid = support_.valid;
  report.supportLabel = supportKindLabel(support_.kind);
  report.leftLoad = support_.leftLoad;
  report.rightLoad = support_.rightLoad;
  report.leftFraction = support_.leftFraction;
  report.rightFraction = support_.rightFraction;
  report.comHeight = pelvis->position.z;

  standing.setHipBalance(true);
  const double comSpeed = horizontalSpeed(velocity);
  const bool wantMove = requested > 0.06;
  const double timingSpeed = wantMove ? requested : comSpeed;
  const bool tall = pelvis->position.z > 0.74 && pelvisUp > 0.72;
  const bool recoverable = pelvis->position.z > 0.68 && pelvisUp > 0.62;

  auto swingState = [&](const std::string& side) {
    return side == "left" ? LocoState::LeftSwing : LocoState::RightSwing;
  };
  auto stanceOf = [&](const std::string& swing) { return swing == "left" ? std::string("right") : std::string("left"); };
  auto fractionOf = [&](const std::string& side) {
    return side == "left" ? support_.leftFraction : support_.rightFraction;
  };

  auto place = [&](const std::string& side) {
    const Side& spec = side == "left" ? leftLeg_ : rightLeg_;
    const BodyKinematics* hip = findBody(state, side + "_upper_leg");
    const BodyKinematics* foot = findBody(state, side + "_foot");
    const BodyKinematics* other = findBody(state, side == "left" ? "right_foot" : "left_foot");
    if (!hip || !foot || !pelvis) return;
    const std::string stance = stanceOf(side);
    Vec3 stancePos = supportPoint;
    if (stance == "left" && support_.leftFound) stancePos = support_.leftFoot;
    if (stance == "right" && support_.rightFound) stancePos = support_.rightFoot;
    const Vec3 velocityError = desiredWorld - velocity;
    Vec3 along = wantMove ? desiredWorld * 0.16 + velocityError * 0.08 : velocity * 0.14;
    Vec3 footPos{foot->position.x, foot->position.y, 0};
    const double sagCapture = dot(capture_ - stancePos, forward);
    const double latCapture = dot(capture_ - footPos, right);
    Vec3 desired = stancePos + forward * (0.50 * sagCapture) + along + right * (0.20 * latCapture);
    desired.z = 0;
    const double ownLat = dot(footPos, right);
    desired = desired + right * (ownLat - dot(desired, right));
    Vec3 aim = wantMove ? desiredWorld : velocity;
    aim.z = 0;
    if (length(aim) > 0.05) {
      const double alongFoot = dot(desired - footPos, normalize(aim));
      const double want = std::clamp(0.08 + 0.08 * timingSpeed, 0.10, 0.18);
      if (alongFoot < want) desired = desired + normalize(aim) * (want - alongFoot);
    }
    const PlannedFoot planned =
        clampFootTarget(pelvis->position, desired, forward, right, hip->position, spec.lengths, spec.sign,
                        metrics.ankleHeight, other, metrics);
    landing_ = planned.landing;
    ankleTarget_ = planned.ankle;
    stepDistance_ = length(Vec3{planned.ankle.x - foot->position.x, planned.ankle.y - foot->position.y, 0});
    duration_ = std::clamp(stepDuration(timingSpeed), 0.45, 0.58);
    if (support_.captureOutside > 0.08) duration_ = std::max(0.36, duration_ * 0.80);
  };

  auto chooseSwing = [&]() {
    const double lat = dot(capture_ - supportPoint, right);
    const bool emergency = captureError_ > 0.10 && std::abs(lat) > 0.07;
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
    stepCounted_ = false;
    Side& spec = swing_ == "left" ? leftLeg_ : rightLeg_;
    const BodyKinematics* hip = findBody(state, swing_ + "_upper_leg");
    if (hip && !spec.calibrated) {
      calibrateHipBias(spec.lengths, state, swing_, hip->position, foot->position, forward, right, up_);
      spec.calibrated = true;
    }
    // The settled hip is much more flexed than the stand reference. Pulling it
    // straight during single support pitches the pelvis backward.
    const std::string stance = stanceOf(swing_);
    for (const char* name : {"_hip_flex", "_hip_abd", "_knee"}) {
      if (const JointState* hinge = findHinge(state, stance + name)) {
        standing.shiftReference(hinge->name, hinge->angle);
      }
    }
    standing.setHoldReferences(true);
  };

  auto finishStep = [&]() {
    if (stepCounted_) return;
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
    stepCounted_ = true;
    haveHold_ = false;
  };

  if (state_ == LocoState::Standing) {
    standing.setSwingFoot("");
    const bool needsBrake = !wantMove && comSpeed > 0.75 && readyTime_ > 2.0 && recoverable;
    if (support_.valid && readyTime_ > 0.8 && (wantMove || needsBrake) && tall) {
      swing_ = chooseSwing();
      state_ = LocoState::Transfer;
      stateTime_ = 0;
      braking_ = needsBrake && !wantMove;
      stepCounted_ = false;
    }
  }

  if (state_ == LocoState::Transfer) {
    stateTime_ += dt;
    standing.setSwingFoot("");
    const std::string stance = stanceOf(swing_);
    const FootContact& stanceContact = feet.of(stance);
    const double stanceFraction = fractionOf(stance);
    const double swingFraction = fractionOf(swing_);
    const double lat = std::abs(dot(capture_ - supportPoint, right));
    const bool captureOk = !support_.valid || (lat < 0.14 && support_.captureOutside < 0.20);
    const bool rising = pelvis->linearVelocity.z > -0.20;
    const bool ready = stanceContact.mode == FootContactMode::Planted && stanceFraction > 0.58 && swingFraction < 0.45 &&
                       captureOk && tall && rising;
    const bool fallback = stateTime_ > 0.55 && stanceFraction > 0.52 && swingFraction < 0.50 && tall && rising;
    if (ready || fallback) beginSwing();
    else if (stateTime_ > 0.85 || pelvis->position.z < 0.78) {
      state_ = LocoState::Standing;
      swing_.clear();
      braking_ = false;
    }
  } else if (state_ == LocoState::LeftSwing || state_ == LocoState::RightSwing) {
    const double urgency = support_.captureOutside > 0.06 ? 1.25 : 1.0;
    stateTime_ += dt * urgency;
    standing.setSwingFoot(swing_);
    standing.setHipBalance(false);
    const double phase = duration_ > 1e-4 ? stateTime_ / duration_ : 1;
    const BodyKinematics* hip = findBody(state, swing_ + "_upper_leg");
    if (hip) {
      const Side& spec = swing_ == "left" ? leftLeg_ : rightLeg_;
      const double clearance = 0.05 + 0.015 * std::min(1.4, timingSpeed);
      const Vec3 ankle = swingAnkle(swingStart_, ankleTarget_, phase, clearance);
      LegPose pose = solveLeg(hip->position, ankle, forward, right, up_, spec.lengths);
      const double stepSag = dot(ankleTarget_ - swingStart_, forward);
      pose.hipFlex += -0.55 * stepSag;
      pose.hipAbd += -0.25 * dot(ankleTarget_ - swingStart_, right);
      // A saturated swing hip folds the stance leg through the pelvis. Keep the
      // requested error inside the torque limit.
      const char* suffix[5] = {"_hip_flex", "_hip_abd", "_knee", "_ankle_pitch", "_ankle_roll"};
      const double raw[5] = {pose.hipFlex, pose.hipAbd, pose.knee, pose.anklePitch, pose.ankleRoll};
      const double kp[5] = {90.0, 80.0, 90.0, 24.0, 24.0};
      const double kd[5] = {12.0, 12.0, 10.0, 4.0, 4.0};
      const double limit[5] = {18.0, 16.0, 22.0, 16.0, 16.0};
      for (int i = 0; i < 5; ++i) {
        const JointState* hinge = findHinge(state, swing_ + suffix[i]);
        if (!hinge) continue;
        JointCommand command;
        const double window = limit[i] / kp[i];
        command.target = hinge->angle + std::clamp(raw[i] - hinge->angle, -window, window);
        command.target = std::clamp(command.target, hinge->rangeMin + 0.02, hinge->rangeMax - 0.02);
        command.kp = kp[i];
        command.kd = kd[i];
        command.limit = limit[i];
        frame.setSwing(hinge->name, command, 1);
      }
    }
    const FootContact& swingingFoot = feet.of(swing_);
    const bool planted = (swingingFoot.mode == FootContactMode::Planted && phase > 0.62) ||
                         (phase > 0.85 && swingingFoot.load > 28.0);
    if (planted || phase >= 1.20) {
      holdContactAngles(state);
      if (const BodyKinematics* foot = findBody(state, swing_ + "_foot")) {
        landing_ = {foot->position.x, foot->position.y, 0};
      }
      state_ = LocoState::Landing;
      stateTime_ = 0;
      swingWeight_ = 1;
      standing.setHoldReferences(false);
    }
    report.phase = std::clamp(phase, 0.0, 1.0);
  } else if (state_ == LocoState::Landing) {
    stateTime_ += dt;
    standing.setSwingFoot("");
    swingWeight_ = 1.0 - std::clamp(stateTime_ / 0.14, 0.0, 1.0);
    commandHold(state, frame, swingWeight_);
    report.phase = 1;
    const std::string newFoot = swing_;
    const std::string oldFoot = stanceOf(swing_);
    const bool newPlanted = feet.of(newFoot).mode == FootContactMode::Planted || feet.of(newFoot).load > 40.0;
    const bool loaded = fractionOf(newFoot) > 0.40;
    const bool oldUnloadable = fractionOf(oldFoot) < 0.72;
    const bool accepted = stateTime_ > 0.16 && newPlanted && loaded && oldUnloadable && recoverable;
    if (swingWeight_ <= 0.02 && (accepted || stateTime_ > 0.65)) finishStep();
    if (stepCounted_ && stateTime_ > 0.20) {
      const std::string landed = swing_;
      const bool canContinue = pelvis->position.z > 0.74 && pelvisUp > 0.82 && support_.captureOutside < 0.12;
      if (pelvisUp < 0.50 || pelvis->position.z < 0.48) {
        state_ = LocoState::Fallen;
      } else if ((wantMove || comSpeed > 0.55) && completedSteps_ < 24 && canContinue) {
        swing_ = stanceOf(landed);
        state_ = LocoState::Transfer;
        stateTime_ = 0;
        braking_ = !wantMove;
        stepCounted_ = false;
      } else {
        swing_.clear();
        state_ = LocoState::Standing;
        stateTime_ = 0;
        braking_ = false;
      }
    }
  }

  const bool swinging = state_ == LocoState::LeftSwing || state_ == LocoState::RightSwing;
  if (swinging || state_ == LocoState::Transfer || state_ == LocoState::Landing || wantMove) {
    lowerArms(state, frame, report.phase, swinging);
  }

  report.state = state_;
  if (braking_ && (swinging || state_ == LocoState::Transfer)) report.state = LocoState::Stopping;
  report.swingFoot = swing_;
  if (state_ == LocoState::Landing && !swing_.empty()) report.stanceFoot = swing_;
  else if (!swing_.empty() && state_ != LocoState::Standing) report.stanceFoot = stanceOf(swing_);
  else if (support_.kind == SupportKind::Left) report.stanceFoot = "left";
  else if (support_.kind == SupportKind::Right) report.stanceFoot = "right";
  else if (support_.kind == SupportKind::Double) report.stanceFoot = "double";
  if (state_ == LocoState::Transfer) report.transfer = "To " + report.stanceFoot;
  else if (swinging) report.transfer = "Single " + report.stanceFoot;
  else if (state_ == LocoState::Landing) report.transfer = "Double";
  else report.transfer = "Quiet";
  if (report.stanceFoot == "left" && support_.leftFound) {
    report.stanceSole = support_.leftFoot;
    report.stanceSoleValid = true;
  } else if (report.stanceFoot == "right" && support_.rightFound) {
    report.stanceSole = support_.rightFoot;
    report.stanceSoleValid = true;
  }
  report.landing = landing_;
  report.landingValid = state_ != LocoState::Standing && state_ != LocoState::Fallen;
  report.stepDistance = stepDistance_;
  report.duration = duration_;
  report.completedSteps = completedSteps_;
  report.sameSideSteps = sameSideSteps_;
  report.usedLeft = usedLeft_;
  report.usedRight = usedRight_;
  if (swinging || state_ == LocoState::Landing) {
    const double clearance = 0.06 + 0.02 * std::min(1.4, timingSpeed);
    for (int i = 0; i < 8; ++i) {
      report.path[i] = swingAnkle(swingStart_, ankleTarget_, static_cast<double>(i) / 7.0, clearance);
    }
    report.pathCount = 8;
  }
  return report;
}

}  // namespace pm
