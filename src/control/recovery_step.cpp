#include "control/recovery_step.h"

#include "control/foot_placement.h"
#include "control/swing_command.h"

#include <algorithm>
#include <cmath>

namespace pm {
namespace {

constexpr double kGravity = 9.81;
constexpr double kEnterMargin = 0.028;
constexpr double kExitMargin = 0.02;
constexpr double kClearance = 0.06;

bool loadedFoot(const ContactState& contact, const std::string& side) {
  const std::string token = side + "_foot";
  const bool foot = contact.geom1.find(token) != std::string::npos || contact.geom2.find(token) != std::string::npos;
  const bool floor = contact.geom1 == "floor" || contact.geom2 == "floor";
  return foot && floor && contact.position.z < 0.12 && contact.normalForce > 8.0;
}

}  // namespace

const char* stepStateLabel(StepState state) {
  switch (state) {
    case StepState::Idle:
      return "Idle";
    case StepState::Prepare:
      return "Prepare";
    case StepState::Swing:
      return "Swing";
    case StepState::Plant:
      return "Plant";
    case StepState::Settle:
      return "Settle";
  }
  return "Idle";
}

void RecoveryStep::reset() {
  state_ = StepState::Idle;
  swing_.clear();
  armed_ = true;
  stateTime_ = 0;
  duration_ = 0.36;
  swingStart_ = {};
  landing_ = {};
  ankleTarget_ = {};
  stepDistance_ = 0;
  outsideTime_ = 0;
  insideTime_ = 0;
  stepsTaken_ = 0;
  swingWeight_ = 0;
  haveHold_ = false;
}

void RecoveryStep::capture(const SkeletonDefinition& skeleton, const BodyState& state) {
  reset();
  metrics_ = defaultPhysicalMetrics();
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
  metrics_.thigh = 0.5 * (leftLeg_.lengths.thigh + rightLeg_.lengths.thigh);
  metrics_.shin = 0.5 * (leftLeg_.lengths.shin + rightLeg_.lengths.shin);
  metrics_.maxReach = 0.96 * (metrics_.thigh + metrics_.shin);
  leftLeg_.calibrated = false;
  rightLeg_.calibrated = false;
  captured_ = true;
}

Vec3 RecoveryStep::axisOf(const BodyKinematics& body, Vec3 local) const {
  const double* r = body.rotation;
  return {r[0] * local.x + r[1] * local.y + r[2] * local.z, r[3] * local.x + r[4] * local.y + r[5] * local.z,
          r[6] * local.x + r[7] * local.y + r[8] * local.z};
}

double RecoveryStep::footLoad(const BodyState& state, const std::string& side) const {
  double load = 0;
  for (const ContactState& contact : state.contacts) {
    if (loadedFoot(contact, side)) load += std::max(0.0, contact.normalForce);
  }
  return load;
}

void RecoveryStep::supportOf(const BodyState& state, Vec3 forward, Vec3 right, Vec3& center, double& sagMin,
                             double& sagMax, double& latMin, double& latMax, bool& valid) const {
  Vec3 sum{0, 0, 0};
  int count = 0;
  auto include = [&](Vec3 point) {
    point.z = 0;
    sum = sum + point;
    ++count;
  };
  auto addFoot = [&](const std::string& side) {
    if (footLoad(state, side) < 12.0) return;
    const BodyKinematics* foot = findBody(state, side + "_foot");
    if (!foot) return;
    const Vec3 origin = foot->position;
    include(origin + forward * metrics_.toe);
    include(origin - forward * metrics_.heel);
    include(origin + right * metrics_.halfWidth);
    include(origin - right * metrics_.halfWidth);
  };
  addFoot("left");
  addFoot("right");
  valid = count > 0;
  if (!valid) {
    center = {};
    sagMin = sagMax = latMin = latMax = 0;
    return;
  }
  center = sum / static_cast<double>(count);
  center.z = 0;
  sagMin = latMin = 1e9;
  sagMax = latMax = -1e9;
  auto expand = [&](const std::string& side) {
    if (footLoad(state, side) < 12.0) return;
    const BodyKinematics* foot = findBody(state, side + "_foot");
    if (!foot) return;
    const Vec3 origin = foot->position;
    const Vec3 corners[4] = {origin + forward * metrics_.toe, origin - forward * metrics_.heel,
                             origin + right * metrics_.halfWidth, origin - right * metrics_.halfWidth};
    for (const Vec3& corner : corners) {
      const double sag = dot(corner - center, forward);
      const double lat = dot(corner - center, right);
      sagMin = std::min(sagMin, sag);
      sagMax = std::max(sagMax, sag);
      latMin = std::min(latMin, lat);
      latMax = std::max(latMax, lat);
    }
  };
  expand("left");
  expand("right");
}

RecoveryReport RecoveryStep::update(const BodyState& state, StandingController& standing, ControlFrame& frame,
                                    const FootContactTracker& feet, double dt, bool enabled) {
  RecoveryReport report;
  report.enabled = enabled;
  if (!captured_) return report;
  dt = std::clamp(dt, 1e-4, 0.05);

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

  const double height = std::clamp(state.centerOfMass.z, 0.45, 1.15);
  const double omega = std::sqrt(kGravity / height);
  Vec3 capture = state.centerOfMass + state.centerOfMassVelocity * (1.0 / omega);
  capture.z = 0;
  report.capture = capture;

  Vec3 support{};
  double sagMin = 0, sagMax = 0, latMin = 0, latMax = 0;
  bool supportValid = false;
  supportOf(state, forward, right, support, sagMin, sagMax, latMin, latMax, supportValid);
  report.support = support;
  report.supportValid = supportValid;

  const double sag = supportValid ? dot(capture - support, forward) : 0;
  const double lat = supportValid ? dot(capture - support, right) : 0;
  const double outsideSag = supportValid ? std::max(0.0, std::max(sagMin - sag, sag - sagMax)) : 0;
  const double outsideLat = supportValid ? std::max(0.0, std::max(latMin - lat, lat - latMax)) : 0;
  report.captureError = std::hypot(outsideSag, outsideLat);
  const bool upright = pelvis && pelvis->position.z > 0.62 && pelvisUp > 0.72;
  const bool recoverable = pelvis && pelvis->position.z > 0.55 && pelvisUp > 0.55;
  const bool fallen = !pelvis || pelvis->position.z < 0.45 || pelvisUp < 0.35;

  if (!enabled || fallen) {
    reset();
    standing.setSwingFoot("");
    report.state = StepState::Idle;
    report.capture = capture;
    report.support = support;
    report.supportValid = supportValid;
    report.captureError = std::hypot(outsideSag, outsideLat);
    return report;
  }

  const bool outside = supportValid && report.captureError > kEnterMargin && (upright || (stepsTaken_ > 0 && recoverable));
  const bool inside = supportValid && report.captureError < kExitMargin;
  if (state_ == StepState::Idle) {
    outsideTime_ = outside ? outsideTime_ + dt : 0;
    insideTime_ = inside ? insideTime_ + dt : 0;
    if (insideTime_ > 0.20) {
      armed_ = true;
      stepsTaken_ = 0;
    }
  }

  auto placeFoot = [&](const std::string& side) {
    const BodyKinematics* foot = findBody(state, side + "_foot");
    const BodyKinematics* hip = findBody(state, side + "_upper_leg");
    if (!foot || !hip || !pelvis) return;
    const Side& spec = side == "left" ? leftLeg_ : rightLeg_;
    Vec3 desired = capture + right * (spec.sign * metrics_.stanceWidth) + forward * (0.02 * (sag >= 0 ? 1.0 : -1.0));
    desired.z = 0;
    const double footSag = dot(foot->position - pelvis->position, forward);
    double placeSag = dot(desired - pelvis->position, forward);
    // The minimum step is measured from the foot, not the pelvis. A target that
    // sits under the hip can be behind the foot that is already there.
    if (sag > 0.04) placeSag = std::max(placeSag, footSag + 0.12);
    if (sag < -0.04) placeSag = std::min(placeSag, footSag - 0.10);
    const double placeLat = dot(desired - pelvis->position, right);
    desired = pelvis->position + forward * placeSag + right * placeLat;
    desired.z = 0;
    const BodyKinematics* other = findBody(state, side == "left" ? "right_foot" : "left_foot");
    const PlannedFoot planned = clampFootTarget(pelvis->position, desired, forward, right, hip->position, spec.lengths,
                                                spec.sign, metrics_.ankleHeight, other, metrics_);
    ankleTarget_ = planned.ankle;
    landing_ = planned.landing;
    stepDistance_ = length(Vec3{ankleTarget_.x - foot->position.x, ankleTarget_.y - foot->position.y, 0});
    duration_ = std::clamp(0.18 + 0.10 * stepDistance_, 0.20, 0.32);
  };

  auto begin = [&](const std::string& side) {
    const BodyKinematics* foot = findBody(state, side + "_foot");
    if (!foot || !pelvis) return;
    swing_ = side;
    state_ = StepState::Prepare;
    stateTime_ = 0;
    armed_ = false;
    swingStart_ = foot->position;
    swingWeight_ = 0;
    haveHold_ = false;
    ++stepsTaken_;
    placeFoot(side);
  };

  const bool another = stepsTaken_ > 0 && stepsTaken_ < 3;
  if (state_ == StepState::Idle && (armed_ || another) && outsideTime_ > 0.06) {
    const double leftLoad = footLoad(state, "left");
    const double rightLoad = footLoad(state, "right");
    std::string side;
    if (std::abs(lat) > std::abs(sag) * 0.75) side = lat < 0 ? "left" : "right";
    else if (leftLoad + 30.0 < rightLoad) side = "left";
    else if (rightLoad + 30.0 < leftLoad) side = "right";
    else side = lat < 0 ? "left" : "right";
    begin(side);
  }

  standing.setSwingFoot(state_ == StepState::Idle || state_ == StepState::Settle || state_ == StepState::Plant ? ""
                                                                                                              : swing_);

  if (state_ == StepState::Prepare) {
    stateTime_ += dt;
    Side& specSide = swing_ == "left" ? leftLeg_ : rightLeg_;
    const BodyKinematics* hip = findBody(state, swing_ + "_upper_leg");
    const BodyKinematics* footNow = findBody(state, swing_ + "_foot");
    if (!specSide.calibrated && hip && footNow) {
      calibrateHipBias(specSide.lengths, state, swing_, hip->position, footNow->position, forward, right, up_);
      specSide.calibrated = true;
    }
    if (hip) {
      Vec3 lifted = swingStart_;
      lifted.z = metrics_.ankleHeight + 0.03;
      const LegPose pose = solveLeg(hip->position, lifted, forward, right, up_, specSide.lengths);
      swingWeight_ = std::clamp(stateTime_ / 0.12, 0.0, 1.0);
      commandLegPose(frame, state, swing_, pose, swingWeight_);
    }
    const FootContact& swingFoot = feet.of(swing_);
    if (stateTime_ > 0.06 || swingFoot.mode == FootContactMode::Air || swingFoot.mode == FootContactMode::Releasing ||
        footLoad(state, swing_) < 25.0) {
      placeFoot(swing_);
      state_ = StepState::Swing;
      stateTime_ = 0;
      if (const BodyKinematics* foot = findBody(state, swing_ + "_foot")) swingStart_ = foot->position;
    }
  } else if (state_ == StepState::Swing) {
    stateTime_ += dt;
    const double phase = std::max(0.0, stateTime_ / std::max(0.05, duration_));
    const BodyKinematics* hip = findBody(state, swing_ + "_upper_leg");
    if (hip) {
      const Side& spec = swing_ == "left" ? leftLeg_ : rightLeg_;
      const Vec3 ankle = swingAnkle(swingStart_, ankleTarget_, phase, kClearance);
      LegPose pose = solveLeg(hip->position, ankle, forward, right, up_, spec.lengths);
      // The geometric hip angle under-reads this skeleton once the leg leaves
      // the pose where the bias was measured. Add the remaining sagittal reach.
      const double sagStep = dot(ankle - swingStart_, forward);
      const double latStep = dot(ankle - swingStart_, right);
      pose.hipFlex += -1.8 * sagStep;
      pose.hipAbd += -1.2 * latStep;
      commandLegPose(frame, state, swing_, pose, 1);
    }
    const FootContact& swingFoot = feet.of(swing_);
    // A brief scuff used to end the swing while the foot was still near the start.
    const bool committed = phase > 0.92 && (swingFoot.mode == FootContactMode::Planted || swingFoot.load > 28.0);
    if (committed || phase >= 1.25) {
      const char* suffix[5] = {"_hip_flex", "_hip_abd", "_knee", "_ankle_pitch", "_ankle_roll"};
      for (int i = 0; i < 5; ++i) {
        const JointState* hinge = findHinge(state, swing_ + suffix[i]);
        hold_[i] = hinge ? hinge->angle : 0;
      }
      haveHold_ = true;
      state_ = StepState::Plant;
      stateTime_ = 0;
      swingWeight_ = 1;
    }
    report.phase = std::min(1.0, phase);
  } else if (state_ == StepState::Plant) {
    stateTime_ += dt;
    // Once the sole is planted, stop chasing the world target. The remaining
    // swing command only holds the joint angles from the contact frame, and
    // that hold fades so the standing balance torque takes the leg back.
    swingWeight_ = 1.0 - std::clamp(stateTime_ / 0.08, 0.0, 1.0);
    if (haveHold_) commandLegAngles(frame, state, swing_, hold_, swingWeight_);
    if (swingWeight_ <= 0.02 || stateTime_ > 0.10) {
      const char* suffix[5] = {"_hip_flex", "_hip_abd", "_knee", "_ankle_pitch", "_ankle_roll"};
      for (const char* name : suffix) {
        if (const JointState* hinge = findHinge(state, swing_ + name)) {
          standing.shiftReference(hinge->name, hinge->angle);
          standing.noteAnkle(hinge->name, hinge->angle);
        }
      }
      state_ = StepState::Settle;
      stateTime_ = 0;
      swingWeight_ = 0;
      standing.setSwingFoot("");
    }
    report.phase = 1;
  } else if (state_ == StepState::Settle) {
    stateTime_ += dt;
    if (stateTime_ > 0.08) {
      state_ = StepState::Idle;
      standing.setSwingFoot("");
      swing_.clear();
      stateTime_ = 0;
      outsideTime_ = 0;
      insideTime_ = 0;
      haveHold_ = false;
    }
    report.phase = 1;
  }

  report.state = state_;
  report.swingFoot = state_ == StepState::Idle ? "" : swing_;
  report.landing = landing_;
  report.landingValid = state_ != StepState::Idle;
  report.stepDistance = stepDistance_;
  report.duration = duration_;
  if (state_ == StepState::Swing || state_ == StepState::Prepare || state_ == StepState::Plant) {
    for (int i = 0; i < 8; ++i) {
      report.path[i] = swingAnkle(swingStart_, ankleTarget_, static_cast<double>(i) / 7.0, kClearance);
    }
    report.pathCount = 8;
  }
  return report;
}

}  // namespace pm
