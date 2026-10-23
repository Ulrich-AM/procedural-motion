#include "control/recovery_step.h"

#include "physics/physical_skeleton.h"

#include <algorithm>
#include <cmath>

namespace pm {
namespace {

constexpr double kGravity = 9.81;
constexpr double kPi = 3.14159265358979323846;
constexpr double kToe = 0.09;
constexpr double kHeel = 0.09;
constexpr double kHalfWidth = 0.040;
constexpr double kAnkleHeight = 0.074;
constexpr double kClearance = 0.08;
constexpr double kEnterMargin = 0.028;
constexpr double kExitMargin = 0.02;

double smoothstep(double phase) {
  const double s = std::clamp(phase, 0.0, 1.0);
  return s * s * (3.0 - 2.0 * s);
}

double kneeSignOf(const JointState* knee) {
  if (!knee || !(knee->rangeMin < knee->rangeMax)) return -1;
  if (std::abs(knee->rangeMin) > std::abs(knee->rangeMax) + 1e-4) return knee->rangeMin < 0 ? -1.0 : 1.0;
  if (std::abs(knee->rangeMax) > std::abs(knee->rangeMin) + 1e-4) return knee->rangeMax > 0 ? 1.0 : -1.0;
  return -1;
}

bool footContact(const ContactState& contact, const std::string& side) {
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
  duration_ = 0.45;
  swingStart_ = {};
  landing_ = {};
  ankleTarget_ = {};
  stepDistance_ = 0;
  outsideTime_ = 0;
  insideTime_ = 0;
}

void RecoveryStep::capture(const SkeletonDefinition& skeleton, const BodyState& state) {
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
    const BodyKinematics* hip = body(state, side.name + "_upper_leg");
    const BodyKinematics* knee = body(state, side.name + "_lower_leg");
    const BodyKinematics* foot = body(state, side.name + "_foot");
    if (hip && knee) side.lengths.thigh = std::max(0.2, length(knee->position - hip->position));
    if (knee && foot) side.lengths.shin = std::max(0.2, length(foot->position - knee->position));
    side.lengths.kneeSign = kneeSignOf(joint(state, side.name + "_knee"));
  };
  measure(leftLeg_, "left", -1);
  measure(rightLeg_, "right", 1);
  leftLeg_.calibrated = false;
  rightLeg_.calibrated = false;
  captured_ = true;
}

Vec3 RecoveryStep::axisOf(const BodyKinematics& body, Vec3 local) const {
  const double* r = body.rotation;
  return {r[0] * local.x + r[1] * local.y + r[2] * local.z, r[3] * local.x + r[4] * local.y + r[5] * local.z,
          r[6] * local.x + r[7] * local.y + r[8] * local.z};
}

const BodyKinematics* RecoveryStep::body(const BodyState& state, const std::string& name) const {
  for (const BodyKinematics& item : state.bodies) {
    if (item.name == name) return &item;
  }
  return nullptr;
}

const JointState* RecoveryStep::joint(const BodyState& state, const std::string& name) const {
  for (const JointState& item : state.joints) {
    if (item.name == name) return &item;
  }
  return nullptr;
}

double RecoveryStep::footLoad(const BodyState& state, const std::string& side) const {
  double load = 0;
  for (const ContactState& contact : state.contacts) {
    if (footContact(contact, side)) load += std::max(0.0, contact.normalForce);
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
    const BodyKinematics* foot = body(state, side + "_foot");
    if (!foot) return;
    const Vec3 origin = foot->position;
    include(origin + forward * kToe);
    include(origin - forward * kHeel);
    include(origin + right * kHalfWidth);
    include(origin - right * kHalfWidth);
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
    const BodyKinematics* foot = body(state, side + "_foot");
    if (!foot) return;
    const Vec3 origin = foot->position;
    const Vec3 corners[4] = {origin + forward * kToe, origin - forward * kHeel, origin + right * kHalfWidth,
                             origin - right * kHalfWidth};
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

void RecoveryStep::trackSwing(const BodyState& state, StandingController& standing, PhysicalSkeleton& physics,
                              const LegPose& pose, double blend) {
  const char* names[5] = {"_hip_flex", "_hip_abd", "_knee", "_ankle_pitch", "_ankle_roll"};
  const double targets[5] = {pose.hipFlex, pose.hipAbd, pose.knee, pose.anklePitch, pose.ankleRoll};
  for (int i = 0; i < 5; ++i) {
    const JointState* hinge = joint(state, swing_ + names[i]);
    if (!hinge) continue;
    const double limited = std::clamp(targets[i], hinge->rangeMin + 0.02, hinge->rangeMax - 0.02);
    standing.trackHinge(*hinge, limited, blend, physics);
  }
}

RecoveryReport RecoveryStep::update(const BodyState& state, StandingController& standing, PhysicalSkeleton& physics,
                                    bool enabled) {
  RecoveryReport report;
  report.enabled = enabled;
  if (!captured_) return report;

  const BodyKinematics* pelvis = body(state, "pelvis");
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
  const bool fallen = !pelvis || pelvis->position.z < 0.45 || pelvisUp < 0.35;
  const double dt = 1.0 / 120.0;

  if (!enabled || fallen) {
    if (fallen || !enabled) reset();
    standing.setSwingFoot("");
    report.state = StepState::Idle;
    return report;
  }

  const bool outside = supportValid && report.captureError > kEnterMargin && upright;
  const bool inside = supportValid && report.captureError < kExitMargin;
  if (state_ == StepState::Idle) {
    outsideTime_ = outside ? outsideTime_ + dt : 0;
    insideTime_ = inside ? insideTime_ + dt : 0;
    if (insideTime_ > 0.20) armed_ = true;
  }

  auto placeFoot = [&](const std::string& side) {
    const BodyKinematics* foot = body(state, side + "_foot");
    const BodyKinematics* hip = body(state, side + "_upper_leg");
    if (!foot || !hip || !pelvis) return;
    const Side& spec = side == "left" ? leftLeg_ : rightLeg_;
    Vec3 desired = capture + right * (spec.sign * 0.11) + forward * (0.02 * (sag >= 0 ? 1.0 : -1.0));
    desired.z = 0;
    const double leg = spec.lengths.thigh + spec.lengths.shin;
    const double vertical = std::max(0.20, hip->position.z - kAnkleHeight);
    const double reach = 0.96 * leg;
    const double reachable = std::sqrt(std::max(0.03, reach * reach - vertical * vertical));
    double placeSag = dot(desired - pelvis->position, forward);
    double placeLat = dot(desired - pelvis->position, right);
    placeSag = std::clamp(placeSag, -std::min(0.16, reachable), std::min(0.22, reachable));
    const BodyKinematics* other = body(state, side == "left" ? "right_foot" : "left_foot");
    if (other) {
      const double otherLat = dot(other->position - pelvis->position, right);
      if (spec.sign < 0) placeLat = std::min(placeLat, otherLat - 0.11);
      else placeLat = std::max(placeLat, otherLat + 0.11);
    }
    const double latLimit = std::min(0.28, std::max(0.08, reachable));
    placeLat = std::clamp(placeLat, spec.sign < 0 ? -latLimit : 0.06, spec.sign < 0 ? -0.06 : latLimit);
    const double horiz = std::hypot(placeSag, placeLat);
    if (horiz > reachable && horiz > 1e-4) {
      placeSag *= reachable / horiz;
      placeLat *= reachable / horiz;
    }
    desired = pelvis->position + forward * placeSag + right * placeLat;
    desired.z = 0;
    ankleTarget_ = desired + Vec3{0, 0, kAnkleHeight};
    landing_ = desired;
    if (swingStart_.x == 0 && swingStart_.y == 0) swingStart_ = foot->position;
    stepDistance_ = length(Vec3{ankleTarget_.x - foot->position.x, ankleTarget_.y - foot->position.y, 0});
    duration_ = std::clamp(0.26 + 0.22 * stepDistance_, 0.28, 0.42);
  };

  auto holdLeg = [&](const std::string& side) {
    const char* names[5] = {"_hip_flex", "_hip_abd", "_knee", "_ankle_pitch", "_ankle_roll"};
    for (const char* suffix : names) {
      if (const JointState* hinge = joint(state, side + suffix)) {
        standing.shiftReference(hinge->name, hinge->angle);
        standing.noteAnkle(hinge->name, hinge->angle);
      }
    }
  };

  auto begin = [&](const std::string& side) {
    const BodyKinematics* foot = body(state, side + "_foot");
    if (!foot || !pelvis) return;
    swing_ = side;
    state_ = StepState::Prepare;
    stateTime_ = 0;
    armed_ = false;
    swingStart_ = foot->position;
    holdLeg(side == "left" ? "right" : "left");
    placeFoot(side);
  };

  if (state_ == StepState::Idle && armed_ && outsideTime_ > 0.06) {
    const double leftLoad = footLoad(state, "left");
    const double rightLoad = footLoad(state, "right");
    std::string side;
    if (std::abs(lat) > std::abs(sag) * 0.75) {
      side = lat < 0 ? "left" : "right";
    } else if (leftLoad + 30.0 < rightLoad) {
      side = "left";
    } else if (rightLoad + 30.0 < leftLoad) {
      side = "right";
    } else {
      side = lat < 0 ? "left" : "right";
    }
    begin(side);
  }

  standing.setSwingFoot(state_ == StepState::Idle || state_ == StepState::Settle ? "" : swing_);

  auto adoptLanded = [&]() {
    const char* names[5] = {"_hip_flex", "_hip_abd", "_knee", "_ankle_pitch", "_ankle_roll"};
    for (const char* suffix : names) {
      if (const JointState* hinge = joint(state, swing_ + suffix)) {
        standing.shiftReference(hinge->name, hinge->angle);
        standing.noteAnkle(hinge->name, hinge->angle);
      }
    }
  };

  if (state_ == StepState::Prepare) {
    stateTime_ += dt;
    Side& specSide = swing_ == "left" ? leftLeg_ : rightLeg_;
    const BodyKinematics* hip = body(state, swing_ + "_upper_leg");
    const BodyKinematics* footNow = body(state, swing_ + "_foot");
    if (!specSide.calibrated && hip && footNow) {
      const LegPose echo = solveLeg(hip->position, footNow->position, forward, right, up_, specSide.lengths);
      if (const JointState* hipJ = joint(state, swing_ + "_hip_flex")) {
        specSide.lengths.hipFlexBias = hipJ->angle - echo.hipFlex;
      }
      if (const JointState* abdJ = joint(state, swing_ + "_hip_abd")) {
        specSide.lengths.hipAbdBias = abdJ->angle - echo.hipAbd;
      }
      specSide.calibrated = true;
    }
    if (hip) {
      Vec3 lifted = swingStart_;
      lifted.z = kAnkleHeight + 0.03;
      const Side& spec = swing_ == "left" ? leftLeg_ : rightLeg_;
      const LegPose pose = solveLeg(hip->position, lifted, forward, right, up_, spec.lengths);
      trackSwing(state, standing, physics, pose, std::clamp(stateTime_ / 0.12, 0.0, 1.0));
    }
    if (stateTime_ > 0.10 || footLoad(state, swing_) < 25.0) {
      placeFoot(swing_);
      state_ = StepState::Swing;
      stateTime_ = 0;
      if (const BodyKinematics* foot = body(state, swing_ + "_foot")) swingStart_ = foot->position;
    }
  } else if (state_ == StepState::Swing) {
    stateTime_ += dt;
    const double phase = std::clamp(stateTime_ / duration_, 0.0, 1.15);
    const double s = smoothstep(std::min(1.0, phase));
    Vec3 ankle = swingStart_ + (ankleTarget_ - swingStart_) * s;
    ankle.z = swingStart_.z + (kAnkleHeight - swingStart_.z) * s + kClearance * 4.0 * s * (1.0 - s);
    const BodyKinematics* hip = body(state, swing_ + "_upper_leg");
    if (hip) {
      const Side& spec = swing_ == "left" ? leftLeg_ : rightLeg_;
      trackSwing(state, standing, physics, solveLeg(hip->position, ankle, forward, right, up_, spec.lengths), 1);
    }
    const bool late = phase > 0.62;
    if ((late && footLoad(state, swing_) > 40.0 && ankle.z < kAnkleHeight + 0.04) || phase >= 1.15) {
      state_ = StepState::Plant;
      stateTime_ = 0;
    }
    report.phase = std::min(1.0, phase);
  } else if (state_ == StepState::Plant) {
    stateTime_ += dt;
    const BodyKinematics* hip = body(state, swing_ + "_upper_leg");
    if (hip) {
      const Side& spec = swing_ == "left" ? leftLeg_ : rightLeg_;
      Vec3 ankle = ankleTarget_;
      ankle.z = kAnkleHeight;
      trackSwing(state, standing, physics, solveLeg(hip->position, ankle, forward, right, up_, spec.lengths), 1);
    }
    if (footLoad(state, swing_) > 40.0 || stateTime_ > 0.12) {
      adoptLanded();
      state_ = StepState::Settle;
      stateTime_ = 0;
      standing.setSwingFoot("");
    }
    report.phase = 1;
  } else if (state_ == StepState::Settle) {
    stateTime_ += dt;
    if (stateTime_ > 0.35) {
      state_ = StepState::Idle;
      standing.setSwingFoot("");
      swing_.clear();
      stateTime_ = 0;
      outsideTime_ = 0;
      insideTime_ = 0;
    }
    report.phase = 1;
  }

  report.state = state_;
  report.swingFoot = state_ == StepState::Idle ? "" : swing_;
  report.landing = landing_;
  report.landingValid = state_ != StepState::Idle;
  report.stepDistance = stepDistance_;
  if (state_ == StepState::Swing || state_ == StepState::Prepare || state_ == StepState::Plant) {
    for (int i = 0; i < 8; ++i) {
      const double phase = static_cast<double>(i) / 7.0;
      const double s = smoothstep(phase);
      Vec3 point = swingStart_ + (ankleTarget_ - swingStart_) * s;
      point.z += kClearance * 4.0 * s * (1.0 - s);
      report.path[i] = point;
    }
    report.pathCount = 8;
  }
  return report;
}

}  // namespace pm
