#include "control/standing_controller.h"

#include "control/foot_placement.h"
#include "control/swing_command.h"
#include "physics/physical_skeleton.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <unordered_map>

namespace pm {
namespace {

constexpr double kPi = 3.14159265358979323846;
// Pose springs only keep the body recognizable. Balance force holds the COM.
constexpr double kSpineLean = 4.0;
constexpr double kSpineRate = 2.0;
constexpr double kNeckLean = 2.0;

struct Indexed {
  std::unordered_map<std::string, const JointState*> joints;
  std::unordered_map<std::string, const BodyKinematics*> bodies;
};

Indexed indexState(const BodyState& state) {
  Indexed indexed;
  for (const JointState& joint : state.joints) indexed.joints.emplace(joint.name, &joint);
  for (const BodyKinematics& body : state.bodies) indexed.bodies.emplace(body.name, &body);
  return indexed;
}

const JointState* findJoint(const Indexed& indexed, const std::string& name) {
  const auto found = indexed.joints.find(name);
  return found == indexed.joints.end() ? nullptr : found->second;
}

const BodyKinematics* findBody(const Indexed& indexed, const std::string& name) {
  const auto found = indexed.bodies.find(name);
  return found == indexed.bodies.end() ? nullptr : found->second;
}

Vec3 rotateByBody(const BodyKinematics& body, Vec3 local) {
  const double* r = body.rotation;
  return {r[0] * local.x + r[1] * local.y + r[2] * local.z, r[3] * local.x + r[4] * local.y + r[5] * local.z,
          r[6] * local.x + r[7] * local.y + r[8] * local.z};
}

bool geomIs(const std::string& geom, const char* token) { return geom.find(token) != std::string::npos; }

bool footFloorContact(const ContactState& contact, const char* footToken) {
  const bool foot = geomIs(contact.geom1, footToken) || geomIs(contact.geom2, footToken);
  const bool floor = contact.geom1 == "floor" || contact.geom2 == "floor";
  return foot && floor && contact.position.z < 0.12 && contact.normalForce > 1.0;
}

double clampToRange(const JointState& joint, double target) {
  if (!(joint.rangeMin < joint.rangeMax)) return target;
  const double margin = 0.03;
  if (joint.rangeMax - joint.rangeMin <= 2.0 * margin) return 0.5 * (joint.rangeMin + joint.rangeMax);
  return std::clamp(target, joint.rangeMin + margin, joint.rangeMax - margin);
}

}  // namespace

const char* balanceLabel(BalanceStatus status) {
  switch (status) {
    case BalanceStatus::Stable:
      return "Stable";
    case BalanceStatus::Unstable:
      return "Unstable";
    case BalanceStatus::NoSupport:
      return "No Support";
  }
  return "No Support";
}

StandingController::Tune StandingController::tuneFor(const std::string& jointName) const {
  // Legs carry the body. Arms only hold the imported T-pose.
  if (jointName.find("ankle") != std::string::npos) return {70.0, 8.0, 70.0};
  if (jointName.find("hip_abd") != std::string::npos) return {220.0, 18.0, 120.0};
  if (jointName.find("hip_flex") != std::string::npos) return {200.0, 16.0, 100.0};
  if (jointName.find("hip") != std::string::npos) return {50.0, 5.0, 30.0};
  if (jointName.find("knee") != std::string::npos) return {400.0, 30.0, 110.0};
  if (jointName.find("spine") != std::string::npos) return {40.0, 6.0, 40.0};
  if (jointName.find("neck") != std::string::npos) return {20.0, 2.0, 12.0};
  if (jointName.find("shoulder") != std::string::npos) return {55.0, 6.0, 28.0};
  if (jointName.find("elbow") != std::string::npos) return {24.0, 2.2, 14.0};
  if (jointName.find("wrist") != std::string::npos) return {6.0, 0.6, 5.0};
  return {16.0, 1.5, 12.0};
}

double StandingController::referenceTarget(const std::string& jointName) const {
  const auto found = targets_.find(jointName);
  return found == targets_.end() ? 0 : found->second;
}

void StandingController::shiftReference(const std::string& jointName, double angle) {
  const auto found = targets_.find(jointName);
  if (found == targets_.end() || !std::isfinite(angle)) return;
  found->second = angle;
}

void StandingController::relaxReferences(double dt) {
  if (dt <= 0 || holdReferences_) return;
  const double follow = 1.0 - std::exp(-dt / 0.55);
  for (auto& entry : targets_) {
    const auto home = home_.find(entry.first);
    if (home == home_.end()) continue;
    entry.second += (home->second - entry.second) * follow;
  }
}

void StandingController::noteAnkle(const std::string& jointName, double angle) {
  if (!std::isfinite(angle) || jointName.find("ankle_pitch") == std::string::npos) return;
  const double held = std::clamp(angle, 0.05, 0.42);
  if (jointName.rfind("left_", 0) == 0) ankleHoldLeft_ = held;
  if (jointName.rfind("right_", 0) == 0) ankleHoldRight_ = held;
}

double StandingController::targetFor(const std::string& jointName, double currentAngle) const {
  const auto found = targets_.find(jointName);
  return found == targets_.end() ? currentAngle : found->second;
}

void StandingController::resetFilters() {
  hipSagittal_ = 0;
  hipLateral_ = 0;
  lastTime_ = -1;
  peakTorque_ = 0;
  ankleHoldLeft_ = 0.234;
  ankleHoldRight_ = 0.234;
}

void StandingController::capture(const SkeletonDefinition& skeleton, const BodyState& state) {
  targets_.clear();
  resetFilters();
  restRight_ = skeleton.right;
  restForward_ = skeleton.forward;
  restUp_ = skeleton.up;
  if (length(restUp_) < 0.5) restUp_ = {0, 0, 1};
  if (length(restRight_) < 0.5) restRight_ = {1, 0, 0};
  if (length(restForward_) < 0.5) restForward_ = {0, 1, 0};

  const Indexed indexed = indexState(state);
  for (const JointState& joint : state.joints) {
    if (joint.type != "hinge") continue;
    targets_[joint.name] = joint.angle;
  }

  Vec3 forward = restForward_;
  Vec3 right = restRight_;
  Vec3 up = restUp_;
  forward.z = 0;
  right.z = 0;
  forward = normalize(forward);
  right = normalize(right);
  if (length(forward) < 0.5) forward = {0, -1, 0};
  if (length(right) < 0.5) right = {-1, 0, 0};
  for (const char* side : {"left", "right"}) {
    const BodyKinematics* hipBody = findBody(state, std::string(side) + "_upper_leg");
    const BodyKinematics* kneeBody = findBody(state, std::string(side) + "_lower_leg");
    const BodyKinematics* footBody = findBody(state, std::string(side) + "_foot");
    const JointState* knee = findJoint(indexed, std::string(side) + "_knee");
    const JointState* hip = findJoint(indexed, std::string(side) + "_hip_flex");
    if (!hipBody || !kneeBody || !footBody || !knee || !hip) continue;
    LegLengths lengths;
    lengths.thigh = std::max(0.2, length(kneeBody->position - hipBody->position));
    lengths.shin = std::max(0.2, length(footBody->position - kneeBody->position));
    lengths.kneeSign = std::abs(knee->rangeMin) >= std::abs(knee->rangeMax) ? -1.0 : 1.0;
    calibrateHipBias(lengths, state, side, hipBody->position, footBody->position, forward, right, up);
    // Slight knee flexion with the ankle directly under the hip. The bind pose
    // is already nearly stacked; this does not copy the settled 13 cm offset.
    const double bend = 10.0 * kPi / 180.0;
    const double span = std::sqrt(lengths.thigh * lengths.thigh + lengths.shin * lengths.shin +
                                   2.0 * lengths.thigh * lengths.shin * std::cos(bend));
    Vec3 ankle = hipBody->position;
    ankle.z = hipBody->position.z - span;
    const LegPose pose = solveLeg(hipBody->position, ankle, forward, right, up, lengths);
    auto set = [&](const std::string& name, double angle) {
      if (const JointState* hinge = findJoint(indexed, name)) targets_[name] = clampToRange(*hinge, angle);
    };
    const bool holdBind = false;
    if (!holdBind) {
      set(std::string(side) + "_hip_flex", pose.hipFlex);
      set(std::string(side) + "_hip_abd", pose.hipAbd);
      set(std::string(side) + "_knee", pose.knee);
      set(std::string(side) + "_ankle_pitch", pose.anklePitch);
      set(std::string(side) + "_ankle_roll", pose.ankleRoll);
    }
    std::cout << "Standing reference: " << side << "_knee " << targets_[std::string(side) + "_knee"] << " rad, "
              << side << "_hip_flex " << targets_[std::string(side) + "_hip_flex"] << " rad, " << side
              << "_ankle_pitch " << targets_[std::string(side) + "_ankle_pitch"] << " rad (stacked span " << span
              << " m)\n";
  }
  home_ = targets_;
}

StandingReport StandingController::diagnose(const BodyState& state) const {
  StandingReport report;
  report.peakTorque = peakTorque_;
  report.comSpeedX = state.centerOfMassVelocity.x;
  report.comSpeedY = state.centerOfMassVelocity.y;

  Vec3 leftWeighted{0, 0, 0};
  Vec3 rightWeighted{0, 0, 0};
  double leftWeight = 0;
  double rightWeight = 0;
  for (const ContactState& contact : state.contacts) {
    const bool left = footFloorContact(contact, "left_foot");
    const bool right = footFloorContact(contact, "right_foot");
    if (!left && !right) continue;
    const double force = std::max(0.0, contact.normalForce);
    if (left) {
      report.leftFootContact = true;
      leftWeighted = leftWeighted + contact.position * force;
      leftWeight += force;
    }
    if (right) {
      report.rightFootContact = true;
      rightWeighted = rightWeighted + contact.position * force;
      rightWeight += force;
    }
  }
  // Each foot's center is force-weighted. If both feet are down, the balance
  // target is the midpoint, so load does not chase the heavier foot.
  // Use the ankle of each planted foot. A flat sole reports contact at whichever
  // corner is loaded, and chasing that corner walks the body off the foot.
  int feet = 0;
  Vec3 support{0, 0, 0};
  const Indexed supportBodies = indexState(state);
  auto addPlanted = [&](double weight, const char* name) {
    if (weight <= 1.0) return;
    const BodyKinematics* foot = findBody(supportBodies, name);
    if (!foot) return;
    support = support + foot->position;
    ++feet;
  };
  addPlanted(leftWeight, "left_foot");
  addPlanted(rightWeight, "right_foot");
  if (feet > 0) {
    report.supportValid = true;
    report.supportCenter = support / static_cast<double>(feet);
  }
  if (stanceFocus_ == "left" || stanceFocus_ == "right") {
    const bool focusLeft = stanceFocus_ == "left";
    const double weight = focusLeft ? leftWeight : rightWeight;
    const BodyKinematics* foot = findBody(supportBodies, focusLeft ? "left_foot" : "right_foot");
    if (foot && weight > 1.0) {
      report.supportValid = true;
      const Vec3 sole = foot->position;
      if (feet > 0) {
        const Vec3 mid = report.supportCenter;
        report.supportCenter = mid * (1.0 - stanceBias_) + sole * stanceBias_;
      } else {
        report.supportCenter = sole;
      }
    }
  }
  if (report.supportValid) {
    report.supportCenter = report.supportCenter + supportShift_;
    report.supportCenter.z = 0;
    report.comErrorX = report.supportCenter.x - state.centerOfMass.x;
    report.comErrorY = report.supportCenter.y - state.centerOfMass.y;
  }

  const Indexed indexed = indexState(state);
  const BodyKinematics* pelvis = findBody(indexed, "pelvis");
  if (pelvis) report.pelvisUp = rotateByBody(*pelvis, restUp_).z;

  const double horizontal = std::hypot(report.comErrorX, report.comErrorY);
  const bool anyFoot = report.leftFootContact || report.rightFootContact;
  if (!anyFoot || !report.supportValid) {
    report.balance = BalanceStatus::NoSupport;
  } else if (report.pelvisUp > 0.92 && horizontal < 0.09 && pelvis && pelvis->position.z > 0.72) {
    report.balance = BalanceStatus::Stable;
  } else {
    report.balance = BalanceStatus::Unstable;
  }
  return report;
}

StandingReport StandingController::update(const BodyState& state, ControlFrame& frame) {
  StandingReport report = diagnose(state);
  report.active = true;
  peakTorque_ = 0;

  const Indexed indexed = indexState(state);
  const BodyKinematics* pelvis = findBody(indexed, "pelvis");
  const BodyKinematics* chest = findBody(indexed, "chest");
  const BodyKinematics* head = findBody(indexed, "head");

  Vec3 forward{0, 1, 0};
  Vec3 right{1, 0, 0};
  Vec3 pelvisUp{0, 0, 1};
  Vec3 chestUp{0, 0, 1};
  Vec3 headUp{0, 0, 1};
  Vec3 angular{0, 0, 0};
  bool directional = false;
  if (pelvis) {
    forward = rotateByBody(*pelvis, restForward_);
    right = rotateByBody(*pelvis, restRight_);
    pelvisUp = rotateByBody(*pelvis, restUp_);
    angular = pelvis->angularVelocity;
    forward.z = 0;
    right.z = 0;
    forward = normalize(forward);
    right = normalize(right);
    directional = length(forward) > 0.5 && length(right) > 0.5 && pelvisUp.z > 0.35;
  }
  if (chest) chestUp = rotateByBody(*chest, restUp_);
  if (head) headUp = rotateByBody(*head, restUp_);

  const double chestSag = directional ? dot(chestUp, forward) : 0;
  const double chestLat = directional ? dot(chestUp, right) : 0;
  const double headSag = directional ? dot(headUp, forward) : 0;
  const double headLat = directional ? dot(headUp, right) : 0;
  const double sagRate = directional ? -dot(angular, right) : 0;
  const double latRate = directional ? dot(angular, forward) : 0;
  const bool supported = report.supportValid && directional && (report.leftFootContact || report.rightFootContact);

  const double dt = lastTime_ >= 0 ? std::clamp(state.time - lastTime_, 0.0, 0.05) : 1.0 / 120.0;
  lastTime_ = state.time;
  relaxReferences(dt);

  for (const JointState& joint : state.joints) {
    if (joint.type != "hinge") continue;
    const Tune tune = tuneFor(joint.name);
    double target = targetFor(joint.name, joint.angle);
    const bool swinging = (!swingFoot_.empty() && joint.name.rfind(swingFoot_ + "_", 0) == 0);
    if (!swinging && joint.name.find("knee") != std::string::npos && kneeExtra_ > 0 &&
        (stanceFocus_.empty() || joint.name.rfind(stanceFocus_ + "_", 0) == 0)) {
      target -= kneeExtra_;
    }
    if (!swinging && pelvisLean_ != 0 && joint.name.find("hip_flex") != std::string::npos) target -= pelvisLean_;
    double feedforward = 0;
    // A light torso regularizer. It does not hold the COM.
    if (supported && joint.name.find("spine") != std::string::npos) {
      if (joint.name.find("_flex") != std::string::npos) feedforward = kSpineLean * chestSag + kSpineRate * sagRate;
      else if (joint.name.find("_abd") != std::string::npos) feedforward = -kSpineLean * chestLat - kSpineRate * latRate;
    }
    if (supported && joint.name.find("neck") != std::string::npos) {
      if (joint.name.find("_flex") != std::string::npos) feedforward = kNeckLean * headSag;
      if (joint.name.find("_abd") != std::string::npos) feedforward = -kNeckLean * headLat;
    }
    if (!std::isfinite(feedforward)) feedforward = 0;
    JointCommand command;
    command.target = target;
    command.kp = tune.kp;
    command.kd = tune.kd;
    command.limit = tune.limit;
    command.feedforward = feedforward;
    frame.setStance(joint.name, command);
  }

  report.peakTorque = peakTorque_;
  report.active = true;
  return report;
}

}  // namespace pm
