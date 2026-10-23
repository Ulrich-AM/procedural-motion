#include "control/standing_controller.h"

#include "physics/physical_skeleton.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <unordered_map>

namespace pm {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kKneeBend = 10.0 * kPi / 180.0;
constexpr double kGravity = 9.81;

// Ankle/hip feedback gains. These scale the gravity moment m*g*capture, then
// the joint torque limit clamps the result. They are not stiff pose springs.
// Capture is a COM-meter gain on m*g. Both ankles share it, so the total moment
// is k * m * g * capture. The pose springs add a little more. Rate damping stays
// modest so a landing spike cannot spend the whole ankle torque budget.
constexpr double kAnkleCapture = 0.8;
constexpr double kAnkleLateral = 2.4;
constexpr double kAnkleLean = 8.0;
constexpr double kAnkleRate = 4.0;
constexpr double kAnkleLatRate = 4.0;
constexpr double kHipCapture = 0.04;
constexpr double kHipLean = 12.0;
constexpr double kHipFilter = 0.28;
constexpr double kSpineLean = 22.0;
constexpr double kSpineRate = 6.0;
constexpr double kNeckLean = 6.0;

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

// Flexion is whichever side of the limit is larger. A straight reset pose is 0.
double flexionSign(const JointState& joint) {
  if (!(joint.rangeMin < joint.rangeMax)) return 0;
  if (std::abs(joint.rangeMin) > std::abs(joint.rangeMax) + 1e-4) return joint.rangeMin < 0 ? -1.0 : 1.0;
  if (std::abs(joint.rangeMax) > std::abs(joint.rangeMin) + 1e-4) return joint.rangeMax > 0 ? 1.0 : -1.0;
  return 0;
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
  if (jointName.find("ankle") != std::string::npos) return {80.0, 8.0, 70.0};
  if (jointName.find("hip_abd") != std::string::npos) return {650.0, 50.0, 120.0};
  if (jointName.find("hip_flex") != std::string::npos) return {300.0, 24.0, 100.0};
  if (jointName.find("hip") != std::string::npos) return {80.0, 8.0, 30.0};
  if (jointName.find("knee") != std::string::npos) return {420.0, 36.0, 110.0};
  if (jointName.find("spine") != std::string::npos) return {80.0, 8.0, 40.0};
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

  for (const char* side : {"left", "right"}) {
    const std::string kneeName = std::string(side) + "_knee";
    const std::string ankleName = std::string(side) + "_ankle_pitch";
    const JointState* knee = findJoint(indexed, kneeName);
    const JointState* ankle = findJoint(indexed, ankleName);
    if (!knee) continue;
    const double sign = flexionSign(*knee);
    const double kneeTarget = clampToRange(*knee, sign * kKneeBend);
    targets_[kneeName] = kneeTarget;
    const std::string hipName = std::string(side) + "_hip_flex";
    const JointState* hip = findJoint(indexed, hipName);
    // A near-straight leg keeps the flat sole under the pelvis. The ankle
    // cancels the remaining knee bend so the sole does not rock onto a corner.
    const double hipTarget = -0.06;
    if (hip) targets_[hipName] = clampToRange(*hip, hipTarget);
    if (ankle) targets_[ankleName] = clampToRange(*ankle, -(hipTarget + kneeTarget));
    std::cout << "Standing reference: " << kneeName << " " << targets_[kneeName] << " rad, " << hipName << " "
              << (hip ? targets_[hipName] : 0) << " rad, " << ankleName << " " << (ankle ? targets_[ankleName] : 0)
              << " rad (flexion sign " << sign << ")\n";
  }
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

StandingReport StandingController::update(const BodyState& state, PhysicalSkeleton& physics) {
  physics.clearJointTorques();
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

  const double sagErr = directional ? dot(report.supportCenter - state.centerOfMass, forward) : 0;
  const double latErr = directional ? dot(report.supportCenter - state.centerOfMass, right) : 0;
  const double sagVel = directional ? dot(state.centerOfMassVelocity, forward) : 0;
  const double latVel = directional ? dot(state.centerOfMassVelocity, right) : 0;
  const double sagLean = directional ? dot(pelvisUp, forward) : 0;
  const double latLean = directional ? dot(pelvisUp, right) : 0;
  const double chestSag = directional ? dot(chestUp, forward) : 0;
  const double chestLat = directional ? dot(chestUp, right) : 0;
  const double headSag = directional ? dot(headUp, forward) : 0;
  const double headLat = directional ? dot(headUp, right) : 0;
  // d/dt of forward lean is -dot(angularVelocity, right) when right/forward/up are orthonormal.
  const double sagRate = directional ? -dot(angular, right) : 0;
  const double latRate = directional ? dot(angular, forward) : 0;

  const double dt = lastTime_ >= 0 ? std::clamp(state.time - lastTime_, 0.0, 0.05) : 1.0 / 120.0;
  lastTime_ = state.time;
  const double blend = 1.0 - std::exp(-dt / kHipFilter);
  const bool supported = report.supportValid && directional && (report.leftFootContact || report.rightFootContact);
  const double height = pelvis ? std::clamp(pelvis->position.z, 0.45, 1.15) : 0.9;
  const double omega = std::sqrt(kGravity / height);
  const double sagCapture = sagErr - sagVel / omega;
  const double latCapture = latErr - latVel / omega;
  if (supported) {
    hipSagittal_ += (sagCapture - hipSagittal_) * blend;
    hipLateral_ += (latCapture - hipLateral_) * blend;
  } else {
    hipSagittal_ *= 1.0 - blend;
    hipLateral_ *= 1.0 - blend;
  }

  const int feet = (report.leftFootContact ? 1 : 0) + (report.rightFootContact ? 1 : 0);
  const double share = feet > 0 ? 1.0 / static_cast<double>(feet) : 0;
  const double weight = std::max(1.0, state.mass) * kGravity;
  double footLoad = 0;
  for (const ContactState& contact : state.contacts) {
    if (footFloorContact(contact, "left_foot") || footFloorContact(contact, "right_foot")) {
      footLoad += std::max(0.0, contact.normalForce);
    }
  }
  const double impactScale = footLoad > 1.8 * weight ? 0.35 : 1.0;

  for (const JointState& joint : state.joints) {
    if (joint.type != "hinge") continue;
    const Tune tune = tuneFor(joint.name);
    double target = targetFor(joint.name, joint.angle);
    // The flat sole settles a little more dorsiflexed than the landing reference.
    // Keep the spring on that settled angle so it does not cancel the capture torque.
    if (joint.name.find("ankle_pitch") != std::string::npos) {
      double& hold = joint.name.rfind("left_", 0) == 0 ? ankleHoldLeft_ : ankleHoldRight_;
      if (state.time > 1.2 && state.time < 2.0 && std::abs(joint.velocity) < 1.2) {
        const double follow = 1.0 - std::exp(-dt / 0.30);
        hold += (std::clamp(joint.angle, 0.05, 0.42) - hold) * follow;
      }
      if (state.time > 1.5) {
        const double blend = std::clamp((state.time - 1.5) / 0.5, 0.0, 1.0);
        target = (1.0 - blend) * target + blend * hold;
      }
    }
    double torque = tune.kp * (target - joint.angle) - tune.kd * joint.velocity;

    const bool left = joint.name.rfind("left_", 0) == 0;
    const bool right = joint.name.rfind("right_", 0) == 0;
    const bool swinging = (!swingFoot_.empty() && joint.name.rfind(swingFoot_ + "_", 0) == 0);
    const bool footDown = (left && report.leftFootContact) || (right && report.rightFootContact);
    if (supported && footDown && share > 0 && !swinging) {
      if (joint.name.find("ankle_pitch") != std::string::npos) {
        // Positive pitch is dorsiflexion and, with the foot planted, tips the body forward.
        torque += share * (kAnkleCapture * weight * sagCapture - kAnkleLean * sagLean - kAnkleRate * sagRate);
      } else if (joint.name.find("ankle_roll") != std::string::npos) {
        // Positive roll tips the body toward character-left, so the sign is opposite the rightward error.
        torque += share * (-kAnkleLateral * 0.25 * weight * latCapture - kAnkleLean * latLean - kAnkleLatRate * latRate);
      } else if (joint.name.find("hip_flex") != std::string::npos) {
        // Positive hip flexion tips the torso backward when the feet stay down.
        torque += impactScale * share * (-kHipCapture * weight * hipSagittal_ + kHipLean * sagLean + 5.0 * sagRate);
      } else if (joint.name.find("hip_abd") != std::string::npos) {
        torque += impactScale * share * (kHipCapture * weight * hipLateral_ + kHipLean * latLean + 5.0 * latRate);
      }
    }
    if (supported && joint.name.find("spine") != std::string::npos) {
      if (joint.name.find("_flex") != std::string::npos) {
        torque += kSpineLean * chestSag + kSpineRate * sagRate;
      } else if (joint.name.find("_abd") != std::string::npos) {
        torque += -kSpineLean * chestLat - kSpineRate * latRate;
      }
    }
    if (supported && joint.name.find("neck") != std::string::npos) {
      if (joint.name.find("_flex") != std::string::npos) torque += kNeckLean * headSag;
      if (joint.name.find("_abd") != std::string::npos) torque += -kNeckLean * headLat;
    }

    if (!std::isfinite(torque)) torque = 0;
    torque = std::clamp(torque, -tune.limit, tune.limit);
    if (!physics.setJointTorque(joint.name, torque, tune.limit)) torque = 0;
    peakTorque_ = std::max(peakTorque_, std::abs(torque));
  }

  report.peakTorque = peakTorque_;
  report.active = true;
  return report;
}

void StandingController::trackHinge(const JointState& joint, double target, double blend, PhysicalSkeleton& physics) {
  Tune tune = tuneFor(joint.name);
  // The standing hip-abduction spring is stiff enough to buzz at this timestep
  // and never reaches a swing target. Swing tracking uses a softer loop.
  if (joint.name.find("hip_abd") != std::string::npos) {
    tune.kp = 420;
    tune.kd = 32;
  } else if (joint.name.find("hip_flex") != std::string::npos) {
    tune.kp = 280;
    tune.kd = 18;
  } else if (joint.name.find("knee") != std::string::npos) {
    tune.kp = 240;
    tune.kd = 16;
  }
  const double goal = blend * target + (1.0 - blend) * targetFor(joint.name, joint.angle);
  double torque = tune.kp * (goal - joint.angle) - tune.kd * joint.velocity;
  if (!std::isfinite(torque)) torque = 0;
  torque = std::clamp(torque, -tune.limit, tune.limit);
  if (!physics.setJointTorque(joint.name, torque, tune.limit)) torque = 0;
  peakTorque_ = std::max(peakTorque_, std::abs(torque));
}

}  // namespace pm
