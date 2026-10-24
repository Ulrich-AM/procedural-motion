#include "control/balance_controller.h"

#include "control/foot_placement.h"
#include "control/support_state.h"
#include "control/swing_command.h"
#include "physics/physical_metrics.h"
#include "physics/physical_skeleton.h"

#include <mujoco/mujoco.h>

#include <algorithm>
#include <cmath>
#include <vector>

namespace pm {
namespace {

bool legJoint(const std::string& name, const std::string& side) {
  if (name.rfind(side + "_", 0) != 0) return false;
  return name.find("hip") != std::string::npos || name.find("knee") != std::string::npos ||
         name.find("ankle") != std::string::npos;
}

double footNormal(const BodyState& state, const char* foot) {
  double load = 0;
  for (const ContactState& contact : state.contacts) {
    const bool onFoot = contact.geom1.find(foot) != std::string::npos || contact.geom2.find(foot) != std::string::npos;
    const bool floor = contact.geom1 == "floor" || contact.geom2 == "floor";
    if (onFoot && floor) load += std::max(0.0, contact.normalForce);
  }
  return load;
}

}  // namespace

void BalanceController::capture(const SkeletonDefinition& skeleton, const BodyState& state) {
  restForward = skeleton.forward;
  restRight = skeleton.right;
  restUp = skeleton.up;
  if (length(restForward) < 0.5) restForward = Vec3{0, -1, 0};
  if (length(restRight) < 0.5) restRight = Vec3{-1, 0, 0};
  if (length(restUp) < 0.5) restUp = Vec3{0, 0, 1};
  const BodyKinematics* pelvis = findBody(state, "pelvis");
  neutralComZ_ = state.centerOfMass.z;
  auto measure = [&](Side& side, const char* name) {
    side.ready = false;
    const BodyKinematics* hip = findBody(state, std::string(name) + "_upper_leg");
    const BodyKinematics* knee = findBody(state, std::string(name) + "_lower_leg");
    const BodyKinematics* foot = findBody(state, std::string(name) + "_foot");
    if (!hip || !knee || !foot) return;
    side.lengths.thigh = std::max(0.2, length(knee->position - hip->position));
    side.lengths.shin = std::max(0.2, length(foot->position - knee->position));
    if (const JointState* hinge = findHinge(state, std::string(name) + "_knee")) {
      side.lengths.kneeSign = std::abs(hinge->rangeMin) >= std::abs(hinge->rangeMax) ? -1.0 : 1.0;
    }
    Vec3 forward = restForward;
    Vec3 right = restRight;
    forward.z = 0;
    right.z = 0;
    forward = normalize(forward);
    right = normalize(right);
    calibrateHipBias(side.lengths, state, name, hip->position, foot->position, forward, right, restUp);
    side.ready = true;
    if (pelvis) {
      const double bend = 10.0 * 3.141592653589793 / 180.0;
      const double span = std::sqrt(side.lengths.thigh * side.lengths.thigh + side.lengths.shin * side.lengths.shin +
                                     2.0 * side.lengths.thigh * side.lengths.shin * std::cos(bend));
      const double pelvisAboveHip = pelvis->position.z - hip->position.z;
      const double comAbovePelvis = state.centerOfMass.z - pelvis->position.z;
      neutralComZ_ = defaultPhysicalMetrics().ankleHeight + span + pelvisAboveHip + comAbovePelvis;
    }
  };
  measure(leftLeg, "left");
  measure(rightLeg, "right");
  captured_ = true;
}

BalanceReport BalanceController::apply(const BodyState& state, ControlFrame& frame, const FootContactTracker& feet,
                                       PhysicalSkeleton& physics, const std::string& focus, const std::string& exclude,
                                       const std::string& holdUp) const {
  BalanceReport report;
  if (!captured_ || !physics.model() || !physics.data()) return report;
  mjModel* model = physics.model();
  mjData* data = physics.data();

  Vec3 forward = restForward;
  Vec3 right = restRight;
  forward.z = 0;
  right.z = 0;
  forward = normalize(forward);
  right = normalize(right);
  const PhysicalMetrics metrics = defaultPhysicalMetrics();
  const SupportState support = measureSupport(state, feet, metrics, forward, right, state.centerOfMass, false);
  report.supportValid = support.valid;
  const BodyKinematics* pelvis = findBody(state, "pelvis");
  if (pelvis && (support.leftFound || support.rightFound)) {
    Vec3 mid = support.center;
    if (support.leftFound && support.rightFound) mid = (support.leftFoot + support.rightFoot) * 0.5;
    else if (support.leftFound) mid = support.leftFoot;
    else mid = support.rightFoot;
    report.pelvisFootOffset = dot(Vec3{pelvis->position.x, pelvis->position.y, 0} - mid, forward);
  }
  if (!support.valid && holdUp.empty()) return report;

  Vec3 desired = support.valid ? support.center : state.centerOfMass;
  desired.z = 0;
  if (focus == "left" && support.leftFound) desired = support.leftFoot;
  if (focus == "right" && support.rightFound) desired = support.rightFoot;
  if (!holdUp.empty()) {
    const bool holdLeft = holdUp == "left";
    if (holdLeft && support.rightFound) desired = support.rightFoot;
    if (!holdLeft && support.leftFound) desired = support.leftFoot;
  } else if (!exclude.empty()) {
    if (exclude == "left" && support.rightFound) desired = support.rightFoot;
    if (exclude == "right" && support.leftFound) desired = support.leftFoot;
  }
  desired.z = neutralComZ_;
  report.desiredCom = desired;

  Vec3 error = desired - state.centerOfMass;
  error.z = neutralComZ_ - state.centerOfMass.z;
  Vec3 velocity = state.centerOfMassVelocity;
  Vec3 force{800.0 * error.x - 180.0 * velocity.x, 800.0 * error.y - 180.0 * velocity.y,
             500.0 * error.z - 90.0 * velocity.z};
  // The foot Jacobian below maps a virtual force into stance-leg torques and
  // discards the free-root columns. Applying that force through the hip folds
  // the floating pelvis, so the virtual force is held at zero. Sagittal balance
  // is the capture-point ankle/hip law further down, with the ankle clamped to
  // the sole.
  force = Vec3{0, 0, 0};
  report.force = force;
  double sagCapture = 0;
  double latCapture = 0;
  double sagLean = 0;
  double latLean = 0;
  double sagRate = 0;
  double latRate = 0;
  if (pelvis && support.valid) {
    const Vec3 pelvisUp{pelvis->rotation[6], pelvis->rotation[7], pelvis->rotation[8]};
    const double height = std::clamp(pelvis->position.z, 0.45, 1.15);
    const double omega = std::sqrt(9.81 / height);
    const double sagErr = dot((support.center - state.centerOfMass), forward);
    const double latErr = dot((support.center - state.centerOfMass), right);
    const double sagVel = dot(state.centerOfMassVelocity, forward);
    const double latVel = dot(state.centerOfMassVelocity, right);
    sagCapture = sagErr - sagVel / omega;
    latCapture = latErr - latVel / omega;
    sagLean = dot(pelvisUp, forward);
    latLean = dot(pelvisUp, right);
    sagRate = -dot(pelvis->angularVelocity, right);
    latRate = dot(pelvis->angularVelocity, forward);
  }

  if (model->nv < 6) return report;
  // Leg joints do not move the pelvis when the free root is held fixed, so the
  // pelvis Jacobian columns are ~0. Map the virtual force through the stance foot.
  // Root columns are skipped.
  Vec3 moment{0, 0, 0};
  if (pelvis) {
    moment = pelvis->angularVelocity * -14.0;
    moment.x = std::clamp(moment.x, -20.0, 20.0);
    moment.y = std::clamp(moment.y, -20.0, 20.0);
    moment.z = std::clamp(moment.z, -12.0, 12.0);
  }

  const bool useLeft = exclude != "left" && holdUp != "left";
  const bool useRight = exclude != "right" && holdUp != "right";
  const int legs = (useLeft ? 1 : 0) + (useRight ? 1 : 0);
  if (legs <= 0) return report;
  const double share = 1.0 / static_cast<double>(legs);
  const double weight = std::max(1.0, state.mass) * 9.81;

  auto addSide = [&](const std::string& side, bool enabled, double normal) {
    if (!enabled || normal < 5.0) return;
    const int footId = mj_name2id(model, mjOBJ_BODY, (side + "_foot").c_str());
    if (footId < 0) return;
    std::vector<mjtNum> jacp(static_cast<size_t>(3 * model->nv), 0);
    std::vector<mjtNum> jacr(static_cast<size_t>(3 * model->nv), 0);
    mj_jacBody(model, data, jacp.data(), jacr.data(), footId);
    const double ankleLimit = std::clamp(normal * metrics.toe, 8.0, 68.0);
    for (int j = 0; j < model->njnt; ++j) {
      if (model->jnt_type[j] != mjJNT_HINGE) continue;
      const int dof = model->jnt_dofadr[j];
      if (dof < 6 || dof >= model->nv) continue;
      const char* raw = mj_id2name(model, mjOBJ_JOINT, j);
      if (!raw) continue;
      const std::string name = raw;
      if (!legJoint(name, side)) continue;
      const auto column = [&](const std::vector<mjtNum>& jac, int row) {
        return jac[static_cast<size_t>(row * model->nv + dof)];
      };
      // Measured foot Jacobian: positive hip/knee motion moves the sole backward
      // and down. Negating J^T F makes an upward, toward-support force extend
      // the knee and tip the torso back over the sole. Root dofs are not written.
      double limited = 0;
      if (name.find("ankle_pitch") != std::string::npos || name.find("ankle_roll") != std::string::npos) {
        // Ankle strategy stays inside the sole. Positive pitch dorsiflexes and
        // tips the body forward, so a COM ahead of the foot needs plantarflexion.
        const BodyKinematics* sole = findBody(state, side + "_foot");
        if (!sole) continue;
        (void)sole;
        if (name.find("ankle_pitch") != std::string::npos) {
          limited = share * (0.8 * weight * sagCapture - 8.0 * sagLean - 4.0 * sagRate);
        } else {
          limited = share * (-0.6 * weight * latCapture - 8.0 * latLean - 4.0 * latRate);
        }
        limited = std::clamp(limited, -ankleLimit, ankleLimit);
      } else {
        const double chain = (column(jacp, 0) * force.x + column(jacp, 1) * force.y + column(jacp, 2) * force.z +
                              column(jacr, 0) * moment.x + column(jacr, 1) * moment.y + column(jacr, 2) * moment.z) *
                             share;
        // Hip flexion's planted reaction is opposite the knee's. Knee -J^T extends
        // into the floor. Hip +J^T pulls the pelvis back over the sole.
        if (name.find("hip_flex") != std::string::npos) {
          // Positive flexion torque moves the pelvis backward over a planted foot.
          const BodyKinematics* sole = findBody(state, side + "_foot");
          const double sag = sole ? dot(state.centerOfMass - sole->position, forward) : 0;
          const double sagVel = dot(state.centerOfMassVelocity, forward);
          // The ankle owns small errors. The hip only adds a capped nudge once
          // the COM is leaving the sole, so the pose spring is not wound up.
          // Angle springs wind up and then throw the pelvis. This term is the
          // hip strategy: positive torque moves the pelvis back toward the feet.
          // sagVel > 0 means the COM is moving further ahead. Both terms then
          // push the pelvis back. The velocity term cuts the torque while returning.
          (void)sag;
          (void)sagVel;
          limited = std::clamp(share * (-0.04 * weight * sagCapture + 8.0 * sagLean + 4.0 * sagRate), -10.0, 10.0);
        } else {
          limited = -chain;
        }
      }
      if (!std::isfinite(limited) || std::abs(limited) < 1e-6) continue;
      auto found = frame.stance.find(name);
      if (found == frame.stance.end()) continue;
      found->second.feedforward += limited;
    }
  };
  addSide("left", useLeft, footNormal(state, "left_foot"));
  addSide("right", useRight, footNormal(state, "right_foot"));

  const double normal = std::max(weight, 1.0);
  Vec3 cop = support.valid ? support.center : desired;
  cop.z = 0;
  cop.x -= force.x * state.centerOfMass.z / normal;
  cop.y -= force.y * state.centerOfMass.z / normal;
  report.cop = cop;
  report.copValid = support.valid;

  if (!holdUp.empty()) {
    const Side& spec = holdUp == "left" ? leftLeg : rightLeg;
    const BodyKinematics* hip = findBody(state, holdUp + "_upper_leg");
    const BodyKinematics* foot = findBody(state, holdUp + "_foot");
    if (spec.ready && hip && foot) {
      Vec3 ankle = foot->position;
      ankle.z = metrics.ankleHeight + 0.045;
      LegPose pose = solveLeg(hip->position, ankle, forward, right, restUp, spec.lengths);
      const char* suffix[5] = {"_hip_flex", "_hip_abd", "_knee", "_ankle_pitch", "_ankle_roll"};
      const double raw[5] = {pose.hipFlex, pose.hipAbd, pose.knee, pose.anklePitch, pose.ankleRoll};
      const double kp[5] = {70.0, 70.0, 80.0, 24.0, 24.0};
      const double kd[5] = {8.0, 8.0, 8.0, 4.0, 4.0};
      const double limit[5] = {36.0, 36.0, 40.0, 24.0, 24.0};
      for (int i = 0; i < 5; ++i) {
        const JointState* hinge = findHinge(state, holdUp + suffix[i]);
        if (!hinge) continue;
        JointCommand command;
        command.target = std::clamp(raw[i], hinge->rangeMin + 0.02, hinge->rangeMax - 0.02);
        command.kp = kp[i];
        command.kd = kd[i];
        command.limit = limit[i];
        frame.setSwing(hinge->name, command, 1);
      }
    }
  }
  return report;
}

}  // namespace pm
