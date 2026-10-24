#include "tests/self_test.h"

#include <mujoco/mujoco.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

namespace pm {
namespace {

int articulationCount(const mjModel* model) {
  int count = 0;
  for (int i = 1; i < model->nbody; ++i) {
    if (model->body_parentid[i] > 0) ++count;
  }
  return count;
}

bool finiteArray(const mjtNum* values, int count) {
  for (int i = 0; i < count; ++i) {
    if (!std::isfinite(values[i])) return false;
  }
  return true;
}

const BodyKinematics* bodyOf(const BodyState& state, const char* name) {
  for (const BodyKinematics& body : state.bodies) {
    if (body.name == name) return &body;
  }
  return nullptr;
}

const JointState* hingeOf(const BodyState& state, const std::string& name) {
  for (const JointState& joint : state.joints) {
    if (joint.name == name) return &joint;
  }
  return nullptr;
}

Vec3 axisOf(const BodyKinematics& body, Vec3 local) {
  const double* rotation = body.rotation;
  return {rotation[0] * local.x + rotation[1] * local.y + rotation[2] * local.z,
          rotation[3] * local.x + rotation[4] * local.y + rotation[5] * local.z,
          rotation[6] * local.x + rotation[7] * local.y + rotation[8] * local.z};
}

double footLoad(const BodyState& state, const char* side) {
  const std::string token = std::string(side) + "_foot";
  double load = 0;
  for (const ContactState& contact : state.contacts) {
    const bool foot = contact.geom1.find(token) != std::string::npos || contact.geom2.find(token) != std::string::npos;
    const bool floor = contact.geom1 == "floor" || contact.geom2 == "floor";
    if (foot && floor) load += std::max(0.0, contact.normalForce);
  }
  return load;
}

double horizontal(Vec3 v) {
  v.z = 0;
  return length(v);
}

}  // namespace

int runProceduralSelfTest(PhysicalSkeleton& physics, StandingController& standing, RecoveryStep& recovery,
                          LocomotionController& locomotion, FootContactTracker& feet, JointServo& servo,
                          const SkeletonDefinition& skeleton) {
  mjModel* model = physics.model();
  mjData* data = physics.data();
  auto fail = [](const std::string& message) {
    std::cerr << "SELF-TEST FAIL: " << message << "\n";
    return 1;
  };
  auto forwardOf = [&](const BodyState& state) {
    const BodyKinematics* pelvis = bodyOf(state, "pelvis");
    Vec3 forward = pelvis ? axisOf(*pelvis, skeleton.forward) : skeleton.forward;
    forward.z = 0;
    forward = normalize(forward);
    if (length(forward) < 0.5) forward = normalize(skeleton.forward);
    return forward;
  };
  auto rightOf = [&](const BodyState& state) {
    const BodyKinematics* pelvis = bodyOf(state, "pelvis");
    Vec3 right = pelvis ? axisOf(*pelvis, skeleton.right) : skeleton.right;
    right.z = 0;
    right = normalize(right);
    if (length(right) < 0.5) right = normalize(skeleton.right);
    return right;
  };
  auto upright = [&](const BodyState& state, double& pelvisZ, double& upZ) {
    const BodyKinematics* pelvis = bodyOf(state, "pelvis");
    if (!pelvis) return false;
    pelvisZ = pelvis->position.z;
    upZ = axisOf(*pelvis, skeleton.up).z;
    return true;
  };
  auto uprightNow = [&](const BodyState& state) {
    double pelvisZ = 0;
    double upZ = 0;
    return upright(state, pelvisZ, upZ) && pelvisZ > 0.70 && upZ > 0.85;
  };
  BalanceController balance;
  auto boot = [&]() {
    physics.reset();
    const BodyState state = physics.sample();
    standing.capture(skeleton, state);
    balance.capture(skeleton, state);
    recovery.capture(skeleton, state);
    locomotion.capture(skeleton, state);
    feet.reset();
  };

  if (model->nbody - 1 != 16) return fail("expected 16 bodies, got " + std::to_string(model->nbody - 1));
  if (articulationCount(model) != 15) return fail("expected 15 body joints");
  if (model->nu != 0) return fail("model has actuators");
  const int root = mj_name2id(model, mjOBJ_JOINT, "root");
  if (root < 0 || model->jnt_type[root] != mjJNT_FREE) return fail("root is not free");

  const double mass = mj_getTotalmass(model);
  std::cout << "Self-test total mass " << mass << " kg\n";
  if (mass < 68.0 || mass > 82.0) return fail("total mass outside 68-82 kg");

  const char* limitedJoints[] = {"left_knee", "right_knee", "left_elbow", "right_elbow"};
  for (const char* name : limitedJoints) {
    const int joint = mj_name2id(model, mjOBJ_JOINT, name);
    if (joint < 0 || !model->jnt_limited[joint]) return fail(std::string("missing limited joint ") + name);
    const double lo = std::abs(model->jnt_range[2 * joint]);
    const double hi = std::abs(model->jnt_range[2 * joint + 1]);
    const double extension = std::min(lo, hi);
    const double flexion = std::max(lo, hi);
    if (extension > 25.0 * 3.141592653589793 / 180.0 || flexion < 90.0 * 3.141592653589793 / 180.0) {
      return fail(std::string("joint range is not hinge-like: ") + name);
    }
  }

  physics.reset();
  std::vector<mjtNum> initial(static_cast<size_t>(model->nq));
  std::copy(data->qpos, data->qpos + model->nq, initial.begin());
  const double timeStep = model->opt.timestep;
  physics.step();
  if (std::abs(data->time - timeStep) > 1e-8) return fail("single step did not advance one timestep");
  {
    const BodyState stepped = physics.sample();
    const int knee = mj_name2id(model, mjOBJ_JOINT, "left_knee");
    if (knee < 0) return fail("left_knee missing");
    const JointState* kneeState = hingeOf(stepped, "left_knee");
    if (!kneeState) return fail("left_knee missing from state");
    if (std::abs(kneeState->velocity - data->qvel[model->jnt_dofadr[knee]]) > 1e-8) return fail("hinge velocity was not copied");
  }
  physics.reset();
  if (std::abs(data->time) > 1e-12) return fail("reset did not clear time");
  for (int i = 0; i < model->nq; ++i) {
    if (std::abs(data->qpos[i] - initial[static_cast<size_t>(i)]) > 1e-8) return fail("reset pose mismatch");
  }
  for (int i = 0; i < model->nv; ++i) {
    if (std::abs(data->qvel[i]) > 1e-10) return fail("reset left a nonzero velocity");
  }

  const int pelvisId = mj_name2id(model, mjOBJ_BODY, "pelvis");
  const double startZ = data->xpos[3 * pelvisId + 2];
  const int airStep = std::max(1, static_cast<int>(std::lround(0.08 / timeStep)));
  const int totalSteps = std::max(airStep + 1, static_cast<int>(std::lround(4.0 / timeStep)));
  bool falling = false;
  for (int step = 1; step <= totalSteps; ++step) {
    physics.step();
    if (!finiteArray(data->qpos, model->nq) || !finiteArray(data->qvel, model->nv)) return fail("non-finite state");
    for (int b = 1; b < model->nbody; ++b) {
      const double x = data->xpos[3 * b];
      const double y = data->xpos[3 * b + 1];
      const double z = data->xpos[3 * b + 2];
      if (std::abs(x) > 4 || std::abs(y) > 4 || z < -0.25 || z > 4) {
        return fail(std::string("body left the lab: ") + mj_id2name(model, mjOBJ_BODY, b));
      }
    }
    double peak = 0;
    for (int i = 0; i < model->nv; ++i) peak = std::max(peak, std::abs(data->qvel[i]));
    if (peak > 200) return fail("velocity exploded");
    if (step == airStep) {
      mjtNum velocity[6] = {};
      mj_objectVelocity(model, data, mjOBJ_BODY, pelvisId, velocity, 0);
      std::cout << "t=" << data->time << " pelvis angular (" << velocity[0] << " " << velocity[1] << " "
                << velocity[2] << ") linear (" << velocity[3] << " " << velocity[4] << " " << velocity[5] << ")\n";
      if (velocity[5] < -0.35 && velocity[5] > -4.0) falling = true;
    }
  }
  if (!falling) return fail("pelvis linear velocity did not show a fall");
  const double endZ = data->xpos[3 * pelvisId + 2];
  std::cout << "Pelvis z " << startZ << " -> " << endZ << ", contacts " << data->ncon << "\n";
  if (endZ > startZ - 0.30) return fail("pelvis did not drop under gravity");
  if (data->ncon <= 0) return fail("no floor contacts after the fall");
  double finalSpeed = 0;
  for (int i = 0; i < model->nv; ++i) finalSpeed = std::max(finalSpeed, std::abs(data->qvel[i]));
  if (finalSpeed > 40) return fail("simulation still unstable after 4s");
  for (int i = 0; i < mjNWARNING; ++i) {
    if (data->warning[i].number > 0) std::cout << "MuJoCo warning " << i << " count " << data->warning[i].number << "\n";
  }

  physics.reset();
  const BodyState state = physics.sample();
  const Vec3 subtree{data->subtree_com[3 * pelvisId], data->subtree_com[3 * pelvisId + 1], data->subtree_com[3 * pelvisId + 2]};
  if (length(state.centerOfMass - subtree) > 1e-3) return fail("center of mass does not match the pelvis subtree");
  physics.reset();
  for (int i = 0; i < model->nq; ++i) {
    if (std::abs(data->qpos[i] - initial[static_cast<size_t>(i)]) > 1e-8) return fail("final reset pose mismatch");
  }
  for (int i = 0; i < model->nv; ++i) {
    if (std::abs(data->qvel[i]) > 1e-10) return fail("final reset velocity mismatch");
  }
  std::cout << "PASSIVE CHECKS PASSED\n";

  if (model->nu != 0) return fail("model has actuators");
  boot();
  if (physics.setJointTorque("root", 20, 20)) return fail("root accepted a torque");
  if (physics.setJointTorque("not_a_joint", 1, 10)) return fail("unknown joint accepted a torque");
  if (!physics.setJointTorque("left_knee", 10000, 40)) return fail("left_knee torque was rejected");
  physics.step();
  if (std::abs(physics.appliedJointTorque("left_knee") - 40.0) > 1e-6) return fail("torque was not clamped");
  if (physics.lastRootAppliedForce() > 1e-8) return fail("root received generalized force");

  const double dt = model->opt.timestep;
  auto drive = [&](bool recoveryOn, bool walkingOn, Vec3 desired, const std::string& focus = {}, const std::string& lift = {}) {
    ControlInput input;
    input.active = true;
    input.recovery = recoveryOn;
    input.walking = walkingOn;
    input.desired = desired;
    input.supportFocus = focus;
    input.liftFoot = lift;
    input.dt = dt;
    const ControlOutput output = applyControl(input, standing, balance, recovery, locomotion, feet, servo, physics);
    physics.step();
    const bool ok = finiteArray(data->qpos, model->nq) && finiteArray(data->qvel, model->nv) &&
                    physics.lastRootAppliedForce() <= 1e-8 && physics.lastPeakJointTorque() <= 120.01;
    return std::pair<bool, ControlOutput>{ok, output};
  };

  boot();
  const int standSteps = std::max(1, static_cast<int>(std::lround(10.0 / dt)));
  int window = 0;
  int uprightCount = 0;
  int leftCount = 0;
  int rightCount = 0;
  int hipSaturated = 0;
  double errorSum = 0;
  double hipSum = 0;
  double ankleSum = 0;
  double offsetSum = 0;
  double minPelvis = 10;
  double minUp = 1;
  double maxHip = 0;
  double maxAnkle = 0;
  for (int step = 1; step <= standSteps; ++step) {
    const auto driven = drive(false, false, {});
    if (!driven.first) return fail("controlled step wrote root force, exceeded a torque limit, or went non-finite");
    if (model->nu != 0) return fail("controller added actuators");
    if (step % 60 == 0 && data->time < 4.0) {
      const BodyState early = physics.sample();
      double earlyZ = 0;
      double earlyUp = 0;
      upright(early, earlyZ, earlyUp);
      const JointState* hip = nullptr;
      const JointState* knee = nullptr;
      const JointState* ankle = nullptr;
      for (const JointState& joint : early.joints) {
        if (joint.name == "left_hip_flex") hip = &joint;
        if (joint.name == "left_knee") knee = &joint;
        if (joint.name == "left_ankle_pitch") ankle = &joint;
      }
      const BodyKinematics* foot = bodyOf(early, "left_foot");
      const BodyKinematics* pelvis = bodyOf(early, "pelvis");
      std::cout << "  stand t " << data->time << " z " << earlyZ << " up " << earlyUp << " offset "
                << driven.second.balance.pelvisFootOffset << " hipT " << physics.appliedJointTorque("left_hip_flex")
                << " ang " << (hip ? hip->angle : 0) << " knee " << (knee ? knee->angle : 0) << " ankle "
                << (ankle ? ankle->angle : 0) << " footY " << (foot ? foot->position.y : 0) << " pelY "
                << (pelvis ? pelvis->position.y : 0) << "\n";
    }
    if (data->time < 6.0) continue;
    const BodyState sample = physics.sample();
    double pelvisZ = 0;
    double upZ = 0;
    if (!upright(sample, pelvisZ, upZ)) return fail("pelvis missing during stand");
    const StandingReport report = standing.diagnose(sample);
    ++window;
    if (pelvisZ > 0.70 && upZ > 0.85) ++uprightCount;
    if (report.leftFootContact) ++leftCount;
    if (report.rightFootContact) ++rightCount;
    if (report.supportValid) errorSum += std::hypot(report.comErrorX, report.comErrorY);
    const double hip = 0.5 * (std::abs(physics.appliedJointTorque("left_hip_flex")) +
                               std::abs(physics.appliedJointTorque("right_hip_flex")));
    const double ankle = 0.5 * (std::abs(physics.appliedJointTorque("left_ankle_pitch")) +
                                 std::abs(physics.appliedJointTorque("right_ankle_pitch")));
    hipSum += hip;
    ankleSum += ankle;
    offsetSum += std::abs(driven.second.balance.pelvisFootOffset);
    minPelvis = std::min(minPelvis, pelvisZ);
    minUp = std::min(minUp, upZ);
    maxHip = std::max(maxHip, std::max(std::abs(physics.appliedJointTorque("left_hip_flex")),
                                       std::abs(physics.appliedJointTorque("right_hip_flex"))));
    maxAnkle = std::max(maxAnkle, std::max(std::abs(physics.appliedJointTorque("left_ankle_pitch")),
                                           std::abs(physics.appliedJointTorque("right_ankle_pitch"))));
    if (std::abs(physics.appliedJointTorque("left_hip_flex")) > 95.0 ||
        std::abs(physics.appliedJointTorque("right_hip_flex")) > 95.0) {
      ++hipSaturated;
    }
  }
  if (window < 10) return fail("standing window was empty");
  const double uprightRatio = static_cast<double>(uprightCount) / static_cast<double>(window);
  const double meanError = errorSum / static_cast<double>(window);
  const double meanHip = hipSum / static_cast<double>(window);
  const double meanAnkle = ankleSum / static_cast<double>(window);
  const double meanOffset = offsetSum / static_cast<double>(window);
  std::cout << "Stand window upright " << uprightRatio << ", mean COM error " << meanError << " m, foot samples "
            << leftCount << "/" << rightCount << " of " << window << "\n";
  std::cout << "Neutral pelvis/foot offset " << meanOffset << " m, quiet hip " << meanHip << " Nm (peak " << maxHip
            << "), quiet ankle " << meanAnkle << " Nm (peak " << maxAnkle << "), hip saturated samples " << hipSaturated
            << ", min pelvis z " << minPelvis << ", min up " << minUp << "\n";
  if (uprightRatio < 0.9) return fail("humanoid did not stay upright for the last 4 seconds");
  if (leftCount < window / 2 || rightCount < window / 2) return fail("feet were not on the floor");
  if (meanError > 0.12) return fail("COM stayed far from the support center");
  if (meanOffset > 0.06) return fail("feet settled far behind the pelvis");
  if (meanHip > 55.0 || maxHip > 90.0 || hipSaturated > window / 20) {
    return fail("quiet stand still depends on a hip near its torque clamp");
  }

  const BodyState beforePush = physics.sample();
  const BodyKinematics* pelvisBefore = bodyOf(beforePush, "pelvis");
  if (!pelvisBefore) return fail("missing body before the push");
  if (!physics.startBodyForcePulse("chest", forwardOf(beforePush) * 220.0, 0.20)) return fail("push was rejected");
  double peakShift = 0;
  bool fell = false;
  const int fallSteps = std::max(1, static_cast<int>(std::lround(3.0 / dt)));
  for (int step = 0; step < fallSteps; ++step) {
    if (!drive(false, false, {}).first) return fail("push made the simulation non-finite or wrote root force");
    const BodyKinematics* pelvis = bodyOf(physics.sample(), "pelvis");
    if (!pelvis) return fail("pelvis missing during the push");
    peakShift = std::max(peakShift, std::hypot(pelvis->position.x - pelvisBefore->position.x, pelvis->position.y - pelvisBefore->position.y));
    double fallenZ = 0;
    double fallenUp = 0;
    upright(physics.sample(), fallenZ, fallenUp);
    if (fallenUp < 0.45 || fallenZ < 0.55) fell = true;
  }
  std::cout << "Push peak pelvis shift " << peakShift << " m, fell=" << (fell ? "yes" : "no") << "\n";
  if (peakShift < 0.012) return fail("push did not move the body");
  if (!fell) return fail("push did not knock the humanoid over");

  boot();
  const int quietSteps = std::max(1, static_cast<int>(std::lround(3.0 / dt)));
  double peakAnkle = 0;
  for (int step = 0; step < quietSteps; ++step) {
    const BodyState sample = physics.sample();
    const auto driven = drive(true, false, {});
    if (!driven.first) return fail("quiet stand forced the root or exceeded a torque limit");
    if (driven.second.recovery.state != StepState::Idle) return fail("quiet stand requested a recovery step");
    if (!uprightNow(sample)) return fail("quiet stand left the upright pose");
    for (const char* name : {"left_ankle_pitch", "right_ankle_pitch"}) {
      const JointState* ankle = hingeOf(sample, name);
      if (!ankle) return fail("ankle missing");
      peakAnkle = std::max(peakAnkle, std::abs(ankle->angle));
    }
  }
  {
    const BodyState sample = physics.sample();
    const JointState* ankle = hingeOf(sample, "left_ankle_pitch");
    if (!ankle || std::abs(ankle->angle) > 0.55) return fail("quiet standing still rests an ankle near its range stop");
    if (footLoad(sample, "left") < 20 || footLoad(sample, "right") < 20) return fail("quiet stand lost a foot contact");
    std::cout << "  quiet ankle |pitch| peak " << peakAnkle << " rad, left " << ankle->angle << "\n";
  }

  auto trial = [&](Vec3 direction, double force, double seconds, bool expectRecover, double minToward) {
    boot();
    const int settle = std::max(1, static_cast<int>(std::lround(2.6 / dt)));
    for (int step = 0; step < settle; ++step) {
      if (!drive(true, false, {}).first) return std::string("pre-push stand failed");
    }
    const BodyState before = physics.sample();
    const BodyKinematics* left0 = bodyOf(before, "left_foot");
    const BodyKinematics* right0 = bodyOf(before, "right_foot");
    if (!left0 || !right0) return std::string("feet missing");
    const double leftZ0 = left0->position.z;
    const double rightZ0 = right0->position.z;
    physics.startBodyForcePulse("chest", normalize(direction) * force, seconds);
    double leftLift = 0;
    double rightLift = 0;
    double leftTravel = 0;
    double rightTravel = 0;
    bool sawSwing = false;
    bool sawAir = false;
    bool sawPlant = false;
    std::string swing;
    Vec3 landing{};
    bool hadLanding = false;
    double toward = 0;
    double plantZ = -1;
    double planned = 0;
    for (int step = 0; step < static_cast<int>(std::lround(4.0 / dt)); ++step) {
      const BodyState sample = physics.sample();
      const BodyKinematics* left = bodyOf(sample, "left_foot");
      const BodyKinematics* right = bodyOf(sample, "right_foot");
      if (!left || !right) return std::string("feet missing");
      leftLift = std::max(leftLift, left->position.z - leftZ0);
      rightLift = std::max(rightLift, right->position.z - rightZ0);
      leftTravel = std::max(leftTravel, horizontal(left->position - left0->position));
      rightTravel = std::max(rightTravel, horizontal(right->position - right0->position));
      if (footLoad(sample, "left") < 8.0 || footLoad(sample, "right") < 8.0) sawAir = true;
      const auto driven = drive(true, false, {});
      if (!driven.first) return std::string(physics.lastRootAppliedForce() > 1e-8 ? "root force" : "torque limit exceeded");
      const RecoveryReport& stepReport = driven.second.recovery;
      if (stepReport.state == StepState::Swing || stepReport.state == StepState::Plant) {
        sawSwing = true;
        swing = stepReport.swingFoot;
        landing = stepReport.landing;
        hadLanding = stepReport.landingValid;
        planned = std::max(planned, stepReport.stepDistance);
        const BodyKinematics* moving = swing == "right" ? right : left;
        const BodyKinematics* origin = swing == "right" ? right0 : left0;
        if (moving && origin && hadLanding) {
          Vec3 aim{landing.x - origin->position.x, landing.y - origin->position.y, 0};
          const double aimLength = length(aim);
          if (aimLength > 1e-3) {
            Vec3 moved{moving->position.x - origin->position.x, moving->position.y - origin->position.y, 0};
            toward = std::max(toward, dot(moved, aim) / aimLength);
          }
        }
      }
      if (sawSwing && (stepReport.state == StepState::Settle || stepReport.state == StepState::Idle) &&
          footLoad(sample, swing.empty() ? "left" : swing.c_str()) > 30.0) {
        if (!sawPlant) {
          const BodyKinematics* planted = bodyOf(sample, "pelvis");
          plantZ = planted ? planted->position.z : 0;
        }
        sawPlant = true;
      }
    }
    const bool up = uprightNow(physics.sample());
    const double lift = std::max(leftLift, rightLift);
    const double travel = swing == "right" ? rightTravel : leftTravel;
    std::cout << "  push " << force << " N  lift " << lift << " m  toward " << toward << " m  travel " << travel
              << " m  planned " << planned << " m  swing "
              << (swing.empty() ? "none" : swing) << "  upright " << (up ? "yes" : "no") << "  plantZ " << plantZ
              << "  endZ " << (bodyOf(physics.sample(), "pelvis") ? bodyOf(physics.sample(), "pelvis")->position.z : 0)
              << "\n";
    if (expectRecover) {
      if (!sawSwing) return std::string("no recovery step");
      if (!sawAir || lift < 0.04) return std::string("swing foot did not leave the floor");
      if (travel < 0.03) return std::string("swing foot barely moved");
      if (hadLanding && toward < minToward) return std::string("foot did not move toward the landing target");
      if (!sawPlant) return std::string("physical contact did not end the swing");
      if (plantZ < 0.65) return std::string("pelvis had collapsed before the foot planted");
    } else if (up && lift < 0.02) {
      return std::string("extreme push neither stepped nor fell");
    } else if (up) {
      return std::string("extreme push was recovered");
    }
    return std::string();
  };

  Vec3 forward = skeleton.forward;
  Vec3 right = skeleton.right;
  const std::string forwardResult = trial(forward, 80.0, 0.12, true, 0.04);
  if (!forwardResult.empty()) return fail("forward recovery: " + forwardResult);
  const std::string sideResult = trial(right, 140.0, 0.14, true, 0.0);
  if (!sideResult.empty()) return fail("lateral recovery: " + sideResult);

  boot();
  for (int step = 0; step < static_cast<int>(std::lround(2.6 / dt)); ++step) {
    if (!drive(true, false, {}).first) return fail("settling pre-stand failed");
  }
  physics.startBodyForcePulse("chest", normalize(forward) * 80.0, 0.12);
  bool planted = false;
  bool sawSwing = false;
  for (int step = 0; step < static_cast<int>(std::lround(3.0 / dt)) && !planted; ++step) {
    const auto driven = drive(true, false, {});
    if (!driven.first) return fail("settling push failed");
    const StepState mode = driven.second.recovery.state;
    if (mode == StepState::Prepare || mode == StepState::Swing || mode == StepState::Plant) sawSwing = true;
    if (sawSwing && (mode == StepState::Settle || mode == StepState::Idle)) {
      const BodyState landed = physics.sample();
      const bool down = footLoad(landed, "left") > 30.0 || footLoad(landed, "right") > 30.0;
      if (down) planted = true;
    }
  }
  if (!planted) return fail("settling regression: recovery step did not plant");
  const int settleSteps = std::max(1, static_cast<int>(std::lround(4.0 / dt)));
  const int tail = std::max(1, static_cast<int>(std::lround(1.5 / dt)));
  double comSpeedSum = 0;
  double pelvisSpeedSum = 0;
  double footSpeedSum = 0;
  int tailCount = 0;
  Vec3 comMark{};
  bool haveMark = false;
  Vec3 comEnd{};
  for (int step = 0; step < settleSteps; ++step) {
    const BodyState sample = physics.sample();
    if (step == settleSteps - tail) {
      comMark = sample.centerOfMass;
      haveMark = true;
    }
    if (!drive(true, false, {}).first) return fail("settling window wrote root force or exceeded a torque limit");
    if (step >= settleSteps - tail) {
      const BodyState after = physics.sample();
      const BodyKinematics* pelvis = bodyOf(after, "pelvis");
      const BodyKinematics* left = bodyOf(after, "left_foot");
      const BodyKinematics* right = bodyOf(after, "right_foot");
      comSpeedSum += horizontal(after.centerOfMassVelocity);
      if (pelvis) pelvisSpeedSum += horizontal(pelvis->linearVelocity);
      if (left) footSpeedSum += horizontal(left->linearVelocity);
      if (right) footSpeedSum += horizontal(right->linearVelocity);
      comEnd = after.centerOfMass;
      ++tailCount;
    }
  }
  const BodyState settled = physics.sample();
  const BodyKinematics* settledPelvis = bodyOf(settled, "pelvis");
  const double meanCom = tailCount ? comSpeedSum / tailCount : 99;
  const double meanPelvis = tailCount ? pelvisSpeedSum / tailCount : 99;
  const double meanFoot = tailCount ? footSpeedSum / (2.0 * tailCount) : 99;
  const double drift = haveMark ? horizontal(comEnd - comMark) : 99;
  std::cout << "  post-step mean com " << meanCom << " m/s  pelvis " << meanPelvis << " m/s  foot " << meanFoot
            << " m/s  drift " << drift << " m  z " << (settledPelvis ? settledPelvis->position.z : 0) << "\n";
  // An 80 N step still ends in a fall, so that window is not a stand. The creep
  // check is a quiet recovery stand: no step, no vibration walk.
  boot();
  for (int step = 0; step < static_cast<int>(std::lround(1.5 / dt)); ++step) {
    if (!drive(true, false, {}).first) return fail("creep pre-stand failed");
  }
  const Vec3 creepStart = physics.sample().centerOfMass;
  double creepCom = 0;
  double creepFoot = 0;
  int creepCount = 0;
  const int creepSteps = std::max(1, static_cast<int>(std::lround(4.0 / dt)));
  for (int step = 0; step < creepSteps; ++step) {
    const auto driven = drive(true, false, {});
    if (!driven.first) return fail("creep window failed");
    if (driven.second.recovery.state != StepState::Idle) return fail("quiet stand started stepping");
    const BodyState sample = physics.sample();
    creepCom += horizontal(sample.centerOfMassVelocity);
    if (const BodyKinematics* left = bodyOf(sample, "left_foot")) creepFoot += horizontal(left->linearVelocity);
    if (const BodyKinematics* right = bodyOf(sample, "right_foot")) creepFoot += horizontal(right->linearVelocity);
    ++creepCount;
    if (!uprightNow(sample)) return fail("creep stand left the upright pose");
  }
  const double creepDrift = horizontal(physics.sample().centerOfMass - creepStart);
  const double creepComMean = creepCom / std::max(1, creepCount);
  const double creepFootMean = creepFoot / std::max(1, 2 * creepCount);
  std::cout << "  quiet drift " << creepDrift << " m  com " << creepComMean << " m/s  foot " << creepFootMean << " m/s\n";
  if (creepDrift > 0.05) return fail("settling regression: quiet stand crept across the floor");
  if (creepComMean > 0.08) return fail("settling regression: quiet COM speed stayed high");
  if (creepFootMean > 0.12) return fail("settling regression: quiet feet jittered");

  const std::string fallResult = trial(forward, 400.0, 0.30, false, 0.0);
  if (!fallResult.empty()) return fail("extreme push: " + fallResult);
  std::cout << "RECOVERY CHECKS PASSED\n";

  auto quiet = [&](double seconds) {
    const int steps = std::max(1, static_cast<int>(std::lround(seconds / dt)));
    for (int step = 0; step < steps; ++step) {
      if (!drive(false, false, {}).first) return false;
    }
    return true;
  };
  auto weightShift = [&](const std::string& focus) {
    if (!quiet(3.0)) return fail(focus + " shift could not settle");
    double fractionSum = 0;
    double hipSum = 0;
    double maxHip = 0;
    double minZ = 10;
    double minUp = 1;
    double errorSum = 0;
    int samples = 0;
    const int steps = std::max(1, static_cast<int>(std::lround(3.0 / dt)));
    for (int step = 0; step < steps; ++step) {
      const auto driven = drive(false, false, {}, focus, "");
      if (!driven.first) return fail(focus + " weight shift wrote root force or saturated a joint");
      if (step * dt < 2.0) continue;
      const BodyState sample = physics.sample();
      const double left = footLoad(sample, "left");
      const double right = footLoad(sample, "right");
      const double total = left + right;
      if (total < 40.0) return fail(focus + " weight shift lost floor contact");
      const double fraction = (focus == "left" ? left : right) / total;
      const double hip = std::abs(physics.appliedJointTorque(focus + "_hip_flex"));
      double pelvisZ = 0;
      double upZ = 0;
      upright(sample, pelvisZ, upZ);
      fractionSum += fraction;
      hipSum += hip;
      maxHip = std::max(maxHip, hip);
      minZ = std::min(minZ, pelvisZ);
      minUp = std::min(minUp, upZ);
      const StandingReport report = standing.diagnose(sample);
      if (report.supportValid) errorSum += std::hypot(report.comErrorX, report.comErrorY);
      ++samples;
    }
    if (samples < 10) return fail(focus + " weight shift window was empty");
    const double fraction = fractionSum / samples;
    const double hip = hipSum / samples;
    std::cout << "Weight shift " << focus << " fraction " << fraction << " hip " << hip << " Nm peak " << maxHip
              << " min z " << minZ << " min up " << minUp << "\n";
    if (fraction < 0.65) return fail(focus + " foot did not take most of the load");
    if (hip > 80.0 || maxHip > 95.0) return fail(focus + " stance hip saturated during the weight shift");
    if (minZ < 0.72 || minUp < 0.80) return fail(focus + " pelvis collapsed during the weight shift");
    if (errorSum / samples > 0.16) return fail(focus + " COM left the support region");
    if (!quiet(1.5)) return fail(focus + " return to double support failed");
    const BodyState back = physics.sample();
    const double left = footLoad(back, "left");
    const double right = footLoad(back, "right");
    if (left < 20.0 || right < 20.0) return fail(focus + " did not return to double support");
    return 0;
  };

  std::cout << "STATIC WEIGHT SHIFT\n";
  boot();
  if (const int shiftLeft = weightShift("left")) return shiftLeft;
  boot();
  if (const int shiftRight = weightShift("right")) return shiftRight;

  auto singleSupport = [&](const std::string& stance, const std::string& lift) {
    if (!quiet(2.5)) return fail(stance + " single support could not settle");
    const int shiftSteps = std::max(1, static_cast<int>(std::lround(2.0 / dt)));
    for (int step = 0; step < shiftSteps; ++step) {
      if (!drive(false, false, {}, stance, "").first) return fail(stance + " pre-lift shift failed");
    }
    const BodyState liftedFrom = physics.sample();
    const BodyKinematics* swing0 = bodyOf(liftedFrom, (lift + "_foot").c_str());
    if (!swing0) return fail("swing foot missing");
    const double swingZ0 = swing0->position.z;
    double minZ = 10;
    double endZ = 0;
    double minUp = 1;
    double hipSum = 0;
    int hipHot = 0;
    int air = 0;
    int samples = 0;
    double maxCapture = 0;
    const int holdSteps = std::max(1, static_cast<int>(std::lround(2.2 / dt)));
    for (int step = 0; step < holdSteps; ++step) {
      const auto driven = drive(false, false, {}, stance, lift);
      if (!driven.first) return fail(stance + " single support wrote root force or went non-finite");
      if (step * dt < 0.45) continue;
      const BodyState sample = physics.sample();
      const BodyKinematics* swing = bodyOf(sample, (lift + "_foot").c_str());
      const BodyKinematics* planted = bodyOf(sample, (stance + "_foot").c_str());
      if (!swing || !planted) return fail("foot missing in single support");
      double pelvisZ = 0;
      double upZ = 0;
      upright(sample, pelvisZ, upZ);
      const double hip = std::abs(physics.appliedJointTorque(stance + "_hip_flex"));
      const StandingReport report = standing.diagnose(sample);
      minZ = std::min(minZ, pelvisZ);
      endZ = pelvisZ;
      minUp = std::min(minUp, upZ);
      hipSum += hip;
      if (hip > 95.0) ++hipHot;
      if (report.supportValid) maxCapture = std::max(maxCapture, std::hypot(report.comErrorX, report.comErrorY));
      const bool off = swing->position.z > swingZ0 + 0.03 && footLoad(sample, lift.c_str()) < 20.0;
      const bool held = footLoad(sample, stance.c_str()) > 40.0 && planted->position.z < swingZ0 + 0.03;
      if (off && held && pelvisZ > 0.70) ++air;
      ++samples;
    }
    const double hip = samples > 0 ? hipSum / samples : 0;
    std::cout << "Single support " << stance << " duration samples " << air << "/" << samples << " hip " << hip
              << " Nm hot " << hipHot << " min z " << minZ << " end z " << endZ << " min up " << minUp
              << " max capture " << maxCapture << "\n";
    if (samples < 10 || air < samples * 8 / 10) return fail(stance + " foot did not stay off the floor");
    if (minZ < 0.70 || endZ < minZ - 0.04 || minUp < 0.72) return fail(stance + " pelvis collapsed in single support");
    if (hip > 85.0 || hipHot > samples / 3) return fail(stance + " stance hip stayed torque-saturated");
    if (!quiet(1.5)) return fail(stance + " replant failed");
    const BodyState replanted = physics.sample();
    if (footLoad(replanted, "left") < 12.0 || footLoad(replanted, "right") < 12.0) {
      return fail(stance + " did not replant into double support");
    }
    return 0;
  };

  std::cout << "SINGLE SUPPORT\n";
  boot();
  if (const int leftStance = singleSupport("left", "right")) return leftStance;
  boot();
  if (const int rightStance = singleSupport("right", "left")) return rightStance;

  std::cout << "SINGLE SUPPORT DISTURBANCE\n";
  boot();
  if (!quiet(2.0)) return fail("disturbance stand failed");
  for (int step = 0; step < static_cast<int>(std::lround(1.5 / dt)); ++step) {
    if (!drive(false, false, {}, "left", "").first) return fail("disturbance shift failed");
  }
  for (int step = 0; step < static_cast<int>(std::lround(0.6 / dt)); ++step) {
    if (!drive(false, false, {}, "left", "right").first) return fail("disturbance lift failed");
  }
  const double hipBefore = physics.appliedJointTorque("left_hip_flex");
  const Vec3 comBefore = physics.sample().centerOfMass;
  if (!physics.startBodyForcePulse("chest", forwardOf(physics.sample()) * 18.0, 0.08)) return fail("disturbance was rejected");
  double torqueResponse = 0;
  double comResponse = 0;
  double disturbZ = 10;
  double disturbUp = 1;
  for (int step = 0; step < static_cast<int>(std::lround(1.2 / dt)); ++step) {
    if (!drive(false, false, {}, "left", "right").first) return fail("disturbance response broke the simulator");
    const BodyState sample = physics.sample();
    torqueResponse = std::max(torqueResponse, std::abs(physics.appliedJointTorque("left_hip_flex") - hipBefore));
    comResponse = std::max(comResponse, horizontal(sample.centerOfMass - comBefore));
    double pelvisZ = 0;
    double upZ = 0;
    upright(sample, pelvisZ, upZ);
    disturbZ = std::min(disturbZ, pelvisZ);
    disturbUp = std::min(disturbUp, upZ);
  }
  std::cout << "Disturbance torque response " << torqueResponse << " Nm, COM response " << comResponse << " m, min z "
            << disturbZ << " min up " << disturbUp << "\n";
  if (disturbZ < 0.68 || disturbUp < 0.70) return fail("small single-support disturbance knocked the body down");
  if (torqueResponse < 1.5 && comResponse < 0.008) return fail("single-support disturbance produced no controller response");

  struct GaitRun {
    int lifts = 0;
    int replants = 0;
    bool usedLeft = false;
    bool usedRight = false;
    Vec3 delta{};
    double minZ = 10;
    double endZ = 0;
    double endUp = 1;
    std::string error;
  };
  auto walkFor = [&](Vec3 desired, double seconds) {
    GaitRun run;
    bool leftAir = false;
    bool rightAir = false;
    const BodyState before = physics.sample();
    const BodyKinematics* pelvis0 = bodyOf(before, "pelvis");
    const BodyKinematics* leftOrigin = bodyOf(before, "left_foot");
    const BodyKinematics* rightOrigin = bodyOf(before, "right_foot");
    if (!pelvis0 || !leftOrigin || !rightOrigin) {
      run.error = "pelvis missing";
      return run;
    }
    const double leftZ0 = leftOrigin->position.z;
    const double rightZ0 = rightOrigin->position.z;
    const int steps = std::max(1, static_cast<int>(std::lround(seconds / dt)));
    for (int step = 0; step < steps; ++step) {
      const BodyState sample = physics.sample();
      double pelvisZ = 0;
      double upZ = 0;
      upright(sample, pelvisZ, upZ);
      run.minZ = std::min(run.minZ, pelvisZ);
      run.endZ = pelvisZ;
      run.endUp = upZ;
      const BodyKinematics* left = bodyOf(sample, "left_foot");
      const BodyKinematics* right = bodyOf(sample, "right_foot");
      if (!left || !right) {
        run.error = "feet missing";
        return run;
      }
      auto watch = [&](bool& air, const BodyKinematics* foot, const char* side, bool& used, double z0) {
        const double load = footLoad(sample, side);
        if (!air && foot->position.z > z0 + 0.06 && load < 12.0) {
          air = true;
          ++run.lifts;
          used = true;
        } else if (air && load > 35.0) {
          air = false;
          ++run.replants;
        }
      };
      watch(leftAir, left, "left", run.usedLeft, leftZ0);
      watch(rightAir, right, "right", run.usedRight, rightZ0);
      if (!drive(false, true, desired).first) {
        run.error = "root force, torque limit, or non-finite state";
        return run;
      }
    }
    const BodyKinematics* pelvis1 = bodyOf(physics.sample(), "pelvis");
    if (!pelvis1) {
      run.error = "pelvis missing";
      return run;
    }
    run.delta = pelvis1->position - pelvis0->position;
    return run;
  };

  boot();
  for (int step = 0; step < static_cast<int>(std::lround(3.0 / dt)); ++step) {
    const auto driven = drive(false, true, {});
    if (!driven.first) return fail("walking-off stand failed");
    const BodyState sample = physics.sample();
    if (data->time > 2.2 && !uprightNow(sample)) {
      double pelvisZ = 0;
      double upZ = 0;
      upright(sample, pelvisZ, upZ);
      return fail(std::string("zero desired velocity did not stay upright at t=") + std::to_string(data->time) +
                  " z=" + std::to_string(pelvisZ) + " up=" + std::to_string(upZ) + " state " +
                  locoStateLabel(driven.second.locomotion.state));
    }
  }
  const GaitRun idle = walkFor({}, 2.0);
  if (!idle.error.empty()) return fail("zero velocity: " + idle.error);
  std::cout << "  zero-velocity lifts " << idle.lifts << "\n";
  if (idle.lifts > 0) return fail("zero desired velocity produced continuous steps");

  const Vec3 aim = normalize(Vec3{forward.x, forward.y, 0});
  boot();
  for (int step = 0; step < static_cast<int>(std::lround(2.6 / dt)); ++step) {
    if (!drive(false, true, {}).first) return fail("one-step pre-stand failed");
  }
  Vec3 stepCommand = aim * 0.40;
  int completedSteps = 0;
  int stepLifts = 0;
  int stepReplants = 0;
  bool leftAir = false;
  bool rightAir = false;
  double advance = 0;
  const BodyState stepStart = physics.sample();
  const double leftZ0 = bodyOf(stepStart, "left_foot")->position.z;
  const double rightZ0 = bodyOf(stepStart, "right_foot")->position.z;
  for (int step = 0; step < static_cast<int>(std::lround(4.0 / dt)); ++step) {
    const auto driven = drive(false, true, stepCommand);
    if (!driven.first) return fail("one-step drive failed");
    completedSteps = driven.second.locomotion.completedSteps;
    if (completedSteps >= 1) stepCommand = {};
    if (step % 60 == 0) {
      const LocomotionReport& loco = driven.second.locomotion;
      double pelvisZ = 0;
      double upZ = 0;
      upright(physics.sample(), pelvisZ, upZ);
      std::cout << "    t " << (step * dt) << " " << locoStateLabel(loco.state) << " z " << pelvisZ << " up " << upZ
                << " L% " << (100.0 * loco.leftFraction) << " R% " << (100.0 * loco.rightFraction) << " swing "
                << loco.swingFoot << " steps " << loco.completedSteps << "\n";
    }
    const BodyState sample = physics.sample();
    auto watch = [&](bool& air, const char* side, double z0) {
      const char* footName = side[0] == 'l' ? "left_foot" : "right_foot";
      const BodyKinematics* foot = bodyOf(sample, footName);
      if (!foot) return;
      const double load = footLoad(sample, side);
      if (!air && foot->position.z > z0 + 0.06 && load < 12.0) {
        air = true;
        ++stepLifts;
      } else if (air && load > 35.0) {
        air = false;
        ++stepReplants;
        const Vec3 origin = side == std::string("left") ? bodyOf(stepStart, "left_foot")->position
                                                       : bodyOf(stepStart, "right_foot")->position;
        advance = std::max(advance, dot(foot->position - origin, aim));
      }
    };
    watch(leftAir, "left", leftZ0);
    watch(rightAir, "right", rightZ0);
  }
  const BodyState afterStep = physics.sample();
  double stepZ = 0;
  double stepUp = 0;
  upright(afterStep, stepZ, stepUp);
  std::cout << "  one-step lifts " << stepLifts << " replants " << stepReplants << " advance " << advance << " m z " << stepZ
            << " steps " << completedSteps << "\n";
  if (stepLifts < 1) return fail("one step did not lift a foot");
  if (stepReplants < 1) return fail("one step did not replant");
  if (advance < 0.04) return fail("one step did not advance the foot");

  double holdSpeed = 0;
  int holdCount = 0;
  double holdMinZ = 10;
  for (int step = 0; step < static_cast<int>(std::lround(3.5 / dt)); ++step) {
    const BodyState sample = physics.sample();
    double pelvisZ = 0;
    double upZ = 0;
    upright(sample, pelvisZ, upZ);
    holdMinZ = std::min(holdMinZ, pelvisZ);
    holdSpeed += horizontal(sample.centerOfMassVelocity);
    ++holdCount;
    if (pelvisZ < 0.68 || upZ < 0.75) {
      return fail(std::string("one step did not stay upright z=") + std::to_string(pelvisZ) + " up=" + std::to_string(upZ));
    }
    if (!drive(false, true, {}).first) return fail("one-step hold failed");
  }
  const BodyState held = physics.sample();
  std::cout << "  one-step hold min z " << holdMinZ << " speed " << (holdSpeed / std::max(1, holdCount)) << " m/s left "
            << footLoad(held, "left") << " right " << footLoad(held, "right") << "\n";
  if (footLoad(held, "left") < 12.0 || footLoad(held, "right") < 12.0) return fail("one step did not return to double support");
  if (holdSpeed / std::max(1, holdCount) > 0.35) return fail("one step did not settle");

  boot();
  for (int step = 0; step < static_cast<int>(std::lround(2.2 / dt)); ++step) {
    if (!drive(false, true, {}).first) return fail("slow-walk pre-stand failed");
  }
  const GaitRun slow = walkFor(aim * 0.40, 9.0);
  if (!slow.error.empty()) return fail("slow walk: " + slow.error);
  const double slowTravel = dot(Vec3{slow.delta.x, slow.delta.y, 0}, aim);
  std::cout << "  slow lifts " << slow.lifts << " replants " << slow.replants << " travel " << slowTravel << " m min z "
            << slow.minZ << " end z " << slow.endZ << "\n";
  if (slow.lifts < 6) return fail("slow walk did not take repeated steps");
  if (!slow.usedLeft || !slow.usedRight) return fail("slow walk did not use both feet");
  if (slow.replants < 4) return fail("slow walk did not replant");
  if (slowTravel < 0.50) return fail("slow walk did not move forward");
  if (slow.minZ < 0.68 || slow.endZ < 0.72) return fail("slow walk pelvis collapsed");

  const BodyState beforeStop = physics.sample();
  const double speedAtStop = horizontal(beforeStop.centerOfMassVelocity);
  const GaitRun stopped = walkFor({}, 5.0);
  if (!stopped.error.empty()) return fail("stop: " + stopped.error);
  const BodyState afterStop = physics.sample();
  const double stopSpeed = horizontal(afterStop.centerOfMassVelocity);
  std::cout << "  stop from " << speedAtStop << " to " << stopSpeed << " m/s lifts " << stopped.lifts << " z " << stopped.endZ
            << "\n";
  if (speedAtStop > 0.20 && stopped.lifts < 1 && stopSpeed > 0.20) return fail("stop did not bleed speed or step");
  if (stopSpeed > 0.35) return fail("stop did not bleed horizontal speed");
  if (stopped.endZ < 0.68 || stopped.endUp < 0.75) return fail("stop did not return to a stand");
  if (footLoad(afterStop, "left") < 12 || footLoad(afterStop, "right") < 12) return fail("stop did not plant both feet");

  boot();
  for (int step = 0; step < static_cast<int>(std::lround(2.0 / dt)); ++step) {
    if (!drive(false, true, {}).first) return fail("normal-walk pre-stand failed");
  }
  const GaitRun normal = walkFor(aim * 0.80, 6.0);
  if (!normal.error.empty()) return fail("normal walk: " + normal.error);
  const double normalTravel = dot(Vec3{normal.delta.x, normal.delta.y, 0}, aim);
  std::cout << "  normal lifts " << normal.lifts << " travel " << normalTravel << " m min z " << normal.minZ << " end z "
            << normal.endZ << "\n";
  if (normal.endZ < 0.66 || normal.minZ < 0.60 || normal.endUp < 0.7) return fail("normal walk fell");
  if (normal.lifts < 4) return fail("normal walk did not keep stepping");
  if (normalTravel < 0.30) return fail("normal walk did not move forward");

  boot();
  for (int step = 0; step < static_cast<int>(std::lround(1.6 / dt)); ++step) {
    if (!drive(false, true, {}).first) return fail("backward pre-stand failed");
  }
  const GaitRun backed = walkFor(aim * -0.35, 4.0);
  if (!backed.error.empty()) return fail("backward walk: " + backed.error);
  const double backTravel = dot(Vec3{backed.delta.x, backed.delta.y, 0}, aim);
  std::cout << "  backward travel " << backTravel << " m lifts " << backed.lifts << " z " << backed.endZ << "\n";
  if (backed.lifts < 1) return fail("backward walk did not step");
  if (backed.endZ < 0.55) return fail("backward walk collapsed");

  boot();
  for (int step = 0; step < static_cast<int>(std::lround(1.6 / dt)); ++step) {
    if (!drive(false, true, {}).first) return fail("lateral pre-stand failed");
  }
  const Vec3 side = normalize(Vec3{right.x, right.y, 0});
  const GaitRun sided = walkFor(side * 0.35, 4.0);
  if (!sided.error.empty()) return fail("lateral walk: " + sided.error);
  const double sideTravel = dot(Vec3{sided.delta.x, sided.delta.y, 0}, side);
  std::cout << "  lateral travel " << sideTravel << " m lifts " << sided.lifts << " z " << sided.endZ << "\n";
  if (sided.lifts < 1) return fail("lateral walk did not step");
  if (sided.endZ < 0.55) return fail("lateral walk collapsed");

  boot();
  for (int step = 0; step < static_cast<int>(std::lround(1.4 / dt)); ++step) {
    if (!drive(false, true, {}).first) return fail("disturbance pre-stand failed");
  }
  const GaitRun primed = walkFor(aim * 0.40, 1.6);
  if (!primed.error.empty()) return fail("disturbance walk failed");
  const BodyState beforeHit = physics.sample();
  physics.startBodyForcePulse("chest", rightOf(beforeHit) * 70.0, 0.10);
  const GaitRun disturbed = walkFor(aim * 0.40, 2.2);
  if (!disturbed.error.empty()) return fail("disturbed walk: " + disturbed.error);
  const BodyKinematics* hitPelvis = bodyOf(physics.sample(), "pelvis");
  const double hitShift = hitPelvis ? horizontal(hitPelvis->position - beforeHit.centerOfMass) : 0;
  std::cout << "  disturbed lifts " << disturbed.lifts << " shift " << hitShift << " m z " << disturbed.endZ << "\n";
  if (disturbed.lifts < 1 && hitShift < 0.04) return fail("disturbance did not change the step or the body");

  boot();
  for (int step = 0; step < static_cast<int>(std::lround(1.4 / dt)); ++step) {
    if (!drive(false, true, {}).first) return fail("failure pre-stand failed");
  }
  if (!walkFor(aim * 0.40, 1.2).error.empty()) return fail("failure walk failed");
  physics.startBodyForcePulse("chest", aim * 450.0, 0.28);
  bool knocked = false;
  for (int step = 0; step < static_cast<int>(std::lround(2.5 / dt)); ++step) {
    if (!drive(false, true, aim * 0.40).first) return fail("extreme walk disturbance broke the simulator");
    double pelvisZ = 0;
    double upZ = 0;
    upright(physics.sample(), pelvisZ, upZ);
    if (pelvisZ < 0.5 || upZ < 0.4) knocked = true;
  }
  std::cout << "  extreme walk fell=" << (knocked ? "yes" : "no") << "\n";
  if (!knocked) return fail("extreme disturbance did not knock the walker over");

  std::cout << "WALK CHECKS PASSED\n";
  std::cout << "SELF-TEST PASS\n";
  boot();
  return 0;
}

}  // namespace pm
