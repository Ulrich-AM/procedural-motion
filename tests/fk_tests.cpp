#include "test_support.h"

#include "body/joint_limits.h"
#include "body/proc_body.h"
#include "motion/interpolation.h"
#include "motion/proc_motion.h"
#include "rig/forward_kinematics.h"
#include "validation/bone_length_monitor.h"
#include "validation/tolerances.h"

#include <glm/geometric.hpp>
#include <glm/gtc/quaternion.hpp>

#include <cmath>
#include <filesystem>
#include <string>

namespace {

std::filesystem::path sourceRoot() { return std::filesystem::path(PROCMOTION_SOURCE_DIR); }

proc::ProcBody loadNeutral(TestStats& stats) {
  proc::ProcBody body;
  const proc::Status status =
      proc::loadProcBodyFromFile(sourceRoot() / "assets" / "bodies" / "neutral.procbody.json", body);
  stats.expect(status.ok(), "neutral body loads for FK: " + status.text());
  return body;
}

bool near(const glm::dvec3& a, const glm::dvec3& b, double tolerance) {
  return glm::length(a - b) <= tolerance;
}

void expectBones(TestStats& stats, const proc::ProcBody& body, const proc::WorldPose& world, const std::string& label) {
  const proc::BoneLengthReport report = proc::measureBoneLengths(body, world, proc::kRuntimeBoneToleranceM);
  stats.expect(report.ok, label + " bone lengths, max error " + std::to_string(report.max_error_m));
}

}  // namespace

int runFkTests() {
  TestStats stats;
  const proc::ProcBody body = loadNeutral(stats);
  if (body.name.empty() && body.skeleton.empty()) return stats.failed;

  const proc::Pose rest;
  const proc::WorldPose rest_world = proc::forwardKinematics(body, rest);
  bool rest_matches = true;
  for (int index = 0; index < proc::kJointCount; ++index) {
    if (!near(rest_world.joints[index].position, body.joints[index].rest_position, 1e-6)) rest_matches = false;
  }
  stats.expect(rest_matches, "identity pose reproduces the rest positions");
  expectBones(stats, body, rest_world, "rest pose");

  const int spine1 = *proc::findJoint("spine1j");
  const int head = *proc::findJoint("headj");
  const int toes = *proc::findJoint("ltoesj");
  proc::Pose bent = rest;
  bent.local_rotation[spine1] = proc::quatAxisAngle(glm::dvec3(1.0, 0.0, 0.0), glm::radians(25.0));
  const proc::WorldPose bent_world = proc::forwardKinematics(body, bent);
  stats.expect(near(bent_world.joints[spine1].position, rest_world.joints[spine1].position, 1e-8),
               "a joint rotation does not translate that joint");
  stats.expect(!near(bent_world.joints[head].position, rest_world.joints[head].position, 1e-4),
               "spine1 rotation moves the head");
  stats.expect(near(bent_world.joints[toes].position, rest_world.joints[toes].position, 1e-8),
               "spine1 rotation leaves the legs in place");
  expectBones(stats, body, bent_world, "bent spine");

  const int left_shoulder = *proc::findJoint("larmuppj");
  const int left_fingers = *proc::findJoint("lfingj");
  const int right_fingers = *proc::findJoint("rfingj");
  proc::Pose arm = rest;
  arm.local_rotation[left_shoulder] = proc::quatAxisAngle(glm::dvec3(0.0, 0.0, 1.0), glm::radians(40.0));
  const proc::WorldPose arm_world = proc::forwardKinematics(body, arm);
  stats.expect(!near(arm_world.joints[left_fingers].position, rest_world.joints[left_fingers].position, 1e-4),
               "left shoulder rotation moves the left hand");
  stats.expect(near(arm_world.joints[right_fingers].position, rest_world.joints[right_fingers].position, 1e-8),
               "left shoulder rotation leaves the right arm");
  stats.expect(near(arm_world.joints[toes].position, rest_world.joints[toes].position, 1e-8),
               "left shoulder rotation leaves the legs");

  proc::Pose shifted = rest;
  shifted.root_translation = glm::dvec3(1.0, -2.0, 0.5);
  const proc::WorldPose shifted_world = proc::forwardKinematics(body, shifted);
  bool shifted_ok = true;
  for (int index = 0; index < proc::kJointCount; ++index) {
    if (!near(shifted_world.joints[index].position, rest_world.joints[index].position + shifted.root_translation, 1e-8)) {
      shifted_ok = false;
    }
  }
  stats.expect(shifted_ok, "root translation moves the whole body");
  expectBones(stats, body, shifted_world, "translated body");

  proc::Pose turned = rest;
  turned.root_rotation = proc::quatAxisAngle(glm::dvec3(0.0, 0.0, 1.0), glm::radians(90.0));
  const proc::WorldPose turned_world = proc::forwardKinematics(body, turned);
  bool turned_ok = true;
  const glm::dvec3 root = rest_world.joints[proc::kRootJoint].position;
  for (int index = 0; index < proc::kJointCount; ++index) {
    const glm::dvec3 offset = rest_world.joints[index].position - root;
    const glm::dvec3 expected = root + glm::dvec3(-offset.y, offset.x, offset.z);
    if (!near(turned_world.joints[index].position, expected, 1e-6)) turned_ok = false;
  }
  stats.expect(turned_ok, "root rotation turns the whole hierarchy about legrootj");
  expectBones(stats, body, turned_world, "rotated body");

  std::uint32_t rng = 1u;
  auto unit = [&rng]() {
    rng = rng * 1664525u + 1013904223u;
    return (rng & 0x00FFFFFFu) / static_cast<double>(0x01000000u);
  };
  bool random_ok = true;
  double worst = 0.0;
  for (int sample = 0; sample < 80; ++sample) {
    proc::Pose pose;
    for (int joint = 0; joint < proc::kJointCount; ++joint) {
      if (joint == proc::kRootJoint) continue;
      glm::dvec3 axis(unit() * 2.0 - 1.0, unit() * 2.0 - 1.0, unit() * 2.0 - 1.0);
      if (glm::length(axis) < 0.2) axis = glm::dvec3(0.0, 0.0, 1.0);
      pose.local_rotation[joint] = proc::quatAxisAngle(axis, (unit() * 2.0 - 1.0) * 2.5);
    }
    pose.root_rotation = proc::quatAxisAngle(glm::dvec3(unit(), unit(), unit()), unit() * 3.0);
    pose.root_translation = glm::dvec3(unit(), unit(), unit());
    const proc::BoneLengthReport report =
        proc::measureBoneLengths(body, proc::forwardKinematics(body, pose), proc::kRuntimeBoneToleranceM);
    worst = std::max(worst, report.max_error_m);
    if (!report.ok) random_ok = false;
  }
  stats.expect(random_ok, "random rotations keep every bone length, worst " + std::to_string(worst));

  proc::ProcBody scaled = body;
  const int thigh = *proc::findBone("rleguppb");
  scaled.bones[thigh].rest_vector *= 1.25;
  scaled.bones[thigh].length_m *= 1.25;
  const proc::BoneLengthReport scaled_report =
      proc::measureBoneLengths(scaled, proc::forwardKinematics(scaled, rest), proc::kRuntimeBoneToleranceM);
  stats.expect(scaled_report.ok, "a different body length stays locked under the identity pose");
  stats.expect(std::abs(scaled_report.samples[thigh].actual_m - body.bones[thigh].length_m) > 0.05,
               "changing the body changes the runtime thigh length");

  const int knee = *proc::findJoint("rleglowj");
  proc::Pose deep = rest;
  deep.local_rotation[knee] = proc::quatAxisAngle(glm::dvec3(1.0, 0.0, 0.0), glm::radians(-165.0));
  const proc::LimitResult warned = proc::applyJointLimits(body, deep, proc::LimitMode::Warn);
  stats.expect(!warned.violations.empty(), "WARN reports the deep knee");
  stats.expect(warned.violations.front().message.find("rleglowj") != std::string::npos, "WARN names rleglowj");
  stats.expect(warned.violations.front().message.find("requested") != std::string::npos, "WARN reports the requested angle");
  stats.expect(warned.pose.local_rotation[knee] == deep.local_rotation[knee], "WARN does not rewrite the pose");
  const proc::LimitResult clamped = proc::applyJointLimits(body, deep, proc::LimitMode::Clamp);
  const glm::dquat knee_before_clamp = deep.local_rotation[knee];
  stats.expect(clamped.pose.local_rotation[knee] != deep.local_rotation[knee], "CLAMP changes the runtime pose");
  stats.expect(deep.local_rotation[knee].w == knee_before_clamp.w && deep.local_rotation[knee].x == knee_before_clamp.x,
               "CLAMP does not rewrite the requested pose");
  const double clamped_dot = std::abs(glm::dot(
      clamped.pose.local_rotation[knee], proc::quatAxisAngle(glm::dvec3(1.0, 0.0, 0.0), glm::radians(-150.0))));
  stats.expect(clamped_dot > 0.999, "CLAMP holds the knee at the flexion limit");
  expectBones(stats, body, proc::forwardKinematics(body, clamped.pose), "clamped knee");
  expectBones(stats, body, proc::forwardKinematics(body, warned.pose), "warned knee");

  std::cout << "fk tests: " << (stats.failed == 0 ? "ok" : "failed") << '\n';
  return stats.failed;
}
