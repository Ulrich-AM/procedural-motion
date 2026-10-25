#include "test_support.h"

#include "body/proc_body.h"
#include "motion/interpolation.h"
#include "motion/proc_motion.h"
#include "retarget/retarget_profile.h"
#include "retarget/retargeter.h"
#include "rig/forward_kinematics.h"
#include "validation/bone_length_monitor.h"
#include "validation/tolerances.h"

#include <cmath>
#include <filesystem>

namespace {

std::filesystem::path sourceFile(const char* relative) {
  return std::filesystem::path(PROCMOTION_SOURCE_DIR) / relative;
}

double dot(const glm::dquat& a, const glm::dquat& b) { return std::abs(glm::dot(proc::quatNormalize(a), proc::quatNormalize(b))); }

proc::SourceClip syntheticClip() {
  proc::SourceClip clip;
  clip.name = "synthetic";
  clip.format = "synthetic";
  clip.nominal_fps = 2.0;
  clip.duration = 0.5;
  clip.joints.resize(3);
  clip.joints[0].name = "Hips";
  clip.joints[0].parent = -1;
  clip.joints[1].name = "LeftUpLeg";
  clip.joints[1].parent = 0;
  clip.joints[2].name = "LeftLeg";
  clip.joints[2].parent = 1;
  const glm::dquat hip_rest = proc::quatAxisAngle(glm::dvec3(0, 0, 1), glm::radians(90.0));
  clip.joints[0].rest_local_translation = glm::dvec3(0, 0, 1);
  clip.joints[0].rest_local_rotation = proc::quatIdentity();
  clip.joints[1].rest_local_translation = glm::dvec3(-0.25, 0.02, -0.08);
  clip.joints[1].rest_local_rotation = hip_rest;
  clip.joints[2].rest_local_translation = glm::dvec3(0, 0, -0.62);
  clip.joints[2].rest_local_rotation = proc::quatIdentity();
  clip.frames.resize(2);
  for (proc::SourceFrame& frame : clip.frames) {
    frame.local_translation.resize(3);
    frame.local_rotation.resize(3);
    for (int index = 0; index < 3; ++index) {
      frame.local_translation[index] = clip.joints[index].rest_local_translation;
      frame.local_rotation[index] = clip.joints[index].rest_local_rotation;
    }
  }
  clip.frames[0].time = 0.0;
  clip.frames[1].time = 0.5;
  clip.frames[1].local_translation[0] = glm::dvec3(1.0, 2.0, 1.25);
  clip.frames[1].local_rotation[1] = proc::quatNormalize(proc::quatAxisAngle(glm::dvec3(1, 0, 0), glm::radians(30.0)) * hip_rest);
  clip.frames[1].local_rotation[2] = proc::quatAxisAngle(glm::dvec3(1, 0, 0), glm::radians(-40.0));
  proc::rebuildSourceTopology(clip);
  proc::rebuildSourceGlobals(clip);
  return clip;
}

void refit(proc::ProcBody& body) {
  for (const proc::CanonicalBone& bone : proc::kCanonicalBones) {
    const int index = *proc::findBone(bone.name);
    proc::BoneGeom& geometry = body.bones[index];
    geometry.length_m = glm::length(geometry.rest_vector);
    geometry.rest_direction = glm::normalize(geometry.rest_vector);
    body.joints[bone.child].rest_position = body.joints[bone.parent].rest_position + geometry.rest_vector;
  }
}

}  // namespace

int runRetargetTests() {
  TestStats stats;
  proc::ProcBody neutral;
  const proc::Status body_status = proc::loadProcBodyFromFile(sourceFile("assets/bodies/neutral.procbody.json"), neutral);
  stats.expect(body_status.ok(), body_status.ok() ? "body" : body_status.text());
  if (!body_status.ok()) return stats.failed;

  const proc::SourceClip clip = syntheticClip();
  std::array<int, proc::kJointCount> mapping{};
  mapping.fill(-1);
  mapping[proc::kRootJoint] = 0;
  mapping[*proc::findJoint("lleguppj")] = 1;
  mapping[*proc::findJoint("lleglowj")] = 2;
  std::array<bool, proc::kJointCount> required{};
  required.fill(false);
  required[proc::kRootJoint] = true;
  required[*proc::findJoint("lleguppj")] = true;
  required[*proc::findJoint("lleglowj")] = true;

  proc::ProcMotion motion;
  proc::RetargetReport report;
  const proc::Status retarget_status =
      proc::retargetClip(neutral, clip, mapping, required, {}, motion, report);
  stats.expect(retarget_status.ok(), retarget_status.text());
  stats.expect(report.rest_max_error_deg < 1e-4, "rest error " + std::to_string(report.rest_max_error_deg));
  stats.expect(report.non_root_translations_exported == 0, "no exported non-root translation");
  stats.expect(std::abs(motion.duration_s - motion.frames.back().time) < 1e-12,
               "retarget duration ends at the last exported sample");

  const int hip = *proc::findJoint("lleguppj");
  const int knee = *proc::findJoint("lleglowj");
  const glm::dquat hip_expected = proc::quatAxisAngle(glm::dvec3(1, 0, 0), glm::radians(30.0));
  stats.expect(dot(motion.frames[1].local_rotation[hip], hip_expected) > 0.9999, "hip flexion survives a twisted source rest");
  const glm::dquat naive = clip.frames[1].local_rotation[1];
  stats.expect(dot(motion.frames[1].local_rotation[hip], naive) < 0.99, "source local quaternion is not copied");
  const glm::dquat knee_expected =
      proc::quatNormalize(proc::quatAxisAngle(glm::dvec3(0, 0, 1), glm::radians(90.0)) *
                          proc::quatAxisAngle(glm::dvec3(1, 0, 0), glm::radians(-40.0)) *
                          proc::quatInverse(proc::quatAxisAngle(glm::dvec3(0, 0, 1), glm::radians(90.0))));
  stats.expect(dot(motion.frames[1].local_rotation[knee], knee_expected) > 0.999, "knee delta is bind-relative");
  stats.expect(glm::length(motion.frames[1].root_translation - glm::dvec3(1, 2, 0.25)) < 1e-9, "root translation delta");
  stats.expect(glm::length(motion.frames[0].root_translation) < 1e-9, "rest root translation is zero");

  const std::string written = proc::writeProcMotionToString(motion);
  stats.expect(written.find("\"translation\"") != std::string::npos, "root translation is present");
  stats.expect(written.find("bone_lengths") == std::string::npos, "no bone lengths");
  proc::ProcMotion loaded;
  const proc::Status load_status = proc::loadProcMotionFromString(written, loaded, "roundtrip");
  stats.expect(load_status.ok(), load_status.ok() ? "roundtrip" : load_status.text());
  if (load_status.ok()) {
    for (double time_s : {0.0, 0.25, 0.5}) {
      const proc::Pose a = proc::sampleMotion(motion, time_s, false);
      const proc::Pose b = proc::sampleMotion(loaded, time_s, false);
      stats.expect(glm::length(a.root_translation - b.root_translation) < 1e-9, "roundtrip translation");
      stats.expect(dot(a.root_rotation, b.root_rotation) > 1.0 - 1e-9, "roundtrip root rotation");
      stats.expect(dot(a.local_rotation[hip], b.local_rotation[hip]) > 1.0 - 1e-9, "roundtrip hip");
      stats.expect(dot(a.local_rotation[knee], b.local_rotation[knee]) > 1.0 - 1e-9, "roundtrip knee");
    }
  }

  proc::ProcBody altered = neutral;
  const int left_thigh = *proc::findBone("lleguppb");
  const int right_thigh = *proc::findBone("rleguppb");
  const int left_hip_bone = *proc::findBone("llegrootb");
  altered.bones[left_thigh].rest_vector *= 0.72;
  altered.bones[right_thigh].rest_vector *= 0.72;
  altered.bones[left_hip_bone].rest_vector.x -= 0.04;
  refit(altered);
  proc::ProcMotion same;
  proc::RetargetReport altered_report;
  stats.expect(proc::retargetClip(altered, clip, mapping, required, {}, same, altered_report).ok(), "altered retarget");
  stats.expect(dot(same.frames[1].local_rotation[hip], motion.frames[1].local_rotation[hip]) > 1.0 - 1e-12,
               "proportions do not change the .procmo rotation");
  const proc::WorldPose neutral_world = proc::forwardKinematics(neutral, motion.frames[1]);
  const proc::WorldPose altered_world = proc::forwardKinematics(altered, motion.frames[1]);
  const int knee_joint = knee;
  stats.expect(glm::length(neutral_world.joints[knee_joint].position - altered_world.joints[knee_joint].position) > 0.05,
               "different bodies put the knee in different places");
  const proc::BoneLengthReport neutral_lengths = proc::measureBoneLengths(neutral, neutral_world, proc::kRuntimeBoneToleranceM);
  const proc::BoneLengthReport altered_lengths = proc::measureBoneLengths(altered, altered_world, proc::kRuntimeBoneToleranceM);
  stats.expect(neutral_lengths.ok && altered_lengths.ok, "each body keeps its own bone lengths");
  stats.expect(std::abs(altered.bones[left_thigh].length_m - neutral.bones[left_thigh].length_m) > 0.05, "thigh length changed");

  std::vector<proc::RetargetProfile> profiles;
  const proc::Status profiles_status = proc::loadRetargetProfiles(sourceFile("config/retarget"), profiles);
  stats.expect(profiles_status.ok(), profiles_status.text());
  stats.expect(profiles.size() == 2, "two profiles");
  std::vector<std::string> mixamo_names = {"mixamorig:Hips", "mixamorig:Spine", "mixamorig:Spine1", "mixamorig:Spine2",
                                           "mixamorig:Neck", "mixamorig:Head", "mixamorig:LeftUpLeg", "mixamorig:LeftLeg",
                                           "mixamorig:LeftFoot", "mixamorig:RightUpLeg", "mixamorig:RightLeg",
                                           "mixamorig:RightFoot", "mixamorig:LeftArm", "mixamorig:LeftForeArm",
                                           "mixamorig:LeftHand", "mixamorig:RightArm", "mixamorig:RightForeArm",
                                           "mixamorig:RightHand"};
  const proc::ProfileMatch matched = proc::matchProfile(profiles, mixamo_names);
  stats.expect(matched.confidence == "HIGH", "mixamo confidence " + matched.confidence);
  stats.expect(matched.matched == 18 && matched.required == 18, "18 required mappings");
  stats.expect(matched.profile_index >= 0 && profiles[matched.profile_index].name == "mixamo", "detected mixamo");
  const int left_toe = *proc::findJoint("ltoesj");
  const int right_toe = *proc::findJoint("rtoesj");
  stats.expect(matched.mapping[left_toe] < 0 && matched.mapping[right_toe] < 0, "missing toes stay unmapped");
  std::vector<std::string> with_toes = mixamo_names;
  with_toes.push_back("mixamorig:LeftToeBase");
  with_toes.push_back("RightToe");
  const proc::ProfileMatch toed = proc::matchProfile(profiles, with_toes);
  stats.expect(toed.confidence == "HIGH" && toed.required == 18, "toes are optional for profile detection");
  stats.expect(toed.mapping[left_toe] >= 0 && toed.mapping[right_toe] >= 0, "present toe names map to ltoesj and rtoesj");

  std::cout << "retarget tests: " << (stats.failed == 0 ? "ok" : "failed") << '\n';
  return stats.failed;
}
