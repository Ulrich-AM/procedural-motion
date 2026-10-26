#include "test_support.h"

#include "body/proc_body.h"
#include "import/bvh_importer.h"
#include "import/fbx_importer.h"
#include "retarget/retarget_profile.h"
#include "retarget/retargeter.h"
#include "rig/forward_kinematics.h"
#include "validation/bone_length_monitor.h"
#include "validation/tolerances.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <string>

namespace {

std::string lowerExtension(const std::filesystem::path& path) {
  std::string extension = path.extension().string();
  for (char& character : extension) {
    if (character >= 'A' && character <= 'Z') character = static_cast<char>(character - 'A' + 'a');
  }
  return extension;
}

int smokeSource(const std::filesystem::path& path) {
  proc::SourceClip clip;
  const std::string extension = lowerExtension(path);
  proc::Status status = extension == ".bvh" ? proc::loadBvhFromFile(path, proc::identityConvention(), clip)
                                             : proc::loadFbxFromFile(path, proc::identityConvention(), clip);
  if (!status.ok()) {
    std::cerr << status.text() << '\n';
    return 1;
  }
  const std::filesystem::path root = std::filesystem::path(PROCMOTION_SOURCE_DIR);
  std::vector<proc::RetargetProfile> profiles;
  status = proc::loadRetargetProfiles(root / "config" / "retarget", profiles);
  if (!status.ok()) {
    std::cerr << status.text() << '\n';
    return 1;
  }
  std::vector<std::string> names;
  for (const proc::SourceJoint& joint : clip.joints) names.push_back(joint.name);
  const proc::ProfileMatch match = proc::matchProfile(profiles, names);
  std::array<int, proc::kJointCount> mapping{};
  mapping.fill(-1);
  std::array<bool, proc::kJointCount> required{};
  required.fill(false);
  if (match.confidence == "HIGH" && match.profile_index >= 0) {
    mapping = match.mapping;
    required = profiles[match.profile_index].required;
    proc::applyAxisConvention(clip, profiles[match.profile_index].coordinates);
  }
  proc::ProcBody body;
  status = proc::loadProcBodyFromFile(root / "assets" / "bodies" / "neutral.procbody.json", body);
  if (!status.ok()) {
    std::cerr << status.text() << '\n';
    return 1;
  }
  proc::ProcMotion motion;
  proc::RetargetReport report;
  status = proc::retargetClip(body, clip, mapping, required, {}, motion, report);
  const proc::Pose pose = motion.frames.empty() ? proc::Pose{} : motion.frames.front();
  const proc::WorldPose world = proc::forwardKinematics(body, pose);
  const proc::BoneLengthReport bones = proc::measureBoneLengths(body, world, proc::kRuntimeBoneToleranceM);
  std::string worst;
  const double direction = proc::segmentDirectionError(clip, 0.0, body, world, mapping, worst);
  std::cout << "format " << clip.format << '\n';
  std::cout << "clip " << clip.name << '\n';
  std::cout << "joints " << clip.joints.size() << '\n';
  std::cout << "frames " << clip.frames.size() << '\n';
  std::cout << "duration " << clip.duration << '\n';
  std::cout << "fps " << clip.nominal_fps << '\n';
  std::cout << "units " << clip.units_detected << '\n';
  std::cout << "coordinates " << clip.coordinate_conversion << '\n';
  std::cout << "profile " << (match.profile_index >= 0 ? profiles[match.profile_index].name : "none") << '\n';
  std::cout << "confidence " << match.confidence << '\n';
  std::cout << "mapped " << report.mapped << " / " << report.required << '\n';
  std::cout << "rest max error deg " << report.rest_max_error_deg << '\n';
  std::cout << "direction max error deg " << direction << " " << worst << '\n';
  std::cout << "bone max error m " << bones.max_error_m << (bones.ok ? " PASS" : " FAIL") << '\n';
  std::cout << "non-root translations exported " << report.non_root_translations_exported << '\n';
  std::cout << "source translation notes " << report.translation_notes.size() << '\n';
  for (const proc::CanonicalBone& bone : proc::kCanonicalBones) {
    if (mapping[bone.parent] < 0 || mapping[bone.child] < 0) continue;
    const glm::dvec3 source_delta = clip.joints[mapping[bone.child]].rest_global_position -
                                    clip.joints[mapping[bone.parent]].rest_global_position;
    const glm::dvec3 target_delta = world.joints[bone.child].position - world.joints[bone.parent].position;
    if (glm::length(source_delta) < 1e-6 || glm::length(target_delta) < 1e-6) continue;
    const double cosine = std::clamp(glm::dot(glm::normalize(source_delta), glm::normalize(target_delta)), -1.0, 1.0);
    const double degrees = glm::degrees(std::acos(cosine));
    if (degrees > 8.0) std::cout << "direction " << bone.name << " " << degrees << " deg\n";
  }
  if (!status.ok()) {
    std::cerr << status.text() << '\n';
    return 1;
  }
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc > 1) {
    const std::filesystem::path path(argv[1]);
    const std::string extension = lowerExtension(path);
    if (extension == ".fbx" || extension == ".bvh") return smokeSource(path);
    proc::ProcBody body;
    const proc::Status status = proc::loadProcBodyFromFile(argv[1], body);
    if (!status.ok()) {
      std::cerr << status.text() << '\n';
      return 1;
    }
    std::cout << "loaded " << body.name << '\n';
    return 0;
  }
  const int body = runBodyTests();
  const int fk = runFkTests();
  const int motion = runMotionTests();
  const int bvh = runBvhTests();
  const int retarget = runRetargetTests();
  const int import = runImportTests();
  const int analysis_signal = runAnalysisSignalTests();
  const int contact = runContactTests();
  const int gait_event = runGaitEventTests();
  const int gait_cycle = runGaitCycleTests();
  const int relationship = runRelationshipTests();
  const int dataset_index = runDatasetIndexTests();
  const int architecture = runArchitectureTests();
  const int locomotion = runLocomotionTests();
  const int gait_validation = runGaitValidationTests();
  const int reference_capture = runReferenceCaptureTests();
  const int failed = body + fk + motion + bvh + retarget + import + analysis_signal + contact + gait_event + gait_cycle + relationship + dataset_index + architecture + locomotion + gait_validation + reference_capture;
  if (failed != 0) {
    std::cerr << failed << " test(s) failed\n";
    return 1;
  }
  std::cout << "architecture tests ok\n";
  std::cout << "body tests ok\n";
  std::cout << "fk tests ok\n";
  std::cout << "motion tests ok\n";
  std::cout << "retarget tests ok\n";
  std::cout << "analysis tests ok\n";
  std::cout << "locomotion tests ok\n";
  std::cout << "all tests passed\n";
  return 0;
}
