#include "test_support.h"

#include "analysis/locomotion_model_builder.h"
#include "analysis/proc_analysis.h"
#include "analysis/synthetic_locomotion.h"
#include "body/joint_limits.h"
#include "body/proc_body.h"
#include "locomotion/body_scale.h"
#include "locomotion/locomotion_audit.h"
#include "locomotion/locomotion_diagnostics.h"
#include "locomotion/locomotion_generator.h"
#include "locomotion/locomotion_model_io.h"
#include "motion/proc_motion.h"
#include "retarget/source_semantics.h"
#include "rig/forward_kinematics.h"
#include "validation/bone_length_monitor.h"
#include "validation/tolerances.h"

#include <glm/geometric.hpp>
#include <glm/gtc/quaternion.hpp>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>

namespace {

void scaleBones(proc::ProcBody& body, const std::vector<std::string>& names, double scale) {
  for (proc::BoneGeom& bone : body.bones) {
    if (std::find(names.begin(), names.end(), bone.name) == names.end()) continue;
    bone.rest_vector *= scale;
    bone.length_m = glm::length(bone.rest_vector);
    if (bone.length_m > 1e-9) bone.rest_direction = bone.rest_vector / bone.length_m;
  }
  for (int pass = 0; pass < proc::kJointCount; ++pass) {
    for (int index = 0; index < proc::kJointCount; ++index) {
      const int parent = body.joints[index].parent;
      const int bone_index = body.joints[index].incoming_bone;
      if (parent < 0 || bone_index < 0) continue;
      body.joints[index].rest_position = body.joints[parent].rest_position + body.bones[bone_index].rest_vector;
    }
  }
}

double maxBoneError(const proc::ProcBody& body, const proc::ProcMotion& motion) {
  double worst = 0.0;
  for (const proc::Pose& pose : motion.frames) {
    const proc::BoneLengthReport report = proc::measureBoneLengths(body, proc::forwardKinematics(body, pose), proc::kRuntimeBoneToleranceM);
    worst = std::max(worst, report.max_error_m);
    if (!report.ok) worst = std::max(worst, report.max_error_m + 1.0);
  }
  return worst;
}

proc::SourceClip clipFromGenerated(const proc::ProcBody& body, const proc::ProcMotion& motion) {
  proc::SourceClip clip;
  clip.name = "generated_locomotion";
  clip.format = "procedural";
  clip.nominal_fps = motion.fps;
  clip.duration = motion.duration_s;
  clip.root = proc::jointIndex(proc::JointId::LegRoot);
  for (int index = 0; index < proc::kJointCount; ++index) {
    proc::SourceJoint joint;
    joint.name = proc::kJointNames[index];
    joint.parent = body.joints[index].parent;
    joint.rest_global_position = body.joints[index].rest_position;
    joint.rest_global_rotation = proc::quatIdentity();
    clip.joints.push_back(joint);
  }
  for (const proc::Pose& pose : motion.frames) {
    const proc::WorldPose world = proc::forwardKinematics(body, pose);
    proc::SourceFrame frame;
    frame.time = pose.time;
    frame.global_position.resize(proc::kJointCount);
    frame.global_rotation.resize(proc::kJointCount, proc::quatIdentity());
    for (int index = 0; index < proc::kJointCount; ++index) frame.global_position[static_cast<std::size_t>(index)] = world.joints[index].position;
    clip.frames.push_back(frame);
  }
  return clip;
}

bool fileHas(const std::filesystem::path& path, const std::string& text) {
  std::ifstream stream(path);
  std::stringstream buffer;
  buffer << stream.rdbuf();
  return buffer.str().find(text) != std::string::npos;
}

}  // namespace

int runLocomotionTests() {
  TestStats stats;
  const std::filesystem::path root = std::filesystem::path(PROCMOTION_SOURCE_DIR);
  proc::ProcBody neutral;
  const proc::Status body_status = proc::loadProcBodyFromFile(root / "assets" / "bodies" / "neutral.procbody.json", neutral);
  stats.expect(body_status.ok(), "neutral body loads for locomotion tests");
  if (!body_status.ok()) return stats.failed;

  const glm::dvec3 from_x(1.0, 0.0, 0.0);
  const glm::dquat same = proc::quatRotateFromTo(from_x, from_x);
  stats.expect(std::abs(glm::dot(same, proc::quatIdentity())) > 0.999999, "parallel vectors rotate by identity");
  const glm::dquat turned = proc::quatRotateFromTo(from_x, glm::dvec3(0.0, 1.0, 0.0));
  const glm::dvec3 turned_x = turned * from_x;
  stats.expect(glm::length(turned_x - glm::dvec3(0.0, 1.0, 0.0)) < 1e-6, "rotation-from-to maps the source vector onto the target");
  const glm::dquat opposite = proc::quatRotateFromTo(from_x, glm::dvec3(-1.0, 0.0, 0.0));
  const glm::dquat opposite_again = proc::quatRotateFromTo(from_x, glm::dvec3(-1.0, 0.0, 0.0));
  stats.expect(glm::length(opposite * from_x - glm::dvec3(-1.0, 0.0, 0.0)) < 1e-6, "opposite vectors rotate by a half turn");
  stats.expect(std::abs(glm::dot(opposite, opposite_again)) > 0.999999, "the opposite-vector rotation is deterministic");
  stats.expect(proc::quatRotateFromTo(glm::dvec3(0.0), from_x).w == 1.0, "a zero vector does not invent a rotation");

  const proc::WorldPose straight = proc::forwardKinematics(neutral, proc::Pose{});
  auto posed = [&](proc::JointId joint, double radians) {
    proc::Pose pose;
    const glm::dvec3 axis = neutral.constraints[proc::jointIndex(joint)].axis;
    pose.local_rotation[proc::jointIndex(joint)] = proc::quatAxisAngle(axis, radians);
    return proc::forwardKinematics(neutral, pose);
  };
  const proc::WorldPose left_knee = posed(proc::JointId::LeftKnee, -0.8);
  const proc::WorldPose right_knee = posed(proc::JointId::RightKnee, -0.8);
  stats.expect(left_knee.joints[proc::jointIndex(proc::JointId::LeftFoot)].position.y <
                       straight.joints[proc::jointIndex(proc::JointId::LeftFoot)].position.y - 0.02 &&
                   left_knee.joints[proc::jointIndex(proc::JointId::LeftFoot)].position.z >
                       straight.joints[proc::jointIndex(proc::JointId::LeftFoot)].position.z + 0.02,
               "negative left-knee hinge flexion folds the shin backward and upward");
  stats.expect(right_knee.joints[proc::jointIndex(proc::JointId::RightFoot)].position.y <
                       straight.joints[proc::jointIndex(proc::JointId::RightFoot)].position.y - 0.02 &&
                   right_knee.joints[proc::jointIndex(proc::JointId::RightFoot)].position.z >
                       straight.joints[proc::jointIndex(proc::JointId::RightFoot)].position.z + 0.02,
               "negative right-knee hinge flexion folds the shin backward and upward");
  const proc::WorldPose left_hyper = posed(proc::JointId::LeftKnee, 0.15);
  stats.expect(left_hyper.joints[proc::jointIndex(proc::JointId::LeftFoot)].position.y >
                   straight.joints[proc::jointIndex(proc::JointId::LeftFoot)].position.y,
               "positive knee rotation is the small hyperextension side");
  const proc::WorldPose left_elbow = posed(proc::JointId::LeftElbow, 0.9);
  const proc::WorldPose right_elbow = posed(proc::JointId::RightElbow, 0.9);
  stats.expect(left_elbow.joints[proc::jointIndex(proc::JointId::LeftHand)].position.y >
                   straight.joints[proc::jointIndex(proc::JointId::LeftHand)].position.y + 0.05,
               "positive left-elbow hinge flexion brings the hand forward");
  stats.expect(right_elbow.joints[proc::jointIndex(proc::JointId::RightHand)].position.y >
                   straight.joints[proc::jointIndex(proc::JointId::RightHand)].position.y + 0.05,
               "positive right-elbow hinge flexion brings the hand forward");
  const proc::WorldPose left_ankle = posed(proc::JointId::LeftFoot, 0.35);
  stats.expect(left_ankle.joints[proc::jointIndex(proc::JointId::LeftToe)].position.z >
                   straight.joints[proc::jointIndex(proc::JointId::LeftToe)].position.z + 0.01,
               "positive ankle hinge rotation lifts the toe");

  const double leg = proc::targetLegLength(neutral);
  stats.expect(leg > 0.7 && leg < 1.1, "target leg length uses the locked thigh and shin");
  const double expected_dim = 1.25 / std::sqrt(proc::kLocomotionGravity * leg);
  stats.expect(std::abs(proc::dimensionlessSpeed(1.25, leg) - expected_dim) < 1e-12, "dimensionless speed is speed over sqrt(g L)");
  stats.expect(proc::dimensionlessSpeed(1.0, 0.0) == 0.0, "dimensionless speed is zero without a leg length");

  glm::dquat spun = proc::quatAxisAngle(glm::dvec3(0.2, 0.7, 0.4), 0.8);
  const glm::dquat restored = proc::mirrorSagittal(proc::mirrorSagittal(spun));
  stats.expect(std::abs(glm::dot(restored, spun)) > 0.999999, "mirroring a rotation twice recovers it");
  std::array<glm::dvec3, proc::kLocoChannels> logs{};
  logs[static_cast<int>(proc::LocoChannel::LeftHip)] = glm::dvec3(0.3, 0.1, -0.2);
  logs[static_cast<int>(proc::LocoChannel::RightKnee)] = glm::dvec3(-0.4, 0.2, 0.05);
  std::array<glm::dvec3, proc::kLocoChannels> twice = logs;
  proc::mirrorLogChannels(twice);
  proc::mirrorLogChannels(twice);
  stats.expect(glm::length(twice[static_cast<int>(proc::LocoChannel::LeftHip)] - logs[static_cast<int>(proc::LocoChannel::LeftHip)]) < 1e-6,
               "mirroring a pose twice recovers the hip log");
  std::array<glm::dvec3, proc::kLocoChannels> once = logs;
  proc::mirrorLogChannels(once);
  stats.expect(glm::length(once[static_cast<int>(proc::LocoChannel::RightHip)]) > 0.2, "a left hip log moves to the right hip");

  std::array<glm::dvec3, proc::kLocoPhaseSamples> curve{};
  curve[99] = glm::dvec3(1.0, 0.0, 0.0);
  curve[0] = glm::dvec3(0.0, 0.0, 0.0);
  const glm::dvec3 wrapped = proc::sampleLog(curve, 0.995);
  stats.expect(std::abs(wrapped.x - 0.5) < 1e-6, "phase sampling wraps from the last sample to the first");
  stats.expect(std::abs(proc::wrapPhase(1.0)) < 1e-12, "phase 1 wraps to 0");
  stats.expect(std::abs(proc::wrapPhase(-0.25) - 0.75) < 1e-12, "negative phase wraps into the cycle");

  proc::ProcLocomotionModel hand;
  proc::SpeedKnot slow;
  proc::SpeedKnot fast;
  slow.dimensionless_speed = 0.40;
  fast.dimensionless_speed = 0.80;
  slow.cadence_spm = 100.0;
  fast.cadence_spm = 140.0;
  slow.normalized_stride_length = 1.1;
  fast.normalized_stride_length = 1.5;
  slow.median_speed_mps = 1.0;
  fast.median_speed_mps = 2.0;
  slow.cycle_count = 4;
  fast.cycle_count = 4;
  for (int sample = 0; sample < proc::kLocoPhaseSamples; ++sample) {
    slow.rotation[static_cast<int>(proc::LocoChannel::LeftKnee)][static_cast<std::size_t>(sample)] = glm::dvec3(0.20, 0.0, 0.0);
    fast.rotation[static_cast<int>(proc::LocoChannel::LeftKnee)][static_cast<std::size_t>(sample)] = glm::dvec3(0.60, 0.10, 0.0);
    slow.pelvis_vertical[static_cast<std::size_t>(sample)] = 0.05;
    fast.pelvis_vertical[static_cast<std::size_t>(sample)] = 0.09;
    slow.contact_left_heel[static_cast<std::size_t>(sample)] = sample < 60 ? 1.0 : 0.0;
  }
  hand.walk.knots = {slow, fast};
  hand.walk.present = true;
  hand.walk.support_min = 0.30;
  hand.walk.support_max = 0.90;
  proc::SpeedKnot run_knot = fast;
  run_knot.dimensionless_speed = 1.10;
  run_knot.cadence_spm = 170.0;
  for (auto& sample : run_knot.rotation[static_cast<int>(proc::LocoChannel::LeftKnee)]) sample = glm::dvec3(0.90, 0.0, 0.0);
  hand.run.knots = {run_knot};
  hand.run.present = true;
  hand.run.support_min = 0.80;
  hand.run.support_max = 1.40;
  proc::SpeedKnot sprint_knot = run_knot;
  sprint_knot.dimensionless_speed = 1.80;
  sprint_knot.cadence_spm = 210.0;
  for (auto& sample : sprint_knot.rotation[static_cast<int>(proc::LocoChannel::LeftKnee)]) sample = glm::dvec3(1.10, 0.0, 0.0);
  hand.sprint.knots = {sprint_knot};
  hand.sprint.present = true;
  hand.sprint.support_min = 1.50;
  hand.sprint.support_max = 2.10;

  const std::string model_text = proc::writeLocomotionModelToString(hand);
  proc::ProcLocomotionModel loaded;
  const proc::Status roundtrip = proc::readLocomotionModelFromString(model_text, loaded);
  stats.expect(roundtrip.ok(), "locomotion model roundtrip");
  stats.expect(loaded.version == 1 && loaded.speed_domain == "dimensionless_speed", "model format is PROC_LOCOMOTION_MODEL v1");
  stats.expect(loaded.walk.knots.size() == 2 && std::abs(loaded.walk.knots[1].cadence_spm - 140.0) < 1e-9, "speed knots survive the file");
  proc::ProcLocomotionModel rejected_model;
  stats.expect(!proc::readLocomotionModelFromString("{\"format\":\"PROC_ANALYSIS\",\"version\":1}", rejected_model).ok(),
               "analysis documents are not locomotion models");

  proc::LocomotionState state;
  state.actual_speed_mps = 1.2;
  state.desired_speed_mps = 1.2;
  state.dimensionless_speed = 0.60;
  state.regime = proc::LocomotionRegime::Walk;
  state.fade = 1.0;
  state.phase = 0.25;
  proc::LocomotionSample mid = proc::generateLocomotionPose(neutral, loaded, state);
  const glm::dquat knee = mid.pose.local_rotation[proc::jointIndex(proc::JointId::LeftKnee)];
  const glm::dquat expected_knee = proc::quatExp(glm::dvec3(0.40, 0.05, 0.0));
  stats.expect(std::abs(glm::dot(knee, expected_knee)) > 0.999, "speed knots interpolate the knee log");
  stats.expect(std::abs(mid.pose.root_translation.z - 0.07 * leg) < 1e-6, "pelvis vertical scales by target leg length");

  proc::ProcBody long_legs = neutral;
  scaleBones(long_legs, {"lleguppb", "lleglowb", "rleguppb", "rleglowb"}, 1.4);
  const double long_leg = proc::targetLegLength(long_legs);
  proc::LocomotionSample tall = proc::generateLocomotionPose(long_legs, loaded, state);
  stats.expect(std::abs(tall.pose.root_translation.z / mid.pose.root_translation.z - long_leg / leg) < 1e-6,
               "the same vertical curve scales with the other body");

  proc::LocomotionCommand command;
  command.desired_speed_mps = 1.2;
  command.max_accel_mps2 = 0.4;
  proc::LocomotionState limited;
  proc::advanceLocomotion(limited, loaded, neutral, command, 1.0);
  stats.expect(std::abs(limited.actual_speed_mps - 0.4) < 1e-9, "acceleration limits the generator speed");
  command.desired_speed_mps = 0.0;
  command.max_accel_mps2 = 0.3;
  limited.actual_speed_mps = 1.0;
  proc::advanceLocomotion(limited, loaded, neutral, command, 1.0);
  stats.expect(std::abs(limited.actual_speed_mps - 0.7) < 1e-9, "deceleration limits the generator speed");

  proc::LocomotionState oscillator;
  oscillator.actual_speed_mps = 1.2;
  oscillator.regime = proc::LocomotionRegime::Walk;
  oscillator.phase = 0.25;
  proc::SpeedKnot cadence_knot = slow;
  cadence_knot.cadence_spm = 120.0;
  cadence_knot.dimensionless_speed = proc::dimensionlessSpeed(1.2, leg);
  proc::ProcLocomotionModel cadence_model;
  cadence_model.walk.knots = {cadence_knot};
  cadence_model.walk.present = true;
  cadence_model.walk.support_min = 0.0;
  cadence_model.walk.support_max = 2.0;
  command.desired_speed_mps = 1.2;
  command.max_accel_mps2 = 0.0;
  oscillator.desired_speed_mps = 1.2;
  proc::advanceLocomotion(oscillator, cadence_model, neutral, command, 1.0);
  stats.expect(std::abs(oscillator.phase - 0.25) < 1e-6, "120 steps per minute advances one same-side cycle per second");

  command.desired_speed_mps = 1.25;
  command.force = proc::RegimeForce::Walk;
  command.max_accel_mps2 = 4.0;
  const proc::ProcMotion travel = proc::bakeLocomotion(neutral, cadence_model, command, 8.0, 30.0);
  const glm::dvec3 end = travel.frames.back().root_translation + neutral.joints[proc::jointIndex(proc::JointId::LegRoot)].rest_position;
  const double distance = std::sqrt(end.x * end.x + end.y * end.y);
  stats.expect(std::abs(distance - 10.0) < 1e-6, "1.25 m/s for 8 s travels 10 m");
  stats.expect(travel.source.type == "procedural", "baked motion is marked procedural");
  stats.expect(maxBoneError(neutral, travel) < proc::kRuntimeBoneToleranceM, "generated frames keep bone lengths");

  auto speedFor = [&](double dimensionless) { return dimensionless * std::sqrt(proc::kLocomotionGravity * leg); };
  command.max_accel_mps2 = 0.0;
  command.force = proc::RegimeForce::Walk;
  command.desired_speed_mps = speedFor(0.50);
  proc::LocomotionState forced;
  forced.actual_speed_mps = command.desired_speed_mps;
  forced.regime = proc::LocomotionRegime::Run;
  forced.phase = 0.37;
  proc::advanceLocomotion(forced, loaded, neutral, command, 0.0);
  stats.expect(forced.regime == proc::LocomotionRegime::Walk, "forced walk stays in walk");
  command.force = proc::RegimeForce::Run;
  proc::advanceLocomotion(forced, loaded, neutral, command, 0.0);
  stats.expect(forced.regime == proc::LocomotionRegime::Run, "forced run selects run");
  command.force = proc::RegimeForce::Auto;
  command.desired_speed_mps = speedFor(0.85);
  forced.actual_speed_mps = command.desired_speed_mps;
  forced.desired_speed_mps = command.desired_speed_mps;
  forced.regime = proc::LocomotionRegime::Walk;
  forced.phase = 0.37;
  proc::advanceLocomotion(forced, loaded, neutral, command, 0.0);
  stats.expect(std::abs(forced.phase - 0.37) < 1e-12, "a walk to run transition keeps the phase");
  stats.expect(forced.blend > 0.2 && forced.blend < 0.8, "overlapping walk and run supports blend");
  command.force = proc::RegimeForce::Sprint;
  proc::advanceLocomotion(forced, loaded, neutral, command, 0.0);
  stats.expect(forced.regime == proc::LocomotionRegime::Sprint, "forced sprint selects sprint when the model has it");

  proc::LocomotionState held;
  held.regime = proc::LocomotionRegime::Walk;
  held.phase = 0.2;
  command.force = proc::RegimeForce::Auto;
  command.max_accel_mps2 = 0.0;
  command.desired_speed_mps = speedFor(loaded.walk.support_max + 0.02);
  held.actual_speed_mps = command.desired_speed_mps;
  proc::advanceLocomotion(held, loaded, neutral, command, 0.0);
  stats.expect(held.regime == proc::LocomotionRegime::Walk, "a small step outside the support does not flicker the regime");
  command.desired_speed_mps = speedFor(1.20);
  held.actual_speed_mps = command.desired_speed_mps;
  proc::advanceLocomotion(held, loaded, neutral, command, 0.0);
  stats.expect(held.regime == proc::LocomotionRegime::Run, "a speed inside run support leaves walk");
  command.desired_speed_mps = speedFor(1.18);
  held.actual_speed_mps = command.desired_speed_mps;
  const double held_phase = held.phase;
  proc::advanceLocomotion(held, loaded, neutral, command, 0.0);
  stats.expect(held.regime == proc::LocomotionRegime::Run && std::abs(held.phase - held_phase) < 1e-12,
               "a small speed change does not flicker back to walk");

  std::vector<proc::ProcAnalysis> analyses = proc::makeSyntheticLocomotionSet(neutral);
  proc::ProcAnalysis other_subject = analyses[1];
  other_subject.source.subject = "other";
  analyses.push_back(other_subject);
  proc::ModelBuildSettings settings;
  settings.bin_width = 0.12;
  settings.min_cycles_per_knot = 1;
  settings.subject = "synthetic";
  std::vector<std::string> warnings;
  const proc::ProcLocomotionModel built = proc::buildLocomotionModel(analyses, settings, warnings);
  int walk_cycles = 0;
  int run_cycles = 0;
  int sprint_cycles = 0;
  for (const proc::SpeedKnot& knot : built.walk.knots) walk_cycles += knot.cycle_count;
  for (const proc::SpeedKnot& knot : built.run.knots) run_cycles += knot.cycle_count;
  for (const proc::SpeedKnot& knot : built.sprint.knots) sprint_cycles += knot.cycle_count;
  stats.expect(built.walk.present && built.run.present && built.sprint.present, "walk, run, and sprint regimes are built");
  stats.expect(walk_cycles == 3 && run_cycles == 2 && sprint_cycles == 2, "rejected, mirrored, and other-subject cycles are excluded");
  stats.expect(built.phase_samples == 100, "the runtime model stores 100 periodic samples");
  stats.expect(built.max_seam_error < 0.08, "periodic synthetic cycles have a small phase seam");
  bool finite_logs = true;
  for (const proc::SpeedKnot& knot : built.walk.knots) {
    if (!std::isfinite(knot.rotation[static_cast<int>(proc::LocoChannel::LeftHip)][0].x)) finite_logs = false;
    if (glm::length(knot.rotation[static_cast<int>(proc::LocoChannel::LeftKnee)][10]) < 1e-6 &&
        glm::length(knot.rotation[static_cast<int>(proc::LocoChannel::LeftKnee)][80]) < 1e-4)
      finite_logs = false;
  }
  stats.expect(finite_logs, "aggregated curves keep a 3D knee swing");

  proc::SyntheticClipSpec right_spec;
  right_spec.speed_mps = 1.2;
  proc::ProcAnalysis right_only = proc::makeSyntheticLocomotionAnalysis(neutral, right_spec);
  right_only.cycles.clear();
  proc::GaitCycle right;
  right.side = "right";
  right.accepted = true;
  right.classification = "straight";
  right.observed_regime = "walk";
  right.start_time = 1.0;
  right.end_time = 2.0;
  right.quality.overall = 0.9;
  right.measurements.speed_mps = 1.2;
  right.measurements.cadence_spm = 120.0;
  right.measurements.normalized_stride_length = 1.2 / leg;
  right.measurements.stance_fraction = 0.6;
  right_only.signals.clear();
  proc::SignalFrame only;
  only.time = 1.0;
  only.root_z = neutral.joints[proc::jointIndex(proc::JointId::LegRoot)].rest_position.z;
  only.log_vector[static_cast<int>(proc::RotChannel::RightKnee)] = glm::dvec3(0.45, 0.0, 0.0);
  right_only.signals.push_back(only);
  only.time = 2.0;
  right_only.signals.push_back(only);
  right_only.cycles.push_back(right);
  std::vector<std::string> mirror_warnings;
  const proc::ProcLocomotionModel mirrored = proc::buildLocomotionModel({right_only}, settings, mirror_warnings);
  stats.expect(!mirrored.walk.knots.empty(), "a right cycle still builds a knot");
  if (!mirrored.walk.knots.empty()) {
    const double mirrored_left_knee = mirrored.walk.knots.front().rotation[static_cast<int>(proc::LocoChannel::LeftKnee)][0].x;
    stats.expect(std::abs(mirrored_left_knee - 0.45) < 1e-3, "mirroring a right cycle canonicalizes it onto the left");
  }

  const std::filesystem::path temp_model = std::filesystem::temp_directory_path() / "proc_synthetic_locomotion_test.procloco.json";
  stats.expect(proc::writeLocomotionModelToFile(built, temp_model).ok(), "synthetic locomotion model writes to a temporary file");
  proc::ProcLocomotionModel from_disk;
  stats.expect(proc::readLocomotionModelFromFile(temp_model, from_disk).ok(), "temporary locomotion model reads back");
  const std::filesystem::path public_model = root / "assets" / "models" / "synthetic_locomotion.procloco.json";
  proc::ProcLocomotionModel published;
  stats.expect(proc::readLocomotionModelFromFile(public_model, published).ok(), "public synthetic locomotion model reads");
  stats.expect(!std::filesystem::exists(root / "assets" / "models" / "synthetic_locomotion.procloco.json.tmp"),
               "tests do not leave a rewrite of the public model");

  command.force = proc::RegimeForce::Walk;
  command.desired_speed_mps = 1.30;
  command.heading = 0.0;
  command.max_accel_mps2 = 5.0;
  const proc::ProcMotion generated = proc::bakeLocomotion(neutral, built, command, 6.0, 50.0);
  stats.expect(maxBoneError(neutral, generated) < proc::kRuntimeBoneToleranceM, "walk generation keeps neutral bone lengths");
  const std::filesystem::path motion_path = std::filesystem::temp_directory_path() / "proc_generated_locomotion.procmo";
  stats.expect(proc::writeProcMotionToFile(generated, motion_path).ok(), "generated procmo writes");
  proc::ProcMotion reloaded;
  stats.expect(proc::loadProcMotionFromFile(motion_path, reloaded).ok(), "generated procmo loads");
  stats.expect(reloaded.frames.size() == generated.frames.size(), "generated procmo roundtrip keeps the frames");
  stats.expect(reloaded.frames.size() > 2 && reloaded.frames[2].contacts.block_present, "generated contacts are stored on the motion");

  proc::ProcBody short_legs = neutral;
  scaleBones(short_legs, {"lleguppb", "lleglowb", "rleguppb", "rleglowb"}, 0.62);
  scaleBones(short_legs, {"larmuppb", "larmlowb", "rarmuppb", "rarmlowb"}, 1.45);
  const proc::ProcMotion short_motion = proc::bakeLocomotion(short_legs, built, command, 2.0, 30.0);
  const double short_error = maxBoneError(short_legs, short_motion);
  stats.expect(short_error < proc::kRuntimeBoneToleranceM, "short-leg long-arm body keeps its own bone lengths");
  double female_error = -1.0;
  double male_error = -1.0;
  auto bakePreset = [&](const std::filesystem::path& path, double& error, const char* label) {
    proc::ProcBody preset;
    if (!proc::loadProcBodyFromFile(path, preset).ok()) return;
    error = maxBoneError(preset, proc::bakeLocomotion(preset, built, command, 2.0, 30.0));
    stats.expect(error < proc::kRuntimeBoneToleranceM, label);
  };
  bakePreset(root / "local_assets" / "ansur_female_mean.procbody.json", female_error, "ANSUR female preset keeps its bone lengths");
  bakePreset(root / "local_assets" / "ansur_male_mean.procbody.json", male_error, "ANSUR male preset keeps its bone lengths");
  bakePreset(root / "assets" / "bodies" / "ansur_male_mean.procbody.json", male_error, "ANSUR male preset keeps its bone lengths");

  const proc::FootSlipReport slip = proc::measureContactSlip(neutral, generated);
  stats.expect(slip.left_mean_mps >= 0.0 && slip.right_mean_mps >= 0.0 && std::isfinite(slip.left_peak_mps) && std::isfinite(slip.right_peak_mps),
               "foot slip diagnostics are finite and non-correcting");

  proc::SourceClip generated_clip = clipFromGenerated(neutral, generated);
  std::array<int, proc::kJointCount> identity{};
  for (int index = 0; index < proc::kJointCount; ++index) identity[static_cast<std::size_t>(index)] = index;
  proc::AnalysisInput input;
  input.source = &generated_clip;
  input.motion = &generated;
  input.semantics = proc::semanticsFromMapping(identity);
  const proc::ProcAnalysis closed = proc::analyzeClip(input);
  double speed_sum = 0.0;
  int speed_count = 0;
  for (std::size_t index = closed.signals.size() / 5; index + closed.signals.size() / 5 < closed.signals.size(); ++index) {
    speed_sum += closed.signals[index].root_horizontal_speed;
    ++speed_count;
  }
  const double measured_speed = speed_count > 0 ? speed_sum / static_cast<double>(speed_count) : 0.0;
  stats.expect(std::abs(measured_speed - 1.30) < 0.15, "closed-loop root speed stays near the command");
  int accepted = 0;
  double cadence_sum = 0.0;
  double stride_sum = 0.0;
  double stance_sum = 0.0;
  for (const proc::GaitCycle& cycle : closed.cycles) {
    if (!cycle.accepted || cycle.mirrored) continue;
    ++accepted;
    cadence_sum += cycle.measurements.cadence_spm;
    stride_sum += cycle.measurements.stride_length_m;
    stance_sum += cycle.measurements.stance_fraction;
  }
  if (accepted > 0) {
    stats.expect(std::abs(cadence_sum / accepted - 120.0) < 20.0, "closed-loop cadence is near the model");
    stats.expect(std::abs(stride_sum / accepted - 1.30) < 0.25, "closed-loop stride is near the commanded speed times the cycle");
  }

  auto curveSpread = [](const proc::SpeedKnot& knot, proc::LocoChannel channel) {
    double spread = 0.0;
    const auto& curve = knot.rotation[static_cast<int>(channel)];
    for (int index = 1; index < proc::kLocoPhaseSamples; ++index) {
      spread = std::max(spread, glm::length(curve[static_cast<std::size_t>(index)] - curve[0]));
    }
    return spread;
  };
  auto kneeFlexion = [&](const proc::RegimeModel& regime, proc::JointId knee) {
    double lowest = 0.0;
    bool inside = true;
    const glm::dvec3 axis = neutral.constraints[proc::jointIndex(knee)].axis;
    const double minimum = neutral.constraints[proc::jointIndex(knee)].min_deg;
    const double maximum = neutral.constraints[proc::jointIndex(knee)].max_deg;
    const int channel = knee == proc::JointId::LeftKnee ? static_cast<int>(proc::LocoChannel::LeftKnee)
                                                        : static_cast<int>(proc::LocoChannel::RightKnee);
    for (const proc::SpeedKnot& knot : regime.knots) {
      for (const glm::dvec3& sample : knot.rotation[static_cast<std::size_t>(channel)]) {
        const double degrees = proc::hingeAngleDegrees(proc::quatExp(sample), axis);
        lowest = std::min(lowest, degrees);
        if (degrees < minimum - 1e-3 || degrees > maximum + 1e-3) inside = false;
      }
    }
    return std::pair<double, bool>{lowest, inside};
  };
  const proc::ProcLocomotionModel& demo = published.walk.present ? published : built;
  for (const proc::RegimeModel* regime : {&demo.walk, &demo.run, &demo.sprint}) {
    stats.expect(!regime->knots.empty(), "each synthetic regime has a knot");
    if (regime->knots.empty()) continue;
    const proc::SpeedKnot& knot = regime->knots.front();
    stats.expect(curveSpread(knot, proc::LocoChannel::LeftHip) > 0.05 && curveSpread(knot, proc::LocoChannel::RightHip) > 0.05,
                 "hip curves move");
    stats.expect(curveSpread(knot, proc::LocoChannel::LeftKnee) > 0.15 && curveSpread(knot, proc::LocoChannel::RightKnee) > 0.15,
                 "knee curves move");
    stats.expect(curveSpread(knot, proc::LocoChannel::LeftAnkle) > 0.05 && curveSpread(knot, proc::LocoChannel::RightAnkle) > 0.05,
                 "ankle curves move");
    stats.expect(curveSpread(knot, proc::LocoChannel::LeftShoulder) > 0.05 && curveSpread(knot, proc::LocoChannel::RightShoulder) > 0.05,
                 "shoulder curves move");
    stats.expect(curveSpread(knot, proc::LocoChannel::LeftElbow) > 0.02 && curveSpread(knot, proc::LocoChannel::RightElbow) > 0.02,
                 "elbow curves move");
    stats.expect(curveSpread(knot, proc::LocoChannel::Pelvis) > 0.01 && curveSpread(knot, proc::LocoChannel::Spine1) > 0.01,
                 "pelvis and spine curves move");
    const auto left_knee_range = kneeFlexion(*regime, proc::JointId::LeftKnee);
    const auto right_knee_range = kneeFlexion(*regime, proc::JointId::RightKnee);
    stats.expect(left_knee_range.second && right_knee_range.second, "synthetic knee angles stay inside the hinge range");
    stats.expect(left_knee_range.first < -25.0 && right_knee_range.first < -25.0, "synthetic knees enter the flexion side of the hinge");
  }

  proc::LocomotionState gait_state;
  gait_state.fade = 1.0;
  gait_state.regime = proc::LocomotionRegime::Walk;
  gait_state.dimensionless_speed = demo.walk.knots.front().dimensionless_speed;
  gait_state.phase = 0.0;
  const proc::WorldPose phase0 = proc::forwardKinematics(neutral, proc::generateLocomotionPose(neutral, demo, gait_state).pose);
  gait_state.phase = 0.5;
  const proc::WorldPose phase_half = proc::forwardKinematics(neutral, proc::generateLocomotionPose(neutral, demo, gait_state).pose);
  const double left_arm0 = phase0.joints[proc::jointIndex(proc::JointId::LeftElbow)].position.y -
                           phase0.joints[proc::jointIndex(proc::JointId::LeftShoulder)].position.y;
  const double right_arm0 = phase0.joints[proc::jointIndex(proc::JointId::RightElbow)].position.y -
                            phase0.joints[proc::jointIndex(proc::JointId::RightShoulder)].position.y;
  const double left_arm_half = phase_half.joints[proc::jointIndex(proc::JointId::LeftElbow)].position.y -
                               phase_half.joints[proc::jointIndex(proc::JointId::LeftShoulder)].position.y;
  const double right_arm_half = phase_half.joints[proc::jointIndex(proc::JointId::RightElbow)].position.y -
                                phase_half.joints[proc::jointIndex(proc::JointId::RightShoulder)].position.y;
  stats.expect(right_arm0 > left_arm0 + 0.02 && left_arm_half > right_arm_half + 0.02, "arms swing opposite each other across the stride");
  stats.expect(phase0.joints[proc::jointIndex(proc::JointId::LeftFoot)].position.y > phase0.joints[proc::jointIndex(proc::JointId::LeftHip)].position.y &&
                   right_arm0 > 0.0 && left_arm0 < 0.0,
               "the right arm is forward while the left leg is forward");
  stats.expect(phase0.joints[proc::jointIndex(proc::JointId::LeftElbow)].position.z <
                       phase0.joints[proc::jointIndex(proc::JointId::LeftShoulder)].position.z - 0.15 &&
                   phase0.joints[proc::jointIndex(proc::JointId::RightElbow)].position.z <
                       phase0.joints[proc::jointIndex(proc::JointId::RightShoulder)].position.z - 0.15,
               "both arms hang below the shoulders");

  const proc::ConstraintAudit audit = proc::auditLocomotionModel(neutral, demo);
  stats.expect(audit.ok && audit.violations == 0, "public synthetic model passes the neutral constraint audit");
  proc::ProcLocomotionModel broken = demo;
  broken.walk.knots.front().rotation[static_cast<int>(proc::LocoChannel::LeftKnee)][70] =
      proc::quatLog(proc::quatAxisAngle(neutral.constraints[proc::jointIndex(proc::JointId::LeftKnee)].axis, 0.85));
  const proc::ConstraintAudit broken_audit = proc::auditLocomotionModel(neutral, broken);
  stats.expect(!broken_audit.ok && broken_audit.worst_joint == "lleglowj", "an inverted knee is reported by the audit");
  stats.expect(broken_audit.worst_message.find("requested") != std::string::npos, "the audit keeps the requested angle");
  const glm::dvec3 broken_sample = broken.walk.knots.front().rotation[static_cast<int>(proc::LocoChannel::LeftKnee)][70];
  proc::LocomotionState broken_state = gait_state;
  broken_state.phase = 0.70;
  broken_state.dimensionless_speed = broken.walk.knots.front().dimensionless_speed;
  const proc::GeneratedPoseResult warned = proc::generateConstrainedLocomotionPose(neutral, broken, broken_state, proc::LimitMode::Warn);
  const proc::GeneratedPoseResult clamped =
      proc::generateConstrainedLocomotionPose(neutral, broken, broken_state, proc::LimitMode::Clamp);
  stats.expect(warned.limits.pose.local_rotation[proc::jointIndex(proc::JointId::LeftKnee)] ==
                   warned.raw.local_rotation[proc::jointIndex(proc::JointId::LeftKnee)],
               "WARN leaves the requested pose unchanged");
  stats.expect(clamped.limits.clamped, "CLAMP reports that it changed the runtime pose");
  const double clamped_knee = proc::hingeAngleDegrees(clamped.limits.pose.local_rotation[proc::jointIndex(proc::JointId::LeftKnee)],
                                                      neutral.constraints[proc::jointIndex(proc::JointId::LeftKnee)].axis);
  stats.expect(clamped_knee <= 10.0 + 1e-3 && clamped_knee >= -150.0, "CLAMP holds the knee inside the hinge");
  stats.expect(broken.walk.knots.front().rotation[static_cast<int>(proc::LocoChannel::LeftKnee)][70] == broken_sample,
               "CLAMP does not mutate the locomotion model");
  const proc::ConstraintAudit short_audit = proc::auditLocomotionModel(short_legs, demo);
  stats.expect(short_audit.poses == audit.poses, "the same model is audited on another body");
  std::cout << proc::formatConstraintAudit(audit, "neutral");

  const std::filesystem::path generator_source = root / "src" / "locomotion" / "locomotion_generator.cpp";
  stats.expect(!fileHas(generator_source, "analysis/") && !fileHas(generator_source, "import/") &&
                   !fileHas(generator_source, "retarget/") && !fileHas(generator_source, "assimp"),
               "the runtime generator does not include offline systems");
  const std::filesystem::path cmake = root / "CMakeLists.txt";
  stats.expect(fileHas(cmake, "target_link_libraries(proc_runtime PUBLIC glm::glm nlohmann_json::nlohmann_json)") &&
                   fileHas(cmake, "target_link_libraries(proc_runtime_link PRIVATE proc_runtime)") &&
                   !fileHas(cmake, "proc_runtime PUBLIC glm::glm nlohmann_json::nlohmann_json assimp"),
               "proc_runtime does not link Assimp");

  std::cout << "locomotion regimes walk " << built.walk.knots.size() << " run " << built.run.knots.size() << " sprint "
            << built.sprint.knots.size() << "\n";
  std::cout << "locomotion cycles walk " << walk_cycles << " run " << run_cycles << " sprint " << sprint_cycles << "\n";
  std::cout << "phase seam " << built.max_seam_error << " " << built.max_seam_channel << "\n";
  std::cout << "root distance error " << (distance - 10.0) << "\n";
  std::cout << "cadence phase error " << (oscillator.phase - 0.25) << "\n";
  std::cout << "neutral bone error " << maxBoneError(neutral, generated) << "\n";
  std::cout << "short-leg bone error " << short_error << "\n";
  std::cout << "female bone error " << female_error << "\n";
  std::cout << "male bone error " << male_error << "\n";
  std::cout << "slip L mean " << slip.left_mean_mps << " peak " << slip.left_peak_mps << " samples " << slip.left_samples << "\n";
  std::cout << "slip R mean " << slip.right_mean_mps << " peak " << slip.right_peak_mps << " samples " << slip.right_samples << "\n";
  std::cout << "closed-loop speed " << measured_speed << " accepted " << accepted << "\n";
  if (accepted > 0) {
    std::cout << "closed-loop cadence " << (cadence_sum / accepted) << " stride " << (stride_sum / accepted) << " stance "
              << (stance_sum / accepted) << "\n";
  } else {
    std::cout << "closed-loop cadence not recovered\n";
    for (const proc::GaitCycle& cycle : closed.cycles) {
      if (!cycle.mirrored) std::cout << "cycle " << cycle.side << " " << cycle.rejection_reason << "\n";
    }
  }
  const bool datasets = std::filesystem::exists(root / "datasets") || std::filesystem::exists(root / "analysis_output");
  std::cout << "real analysis present " << (datasets ? "yes" : "no") << "\n";
  return stats.failed;
}
