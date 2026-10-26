#include "test_support.h"

#include "analysis/reference_capture.h"
#include "locomotion/locomotion_diagnostics.h"
#include "locomotion/locomotion_generator.h"
#include "locomotion/locomotion_model_io.h"
#include "validation/bone_length_monitor.h"
#include "validation/tolerances.h"

#include <cmath>
#include <fstream>
#include <sstream>

namespace {

void fill(proc::ProcAnalysis& analysis, const std::string& name, double speed, double hip, bool in_place) {
  analysis.source.clip = name;
  analysis.source.theme = name;
  analysis.source_leg_length_m = 0.8;
  for (int frame = 0; frame <= 20; ++frame) {
    proc::SignalFrame signal;
    signal.time = frame / 20.0;
    signal.root_horizontal_speed = in_place ? 0.0 : speed;
    signal.root_z = 0.95 + 0.01 * std::sin(signal.time * 6.283185307179586);
    const double wave = std::sin(signal.time * 6.283185307179586) * hip;
    signal.log_vector[static_cast<int>(proc::LocoChannel::LeftHip)] = glm::dvec3(hip == 0.0 ? 0.2 : wave, 0.0, 0.0);
    signal.log_vector[static_cast<int>(proc::LocoChannel::LeftKnee)] = glm::dvec3(wave * 0.5, 0.0, 0.0);
    signal.log_vector[static_cast<int>(proc::LocoChannel::LeftAnkle)] = glm::dvec3(wave * 0.25, 0.0, 0.0);
    signal.log_vector[static_cast<int>(proc::LocoChannel::LeftShoulder)] = glm::dvec3(-wave * 0.2, 0.0, 0.0);
    signal.log_vector[static_cast<int>(proc::LocoChannel::LeftElbow)] = glm::dvec3(std::abs(wave) * 0.1, 0.0, 0.0);
    analysis.signals.push_back(signal);
  }
}

proc::GaitCycle cycle(int index, bool accepted, double speed, const std::string& regime) {
  proc::GaitCycle value;
  value.index = index;
  value.side = "left";
  value.accepted = accepted;
  value.classification = "straight";
  value.observed_regime = regime;
  value.start_time = 0.0;
  value.end_time = 1.0;
  value.measurements.speed_mps = speed;
  value.measurements.cadence_spm = 112.0;
  value.measurements.stance_fraction = 0.62;
  value.measurements.flight_fraction = 0.0;
  value.quality.overall = 0.8;
  if (!accepted) value.rejection_reason = "turn";
  return value;
}

proc::ReferenceClipRecord clipFrom(proc::ProcAnalysis analysis, const std::string& regime, std::optional<double> speed) {
  proc::ReferenceClipRecord clip;
  clip.source_path = analysis.source.clip;
  clip.format = "fixture";
  clip.regime = regime;
  clip.measured_speed_mps = proc::measuredRootSpeed(analysis);
  clip.in_place = proc::sourceAppearsInPlace(analysis);
  clip.speed_override_mps = speed;
  clip.source_leg_length_m = analysis.source_leg_length_m;
  clip.analysis = std::move(analysis);
  proc::selectAcceptedCycles(clip);
  return clip;
}

void scaleLegs(proc::ProcBody& body, double scale) {
  for (proc::BoneGeom& bone : body.bones) {
    if (bone.name != "lleguppb" && bone.name != "lleglowb" && bone.name != "rleguppb" && bone.name != "rleglowb") continue;
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

std::string readFile(const std::filesystem::path& path) {
  std::ifstream stream(path);
  std::stringstream buffer;
  buffer << stream.rdbuf();
  return buffer.str();
}

}  // namespace

int runReferenceCaptureTests() {
  TestStats stats;
  const std::filesystem::path root = std::filesystem::path(PROCMOTION_SOURCE_DIR);
  proc::ProcAnalysis walk;
  fill(walk, "Walk.fbx", 1.25, 0.2, false);
  walk.cycles.push_back(cycle(0, true, 1.25, "walk"));
  walk.cycles.push_back(cycle(1, false, 1.25, "walk"));
  proc::ReferenceSet set;
  stats.expect(proc::addReferenceClip(set, clipFrom(walk, "walk", std::nullopt)), "reference set accepts a clip");
  stats.expect(set.clips.front().included_cycles.size() == 1, "rejected cycles are excluded by default");
  stats.expect(proc::removeReferenceClip(set, 0) && set.clips.empty(), "reference set can remove a clip");

  proc::ProcAnalysis kept = walk;
  const double original_speed = kept.cycles.front().measurements.speed_mps;
  proc::addReferenceClip(set, clipFrom(kept, "run", std::nullopt));
  const proc::ReferenceBuildResult run_regime = proc::buildReferenceModel(set);
  stats.expect(run_regime.ok && run_regime.model.run.present && !run_regime.model.walk.present, "explicit regime assignment");
  stats.expect(run_regime.model.run.knots.size() == 1, "single reference clip produces one knot");
  stats.expect(kept.cycles.front().measurements.speed_mps == original_speed, "saved analysis is not mutated by the build");
  set.clips.clear();

  proc::ProcAnalysis slow;
  fill(slow, "WalkSlow.fbx", 1.10, 0.15, false);
  slow.cycles.push_back(cycle(0, true, 1.10, "walk"));
  proc::ProcAnalysis fast;
  fill(fast, "WalkFast.fbx", 2.40, 0.35, false);
  fast.cycles.push_back(cycle(0, true, 2.40, "walk"));
  proc::addReferenceClip(set, clipFrom(slow, "walk", std::nullopt));
  proc::addReferenceClip(set, clipFrom(fast, "walk", std::nullopt));
  const proc::ReferenceBuildResult multi = proc::buildReferenceModel(set);
  stats.expect(multi.ok && multi.model.walk.knots.size() == 2, "multiple speeds produce multiple knots");
  stats.expect(std::abs(multi.model.walk.knots.front().median_speed_mps - 1.10) < 1e-6, "measured speed is used when root travel exists");
  set.clips.clear();

  proc::ProcAnalysis planted;
  fill(planted, "WalkInPlace.fbx", 0.0, 0.2, true);
  planted.cycles.push_back(cycle(0, true, 0.0, "walk"));
  proc::addReferenceClip(set, clipFrom(planted, "walk", std::nullopt));
  stats.expect(set.clips.front().in_place, "in-place source is detected");
  const proc::ReferenceBuildResult refused = proc::buildReferenceModel(set);
  stats.expect(!refused.ok && refused.error.find("in-place") != std::string::npos, "in-place source without manual speed refuses the build");
  set.clips.front().speed_override_mps = 1.25;
  const proc::ReferenceBuildResult manual = proc::buildReferenceModel(set);
  stats.expect(manual.ok && std::abs(manual.model.walk.knots.front().median_speed_mps - 1.25) < 1e-6, "manual speed override");
  stats.expect(proc::speedProvenance(set.clips.front()) == proc::SpeedProvenance::Manual, "manual speed provenance is preserved");
  stats.expect(set.clips.front().analysis.cycles.front().measurements.speed_mps == 0.0, "manual speed does not rewrite the analysis");

  proc::ProcAnalysis one;
  fill(one, "OneCycle.fbx", 1.30, 0.0, false);
  for (proc::SignalFrame& signal : one.signals) {
    signal.log_vector[static_cast<int>(proc::LocoChannel::LeftHip)] = glm::dvec3(0.2, 0.0, 0.0);
  }
  one.cycles.push_back(cycle(0, true, 1.30, "walk"));
  proc::ReferenceSet single;
  proc::addReferenceClip(single, clipFrom(one, "walk", std::nullopt));
  const proc::ReferenceBuildResult one_knot = proc::buildReferenceModel(single);
  stats.expect(one_knot.fidelity.available && one_knot.fidelity.hip_rms < 1e-8 && one_knot.fidelity.knee_rms < 1e-8 &&
                   one_knot.fidelity.ankle_rms < 1e-8 && one_knot.fidelity.pelvis_rms < 1e-8 && one_knot.fidelity.shoulder_rms < 1e-8 &&
                   one_knot.fidelity.elbow_rms < 1e-8,
               "single-cycle model reproduces its normalized curves");
  stats.expect(one_knot.model.max_seam_error < 1e-8, "phase seam remains valid");
  const proc::ReferenceFidelity again = proc::measureReferenceFidelity(one_knot.model, proc::inputsFromReferenceSet(single), proc::referenceBuildSettings());
  stats.expect(std::abs(again.hip_rms - one_knot.fidelity.hip_rms) < 1e-12, "source/model phase comparison is deterministic");
  proc::GaitCycle timed = one.cycles.front();
  const double at_half = proc::referencePhaseTime(timed, 0.50);
  stats.expect(std::abs(at_half - 0.50) < 1e-12, "A/B preview keeps the same phase");

  proc::ProcAnalysis median_a;
  fill(median_a, "A.fbx", 1.30, 0.0, false);
  for (proc::SignalFrame& signal : median_a.signals) signal.log_vector[static_cast<int>(proc::LocoChannel::LeftHip)] = glm::dvec3(0.20, 0.0, 0.0);
  median_a.cycles.push_back(cycle(0, true, 1.30, "walk"));
  proc::ProcAnalysis median_b = median_a;
  median_b.source.clip = "B.fbx";
  for (proc::SignalFrame& signal : median_b.signals) signal.log_vector[static_cast<int>(proc::LocoChannel::LeftHip)] = glm::dvec3(0.40, 0.0, 0.0);
  median_b.cycles.front().index = 0;
  proc::ReferenceSet medians;
  proc::addReferenceClip(medians, clipFrom(median_a, "walk", std::nullopt));
  proc::addReferenceClip(medians, clipFrom(median_b, "walk", std::nullopt));
  const proc::ReferenceBuildResult median_model = proc::buildReferenceModel(medians);
  stats.expect(median_model.ok && median_model.model.walk.knots.size() == 1 && median_model.model.walk.knots.front().cycle_count == 2,
               "multi-cycle aggregation uses one knot");
  stats.expect(std::abs(median_model.model.walk.knots.front().rotation[static_cast<int>(proc::LocoChannel::LeftHip)][0].x - 0.30) < 1e-9,
               "multi-cycle aggregation uses the median");

  proc::ProcBody neutral;
  stats.expect(proc::loadProcBodyFromFile(root / "assets" / "bodies" / "neutral.procbody.json", neutral).ok(), "neutral body loads");
  const proc::ConstraintAudit audit = proc::auditLocomotionModel(neutral, one_knot.model);
  stats.expect(audit.poses > 0, "constraint audit runs on the built model");
  const std::filesystem::path temp = std::filesystem::temp_directory_path() / "proc_reference_capture";
  std::filesystem::create_directories(temp);
  stats.expect(proc::exportReferenceModel(single, one_knot, audit.violations, temp / "one.procloco.json").ok(), "reference model exports");
  proc::ProcLocomotionModel loaded;
  stats.expect(proc::readLocomotionModelFromFile(temp / "one.procloco.json", loaded).ok() && loaded.walk.knots.size() == 1, "exported model roundtrips");
  proc::LocomotionCommand command;
  command.desired_speed_mps = 1.30;
  command.force = proc::RegimeForce::Walk;
  const proc::ProcMotion generated = proc::bakeLocomotion(neutral, loaded, command, 1.0, 30.0);
  stats.expect(!generated.frames.empty(), "exported model loads in generator mode");
  proc::ProcBody short_legs = neutral;
  scaleLegs(short_legs, 0.62);
  const proc::BoneLengthReport neutral_bones = proc::measureBoneLengths(neutral, proc::forwardKinematics(neutral, generated.frames.front()), proc::kRuntimeBoneToleranceM);
  proc::LocomotionState state;
  state.phase = 0.25;
  state.actual_speed_mps = 1.30;
  const proc::Pose short_pose = proc::generateLocomotionPose(short_legs, loaded, state).pose;
  const proc::BoneLengthReport short_bones = proc::measureBoneLengths(short_legs, proc::forwardKinematics(short_legs, short_pose), proc::kRuntimeBoneToleranceM);
  stats.expect(neutral_bones.ok && short_bones.ok, "reference-derived model remains body-independent");
  const proc::FootSlipReport slip = proc::measureContactSlip(neutral, generated);
  stats.expect(std::isfinite(slip.left_mean_mps) && std::isfinite(slip.left_peak_mps), "foot-slip diagnostic works");

  const std::string before = readFile(root / "assets" / "models" / "synthetic_locomotion.procloco.json");
  const proc::Status blocked = proc::exportReferenceModel(single, one_knot, 0, root / "assets" / "models" / "reference.procloco.json");
  stats.expect(!blocked.ok() && readFile(root / "assets" / "models" / "synthetic_locomotion.procloco.json") == before,
               "reference models are not written into public assets");
  stats.expect(proc::writeReferenceSet(set, temp / "set.procref.json").ok(), "reference set metadata writes");
  proc::ReferenceSet restored;
  stats.expect(proc::readReferenceSet(temp / "set.procref.json", restored).ok() && restored.clips.size() == 1 &&
                   proc::speedProvenance(restored.clips.front()) == proc::SpeedProvenance::Manual,
               "reference metadata roundtrips the manual provenance");

  char program[] = "proc_motion_reference";
  char input_arg[] = "--input";
  char input_value[] = "Walk.fbx";
  char regime_arg[] = "--regime";
  char regime_value[] = "walk";
  char speed_arg[] = "--speed";
  char speed_value[] = "1.25";
  char* argv[] = {program, input_arg, input_value, regime_arg, regime_value, speed_arg, speed_value};
  const proc::ReferenceArgumentParse parsed = proc::parseReferenceArgs(7, argv);
  stats.expect(parsed.ok && parsed.request.jobs.size() == 1 && parsed.request.jobs.front().regime == "walk" &&
                   parsed.request.jobs.front().speed_override_mps && *parsed.request.jobs.front().speed_override_mps == 1.25,
               "reference CLI parsing");

  const std::string generator = readFile(root / "src" / "locomotion" / "locomotion_generator.cpp");
  stats.expect(generator.find("reference_capture") == std::string::npos && generator.find("SourceClip") == std::string::npos,
               "proc_runtime stays independent of reference capture");
  std::cout << "reference capture tests ok\n";
  return stats.failed;
}
