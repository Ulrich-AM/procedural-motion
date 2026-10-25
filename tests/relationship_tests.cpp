#include "test_support.h"

#include "analysis/analysis_io.h"
#include "analysis/proc_analysis.h"
#include "analysis/relationship_model.h"
#include "body/proc_body.h"
#include "retarget/retarget_profile.h"
#include "retarget/retargeter.h"
#include "rig/forward_kinematics.h"
#include "synthetic_gait.h"

#include <glm/geometric.hpp>

#include <cmath>
#include <filesystem>

namespace {

std::filesystem::path sourceRoot() { return std::filesystem::path(PROCMOTION_SOURCE_DIR); }

proc::GaitCycle cycleAt(const std::string& regime, double speed, double stride) {
  proc::GaitCycle cycle;
  cycle.accepted = true;
  cycle.automatic_accepted = true;
  cycle.classification = "straight";
  cycle.observed_regime = regime;
  cycle.side = "left";
  cycle.measurements.speed_mps = speed;
  cycle.measurements.stride_length_m = stride;
  cycle.measurements.cadence_spm = 100.0;
  cycle.measurements.stance_fraction = regime == "walk" ? 0.6 : 0.3;
  cycle.measurements.flight_fraction = regime == "walk" ? 0.0 : 0.2;
  return cycle;
}

}  // namespace

int runRelationshipTests() {
  TestStats stats;
  proc::ProcAnalysis pooled;
  pooled.cycles.push_back(cycleAt("walk", 1.0, 0.50));
  pooled.cycles.push_back(cycleAt("walk", 1.2, 0.60));
  pooled.cycles.push_back(cycleAt("walk", 1.4, 0.70));
  pooled.cycles.push_back(cycleAt("run", 3.0, 2.10));
  proc::fitRelationships(pooled);
  const proc::RelationshipFit* walk_stride = nullptr;
  const proc::RelationshipFit* run_stride = nullptr;
  for (const proc::RelationshipFit& fit : pooled.relationships) {
    if (fit.name == "stride_length_m" && fit.regime == "walk") walk_stride = &fit;
    if (fit.name == "stride_length_m" && fit.regime == "run") run_stride = &fit;
  }
  stats.expect(walk_stride != nullptr && walk_stride->sample_count == 3, "walk fit uses only walk cycles");
  stats.expect(run_stride != nullptr && run_stride->sample_count == 1, "run fit stays in its own regime");
  stats.expect(walk_stride != nullptr && walk_stride->has_linear_fit && std::abs(walk_stride->slope - 0.5) < 0.05, "stride-speed slope");
  stats.expect(walk_stride != nullptr && !walk_stride->bins.empty(), "speed bins are stored");

  proc::test::SyntheticGait walk = proc::test::makeSyntheticGait(proc::test::SyntheticKind::Walk);
  proc::AnalysisInput input;
  input.source = &walk.source;
  input.motion = &walk.motion;
  proc::ProcAnalysis analysis = proc::analyzeClip(input);
  stats.expect(!analysis.events.empty(), "roundtrip source has events");
  const double automatic = analysis.events.front().automatic_time;
  proc::moveEvent(analysis, 0, analysis.events.front().time + 0.04);
  proc::overrideContactRange(analysis, "left_heel", 1, 2, false);
  const std::string text = proc::writeAnalysisToString(analysis);
  proc::ProcAnalysis loaded;
  const proc::Status status = proc::readAnalysisFromString(text, loaded);
  stats.expect(status.ok(), "analysis JSON parses");
  stats.expect(loaded.format == "PROC_ANALYSIS" && loaded.version == 1, "PROC_ANALYSIS v1");
  stats.expect(!loaded.events.empty() && loaded.events.front().manual_override, "manual event flag survives");
  stats.expect(!loaded.events.empty() && std::abs(loaded.events.front().automatic_time - automatic) < 1e-9, "automatic event time is preserved");
  stats.expect(std::abs(loaded.ground.height_m - analysis.ground.height_m) < 1e-9, "ground height roundtrip");
  stats.expect(std::abs(loaded.source_leg_length_m - analysis.source_leg_length_m) < 1e-9, "leg length roundtrip");
  stats.expect(loaded.contact_overrides.size() == 1, "contact override roundtrip");
  stats.expect(loaded.cycles.size() == analysis.cycles.size(), "cycle count roundtrip");

  proc::ProcBody neutral;
  proc::Status body_status = proc::loadProcBodyFromFile(sourceRoot() / "assets" / "bodies" / "neutral.procbody.json", neutral);
  stats.expect(body_status.ok(), "neutral body loads");
  proc::ProcBody scaled = neutral;
  for (proc::BoneGeom& bone : scaled.bones) {
    if (bone.name == "lleguppb" || bone.name == "rleguppb") {
      bone.rest_vector *= 1.35;
      bone.length_m = glm::length(bone.rest_vector);
    }
  }
  proc::RetargetProfile profile;
  body_status = proc::loadRetargetProfileFromFile(sourceRoot() / "config" / "retarget" / "lafan1.json", profile);
  stats.expect(body_status.ok(), "lafan profile loads");
  std::vector<std::string> names;
  for (const proc::SourceJoint& joint : walk.source.joints) names.push_back(joint.name);
  const std::array<int, proc::kJointCount> mapping = proc::mappingFromProfile(profile, names);
  proc::ProcMotion neutral_motion;
  proc::ProcMotion scaled_motion;
  proc::RetargetReport report;
  proc::retargetClip(neutral, walk.source, mapping, profile.required, {}, neutral_motion, report);
  proc::retargetClip(scaled, walk.source, mapping, profile.required, {}, scaled_motion, report);
  input.motion = &neutral_motion;
  const proc::ProcAnalysis neutral_analysis = proc::analyzeClip(input);
  input.motion = &scaled_motion;
  const proc::ProcAnalysis scaled_analysis = proc::analyzeClip(input);
  stats.expect(std::abs(neutral_analysis.source_leg_length_m - scaled_analysis.source_leg_length_m) < 1e-9, "source leg length ignores the PROC body");
  const proc::GaitCycle* neutral_cycle = nullptr;
  const proc::GaitCycle* scaled_cycle = nullptr;
  for (const proc::GaitCycle& cycle : neutral_analysis.cycles) {
    if (!cycle.mirrored && cycle.side == "left") {
      neutral_cycle = &cycle;
      break;
    }
  }
  for (const proc::GaitCycle& cycle : scaled_analysis.cycles) {
    if (!cycle.mirrored && cycle.side == "left") {
      scaled_cycle = &cycle;
      break;
    }
  }
  stats.expect(neutral_cycle != nullptr && scaled_cycle != nullptr, "both bodies produce the same source cycle");
  if (neutral_cycle != nullptr && scaled_cycle != nullptr) {
    stats.expect(std::abs(neutral_cycle->measurements.speed_mps - scaled_cycle->measurements.speed_mps) < 1e-6, "source speed is body-independent");
    stats.expect(std::abs(neutral_cycle->measurements.stride_length_m - scaled_cycle->measurements.stride_length_m) < 1e-6, "source stride is body-independent");
    stats.expect(std::abs(neutral_cycle->start_time - scaled_cycle->start_time) < 1e-9, "contact time is body-independent");
  }
  if (!neutral_motion.frames.empty() && !scaled_motion.frames.empty()) {
    const proc::WorldPose neutral_world = proc::forwardKinematics(neutral, neutral_motion.frames[neutral_motion.frames.size() / 2]);
    const proc::WorldPose scaled_world = proc::forwardKinematics(scaled, scaled_motion.frames[scaled_motion.frames.size() / 2]);
    const int foot = *proc::findJoint("lfootj");
    stats.expect(glm::length(neutral_world.joints[foot].position - scaled_world.joints[foot].position) > 0.02, "PROC endpoints still follow the loaded body");
  }
  if (stats.failed == 0) std::cout << "relationship tests ok\n";
  return stats.failed;
}
