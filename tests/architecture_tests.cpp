#include "test_support.h"

#include "analysis/analysis_io.h"
#include "analysis/proc_analysis.h"
#include "body/proc_body.h"
#include "body/skeleton.h"
#include "retarget/retargeter.h"
#include "retarget/source_semantics.h"
#include "rig/forward_kinematics.h"
#include "synthetic_gait.h"
#include "validation/bone_length_monitor.h"
#include "validation/tolerances.h"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>

namespace {

std::filesystem::path sourceRoot() { return std::filesystem::path(PROCMOTION_SOURCE_DIR); }

int walkSamples(const proc::ProcAnalysis& analysis) {
  for (const proc::RelationshipFit& fit : analysis.relationships) {
    if (fit.regime == "walk" && fit.name == "stride_length_m") return fit.sample_count;
  }
  return 0;
}

int rawCycles(const proc::ProcAnalysis& analysis, const std::string& side) {
  int count = 0;
  for (const proc::GaitCycle& cycle : analysis.cycles) {
    if (!cycle.mirrored && cycle.side == side) ++count;
  }
  return count;
}

bool warningHas(const proc::ProcAnalysis& analysis, const std::string& text) {
  for (const std::string& warning : analysis.warnings) {
    if (warning.find(text) != std::string::npos) return true;
  }
  return false;
}

proc::AnalysisInput gaitInput(const proc::test::SyntheticGait& gait) {
  proc::AnalysisInput input;
  input.source = &gait.source;
  input.motion = &gait.motion;
  input.semantics = proc::test::syntheticSemantics();
  return input;
}

}  // namespace

int runArchitectureTests() {
  TestStats stats;
  const char* expected_names[] = {
      "legrootj", "spine1j", "spine2j", "armrootj", "neckj", "headj", "lleguppj", "lleglowj", "lfootj", "ltoesj",
      "rleguppj", "rleglowj", "rfootj", "rtoesj", "larmuppj", "larmlowj", "lhandj", "lfingj", "rarmuppj", "rarmlowj",
      "rhandj", "rfingj"};
  stats.expect(static_cast<int>(proc::JointId::Count) == proc::kJointCount, "joint id count matches the frozen skeleton");
  bool names_match = true;
  for (int index = 0; index < proc::kJointCount; ++index) {
    if (std::string(proc::kJointNames[index]) != expected_names[index]) names_match = false;
    if (proc::jointIndex(static_cast<proc::JointId>(index)) != index) names_match = false;
  }
  stats.expect(names_match, "named joint ids follow the frozen joint order");
  stats.expect(std::string(proc::kJointNames[proc::jointIndex(proc::JointId::LeftKnee)]) == "lleglowj", "LeftKnee is lleglowj");

  std::ifstream source(sourceRoot() / "src" / "analysis" / "proc_analysis.cpp");
  std::stringstream buffer;
  buffer << source.rdbuf();
  const std::string analyzer = buffer.str();
  stats.expect(analyzer.find("leftupleg") == std::string::npos, "analysis has no private left-hip alias");
  stats.expect(analyzer.find("lefttoebase") == std::string::npos, "analysis has no private toe alias");
  stats.expect(analyzer.find("findSource") == std::string::npos, "analysis does not search source names");

  proc::test::SyntheticGait named = proc::test::makeSyntheticGait(proc::test::SyntheticKind::Walk);
  proc::AnalysisInput unnamed = gaitInput(named);
  unnamed.semantics = {};
  const proc::ProcAnalysis missed = proc::analyzeClip(unnamed);
  stats.expect(missed.source_leg_length_m == 0.0, "unmapped source names do not imply a foot");
  stats.expect(warningHas(missed, "left foot was not mapped"), "missing foot mapping is an explicit warning");

  proc::test::SyntheticGait custom = proc::test::makeSyntheticGait(proc::test::SyntheticKind::Walk);
  const char* custom_names[] = {"PelvisNode_X", "UpperLeg_L_Custom", "Shin_L_Custom", "Ankle_L_Custom", "Toe_L_Custom",
                                 "UpperLeg_R_Custom", "Shin_R_Custom", "Ankle_R_Custom", "Toe_R_Custom"};
  for (int index = 0; index < 9; ++index) custom.source.joints[static_cast<std::size_t>(index)].name = custom_names[index];
  proc::ProcBody body;
  proc::Status status = proc::loadProcBodyFromFile(sourceRoot() / "assets" / "bodies" / "neutral.procbody.json", body);
  stats.expect(status.ok(), "neutral body loads for the custom mapping");
  const std::array<int, proc::kJointCount> mapping = proc::test::syntheticMapping();
  std::array<bool, proc::kJointCount> required{};
  required.fill(false);
  proc::ProcMotion motion;
  proc::RetargetReport report;
  status = proc::retargetClip(body, custom.source, mapping, required, {}, motion, report);
  stats.expect(status.ok() && !motion.frames.empty(), "custom source names retarget through the explicit mapping");
  const proc::WorldPose world = proc::forwardKinematics(body, motion.frames.front());
  const proc::BoneLengthReport bones = proc::measureBoneLengths(body, world, proc::kRuntimeBoneToleranceM);
  stats.expect(bones.ok, "custom retarget keeps bone lengths");

  proc::AnalysisInput mapped = gaitInput(custom);
  mapped.source = &custom.source;
  mapped.motion = &motion;
  mapped.semantics = proc::semanticsFromMapping(mapping);
  proc::ProcAnalysis analysis = proc::analyzeClip(mapped);
  stats.expect(std::abs(analysis.source_leg_length_m - custom.leg_length) < 1e-6, "analysis uses the mapped custom joints");
  stats.expect(!warningHas(analysis, "left foot was not mapped"), "mapped custom foot is not reported missing");
  int accepted = 0;
  for (const proc::GaitCycle& cycle : analysis.cycles) {
    if (!cycle.mirrored && cycle.accepted) ++accepted;
  }
  stats.expect(accepted > 0, "mapped custom locomotion still yields accepted cycles");

  const int events_before = static_cast<int>(analysis.events.size());
  proc::overrideContactRange(analysis, "left_heel", 40, 40, true);
  proc::overrideContactRange(analysis, "left_toe", 40, 40, true);
  stats.expect(static_cast<int>(analysis.events.size()) != events_before, "contact override rebuilds events");
  stats.expect(analysis.derivation_note.find("Contact edit") != std::string::npos, "contact rebuild is visible");
  stats.expect(analysis.contact_overrides.size() == 2, "both contact overrides are recorded");
  proc::ProcAnalysis repeated = analysis;
  proc::rebuildDerivedAnalysis(repeated, proc::AnalysisDirtyFrom::Contacts);
  stats.expect(proc::writeAnalysisToString(repeated) == proc::writeAnalysisToString(analysis), "a second contact rebuild is identical");

  stats.expect(!analysis.events.empty(), "events remain after the contact rebuild");
  const double automatic_time = analysis.events.front().automatic_time;
  const int moved = proc::moveEvent(analysis, 0, analysis.events.front().time + 0.08);
  stats.expect(moved >= 0, "moved event stays addressable");
  if (moved >= 0) {
    const proc::GaitEvent& event = analysis.events[static_cast<std::size_t>(moved)];
    stats.expect(event.manual_override, "moved event keeps the manual flag");
    stats.expect(std::abs(event.automatic_time - automatic_time) < 1e-9, "moved event keeps the automatic time");
    stats.expect(std::abs(event.time - automatic_time) > 0.02, "moved event time is the manual value");
    stats.expect(event.type == "initial_contact", "the moved event is still an initial contact");
    bool boundary = false;
    for (const proc::GaitCycle& cycle : analysis.cycles) {
      if (!cycle.mirrored && cycle.side == event.side && std::abs(cycle.start_time - event.time) < 1e-9) boundary = true;
    }
    stats.expect(boundary, "event move rebuilds cycle boundaries");
  }
  stats.expect(analysis.derivation_note.find("Event edit") != std::string::npos, "event rebuild is visible");

  const int left_before = rawCycles(analysis, "left");
  if (moved >= 0) {
    const int typed = proc::setEventType(analysis, moved, "toe_off");
    stats.expect(typed >= 0, "event type change sticks");
    if (typed >= 0) {
      stats.expect(analysis.events[static_cast<std::size_t>(typed)].type == "toe_off", "event type is the manual value");
      stats.expect(analysis.events[static_cast<std::size_t>(typed)].automatic_type == "initial_contact", "automatic event type is preserved");
    }
  }
  stats.expect(rawCycles(analysis, "left") < left_before, "event type change rebuilds cycles");

  int right_index = -1;
  for (int index = 0; index < static_cast<int>(analysis.cycles.size()); ++index) {
    const proc::GaitCycle& cycle = analysis.cycles[static_cast<std::size_t>(index)];
    if (!cycle.mirrored && cycle.side == "right" && cycle.accepted) {
      right_index = index;
      break;
    }
  }
  stats.expect(right_index >= 0, "a right cycle is available to reject");
  const int samples_before = walkSamples(analysis);
  const double rejected_start = right_index >= 0 ? analysis.cycles[static_cast<std::size_t>(right_index)].start_time : 0.0;
  if (right_index >= 0) proc::setCycleAccepted(analysis, right_index, false, "manual");
  stats.expect(walkSamples(analysis) == samples_before - 1, "rejecting a cycle refits relationships");
  bool stale_mirror = false;
  for (const proc::GaitCycle& cycle : analysis.cycles) {
    if (cycle.mirrored && std::abs(cycle.start_time - rejected_start) < 1e-8 && cycle.accepted) stale_mirror = true;
  }
  stats.expect(!stale_mirror, "a rejected cycle has no accepted mirror");
  stats.expect(analysis.derivation_note.find("Cycle decision") != std::string::npos, "cycle rebuild is visible");

  const std::string saved = proc::writeAnalysisToString(analysis);
  proc::ProcAnalysis loaded;
  status = proc::readAnalysisFromString(saved, loaded);
  stats.expect(status.ok(), "edited analysis reloads");
  stats.expect(loaded.contact_overrides.size() == analysis.contact_overrides.size(), "reloaded contact overrides");
  stats.expect(loaded.derivation_note == analysis.derivation_note, "reloaded derivation note");
  stats.expect(walkSamples(loaded) == walkSamples(analysis), "reloaded relationships match the cycles");
  bool manual_event = false;
  for (const proc::GaitEvent& event : loaded.events) {
    if (event.manual_override && event.type == "toe_off" && event.automatic_type == "initial_contact") manual_event = true;
  }
  stats.expect(manual_event, "reloaded event keeps automatic and manual type");
  bool manual_cycle = false;
  for (const proc::GaitCycle& cycle : loaded.cycles) {
    if (!cycle.mirrored && cycle.manual_override && !cycle.accepted && cycle.rejection_reason == "manual") manual_cycle = true;
  }
  stats.expect(manual_cycle, "reloaded cycle keeps the manual rejection");
  const proc::ContactTrack* heel = nullptr;
  for (const proc::ContactTrack& track : loaded.contacts) {
    if (track.marker == "left_heel") heel = &track;
  }
  stats.expect(heel != nullptr && heel->contact.size() > 40 && heel->automatic_contact.size() > 40, "reloaded heel track");
  if (heel != nullptr && heel->contact.size() > 40) {
    stats.expect(heel->contact[40] == 1 && heel->automatic_contact[40] == 0, "effective contact is the override and automatic contact is unchanged");
  }

  if (stats.failed == 0) std::cout << "architecture tests ok\n";
  return stats.failed;
}
