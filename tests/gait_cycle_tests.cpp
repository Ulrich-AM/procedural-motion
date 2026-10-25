#include "test_support.h"

#include "analysis/proc_analysis.h"
#include "synthetic_gait.h"

#include <cmath>

namespace {

const proc::GaitCycle* firstAccepted(const proc::ProcAnalysis& analysis, const std::string& side) {
  for (const proc::GaitCycle& cycle : analysis.cycles) {
    if (!cycle.mirrored && cycle.accepted && cycle.side == side) return &cycle;
  }
  return nullptr;
}

int acceptedCount(const proc::ProcAnalysis& analysis) {
  int count = 0;
  for (const proc::GaitCycle& cycle : analysis.cycles) {
    if (!cycle.mirrored && cycle.accepted) ++count;
  }
  return count;
}

}  // namespace

int runGaitCycleTests() {
  TestStats stats;
  proc::test::SyntheticGait walk = proc::test::makeSyntheticGait(proc::test::SyntheticKind::Walk);
  proc::AnalysisInput input;
  input.source = &walk.source;
  input.motion = &walk.motion;
  input.semantics = proc::test::syntheticSemantics();
  input.theme = "walk";
  const proc::ProcAnalysis walked = proc::analyzeClip(input);
  const proc::GaitCycle* cycle = firstAccepted(walked, "left");
  stats.expect(cycle != nullptr, "synthetic walk cycle accepted");
  if (cycle != nullptr) {
    stats.expect(std::abs(cycle->measurements.stride_time - walk.period) <= 0.06, "stride time");
    stats.expect(std::abs(cycle->measurements.speed_mps - walk.speed) <= 0.15, "speed");
    stats.expect(std::abs(cycle->measurements.stride_length_m - walk.speed * walk.period) <= 0.15, "stride length");
    stats.expect(std::abs(cycle->measurements.cadence_spm - 120.0) <= 15.0, "cadence");
    stats.expect(static_cast<int>(cycle->curves.knee_flex_deg.size()) == 101, "phase grid is 101 samples");
    stats.expect(!cycle->curves.foot_height_m.empty() && cycle->curves.foot_height_m.front() < 0.03, "phase 0 is near contact");
    double peak = 0.0;
    for (double height : cycle->curves.foot_height_m) peak = std::max(peak, height);
    stats.expect(peak > 0.04, "swing foot clears the ground");
    stats.expect(cycle->measurements.normalized_stride_length > 0.5, "stride is normalized by source leg length");
  }
  bool mirrored = false;
  for (const proc::GaitCycle& candidate : walked.cycles) {
    if (candidate.mirrored && candidate.side == "right" && candidate.canonical_side == "left") mirrored = true;
  }
  stats.expect(mirrored, "right cycles produce a canonical mirrored copy");

  proc::test::SyntheticGait run = proc::test::makeSyntheticGait(proc::test::SyntheticKind::Run);
  input.source = &run.source;
  input.motion = &run.motion;
  const proc::ProcAnalysis ran = proc::analyzeClip(input);
  const proc::GaitCycle* run_cycle = firstAccepted(ran, "left");
  stats.expect(run_cycle != nullptr && run_cycle->measurements.flight_fraction > 0.10, "synthetic run flight");
  stats.expect(run_cycle != nullptr && run_cycle->observed_regime == "run", "run regime from observed flight");

  proc::test::SyntheticGait dance = proc::test::makeSyntheticGait(proc::test::SyntheticKind::NonGait);
  input.source = &dance.source;
  input.motion = &dance.motion;
  input.theme = "dance";
  const proc::ProcAnalysis irregular = proc::analyzeClip(input);
  stats.expect(acceptedCount(irregular) == 0, "non-gait does not become clean locomotion");

  proc::test::SyntheticGait turning = proc::test::makeSyntheticGait(proc::test::SyntheticKind::Turning);
  input.source = &turning.source;
  input.motion = &turning.motion;
  const proc::ProcAnalysis turned = proc::analyzeClip(input);
  bool kept = false;
  for (const proc::GaitCycle& candidate : turned.cycles) {
    if (!candidate.mirrored && candidate.rejection_reason == "turning") kept = true;
  }
  stats.expect(kept, "turning cycles are kept with an explicit reason");
  if (stats.failed == 0) std::cout << "gait cycle tests ok\n";
  return stats.failed;
}
