#include "test_support.h"

#include "analysis/contact_detector.h"
#include "analysis/proc_analysis.h"
#include "synthetic_gait.h"

#include <vector>

namespace {

const proc::GaitCycle* firstAccepted(const proc::ProcAnalysis& analysis, const std::string& side) {
  for (const proc::GaitCycle& cycle : analysis.cycles) {
    if (!cycle.mirrored && cycle.accepted && (side.empty() || cycle.side == side)) return &cycle;
  }
  return nullptr;
}

}  // namespace

int runContactTests() {
  TestStats stats;
  const std::vector<std::uint8_t> states = proc::applyHysteresis({0.10, 0.90, 0.50, 0.50, 0.20}, 0.60, 0.40);
  stats.expect(states.size() == 5, "hysteresis length");
  stats.expect(states[0] == 0 && states[1] == 1 && states[2] == 1 && states[3] == 1 && states[4] == 0, "hysteresis holds between thresholds");

  proc::test::SyntheticGait walk = proc::test::makeSyntheticGait(proc::test::SyntheticKind::Walk);
  proc::AnalysisInput input;
  input.source = &walk.source;
  input.motion = &walk.motion;
  input.semantics = proc::test::syntheticSemantics();
  const proc::ProcAnalysis walked = proc::analyzeClip(input);
  const proc::GaitCycle* walk_cycle = firstAccepted(walked, "left");
  stats.expect(walk_cycle != nullptr, "walk produces an accepted cycle");
  if (walk_cycle != nullptr) {
    stats.expect(walk_cycle->measurements.double_support_fraction > 0.05, "walk keeps double support");
    stats.expect(walk_cycle->measurements.flight_fraction < 0.05, "walk has no flight phase");
  }

  proc::test::SyntheticGait run = proc::test::makeSyntheticGait(proc::test::SyntheticKind::Run);
  input.source = &run.source;
  input.motion = &run.motion;
  const proc::ProcAnalysis ran = proc::analyzeClip(input);
  const proc::GaitCycle* run_cycle = firstAccepted(ran, "left");
  stats.expect(run_cycle != nullptr, "run produces an accepted cycle");
  if (run_cycle != nullptr) {
    stats.expect(run_cycle->measurements.flight_fraction > 0.10, "run keeps a flight phase");
    stats.expect(run_cycle->measurements.double_support_fraction < 0.05, "run is not forced into double support");
  }
  if (stats.failed == 0) std::cout << "contact tests ok\n";
  return stats.failed;
}
