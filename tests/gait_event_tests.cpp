#include "test_support.h"

#include "analysis/proc_analysis.h"
#include "synthetic_gait.h"

#include <algorithm>
#include <cmath>
#include <vector>

namespace {

std::vector<double> timesOf(const proc::ProcAnalysis& analysis, const std::string& side, const std::string& type) {
  std::vector<double> times;
  for (const proc::GaitEvent& event : analysis.events) {
    if (event.side == side && event.type == type) times.push_back(event.time);
  }
  return times;
}

double nearest(const std::vector<double>& times, double target) {
  double best = 1e9;
  for (double time_s : times) best = std::min(best, std::abs(time_s - target));
  return best;
}

}  // namespace

int runGaitEventTests() {
  TestStats stats;
  const proc::test::SyntheticGait walk = proc::test::makeSyntheticGait(proc::test::SyntheticKind::Walk);
  proc::AnalysisInput input;
  input.source = &walk.source;
  input.motion = &walk.motion;
  const proc::ProcAnalysis analysis = proc::analyzeClip(input);
  const std::vector<double> left_ic = timesOf(analysis, "left", "initial_contact");
  const std::vector<double> left_to = timesOf(analysis, "left", "toe_off");
  const std::vector<double> right_ic = timesOf(analysis, "right", "initial_contact");
  double worst_ic = 0.0;
  double worst_to = 0.0;
  for (double expected : walk.left_ic) worst_ic = std::max(worst_ic, nearest(left_ic, expected));
  for (double expected : walk.right_ic) worst_ic = std::max(worst_ic, nearest(right_ic, expected));
  for (double expected : walk.left_to) worst_to = std::max(worst_to, nearest(left_to, expected));
  stats.expect(worst_ic <= 0.06, "initial contact timing within 60 ms, worst " + std::to_string(worst_ic));
  stats.expect(worst_to <= 0.06, "toe-off timing within 60 ms, worst " + std::to_string(worst_to));
  double lowest = 1.0;
  for (const proc::GaitEvent& event : analysis.events) lowest = std::min(lowest, event.confidence);
  bool confident = !analysis.events.empty() && lowest > 0.4;
  stats.expect(confident, "events carry contact confidence, lowest " + std::to_string(lowest));
  if (stats.failed == 0) std::cout << "gait event tests ok\n";
  return stats.failed;
}
