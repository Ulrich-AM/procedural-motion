#include "test_support.h"

#include "analysis/contact_detector.h"
#include "analysis/ground_estimator.h"
#include "analysis/proc_analysis.h"
#include "analysis/signal_processing.h"
#include "synthetic_gait.h"

#include <cmath>
#include <vector>

namespace {

double absDiff(double a, double b) { return std::abs(a - b); }

}  // namespace

int runAnalysisSignalTests() {
  TestStats stats;
  const std::vector<double> time = {0.0, 0.1, 0.2, 0.3, 0.4};
  std::vector<glm::dvec3> ramp;
  for (double sample : time) ramp.push_back(glm::dvec3(0.0, 1.2 * sample, 0.0));
  const std::vector<glm::dvec3> smoothed = proc::smoothPositions(time, ramp, 0.05);
  stats.expect(absDiff(smoothed[2].y, 1.2 * 0.2) < 1e-9, "smoothing preserves a linear ramp");
  const std::vector<glm::dvec3> velocity = proc::centralDifference(time, ramp);
  stats.expect(absDiff(velocity[2].y, 1.2) < 1e-9, "central difference of a 1.2 m/s ramp");

  const glm::dquat turned = proc::quatAxisAngle(glm::dvec3(0.0, 1.0, 0.0), 0.4);
  const glm::dquat flipped = -turned;
  const glm::dquat aligned = proc::hemisphereContinuous(turned, flipped);
  const glm::dvec3 log_a = proc::quaternionLog(turned);
  const glm::dvec3 log_b = proc::quaternionLog(aligned);
  stats.expect(glm::length(log_a - log_b) < 1e-9, "hemisphere alignment removes the quaternion sign flip");
  stats.expect(glm::length(proc::quaternionLog(turned) - proc::quaternionLog(flipped)) > 1e-3, "raw logs of q and -q are not the same");

  std::vector<double> flat(80, 0.0);
  std::vector<double> flat_speed(80, 0.0);
  for (int index = 0; index < 20; ++index) {
    flat.push_back(0.12);
    flat_speed.push_back(1.5);
  }
  const proc::GroundEstimate ground = proc::estimateGround(flat, flat_speed, {});
  stats.expect(absDiff(ground.height_m, 0.0) < 0.01, "flat-ground height");
  stats.expect(ground.confidence > 0.7, "flat-ground confidence");
  std::vector<double> stairs;
  std::vector<double> stair_speed;
  for (int index = 0; index < 80; ++index) {
    stairs.push_back(0.01 * index);
    stair_speed.push_back(0.05);
  }
  const proc::GroundEstimate uneven = proc::estimateGround(stairs, stair_speed, {});
  stats.expect(uneven.confidence < 0.45, "uneven envelope is low confidence");

  const proc::test::SyntheticGait walk = proc::test::makeSyntheticGait(proc::test::SyntheticKind::Walk);
  proc::AnalysisInput input;
  input.source = &walk.source;
  input.motion = &walk.motion;
  input.semantics = proc::test::syntheticSemantics();
  const proc::ProcAnalysis analysis = proc::analyzeClip(input);
  stats.expect(absDiff(analysis.source_leg_length_m, walk.leg_length) < 1e-6, "source leg length is hip-knee plus knee-ankle");
  stats.expect(analysis.heel_provenance == "inferred_virtual", "heel provenance is virtual");
  if (stats.failed == 0) std::cout << "analysis signal tests ok\n";
  return stats.failed;
}
