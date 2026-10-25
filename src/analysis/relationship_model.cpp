#include "analysis/relationship_model.h"

#include <algorithm>
#include <cmath>

namespace proc {
namespace {

struct Sample {
  double speed = 0.0;
  double value = 0.0;
};

RelationshipFit fitOne(const std::string& regime, const std::string& name, const std::vector<Sample>& samples) {
  RelationshipFit fit;
  fit.name = name;
  fit.regime = regime;
  fit.sample_count = static_cast<int>(samples.size());
  if (samples.empty()) return fit;
  std::vector<double> values;
  values.reserve(samples.size());
  double speed_min = samples.front().speed;
  double speed_max = samples.front().speed;
  for (const Sample& sample : samples) {
    values.push_back(sample.value);
    speed_min = std::min(speed_min, sample.speed);
    speed_max = std::max(speed_max, sample.speed);
  }
  fit.speed_min = speed_min;
  fit.speed_max = speed_max;
  fit.median = medianOf(values);
  fit.p25 = percentile(values, 0.25);
  fit.p75 = percentile(values, 0.75);

  const double width = 0.25;
  int min_bin = static_cast<int>(std::floor(speed_min / width));
  int max_bin = static_cast<int>(std::floor(speed_max / width));
  for (int bin = min_bin; bin <= max_bin; ++bin) {
    std::vector<double> bin_values;
    for (const Sample& sample : samples) {
      const int sample_bin = static_cast<int>(std::floor(sample.speed / width));
      if (sample_bin == bin) bin_values.push_back(sample.value);
    }
    if (bin_values.empty()) continue;
    SpeedBin summary;
    summary.speed_center = (static_cast<double>(bin) + 0.5) * width;
    summary.count = static_cast<int>(bin_values.size());
    summary.median = medianOf(bin_values);
    summary.p25 = percentile(bin_values, 0.25);
    summary.p75 = percentile(bin_values, 0.75);
    fit.bins.push_back(summary);
  }

  if (samples.size() >= 2) {
    double sum_x = 0.0;
    double sum_y = 0.0;
    double sum_xx = 0.0;
    double sum_xy = 0.0;
    for (const Sample& sample : samples) {
      sum_x += sample.speed;
      sum_y += sample.value;
      sum_xx += sample.speed * sample.speed;
      sum_xy += sample.speed * sample.value;
    }
    const double n = static_cast<double>(samples.size());
    const double denominator = n * sum_xx - sum_x * sum_x;
    if (std::abs(denominator) > 1e-12) {
      fit.slope = (n * sum_xy - sum_x * sum_y) / denominator;
      fit.intercept = (sum_y - fit.slope * sum_x) / n;
      fit.has_linear_fit = true;
      double ss_tot = 0.0;
      double ss_res = 0.0;
      const double mean = sum_y / n;
      for (const Sample& sample : samples) {
        const double predicted = fit.intercept + fit.slope * sample.speed;
        ss_tot += (sample.value - mean) * (sample.value - mean);
        ss_res += (sample.value - predicted) * (sample.value - predicted);
      }
      fit.r2 = ss_tot > 1e-12 ? std::clamp(1.0 - ss_res / ss_tot, 0.0, 1.0) : 1.0;
    }
  }
  return fit;
}

}  // namespace

void fitRelationships(ProcAnalysis& analysis) {
  analysis.relationships.clear();
  const std::vector<const char*> fields = {
      "stride_length_m", "cadence_spm", "stance_fraction", "flight_fraction", "hip_amplitude_deg",
      "knee_amplitude_deg", "ankle_amplitude_deg", "pelvis_bob_m", "foot_clearance_m"};
  for (const char* regime : {"walk", "run", "sprint"}) {
    std::vector<const GaitCycle*> cycles;
    for (const GaitCycle& cycle : analysis.cycles) {
      if (cycle.mirrored || !cycle.accepted || cycle.observed_regime != regime) continue;
      cycles.push_back(&cycle);
    }
    if (cycles.empty()) continue;
    for (const char* field : fields) {
      std::vector<Sample> samples;
      const std::string name = field;
      for (const GaitCycle* cycle : cycles) {
        Sample sample;
        sample.speed = cycle->measurements.speed_mps;
        const CycleMeasurements& measured = cycle->measurements;
        if (name == "stride_length_m") sample.value = measured.stride_length_m;
        else if (name == "cadence_spm") sample.value = measured.cadence_spm;
        else if (name == "stance_fraction") sample.value = measured.stance_fraction;
        else if (name == "flight_fraction") sample.value = measured.flight_fraction;
        else if (name == "hip_amplitude_deg") sample.value = measured.hip_flex_max_deg - measured.hip_flex_min_deg;
        else if (name == "knee_amplitude_deg") sample.value = measured.knee_flex_max_deg - measured.knee_flex_min_deg;
        else if (name == "ankle_amplitude_deg") sample.value = measured.ankle_flex_max_deg - measured.ankle_flex_min_deg;
        else if (name == "pelvis_bob_m") sample.value = measured.pelvis_vertical_excursion_m;
        else sample.value = measured.foot_clearance_m;
        samples.push_back(sample);
      }
      analysis.relationships.push_back(fitOne(regime, field, samples));
    }
  }
}

}  // namespace proc
