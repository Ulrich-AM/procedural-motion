#include "analysis/ground_estimator.h"

#include <algorithm>
#include <cmath>

namespace proc {

GroundEstimate estimateGround(const std::vector<double>& height_m, const std::vector<double>& speed_mps,
                              const AnalysisSettings& settings) {
  GroundEstimate estimate;
  if (height_m.empty()) {
    estimate.method = "empty";
    estimate.confidence = 0.0;
    return estimate;
  }
  const double low_band = percentile(height_m, std::clamp(settings.ground_low_percentile, 0.0, 0.4));
  const double p30 = percentile(height_m, 0.30);
  std::vector<double> planted;
  planted.reserve(height_m.size());
  const std::size_t count = std::min(height_m.size(), speed_mps.size());
  for (std::size_t index = 0; index < count; ++index) {
    if (speed_mps[index] <= settings.ground_speed_mps && height_m[index] <= p30 + 0.03) planted.push_back(height_m[index]);
  }
  if (planted.size() >= std::max<std::size_t>(8, height_m.size() / 20)) {
    estimate.height_m = medianOf(planted);
    estimate.method = "low_velocity_lower_envelope";
  } else {
    estimate.height_m = low_band;
    estimate.method = "low_percentile";
  }
  const double spread = percentile(height_m, 0.40) - percentile(height_m, 0.05);
  const double envelope = std::clamp(1.0 - spread / 0.08, 0.0, 1.0);
  double agreement = 1.0;
  if (!planted.empty()) {
    const double iqr = percentile(planted, 0.75) - percentile(planted, 0.25);
    agreement = std::clamp(1.0 - iqr / 0.04, 0.0, 1.0);
  } else {
    agreement = 0.35;
  }
  const double support = std::clamp(static_cast<double>(planted.size()) / static_cast<double>(height_m.size()) / 0.25, 0.0, 1.0);
  estimate.confidence = std::clamp(0.55 * envelope + 0.30 * agreement + 0.15 * support, 0.0, 1.0);
  if (spread > 0.15) estimate.confidence = std::min(estimate.confidence, 0.35);
  return estimate;
}

}  // namespace proc
