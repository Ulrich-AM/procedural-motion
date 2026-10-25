#include "analysis/contact_detector.h"

#include <algorithm>
#include <cmath>

namespace proc {

std::vector<std::uint8_t> applyHysteresis(const std::vector<double>& confidence, double enter, double exit) {
  std::vector<std::uint8_t> state(confidence.size(), 0);
  bool held = false;
  for (std::size_t index = 0; index < confidence.size(); ++index) {
    if (!held && confidence[index] >= enter) held = true;
    else if (held && confidence[index] < exit) held = false;
    state[index] = held ? 1 : 0;
  }
  return state;
}

ContactObservation detectMarkerContact(const std::vector<double>& time, const std::vector<double>& height_above_ground,
                                       const std::vector<double>& horizontal_speed, const std::vector<double>& vertical_speed,
                                       const AnalysisSettings& settings) {
  ContactObservation observation;
  const std::size_t count = std::min(height_above_ground.size(), std::min(horizontal_speed.size(), vertical_speed.size()));
  observation.confidence.assign(count, 0.0);
  if (count == 0) return observation;
  std::vector<double> raw(count, 0.0);
  const double height_scale = std::max(settings.contact_height_m, 1e-4);
  const double speed_scale = std::max(settings.contact_speed_mps, 1e-4);
  const double vertical_scale = std::max(settings.contact_vertical_speed_mps, 1e-4);
  for (std::size_t index = 0; index < count; ++index) {
    const double height = height_above_ground[index];
    const double height_score = height <= 0.0 ? 1.0 : std::clamp(1.0 - height / height_scale, 0.0, 1.0);
    const double speed_score = std::clamp(1.0 - horizontal_speed[index] / speed_scale, 0.0, 1.0);
    const double vertical_score = std::clamp(1.0 - std::abs(vertical_speed[index]) / vertical_scale, 0.0, 1.0);
    raw[index] = height_score * speed_score * vertical_score;
  }
  for (std::size_t index = 0; index < count; ++index) {
    const double half = std::max(settings.stability_window_s, 0.0) * 0.5;
    int begin = static_cast<int>(index);
    int end = begin;
    if (!time.empty() && time.size() >= count) {
      while (begin > 0 && time[static_cast<std::size_t>(index)] - time[static_cast<std::size_t>(begin - 1)] <= half + 1e-12) --begin;
      while (end + 1 < static_cast<int>(count) &&
             time[static_cast<std::size_t>(end + 1)] - time[static_cast<std::size_t>(index)] <= half + 1e-12)
        ++end;
    }
    double sum = 0.0;
    for (int sample = begin; sample <= end; ++sample) sum += raw[static_cast<std::size_t>(sample)];
    const double stability = sum / static_cast<double>(end - begin + 1);
    observation.confidence[index] = std::clamp(raw[index] * (0.35 + 0.65 * stability), 0.0, 1.0);
  }
  observation.contact = applyHysteresis(observation.confidence, settings.contact_enter, settings.contact_exit);
  return observation;
}

}  // namespace proc
