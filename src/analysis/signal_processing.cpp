#include "analysis/signal_processing.h"

#include <algorithm>
#include <cmath>

namespace proc {
namespace {

constexpr double kPi = 3.14159265358979323846;

double safeDt(double dt) { return std::abs(dt) < 1e-9 ? 1e-9 : dt; }

glm::dquat unitQuaternion(glm::dquat rotation) {
  const double length = quatLength(rotation);
  if (length < 1e-12) return quatIdentity();
  return glm::dquat(rotation.w / length, rotation.x / length, rotation.y / length, rotation.z / length);
}

}  // namespace

std::vector<glm::dvec3> smoothPositions(const std::vector<double>& time, const std::vector<glm::dvec3>& values, double window_s) {
  const int count = static_cast<int>(values.size());
  if (count == 0 || window_s <= 1e-6) return values;
  const double half = window_s * 0.5;
  std::vector<glm::dvec3> smoothed(static_cast<std::size_t>(count));
  for (int index = 0; index < count; ++index) {
    int begin = index;
    int end = index;
    while (begin > 0 && time[static_cast<std::size_t>(index)] - time[static_cast<std::size_t>(begin - 1)] <= half + 1e-12) --begin;
    while (end + 1 < count && time[static_cast<std::size_t>(end + 1)] - time[static_cast<std::size_t>(index)] <= half + 1e-12) ++end;
    glm::dvec3 sum(0.0);
    for (int sample = begin; sample <= end; ++sample) sum += values[static_cast<std::size_t>(sample)];
    smoothed[static_cast<std::size_t>(index)] = sum / static_cast<double>(end - begin + 1);
  }
  return smoothed;
}

std::vector<double> smoothScalars(const std::vector<double>& time, const std::vector<double>& values, double window_s) {
  std::vector<glm::dvec3> packed;
  packed.reserve(values.size());
  for (double value : values) packed.push_back(glm::dvec3(value, 0.0, 0.0));
  const std::vector<glm::dvec3> smoothed = smoothPositions(time, packed, window_s);
  std::vector<double> out;
  out.reserve(smoothed.size());
  for (const glm::dvec3& value : smoothed) out.push_back(value.x);
  return out;
}

std::vector<glm::dvec3> centralDifference(const std::vector<double>& time, const std::vector<glm::dvec3>& values) {
  const int count = static_cast<int>(values.size());
  std::vector<glm::dvec3> velocity(static_cast<std::size_t>(std::max(count, 0)));
  if (count == 0) return velocity;
  if (count == 1) return velocity;
  velocity[0] = (values[1] - values[0]) / safeDt(time[1] - time[0]);
  velocity[static_cast<std::size_t>(count - 1)] =
      (values[static_cast<std::size_t>(count - 1)] - values[static_cast<std::size_t>(count - 2)]) /
      safeDt(time[static_cast<std::size_t>(count - 1)] - time[static_cast<std::size_t>(count - 2)]);
  for (int index = 1; index + 1 < count; ++index) {
    velocity[static_cast<std::size_t>(index)] =
        (values[static_cast<std::size_t>(index + 1)] - values[static_cast<std::size_t>(index - 1)]) /
        safeDt(time[static_cast<std::size_t>(index + 1)] - time[static_cast<std::size_t>(index - 1)]);
  }
  return velocity;
}

glm::dquat hemisphereContinuous(const glm::dquat& previous, glm::dquat current) {
  current = unitQuaternion(current);
  if (glm::dot(unitQuaternion(previous), current) < 0.0) current = -current;
  return current;
}

glm::dvec3 quaternionLog(glm::dquat rotation) {
  rotation = unitQuaternion(rotation);
  const double vector_length = std::sqrt(rotation.x * rotation.x + rotation.y * rotation.y + rotation.z * rotation.z);
  if (vector_length < 1e-12) return glm::dvec3(rotation.x, rotation.y, rotation.z);
  const double half_angle = std::atan2(vector_length, rotation.w);
  return glm::dvec3(rotation.x, rotation.y, rotation.z) * (half_angle / vector_length);
}

double sagittalDegrees(const glm::dvec3& log_vector) { return log_vector.x * 2.0 * (180.0 / kPi); }

double rollDegrees(const glm::dvec3& log_vector) { return log_vector.y * 2.0 * (180.0 / kPi); }

double percentile(std::vector<double> values, double fraction) {
  if (values.empty()) return 0.0;
  std::sort(values.begin(), values.end());
  const double clamped = std::clamp(fraction, 0.0, 1.0);
  const double position = clamped * static_cast<double>(values.size() - 1);
  const std::size_t lower = static_cast<std::size_t>(std::floor(position));
  const std::size_t upper = std::min(lower + 1, values.size() - 1);
  const double weight = position - static_cast<double>(lower);
  return values[lower] * (1.0 - weight) + values[upper] * weight;
}

double medianOf(std::vector<double> values) { return percentile(std::move(values), 0.5); }

double sampleSeries(const std::vector<double>& time, const std::vector<double>& values, double time_s) {
  if (time.empty() || values.empty()) return 0.0;
  const std::size_t count = std::min(time.size(), values.size());
  if (time_s <= time.front()) return values.front();
  if (time_s >= time[count - 1]) return values[count - 1];
  std::size_t upper = 1;
  while (upper + 1 < count && time[upper] < time_s) ++upper;
  const double span = time[upper] - time[upper - 1];
  const double weight = span > 1e-12 ? (time_s - time[upper - 1]) / span : 0.0;
  return values[upper - 1] * (1.0 - weight) + values[upper] * weight;
}

}  // namespace proc
