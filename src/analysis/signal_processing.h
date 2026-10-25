#pragma once

#include "math/quat.h"

#include <vector>

namespace proc {

// Symmetric box smooth. The window is a duration in seconds, so 30 fps and 60 fps
// clips with the same window_s are filtered over the same amount of time.
std::vector<glm::dvec3> smoothPositions(const std::vector<double>& time, const std::vector<glm::dvec3>& values, double window_s);
std::vector<double> smoothScalars(const std::vector<double>& time, const std::vector<double>& values, double window_s);

// Central difference. End samples use a one-sided step.
std::vector<glm::dvec3> centralDifference(const std::vector<double>& time, const std::vector<glm::dvec3>& values);

// Flip current when it falls in the opposite hemisphere from previous, then return it normalized.
glm::dquat hemisphereContinuous(const glm::dquat& previous, glm::dquat current);

// Quaternion logarithm as a 3-vector (axis * half-angle). Do not average raw xyzw components.
glm::dvec3 quaternionLog(glm::dquat rotation);

double sagittalDegrees(const glm::dvec3& log_vector);
double rollDegrees(const glm::dvec3& log_vector);

double percentile(std::vector<double> values, double fraction);
double medianOf(std::vector<double> values);

double sampleSeries(const std::vector<double>& time, const std::vector<double>& values, double time_s);

}  // namespace proc
