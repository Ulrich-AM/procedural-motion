#pragma once

#include <glm/geometric.hpp>
#include <glm/gtc/quaternion.hpp>

#include <algorithm>
#include <cmath>

namespace proc {

inline glm::dquat quatIdentity() { return glm::dquat(1.0, 0.0, 0.0, 0.0); }

inline double quatLength(const glm::dquat& q) {
  return std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
}

inline glm::dquat quatFromXYZW(double x, double y, double z, double w) {
  return glm::dquat(w, x, y, z);
}

inline glm::dquat quatCanonical(glm::dquat q) {
  if (q.w < 0.0) {
    q = -q;
  }
  return q;
}

inline bool quatFinite(const glm::dquat& q) {
  return std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z) && std::isfinite(q.w);
}

inline glm::dquat quatNormalize(const glm::dquat& q) {
  const double length = quatLength(q);
  return quatCanonical(glm::dquat(q.w / length, q.x / length, q.y / length, q.z / length));
}

inline glm::dquat quatInverse(const glm::dquat& q) {
  const glm::dquat n = quatNormalize(q);
  return glm::dquat(n.w, -n.x, -n.y, -n.z);
}

inline glm::dquat quatAxisAngle(const glm::dvec3& axis, double radians) {
  const glm::dvec3 a = glm::normalize(axis);
  const double half = radians * 0.5;
  const double s = std::sin(half);
  return quatCanonical(glm::dquat(std::cos(half), a.x * s, a.y * s, a.z * s));
}

inline glm::dquat quatSlerp(glm::dquat a, glm::dquat b, double t) {
  a = quatNormalize(a);
  b = quatNormalize(b);
  double dot = glm::dot(a, b);
  if (dot < 0.0) {
    b = -b;
    dot = -dot;
  }
  if (dot > 0.9995) {
    return quatNormalize(a * (1.0 - t) + b * t);
  }
  const double theta = std::acos(std::clamp(dot, -1.0, 1.0));
  const double sine = std::sin(theta);
  const double w1 = std::sin((1.0 - t) * theta) / sine;
  const double w2 = std::sin(t * theta) / sine;
  return quatNormalize(a * w1 + b * w2);
}

inline double quatAngleDegrees(const glm::dquat& q) {
  const double w = std::clamp(q.w, -1.0, 1.0);
  return glm::degrees(2.0 * std::acos(w));
}

// Length-only. Does not flip the sign, so a continuous log chain can keep its hemisphere.
inline glm::dquat quatUnit(glm::dquat rotation) {
  const double length = quatLength(rotation);
  if (length < 1e-12) return quatIdentity();
  return glm::dquat(rotation.w / length, rotation.x / length, rotation.y / length, rotation.z / length);
}

inline glm::dvec3 quatLog(glm::dquat rotation) {
  rotation = quatUnit(rotation);
  const double vector_length = std::sqrt(rotation.x * rotation.x + rotation.y * rotation.y + rotation.z * rotation.z);
  if (vector_length < 1e-12) return glm::dvec3(rotation.x, rotation.y, rotation.z);
  const double half_angle = std::atan2(vector_length, rotation.w);
  return glm::dvec3(rotation.x, rotation.y, rotation.z) * (half_angle / vector_length);
}

inline glm::dquat quatExp(const glm::dvec3& log_vector) {
  const double half_angle = glm::length(log_vector);
  if (half_angle < 1e-12) return quatIdentity();
  const double scale = std::sin(half_angle) / half_angle;
  return quatUnit(glm::dquat(std::cos(half_angle), log_vector.x * scale, log_vector.y * scale, log_vector.z * scale));
}

}  // namespace proc
