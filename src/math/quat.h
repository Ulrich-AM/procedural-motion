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

}  // namespace proc
