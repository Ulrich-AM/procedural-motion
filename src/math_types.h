#pragma once

#include <cmath>
#include <sstream>
#include <string>

namespace pm {

struct Vec3 {
  double x = 0;
  double y = 0;
  double z = 0;

  Vec3() = default;
  Vec3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}
};

inline Vec3 operator+(Vec3 a, Vec3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
inline Vec3 operator-(Vec3 a, Vec3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
inline Vec3 operator-(Vec3 a) { return {-a.x, -a.y, -a.z}; }
inline Vec3 operator*(Vec3 a, double s) { return {a.x * s, a.y * s, a.z * s}; }
inline Vec3 operator*(double s, Vec3 a) { return a * s; }
inline Vec3 operator/(Vec3 a, double s) { return {a.x / s, a.y / s, a.z / s}; }

inline double dot(Vec3 a, Vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }

inline Vec3 cross(Vec3 a, Vec3 b) {
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

inline double length(Vec3 a) { return std::sqrt(dot(a, a)); }

inline Vec3 normalize(Vec3 a) {
  const double len = length(a);
  if (len < 1e-12) {
    return {};
  }
  return a / len;
}

inline Vec3 rotateAround(Vec3 v, Vec3 axis, double radians) {
  axis = normalize(axis);
  const double c = std::cos(radians);
  const double s = std::sin(radians);
  return v * c + cross(axis, v) * s + axis * dot(axis, v) * (1.0 - c);
}

inline std::string formatVec(Vec3 a) {
  std::ostringstream stream;
  stream.setf(std::ios::fixed);
  stream.precision(4);
  stream << a.x << " " << a.y << " " << a.z;
  return stream.str();
}

}  // namespace pm
