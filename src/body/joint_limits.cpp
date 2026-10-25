#include "body/joint_limits.h"

#include "math/quat.h"

#include <glm/gtc/quaternion.hpp>

#include <algorithm>
#include <cmath>
#include <sstream>

namespace proc {
namespace {

std::string formatDeg(double degrees) {
  std::ostringstream stream;
  stream.setf(std::ios::fixed);
  stream.precision(1);
  stream << degrees;
  return stream.str();
}

double wrapDegrees(double degrees) {
  while (degrees > 180.0) degrees -= 360.0;
  while (degrees < -180.0) degrees += 360.0;
  return degrees;
}

void swingTwist(const glm::dquat& rotation, const glm::dvec3& axis, glm::dquat& swing, glm::dquat& twist) {
  const glm::dquat q = quatNormalize(rotation);
  const glm::dvec3 unit = glm::normalize(axis);
  const double projection = glm::dot(glm::dvec3(q.x, q.y, q.z), unit);
  const glm::dquat raw(q.w, unit.x * projection, unit.y * projection, unit.z * projection);
  const double length = quatLength(raw);
  if (length < 1e-10) {
    twist = quatIdentity();
    swing = q;
    return;
  }
  twist = glm::dquat(raw.w / length, raw.x / length, raw.y / length, raw.z / length);
  swing = glm::normalize(q * glm::conjugate(twist));
  if (swing.w < 0.0) {
    swing = -swing;
    twist = -twist;
  }
}

double twistDegrees(const glm::dquat& twist, const glm::dvec3& axis) {
  const glm::dvec3 unit = glm::normalize(axis);
  const double sine = glm::dot(glm::dvec3(twist.x, twist.y, twist.z), unit);
  return wrapDegrees(glm::degrees(2.0 * std::atan2(sine, twist.w)));
}

}  // namespace

LimitResult applyJointLimits(const ProcBody& body, const Pose& pose, LimitMode mode) {
  LimitResult result;
  result.pose = pose;
  const bool clamp = mode == LimitMode::Clamp;

  for (int index = 0; index < kJointCount; ++index) {
    if (index == kRootJoint) continue;
    const JointConstraint& constraint = body.constraints[index];
    const glm::dquat requested = quatNormalize(pose.local_rotation[index]);
    const std::string name = kJointNames[index];

    if (constraint.type == JointType::Endpoint) {
      const double degrees = quatAngleDegrees(requested);
      if (degrees > 0.5) {
        result.violations.push_back(
            {name, name + " requested " + formatDeg(degrees) + " deg\nallowed max 0.0 deg"});
        if (clamp) result.pose.local_rotation[index] = quatIdentity();
      }
      continue;
    }

    if (constraint.type == JointType::Hinge) {
      glm::dquat swing;
      glm::dquat twist;
      swingTwist(requested, constraint.axis, swing, twist);
      const double off_axis = quatAngleDegrees(swing);
      const double angle = twistDegrees(twist, constraint.axis);
      if (off_axis > 1.0) {
        result.violations.push_back(
            {name, name + " off-axis rotation " + formatDeg(off_axis) + " deg exceeds hinge tolerance 1.0 deg"});
      }
      if (angle < constraint.min_deg || angle > constraint.max_deg) {
        const double limit = angle > constraint.max_deg ? constraint.max_deg : constraint.min_deg;
        result.violations.push_back({name, name + " requested " + formatDeg(angle) + " deg\nallowed " +
                                               (angle > constraint.max_deg ? "max " : "min ") + formatDeg(limit) +
                                               " deg"});
      }
      if (clamp && !result.violations.empty() && result.violations.back().joint == name) {
        const double clamped = std::clamp(angle, constraint.min_deg, constraint.max_deg);
        result.pose.local_rotation[index] = quatAxisAngle(constraint.axis, glm::radians(clamped));
      } else if (clamp && off_axis > 1.0) {
        result.pose.local_rotation[index] = quatAxisAngle(constraint.axis, glm::radians(angle));
      }
      continue;
    }

    if (constraint.type == JointType::Ball) {
      glm::dquat swing;
      glm::dquat twist;
      swingTwist(requested, constraint.axis, swing, twist);
      const double swing_deg = quatAngleDegrees(swing);
      const double twist_deg = twistDegrees(twist, constraint.axis);
      bool violated = false;
      if (swing_deg > constraint.max_swing_deg + 1e-6) {
        violated = true;
        result.violations.push_back({name, name + " requested " + formatDeg(swing_deg) + " deg\nallowed max " +
                                                formatDeg(constraint.max_swing_deg) + " deg"});
      }
      if (std::abs(twist_deg) > constraint.max_twist_deg + 1e-6) {
        violated = true;
        result.violations.push_back({name, name + " twist requested " + formatDeg(twist_deg) + " deg\nallowed max " +
                                                formatDeg(constraint.max_twist_deg) + " deg"});
      }
      if (clamp && violated) {
        glm::dquat swung = swing;
        if (swing_deg > constraint.max_swing_deg && swing_deg > 1e-8) {
          const glm::dvec3 swing_axis(swing.x, swing.y, swing.z);
          swung = quatAxisAngle(swing_axis, glm::radians(constraint.max_swing_deg));
        }
        const double clamped_twist = std::clamp(twist_deg, -constraint.max_twist_deg, constraint.max_twist_deg);
        const glm::dquat twisted = quatAxisAngle(constraint.axis, glm::radians(clamped_twist));
        result.pose.local_rotation[index] = quatNormalize(swung * twisted);
      }
    }
  }

  result.clamped = clamp && !result.violations.empty();
  return result;
}

}  // namespace proc
