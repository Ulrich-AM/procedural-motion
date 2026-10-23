#include "control/leg_ik.h"

#include <algorithm>
#include <cmath>

namespace pm {
namespace {

constexpr double kPi = 3.14159265358979323846;

double clampUnit(double value) { return std::clamp(value, -1.0, 1.0); }

}  // namespace

LegPose solveLeg(Vec3 hip, Vec3 ankleTarget, Vec3 forward, Vec3 right, Vec3 up, const LegLengths& lengths) {
  LegPose pose;
  const double thigh = std::max(0.05, lengths.thigh);
  const double shin = std::max(0.05, lengths.shin);
  Vec3 delta = ankleTarget - hip;
  double distance = length(delta);
  const double minReach = std::abs(thigh - shin) + 0.02;
  const double maxReach = thigh + shin - 0.02;
  if (distance < 1e-4) return pose;
  pose.reachable = distance <= maxReach + 1e-3 && distance >= minReach - 1e-3;
  distance = std::clamp(distance, minReach, maxReach);
  delta = normalize(ankleTarget - hip) * distance;

  const double cosKnee = clampUnit((thigh * thigh + shin * shin - distance * distance) / (2.0 * thigh * shin));
  const double bend = kPi - std::acos(cosKnee);
  pose.knee = lengths.kneeSign * bend;

  const double sag = dot(delta, forward);
  const double lat = dot(delta, right);
  const double down = std::max(0.05, -dot(delta, up));
  // Geometric flexion is positive when the ankle is forward of the hip. On this
  // skeleton the hip and knee flexion ranges are negative, so the joint target
  // uses kneeSign.
  const double cosOffset = clampUnit((thigh * thigh + distance * distance - shin * shin) / (2.0 * thigh * distance));
  const double offset = std::acos(cosOffset);
  const double flexion = std::atan2(sag, down) + offset;
  pose.hipFlex = (lengths.kneeSign < 0.0 ? -flexion : flexion) + lengths.hipFlexBias;
  // Positive abduction about the shared forward axis swings the thigh toward
  // character-left, which is the negative lateral direction.
  pose.hipAbd = -std::atan2(lat, down) + lengths.hipAbdBias;

  // Keep the sole roughly level: foot pitch cancels the sagittal fold.
  pose.anklePitch = std::clamp(-(pose.hipFlex + pose.knee), -0.65, 0.65);
  pose.ankleRoll = std::clamp(-pose.hipAbd, -0.4, 0.4);
  return pose;
}

}  // namespace pm
