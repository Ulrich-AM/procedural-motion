#pragma once

#include "physics/body_state.h"
#include "skeleton/skeleton_definition.h"

#include <string>
#include <unordered_map>

namespace pm {

class PhysicalSkeleton;

enum class BalanceStatus { Stable, Unstable, NoSupport };

// Diagnostics for the debug panel. The viewer does not interpret these.
struct StandingReport {
  bool active = false;
  BalanceStatus balance = BalanceStatus::NoSupport;
  double comErrorX = 0;
  double comErrorY = 0;
  double comSpeedX = 0;
  double comSpeedY = 0;
  bool leftFootContact = false;
  bool rightFootContact = false;
  bool supportValid = false;
  Vec3 supportCenter;
  double peakTorque = 0;
  double pelvisUp = 1;
};

// Torque-limited standing. Reads BodyState, writes hinge torques only.
// The free root is never commanded from here.
class StandingController {
 public:
  void capture(const SkeletonDefinition& skeleton, const BodyState& state);
  void resetFilters();
  StandingReport diagnose(const BodyState& state) const;
  StandingReport update(const BodyState& state, PhysicalSkeleton& physics);

  double peakTorque() const { return peakTorque_; }
  double referenceTarget(const std::string& jointName) const;

 private:
  struct Tune {
    double kp = 20;
    double kd = 2;
    double limit = 15;
  };

  Tune tuneFor(const std::string& jointName) const;
  double targetFor(const std::string& jointName, double currentAngle) const;

  Vec3 restRight_{1, 0, 0};
  Vec3 restForward_{0, 1, 0};
  Vec3 restUp_{0, 0, 1};
  std::unordered_map<std::string, double> targets_;
  double hipSagittal_ = 0;
  double hipLateral_ = 0;
  double lastTime_ = -1;
  double peakTorque_ = 0;
};

const char* balanceLabel(BalanceStatus status);

}  // namespace pm
