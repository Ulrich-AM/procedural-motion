#pragma once

#include "control/control_frame.h"
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
  // Writes stance objectives. The servo applies them. The free root is never commanded.
  StandingReport update(const BodyState& state, ControlFrame& frame);

  // Empty, "left", or "right". That leg keeps its pose spring but does not
  // receive the two-foot balance term while it is the swing leg.
  void setSwingFoot(const std::string& side) { swingFoot_ = side; }
  // Added to the support center before the capture term. Walking uses this to
  // let the body fall toward the desired direction instead of pinning the COM.
  void setSupportShift(Vec3 shift) { supportShift_ = shift; }

  double peakTorque() const { return peakTorque_; }
  void notePeak(double peak) { peakTorque_ = peak; }
  double referenceTarget(const std::string& jointName) const;
  // Move one standing reference onto the pose the leg actually landed in,
  // so the pose spring does not yank the hip back to the pre-step crouch.
  void shiftReference(const std::string& jointName, double angle);
  void noteAnkle(const std::string& jointName, double angle);
  // Ease shifted joints back toward the captured stand so a landed step does not
  // stay frozen in the swing crouch.
  void relaxReferences(double dt);

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
  std::unordered_map<std::string, double> home_;
  double hipSagittal_ = 0;
  double hipLateral_ = 0;
  double lastTime_ = -1;
  double peakTorque_ = 0;
  // After the drop, the flat-foot crouch sits more flexed than the reference.
  // These holds slide onto that angle so the pose spring stops pitching backward.
  double ankleHoldLeft_ = 0.234;
  double ankleHoldRight_ = 0.234;
  std::string swingFoot_;
  Vec3 supportShift_{};
};

const char* balanceLabel(BalanceStatus status);

}  // namespace pm
