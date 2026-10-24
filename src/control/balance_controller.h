#pragma once

#include "control/control_frame.h"
#include "control/foot_contact.h"
#include "control/leg_ik.h"
#include "math_types.h"
#include "physics/body_state.h"
#include "skeleton/skeleton_definition.h"

#include <string>

namespace pm {

class PhysicalSkeleton;

// Task-space balance. Virtual forces become stance-leg hinge torques.
// The free root is never written.
struct BalanceReport {
  Vec3 desiredCom{};
  Vec3 force{};
  Vec3 cop{};
  bool copValid = false;
  double pelvisFootOffset = 0;
  bool supportValid = false;
};

class BalanceController {
 public:
  void capture(const SkeletonDefinition& skeleton, const BodyState& state);
  // focus is "", "left", or "right". exclude drops that leg from the force map.
  // holdUp also raises that foot a few centimeters and excludes it.
  BalanceReport apply(const BodyState& state, ControlFrame& frame, const FootContactTracker& feet,
                      PhysicalSkeleton& physics, const std::string& focus, const std::string& exclude,
                      const std::string& holdUp) const;

 private:
  struct Side {
    LegLengths lengths;
    bool ready = false;
  };

  Vec3 restForward{0, -1, 0};
  Vec3 restRight{-1, 0, 0};
  Vec3 restUp{0, 0, 1};
  Side leftLeg;
  Side rightLeg;
  double neutralComZ_ = 0.9;
  bool captured_ = false;
};
}  // namespace pm
