#pragma once

#include "control/control_frame.h"
#include "physics/body_state.h"

namespace pm {

class PhysicalSkeleton;

// The only stage that writes hinge torques from a control frame.
class JointServo {
 public:
  double apply(const ControlFrame& frame, const BodyState& state, PhysicalSkeleton& physics);
  double lastPeak() const { return lastPeak_; }

 private:
  double lastPeak_ = 0;
};

}  // namespace pm
