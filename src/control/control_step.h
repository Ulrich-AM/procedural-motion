#pragma once

#include "control/foot_contact.h"
#include "control/joint_servo.h"
#include "control/locomotion_controller.h"
#include "control/recovery_step.h"
#include "control/standing_controller.h"
#include "math_types.h"

namespace pm {

class PhysicalSkeleton;

struct ControlInput {
  bool active = false;
  bool recovery = false;
  bool walking = false;
  Vec3 desired{};
  double dt = 1.0 / 120.0;
};

struct ControlOutput {
  StandingReport standing;
  RecoveryReport recovery;
  LocomotionReport locomotion;
  double peakTorque = 0;
};

// Standing fills the frame, then either walking or recovery adds swing
// objectives. The servo is the only writer of hinge torque.
ControlOutput applyControl(const ControlInput& input, StandingController& standing, RecoveryStep& recovery,
                           LocomotionController& locomotion, FootContactTracker& feet, JointServo& servo,
                           PhysicalSkeleton& physics);

}  // namespace pm
