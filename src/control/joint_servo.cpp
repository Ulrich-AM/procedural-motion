#include "control/joint_servo.h"

#include "physics/physical_skeleton.h"

#include <algorithm>
#include <cmath>

namespace pm {

double JointServo::apply(const ControlFrame& frame, const BodyState& state, PhysicalSkeleton& physics) {
  physics.clearJointTorques();
  lastPeak_ = 0;
  for (const JointState& joint : state.joints) {
    if (joint.type != "hinge") continue;
    const JointCommand command = frame.resolve(joint.name);
    if (!command.used) continue;
    double torque = command.kp * (command.target - joint.angle) - command.kd * joint.velocity + command.feedforward;
    if (!std::isfinite(torque)) torque = 0;
    const double limit = std::max(0.0, command.limit);
    torque = std::clamp(torque, -limit, limit);
    if (!physics.setJointTorque(joint.name, torque, limit)) torque = 0;
    lastPeak_ = std::max(lastPeak_, std::abs(torque));
  }
  return lastPeak_;
}

}  // namespace pm
