#include "control/control_step.h"

#include "physics/physical_skeleton.h"

namespace pm {

ControlOutput applyControl(const ControlInput& input, StandingController& standing, RecoveryStep& recovery,
                           LocomotionController& locomotion, FootContactTracker& feet, JointServo& servo,
                           PhysicalSkeleton& physics) {
  const BodyState state = physics.sample();
  ControlOutput output;
  if (!input.active) {
    physics.clearJointTorques();
    standing.setSwingFoot("");
    recovery.reset();
    locomotion.reset();
    feet.reset();
    output.standing = standing.diagnose(state);
    output.standing.active = false;
    return output;
  }

  feet.update(state, input.dt);
  ControlFrame frame;
  standing.setSupportShift({});
  if (input.walking) {
    const LocoBalance hint = locomotion.balanceHint(input.desired);
    standing.setSwingFoot(hint.swingFoot);
    standing.setStanceFocus(hint.stanceFocus);
    standing.setCaptureScales(hint.sagittalScale, hint.lateralScale);
    standing.setWalkPosture(hint.kneeExtra, hint.pelvisLean, hint.heightAssist);
    standing.setStanceBias(hint.stanceBias);
  } else {
    standing.setStanceFocus("");
    standing.setCaptureScales(1, 1);
    standing.setWalkPosture(0, 0, false);
    standing.setStanceBias(0);
    standing.setHoldReferences(false);
    standing.setHipBalance(true);
  }
  output.standing = standing.update(state, frame);
  if (input.walking) {
    recovery.reset();
    output.locomotion = locomotion.update(state, standing, frame, feet, recovery.metrics(), input.desired, input.dt, true);
    output.recovery.enabled = false;
    output.recovery.capture = output.locomotion.capture;
    output.recovery.captureError = output.locomotion.captureError;
  } else {
    locomotion.reset();
    output.recovery = recovery.update(state, standing, frame, feet, input.dt, input.recovery);
  }
  output.peakTorque = servo.apply(frame, state, physics);
  standing.notePeak(output.peakTorque);
  output.standing.peakTorque = output.peakTorque;
  output.standing.active = true;
  return output;
}

}  // namespace pm
