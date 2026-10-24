#pragma once

#include "control/control_step.h"
#include "physics/physical_skeleton.h"
#include "skeleton/skeleton_definition.h"

namespace pm {

int runProceduralSelfTest(PhysicalSkeleton& physics, StandingController& standing, RecoveryStep& recovery,
                          LocomotionController& locomotion, FootContactTracker& feet, JointServo& servo,
                          const SkeletonDefinition& skeleton);

}  // namespace pm
