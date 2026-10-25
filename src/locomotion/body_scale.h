#pragma once

#include "body/proc_body.h"
#include "locomotion/locomotion_model.h"

namespace proc {

// Mean of the locked hip→knee and knee→ankle lengths. Not an animated world length.
double targetLegLength(const ProcBody& body);

double dimensionlessSpeed(double speed_mps, double leg_length_m, double gravity_mps2 = kLocomotionGravity);

}  // namespace proc
