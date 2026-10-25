#pragma once

#include "motion/proc_motion.h"

namespace proc {

Pose interpolatePoses(const Pose& a, const Pose& b, double u);
Pose sampleMotion(const ProcMotion& motion, double time_s, bool loop);

}  // namespace proc
