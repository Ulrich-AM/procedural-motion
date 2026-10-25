#pragma once

#include "motion/proc_motion.h"
#include "status.h"

namespace proc {

void validateProcMotion(const ProcMotion& motion, Status& status);

}  // namespace proc
