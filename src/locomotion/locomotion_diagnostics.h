#pragma once

#include "body/proc_body.h"
#include "motion/proc_motion.h"

namespace proc {

struct FootSlipReport {
  double left_mean_mps = 0.0;
  double left_peak_mps = 0.0;
  double right_mean_mps = 0.0;
  double right_peak_mps = 0.0;
  int left_samples = 0;
  int right_samples = 0;
};

// Horizontal foot speed while the generated contact says that side is down.
// This reports slip. It does not move the foot.
FootSlipReport measureContactSlip(const ProcBody& body, const ProcMotion& motion);

}  // namespace proc
