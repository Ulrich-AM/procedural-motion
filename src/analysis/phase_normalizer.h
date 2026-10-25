#pragma once

#include "analysis/proc_analysis.h"

namespace proc {

// phase_normalizer is implemented with the cycle pass. This header exists so the
// phase step stays a named stage: normalizeCycle resamples an accepted cycle to
// settings.phase_samples (101) on phase [0, 1].
void normalizeCycle(ProcAnalysis& analysis, GaitCycle& cycle);

}  // namespace proc
