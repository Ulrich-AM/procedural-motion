#pragma once

#include "analysis/proc_analysis.h"

namespace proc {

void measureCycle(ProcAnalysis& analysis, GaitCycle& cycle);
void normalizeCycle(ProcAnalysis& analysis, GaitCycle& cycle);
void extractGaitCycles(ProcAnalysis& analysis);
void mirrorCycles(ProcAnalysis& analysis);

}  // namespace proc
