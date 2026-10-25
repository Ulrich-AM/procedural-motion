#pragma once

#include "body/proc_body.h"
#include "rig/forward_kinematics.h"

#include <string>
#include <vector>

namespace proc {

struct BoneLengthSample {
  std::string bone;
  double expected_m = 0.0;
  double actual_m = 0.0;
  double error_m = 0.0;
};

struct BoneLengthReport {
  std::vector<BoneLengthSample> samples;
  double max_error_m = 0.0;
  int worst_index = -1;
  bool ok = true;
};

BoneLengthReport measureBoneLengths(const ProcBody& body, const WorldPose& world, double tolerance_m);

}  // namespace proc
