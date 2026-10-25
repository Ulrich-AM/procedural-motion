#pragma once

#include "body/proc_body.h"
#include "import/source_motion.h"
#include "motion/proc_motion.h"
#include "rig/forward_kinematics.h"
#include "status.h"

#include <array>
#include <string>
#include <vector>

namespace proc {

struct RetargetOptions {
  bool rebase_horizontal_start = false;
  bool rebase_initial_facing = false;
};

struct RetargetReport {
  double rest_max_error_deg = 0.0;
  std::array<double, kJointCount> rest_error_deg{};
  int mapped = 0;
  int required = 0;
  int non_root_translations_exported = 0;
  std::vector<std::string> translation_notes;
  std::string direction_worst_bone;
  double direction_max_error_deg = 0.0;
};

Status retargetClip(const ProcBody& body, const SourceClip& clip, const std::array<int, kJointCount>& mapping,
                    const std::array<bool, kJointCount>& required, const RetargetOptions& options, ProcMotion& out,
                    RetargetReport& report);

double segmentDirectionError(const SourceClip& clip, double time_s, const ProcBody& body, const WorldPose& target,
                             const std::array<int, kJointCount>& mapping, std::string& worst_bone);

}  // namespace proc
