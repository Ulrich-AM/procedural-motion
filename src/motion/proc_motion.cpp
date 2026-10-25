#include "motion/proc_motion.h"

#include "body/skeleton.h"

namespace proc {

std::optional<bool> ContactSample::get(std::string_view name) const {
  if (name == "left_heel") return left_heel;
  if (name == "left_toe") return left_toe;
  if (name == "right_heel") return right_heel;
  if (name == "right_toe") return right_toe;
  return std::nullopt;
}

Pose::Pose() {
  root_rotation = quatIdentity();
  local_rotation.fill(quatIdentity());
}

}  // namespace proc
