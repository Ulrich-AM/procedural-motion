#include "validation/motion_validator.h"

#include <cmath>
#include <sstream>

namespace proc {
namespace {

std::string quoteName(std::string_view text) { return "\"" + std::string(text) + "\""; }

}  // namespace

void validateProcMotion(const ProcMotion& motion, Status& status) {
  if (motion.skeleton != kSkeletonId) {
    status.error("skeleton is " + quoteName(motion.skeleton) + ", expected " + quoteName(kSkeletonId));
  }
  if (!std::isfinite(motion.fps) || motion.fps <= 0.0) {
    status.error("timing.fps must be a positive finite number");
  }
  if (!std::isfinite(motion.duration_s) || motion.duration_s <= 0.0) {
    status.error("timing.duration_s must be a positive finite number");
  }
  if (motion.frame_count != static_cast<int>(motion.frames.size()) || motion.frame_count <= 0) {
    status.error("timing.frame_count does not match the frames array");
  }
  if (std::isfinite(motion.fps) && motion.fps > 0.0 && motion.frame_count > 0 &&
      std::abs(motion.duration_s - static_cast<double>(motion.frame_count) / motion.fps) > 1e-2) {
    status.warn("timing.duration_s does not match frame_count / fps");
  }

  double previous = -1.0e300;
  for (int index = 0; index < static_cast<int>(motion.frames.size()); ++index) {
    const Pose& frame = motion.frames[index];
    if (!std::isfinite(frame.time)) {
      status.error("frame " + std::to_string(index) + " has a non-finite time");
      continue;
    }
    if (!(frame.time > previous)) {
      status.error("frame times must be strictly increasing (frame " + std::to_string(index) + ")");
    }
    previous = frame.time;
    if (!std::isfinite(frame.root_translation.x) || !std::isfinite(frame.root_translation.y) ||
        !std::isfinite(frame.root_translation.z) || !quatFinite(frame.root_rotation)) {
      status.error("frame " + std::to_string(index) + " has a non-finite root transform");
    }
    for (int joint = 0; joint < kJointCount; ++joint) {
      if (!quatFinite(frame.local_rotation[joint]) || std::abs(quatLength(frame.local_rotation[joint]) - 1.0) > 1e-4) {
        status.error("frame " + std::to_string(index) + " joint " + quoteName(kJointNames[joint]) +
                     " quaternion is not finite and normalized");
      }
    }
  }
  if (!motion.frames.empty()) {
    if (motion.frames.front().time < -1e-6) status.error("the first frame time must be >= 0");
    if (std::isfinite(motion.duration_s) && motion.frames.back().time > motion.duration_s + 1e-3) {
      status.error("a frame time is later than timing.duration_s");
    }
  }
}

}  // namespace proc
