#include "motion/proc_motion.h"

#include <nlohmann/json.hpp>

#include <cmath>
#include <fstream>

namespace proc {
namespace {

nlohmann::json vec3Json(const glm::dvec3& value) {
  return {value.x, value.y, value.z};
}

nlohmann::json quatJson(const glm::dquat& value) {
  const glm::dquat normalized = quatNormalize(value);
  return {normalized.x, normalized.y, normalized.z, normalized.w};
}

}  // namespace

std::string writeProcMotionToString(const ProcMotion& motion) {
  nlohmann::json frames = nlohmann::json::array();
  for (const Pose& pose : motion.frames) {
    nlohmann::json joints = nlohmann::json::object();
    for (int index = 0; index < kJointCount; ++index) {
      if (index == kRootJoint) continue;
      joints[kJointNames[index]] = quatJson(pose.local_rotation[index]);
    }
    nlohmann::json frame = {{"time", pose.time},
                            {"root", {{"translation", vec3Json(pose.root_translation)}, {"rotation", quatJson(pose.root_rotation)}}},
                            {"joints", joints}};
    frames.push_back(std::move(frame));
  }
  nlohmann::json document = {
      {"format", "PROC_MOTION"},
      {"version", 1},
      {"skeleton", motion.skeleton.empty() ? kSkeletonId : motion.skeleton},
      {"source", {{"type", motion.source.type},
                  {"dataset", motion.source.dataset ? nlohmann::json(*motion.source.dataset) : nlohmann::json(nullptr)},
                  {"clip", motion.source.clip}}},
      {"timing", {{"fps", motion.fps}, {"frame_count", motion.frame_count}, {"duration_s", motion.duration_s}}},
      {"frames", frames}};
  return document.dump(2) + "\n";
}

Status writeProcMotionToFile(const ProcMotion& motion, const std::filesystem::path& path) {
  std::ofstream stream(path, std::ios::binary);
  if (!stream) {
    Status status;
    status.error("could not write " + path.string());
    return status;
  }
  stream << writeProcMotionToString(motion);
  return {};
}

}  // namespace proc
