#pragma once

#include "body/skeleton.h"
#include "math/quat.h"
#include "status.h"

#include <glm/gtc/quaternion.hpp>
#include <glm/vec3.hpp>

#include <array>
#include <filesystem>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace proc {

struct ContactSample {
  bool block_present = false;
  std::optional<bool> left_heel;
  std::optional<bool> left_toe;
  std::optional<bool> right_heel;
  std::optional<bool> right_toe;

  // Missing block and an explicit false are different states.
  std::optional<bool> get(std::string_view name) const;
};

struct MotionSource {
  std::string type;
  std::optional<std::string> dataset;
  std::string clip;
};

struct Pose {
  double time = 0.0;
  glm::dvec3 root_translation{0.0, 0.0, 0.0};
  glm::dquat root_rotation{1.0, 0.0, 0.0, 0.0};
  std::array<glm::dquat, kJointCount> local_rotation{};
  ContactSample contacts;

  Pose();
};

struct ProcMotion {
  std::string skeleton;
  MotionSource source;
  double fps = 0.0;
  int frame_count = 0;
  double duration_s = 0.0;
  std::vector<Pose> frames;
  std::string source_path;
};

Status loadProcMotionFromString(std::string_view json_text, ProcMotion& out, std::string_view label = "motion");
Status loadProcMotionFromFile(const std::filesystem::path& path, ProcMotion& out);

}  // namespace proc
