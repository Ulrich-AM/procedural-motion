#pragma once

#include "body/skeleton.h"
#include "status.h"

#include <glm/vec3.hpp>

#include <array>
#include <filesystem>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace proc {

enum class JointType { Root, Hinge, Ball, Endpoint };

struct Anthropometry {
  std::string description;
  std::vector<std::pair<std::string, double>> measurements;

  const double* find(std::string_view key) const;
};

struct JointGeom {
  std::string name;
  glm::dvec3 rest_position{0.0, 0.0, 0.0};
  int parent = -1;
  int incoming_bone = -1;
  std::vector<int> children;
};

struct BoneGeom {
  std::string name;
  int parent_joint = -1;
  int child_joint = -1;
  double length_m = 0.0;
  glm::dvec3 rest_direction{0.0, 0.0, 0.0};
  glm::dvec3 rest_vector{0.0, 0.0, 0.0};
  bool locked = false;
};

struct JointConstraint {
  JointType type = JointType::Ball;
  glm::dvec3 axis{0.0, 0.0, 1.0};
  double min_deg = 0.0;
  double max_deg = 0.0;
  double max_swing_deg = 0.0;
  double max_twist_deg = 0.0;
  std::string authority;
  std::string note;
};

struct ProcBody {
  std::string name;
  std::string skeleton;
  Anthropometry anthropometry;
  std::array<JointGeom, kJointCount> joints;
  std::array<BoneGeom, kBoneCount> bones;
  std::array<JointConstraint, kJointCount> constraints;
  std::string source_path;
};

Status loadProcBodyFromString(std::string_view json_text, ProcBody& out, std::string_view label = "body");
Status loadProcBodyFromFile(const std::filesystem::path& path, ProcBody& out);

const char* jointTypeName(JointType type);

}  // namespace proc
