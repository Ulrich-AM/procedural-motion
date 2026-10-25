#pragma once

#include "body/joint_limits.h"
#include "body/proc_body.h"
#include "locomotion/locomotion_audit.h"
#include "locomotion/locomotion_generator.h"
#include "rig/forward_kinematics.h"
#include "status.h"

#include <filesystem>
#include <string>

struct GLFWwindow;

namespace proc {

struct GeneratorUi {
  bool enabled = false;
  ProcLocomotionModel model;
  bool has_model = false;
  std::filesystem::path path;
  Status status;
  LocomotionState state;
  bool playing = false;
  int force = 0;
  int inspect = 0;
  float desired_speed = 1.2f;
  float acceleration = 2.5f;
  float heading_deg = 0.0f;
  float bake_seconds = 10.0f;
  double left_mean_mps = 0.0;
  double left_peak_mps = 0.0;
  double right_mean_mps = 0.0;
  double right_peak_mps = 0.0;
  int left_samples = 0;
  int right_samples = 0;
  glm::dvec3 previous_left{0.0};
  glm::dvec3 previous_right{0.0};
  bool has_previous = false;
  std::string bake_status;
  ConstraintAudit audit;
  bool audit_valid = false;
  std::string audited_body;
};

void loadLocomotionModel(GeneratorUi& ui, const std::filesystem::path& path);
void drawGeneratorPanel(GeneratorUi& ui, GLFWwindow* window, const ProcBody* body, bool has_body,
                        const std::filesystem::path& asset_root, const std::filesystem::path& body_path, LimitMode* limit_mode,
                        const LimitResult* limits);
void accumulateGeneratorSlip(GeneratorUi& ui, const WorldPose& world, const Pose& pose, double dt);

}  // namespace proc
