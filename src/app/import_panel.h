#pragma once

#include "body/proc_body.h"
#include "import/source_motion.h"
#include "motion/motion_player.h"
#include "retarget/retarget_profile.h"
#include "retarget/retargeter.h"
#include "status.h"

#include <array>
#include <filesystem>
#include <string>
#include <vector>

struct GLFWwindow;

namespace proc {

enum class ViewportMode { Target, Source, Overlay };

struct ImportSession {
  bool has_source = false;
  bool profiles_loaded = false;
  SourceClip clip;
  Status status;
  std::filesystem::path path;
  std::vector<RetargetProfile> profiles;
  int profile_index = -1;
  std::string detection;
  std::array<int, kJointCount> mapping{};
  std::array<bool, kJointCount> required{};
  bool rebase_horizontal = false;
  bool rebase_facing = false;
  ViewportMode view = ViewportMode::Target;
  RetargetReport report;
  int hierarchy_open = -1;

  ImportSession() {
    mapping.fill(-1);
    required.fill(false);
  }
};

struct ImportResult {
  bool replaced_motion = false;
};

ImportResult drawImportPanel(ImportSession& session, GLFWwindow* window, const ProcBody* body, bool has_body,
                             const std::filesystem::path& asset_root, ProcMotion& motion, bool& has_motion,
                             std::filesystem::path& motion_path, Status& motion_status, MotionPlayer& player);

void sourcePositionsAt(const ImportSession& session, double time_s, std::vector<glm::dvec3>& positions, std::vector<int>& parents);

void loadDroppedSource(ImportSession& session, const std::filesystem::path& path, const std::filesystem::path& asset_root,
                       const ProcBody* body, bool has_body, ProcMotion& motion, bool& has_motion,
                       std::filesystem::path& motion_path, Status& motion_status, MotionPlayer& player);

bool retargetCurrent(ImportSession& session, const ProcBody& body, ProcMotion& motion, bool& has_motion,
                     std::filesystem::path& motion_path, Status& motion_status, MotionPlayer& player);

}  // namespace proc
