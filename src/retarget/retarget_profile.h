#pragma once

#include "body/skeleton.h"
#include "import/coordinate_normalizer.h"
#include "status.h"

#include <array>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace proc {

struct RetargetProfile {
  std::string name;
  std::string path;
  bool has_coordinates = false;
  AxisConvention coordinates = identityConvention();
  std::vector<std::pair<std::string, std::vector<std::string>>> aliases;
  std::array<std::string, kJointCount> target_to_source{};
  std::array<bool, kJointCount> required{};
  std::vector<std::string> detect_any;
  std::vector<std::string> detect_none;
};

struct ProfileMatch {
  int profile_index = -1;
  int matched = 0;
  int required = 0;
  std::string confidence;
  std::array<int, kJointCount> mapping{};
};

std::string normalizeSourceName(std::string name);
Status loadRetargetProfileFromFile(const std::filesystem::path& path, RetargetProfile& out);
Status loadRetargetProfiles(const std::filesystem::path& directory, std::vector<RetargetProfile>& out);

int findSourceBySemantic(const std::vector<std::string>& source_names, const RetargetProfile& profile, const std::string& semantic);
ProfileMatch matchProfile(const std::vector<RetargetProfile>& profiles, const std::vector<std::string>& source_names);
std::array<int, kJointCount> mappingFromProfile(const RetargetProfile& profile, const std::vector<std::string>& source_names);

Status saveMappingFile(const std::filesystem::path& path, const std::array<int, kJointCount>& mapping,
                       const std::vector<std::string>& source_names);
Status loadMappingFile(const std::filesystem::path& path, const std::vector<std::string>& source_names,
                       std::array<int, kJointCount>& mapping);

}  // namespace proc
