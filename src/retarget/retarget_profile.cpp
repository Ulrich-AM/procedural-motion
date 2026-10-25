#include "retarget/retarget_profile.h"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cctype>
#include <fstream>
#include <sstream>

namespace proc {
namespace {

using json = nlohmann::json;

std::string lowerCopy(std::string text) {
  for (char& character : text) character = static_cast<char>(std::tolower(static_cast<unsigned char>(character)));
  return text;
}

bool sameName(const std::string& a, const std::string& b) {
  return a == b || lowerCopy(a) == lowerCopy(b);
}

}  // namespace

std::string normalizeSourceName(std::string name) {
  const auto colon = name.rfind(':');
  if (colon != std::string::npos) name = name.substr(colon + 1);
  const std::string prefix = "mixamorig";
  if (name.size() > prefix.size() && lowerCopy(name.substr(0, prefix.size())) == prefix) {
    name = name.substr(prefix.size());
  }
  return name;
}

Status loadRetargetProfileFromFile(const std::filesystem::path& path, RetargetProfile& out) {
  Status status;
  std::ifstream stream(path, std::ios::binary);
  if (!stream) {
    status.error("could not open " + path.string());
    return status;
  }
  std::ostringstream buffer;
  buffer << stream.rdbuf();
  json document;
  try {
    document = json::parse(buffer.str());
  } catch (const json::exception& error) {
    status.error(path.filename().string() + ": " + error.what());
    return status;
  }
  if (!document.is_object() || !document.contains("profile") || !document["profile"].is_string()) {
    status.error(path.filename().string() + ": profile must be a string");
    return status;
  }
  RetargetProfile profile;
  profile.name = document["profile"].get<std::string>();
  profile.path = path.string();
  if (document.contains("coordinates") && document["coordinates"].is_object()) {
    const json& coordinates = document["coordinates"];
    if (!coordinates.contains("up") || !coordinates.contains("forward") || !coordinates.contains("unit_to_meters") ||
        !coordinates["up"].is_string() || !coordinates["forward"].is_string() || !coordinates["unit_to_meters"].is_number()) {
      status.error(profile.name + ": coordinates need up, forward, and unit_to_meters");
      return status;
    }
    std::string error;
    profile.coordinates = conventionFromAxes(coordinates["up"].get<std::string>(), coordinates["forward"].get<std::string>(),
                                             coordinates["unit_to_meters"].get<double>(), error);
    if (!error.empty()) {
      status.error(profile.name + ": " + error);
      return status;
    }
    profile.has_coordinates = true;
  }
  auto readTokens = [](const json& document_json, const char* key, std::vector<std::string>& out_tokens) {
    if (!document_json.contains(key) || !document_json[key].is_array()) return;
    for (const json& token : document_json[key]) {
      if (token.is_string()) out_tokens.push_back(lowerCopy(token.get<std::string>()));
    }
  };
  readTokens(document, "detect_any", profile.detect_any);
  readTokens(document, "detect_none", profile.detect_none);
  if (document.contains("aliases") && document["aliases"].is_object()) {
    for (auto it = document["aliases"].begin(); it != document["aliases"].end(); ++it) {
      std::vector<std::string> names;
      if (it.value().is_array()) {
        for (const json& name : it.value()) {
          if (name.is_string()) names.push_back(name.get<std::string>());
        }
      }
      profile.aliases.emplace_back(it.key(), std::move(names));
    }
  }
  if (!document.contains("target_to_source") || !document["target_to_source"].is_object()) {
    status.error(profile.name + ": target_to_source is required");
    return status;
  }
  for (auto it = document["target_to_source"].begin(); it != document["target_to_source"].end(); ++it) {
    const std::optional<int> joint = findJoint(it.key());
    if (!joint || !it.value().is_string()) {
      status.error(profile.name + ": unknown PROC joint \"" + it.key() + "\"");
      return status;
    }
    profile.target_to_source[*joint] = it.value().get<std::string>();
    profile.required[*joint] = true;
  }
  out = std::move(profile);
  return status;
}

Status loadRetargetProfiles(const std::filesystem::path& directory, std::vector<RetargetProfile>& out) {
  Status status;
  out.clear();
  if (!std::filesystem::exists(directory)) return status;
  std::vector<std::filesystem::path> files;
  for (const std::filesystem::directory_entry& entry : std::filesystem::directory_iterator(directory)) {
    if (entry.path().extension() == ".json") files.push_back(entry.path());
  }
  std::sort(files.begin(), files.end());
  for (const std::filesystem::path& file : files) {
    RetargetProfile profile;
    Status loaded = loadRetargetProfileFromFile(file, profile);
    if (!loaded.ok()) {
      status.error(loaded.text());
      continue;
    }
    out.push_back(std::move(profile));
  }
  return status;
}

int findSourceBySemantic(const std::vector<std::string>& source_names, const RetargetProfile& profile, const std::string& semantic) {
  std::vector<std::string> candidates;
  for (const auto& alias : profile.aliases) {
    if (alias.first == semantic) candidates = alias.second;
  }
  candidates.push_back(semantic);
  for (const std::string& candidate : candidates) {
    const std::string normalized = normalizeSourceName(candidate);
    for (int index = 0; index < static_cast<int>(source_names.size()); ++index) {
      if (sameName(normalizeSourceName(source_names[index]), normalized) || sameName(source_names[index], candidate)) {
        return index;
      }
    }
  }
  return -1;
}

std::array<int, kJointCount> mappingFromProfile(const RetargetProfile& profile, const std::vector<std::string>& source_names) {
  std::array<int, kJointCount> mapping{};
  mapping.fill(-1);
  for (int joint = 0; joint < kJointCount; ++joint) {
    if (!profile.required[joint]) continue;
    mapping[joint] = findSourceBySemantic(source_names, profile, profile.target_to_source[joint]);
  }
  return mapping;
}

bool profileEligible(const RetargetProfile& profile, const std::vector<std::string>& source_names) {
  auto containsToken = [&](const std::string& token) {
    for (const std::string& name : source_names) {
      if (lowerCopy(name).find(token) != std::string::npos) return true;
    }
    return false;
  };
  for (const std::string& token : profile.detect_none) {
    if (containsToken(token)) return false;
  }
  if (profile.detect_any.empty()) return true;
  for (const std::string& token : profile.detect_any) {
    if (containsToken(token)) return true;
  }
  return false;
}

ProfileMatch matchProfile(const std::vector<RetargetProfile>& profiles, const std::vector<std::string>& source_names) {
  ProfileMatch best;
  best.mapping.fill(-1);
  best.confidence = "NONE";
  for (int index = 0; index < static_cast<int>(profiles.size()); ++index) {
    const RetargetProfile& profile = profiles[index];
    if (!profileEligible(profile, source_names)) continue;
    const std::array<int, kJointCount> mapping = mappingFromProfile(profile, source_names);
    int required = 0;
    int matched = 0;
    for (int joint = 0; joint < kJointCount; ++joint) {
      if (!profile.required[joint]) continue;
      ++required;
      if (mapping[joint] >= 0) ++matched;
    }
    if (required > 0 && matched == required && matched >= best.matched) {
      best.profile_index = index;
      best.matched = matched;
      best.required = required;
      best.mapping = mapping;
      best.confidence = "HIGH";
    } else if (best.profile_index < 0 && matched > best.matched) {
      best.profile_index = -1;
      best.matched = matched;
      best.required = required;
      best.confidence = "INCOMPLETE";
    }
  }
  return best;
}

Status saveMappingFile(const std::filesystem::path& path, const std::array<int, kJointCount>& mapping,
                       const std::vector<std::string>& source_names) {
  json target = json::object();
  for (int joint = 0; joint < kJointCount; ++joint) {
    if (mapping[joint] < 0 || mapping[joint] >= static_cast<int>(source_names.size())) continue;
    target[kJointNames[joint]] = source_names[mapping[joint]];
  }
  json document = {{"profile", "custom"}, {"target_to_source", target}};
  std::ofstream stream(path, std::ios::binary);
  if (!stream) {
    Status status;
    status.error("could not write " + path.string());
    return status;
  }
  stream << document.dump(2) << '\n';
  return {};
}

Status loadMappingFile(const std::filesystem::path& path, const std::vector<std::string>& source_names,
                       std::array<int, kJointCount>& mapping) {
  RetargetProfile profile;
  Status status = loadRetargetProfileFromFile(path, profile);
  if (!status.ok()) return status;
  mapping = mappingFromProfile(profile, source_names);
  return status;
}

}  // namespace proc
