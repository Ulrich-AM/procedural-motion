#include "skeleton/skeleton_definition.h"

#include "skeleton/canonical_bones.h"

#include <nlohmann/json.hpp>

#include <fstream>

namespace pm {
namespace {

Vec3 readVec3(const nlohmann::json& value, const char* what, std::string& error) {
  if (!value.is_array() || value.size() != 3) {
    error = std::string("Expected a 3-vector for ") + what;
    return {};
  }
  return {value.at(0).get<double>(), value.at(1).get<double>(), value.at(2).get<double>()};
}

Bone readBone(const nlohmann::json& item, std::string& error) {
  Bone bone;
  if (error.size()) {
    return bone;
  }
  bone.name = item.at("name").get<std::string>();
  bone.parent = item.value("parent", "");
  bone.source = item.value("source", "");
  bone.sourceParent = item.value("source_parent", "");
  bone.position = readVec3(item.at("position"), bone.name.c_str(), error);
  return bone;
}

}  // namespace

const Bone* SkeletonDefinition::findBone(const std::string& name) const {
  for (const Bone& bone : bones) {
    if (bone.name == name) {
      return &bone;
    }
  }
  return nullptr;
}

const Bone* SkeletonDefinition::findAny(const std::string& name) const {
  if (const Bone* bone = findBone(name)) {
    return bone;
  }
  for (const Bone& bone : auxiliary) {
    if (bone.name == name) {
      return &bone;
    }
  }
  return nullptr;
}

bool SkeletonDefinition::loadFromFile(const std::filesystem::path& path, std::string& error) {
  error.clear();
  std::ifstream input(path);
  if (!input) {
    error = "Could not open skeleton file: " + path.string();
    return false;
  }

  nlohmann::json json;
  try {
    input >> json;
    if (json.value("version", 0) != 1) {
      error = "Unsupported skeleton.json version";
      return false;
    }
    if (json.value("animation_curves_imported", true)) {
      error = "skeleton.json claims animation curves were imported; Phase 0 refuses that";
      return false;
    }

    bones.clear();
    auxiliary.clear();
    for (const auto& item : json.at("bones")) {
      bones.push_back(readBone(item, error));
      if (!error.empty()) {
        return false;
      }
    }
    if (json.contains("auxiliary")) {
      for (const auto& item : json.at("auxiliary")) {
        auxiliary.push_back(readBone(item, error));
        if (!error.empty()) {
          return false;
        }
      }
    }

    right = readVec3(json.at("axes").at("right"), "axes.right", error);
    forward = readVec3(json.at("axes").at("forward"), "axes.forward", error);
    up = readVec3(json.at("axes").at("up"), "axes.up", error);
    if (!error.empty()) {
      return false;
    }
    sourceScale = json.value("source_scale_to_meters", 1.0);
    sourceFile = json.value("source_file", "");
    ignoredAnimations = json.value("ignored_animation_count", 0);
  } catch (const std::exception& exception) {
    error = std::string("Failed to parse skeleton.json: ") + exception.what();
    return false;
  }

  for (const CanonicalBone& spec : kCanonicalBones) {
    const Bone* bone = findBone(spec.name);
    if (!bone) {
      error = std::string("skeleton.json is missing canonical bone ") + spec.name;
      return false;
    }
    if (bone->parent != spec.parent) {
      error = std::string("Canonical parent mismatch for ") + spec.name;
      return false;
    }
  }
  return true;
}

}  // namespace pm
