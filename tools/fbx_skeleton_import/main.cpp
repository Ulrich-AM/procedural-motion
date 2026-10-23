// Reads a Mixamo/Y-Bot FBX and writes rest-pose skeleton.json.
// Mesh, materials, and animation curves are inspected only so they can be ignored.

#include "math_types.h"
#include "skeleton/canonical_bones.h"

#include <assimp/Importer.hpp>
#include <assimp/config.h>
#include <assimp/postprocess.h>
#include <assimp/scene.h>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <set>
#include <string>
#include <vector>

namespace {
using pm::Vec3;
using pm::cross;
using pm::dot;
using pm::formatVec;
using pm::length;
using pm::normalize;

struct SourceNode {
  std::string name;
  std::string key;
  int parent = -1;
  int depth = 0;
  Vec3 translation;
  unsigned meshCount = 0;
};

std::string boneKey(std::string name) {
  const auto slash = name.find_last_of("/\\");
  if (slash != std::string::npos) name = name.substr(slash + 1);
  const auto pivot = name.find("_$");
  if (pivot != std::string::npos) name = name.substr(0, pivot);
  const auto colon = name.rfind(':');
  if (colon != std::string::npos) name = name.substr(colon + 1);
  const std::string prefix = "mixamorig_";
  if (name.rfind(prefix, 0) == 0) name = name.substr(prefix.size());
  return name;
}

std::string lowerCopy(std::string value) {
  for (char& character : value) character = static_cast<char>(std::tolower(static_cast<unsigned char>(character)));
  return value;
}

void walk(const aiNode* node, int parent, int depth, const aiMatrix4x4& parentGlobal, bool parentTimesLocal,
          std::vector<SourceNode>& nodes) {
  const aiMatrix4x4 global = parentTimesLocal ? (parentGlobal * node->mTransformation) : (node->mTransformation * parentGlobal);
  SourceNode item;
  item.name = node->mName.C_Str();
  item.key = boneKey(item.name);
  item.parent = parent;
  item.depth = depth;
  item.translation = {global.a4, global.b4, global.c4};
  item.meshCount = node->mNumMeshes;
  const int index = static_cast<int>(nodes.size());
  nodes.push_back(item);
  for (unsigned child = 0; child < node->mNumChildren; ++child) {
    walk(node->mChildren[child], index, depth + 1, global, parentTimesLocal, nodes);
  }
}

std::map<std::string, int> indexKeys(const std::vector<SourceNode>& nodes) {
  std::map<std::string, int> exact;
  std::map<std::string, int> folded;
  for (int i = 0; i < static_cast<int>(nodes.size()); ++i) {
    if (nodes[i].key.empty()) continue;
    const auto found = exact.find(nodes[i].key);
    if (found == exact.end() || nodes[i].depth > nodes[found->second].depth) exact[nodes[i].key] = i;
    const std::string foldedKey = lowerCopy(nodes[i].key);
    const auto foldedFound = folded.find(foldedKey);
    if (foldedFound == folded.end() || nodes[i].depth > nodes[foldedFound->second].depth) folded[foldedKey] = i;
  }
  for (const auto& [key, index] : folded) {
    if (!exact.count(key)) {
      bool already = false;
      for (const auto& [existing, existingIndex] : exact) {
        if (lowerCopy(existing) == key) {
          already = true;
          break;
        }
      }
      if (!already) exact[nodes[index].key] = index;
    }
  }
  return exact;
}

int findKey(const std::map<std::string, int>& keys, const std::string& wanted) {
  const auto exact = keys.find(wanted);
  if (exact != keys.end()) return exact->second;
  const std::string folded = lowerCopy(wanted);
  for (const auto& [key, index] : keys) {
    if (lowerCopy(key) == folded) return index;
  }
  return -1;
}

int findFirst(const std::map<std::string, int>& keys, std::initializer_list<const char*> names) {
  for (const char* name : names) {
    const int index = findKey(keys, name);
    if (index >= 0) return index;
  }
  return -1;
}

Vec3 convertPoint(Vec3 point, int mode) {
  switch (mode) {
    case 0:
      return {point.x, -point.z, point.y};
    case 1:
      return {point.x, point.z, point.y};
    case 2:
      return point;
    default:
      return {point.x, point.y, -point.z};
  }
}

double guessScale(double span) {
  if (span > 0.9 && span < 3.2) return 1.0;
  if (span > 9.0 && span < 32.0) return 0.1;
  if (span > 90.0 && span < 320.0) return 0.01;
  if (span > 900.0 && span < 3200.0) return 0.001;
  if (span > 1e-6) return 1.7 / span;
  return 1.0;
}

void printHierarchy(const std::vector<SourceNode>& nodes, std::ostream& output) {
  std::vector<std::vector<int>> children(nodes.size());
  std::vector<int> roots;
  for (int i = 0; i < static_cast<int>(nodes.size()); ++i) {
    if (nodes[i].parent >= 0) children[nodes[i].parent].push_back(i);
    else roots.push_back(i);
  }
  auto walkPrint = [&](auto&& self, int index) -> void {
    const SourceNode& node = nodes[index];
    output << std::string(static_cast<size_t>(node.depth) * 2, ' ') << node.name << "  [" << node.key << "]  t=("
           << node.translation.x << ", " << node.translation.y << ", " << node.translation.z << ")";
    if (node.meshCount) output << "  meshes=" << node.meshCount << " (ignored)";
    output << "\n";
    for (int child : children[index]) self(self, child);
  };
  for (int root : roots) walkPrint(walkPrint, root);
}

bool underHips(const std::vector<SourceNode>& nodes, int index) {
  while (index >= 0) {
    if (nodes[index].key == "Hips") return true;
    index = nodes[index].parent;
  }
  return false;
}

struct Loaded {
  const aiScene* scene = nullptr;
  std::vector<SourceNode> nodes;
};

bool loadScene(Assimp::Importer& importer, const std::string& path, bool preservePivots, bool parentTimesLocal,
               Loaded& loaded, std::string& error) {
  importer.SetPropertyBool(AI_CONFIG_IMPORT_FBX_PRESERVE_PIVOTS, preservePivots);
  importer.SetPropertyBool(AI_CONFIG_IMPORT_FBX_READ_ALL_MATERIALS, false);
  try {
    loaded.scene = importer.ReadFile(path, 0);
  } catch (const std::exception& exception) {
    error = exception.what();
    return false;
  }
  if (!loaded.scene || !loaded.scene->mRootNode) {
    error = importer.GetErrorString();
    return false;
  }
  loaded.nodes.clear();
  walk(loaded.scene->mRootNode, -1, 0, aiMatrix4x4(), parentTimesLocal, loaded.nodes);
  return true;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 3) {
    std::cerr << "Usage: fbx_skeleton_import <input.fbx> <output.json>\n";
    return 2;
  }
  const std::filesystem::path input = argv[1];
  const std::filesystem::path output = argv[2];
  if (!std::filesystem::exists(input)) {
    std::cerr << "FBX not found: " << input.string() << "\n";
    return 1;
  }

  Assimp::Importer importer;
  Loaded loaded;
  std::string error;
  bool preservePivots = false;
  bool parentTimesLocal = true;
  if (!loadScene(importer, input.string(), preservePivots, parentTimesLocal, loaded, error)) {
    std::cerr << "FBX import failed: " << error << "\n";
    return 1;
  }

  auto keys = indexKeys(loaded.nodes);
  auto missingRequired = [&]() {
    std::vector<std::string> missing;
    for (const pm::CanonicalBone& bone : pm::kCanonicalBones) {
      if (findKey(keys, bone.sourceKey) < 0) missing.emplace_back(bone.sourceKey);
    }
    return missing;
  };

  auto hipsToHead = [&]() {
    const int hips = findKey(keys, "Hips");
    const int head = findKey(keys, "Head");
    if (hips < 0 || head < 0) return 0.0;
    return length(loaded.nodes[head].translation - loaded.nodes[hips].translation);
  };

  if (!missingRequired().empty() || hipsToHead() < 1e-3) {
    parentTimesLocal = false;
    if (!loadScene(importer, input.string(), preservePivots, parentTimesLocal, loaded, error)) {
      std::cerr << "FBX import failed: " << error << "\n";
      return 1;
    }
    keys = indexKeys(loaded.nodes);
  }
  if (!missingRequired().empty()) {
    preservePivots = true;
    parentTimesLocal = true;
    if (!loadScene(importer, input.string(), preservePivots, parentTimesLocal, loaded, error)) {
      std::cerr << "FBX import failed: " << error << "\n";
      return 1;
    }
    keys = indexKeys(loaded.nodes);
  }

  const auto stillMissing = missingRequired();
  std::cout << "FBX: " << input.string() << "\n";
  std::cout << "Nodes: " << loaded.nodes.size() << "\n";
  std::cout << "Meshes in file: " << loaded.scene->mNumMeshes << " (ignored, not rendered)\n";
  std::cout << "Animation clips in file: " << loaded.scene->mNumAnimations
            << " (ignored, curves are not imported)\n";
  for (unsigned i = 0; i < loaded.scene->mNumAnimations; ++i) {
    const aiAnimation* animation = loaded.scene->mAnimations[i];
    std::cout << "  (ignored) " << (animation->mName.length ? animation->mName.C_Str() : "(unnamed)") << "\n";
  }
  std::cout << "Preserve pivots: " << (preservePivots ? "yes" : "no")
            << ", transform order: " << (parentTimesLocal ? "parent*local" : "local*parent") << "\n";
  std::cout << "Source hierarchy:\n";
  printHierarchy(loaded.nodes, std::cout);

  if (!stillMissing.empty()) {
    std::cerr << "Missing required Mixamo bones:\n";
    for (const std::string& name : stillMissing) std::cerr << "  " << name << "\n";
    std::cerr << "Available keys:\n";
    for (const auto& [key, index] : keys) std::cerr << "  " << key << "\n";
    return 1;
  }

  struct Attempt {
    int mode = 0;
    double scale = 1;
    double height = 0;
    std::map<std::string, Vec3> position;
  };
  Attempt best;
  bool found = false;
  for (int mode = 0; mode < 4; ++mode) {
    std::map<std::string, Vec3> raw;
    for (const pm::CanonicalBone& bone : pm::kCanonicalBones) {
      const int index = findKey(keys, bone.sourceKey);
      raw[bone.name] = convertPoint(loaded.nodes[index].translation, mode);
    }
    double minZ = 1e9;
    double maxZ = -1e9;
    for (const auto& [name, position] : raw) {
      minZ = std::min(minZ, position.z);
      maxZ = std::max(maxZ, position.z);
    }
    const double scale = guessScale(maxZ - minZ);
    for (auto& [name, position] : raw) position = position * scale;
    const double height = raw["head"].z - std::min(raw["left_foot"].z, raw["right_foot"].z);
    const double shoulders = length(raw["left_upper_arm"] - raw["right_upper_arm"]);
    if (height < 1.15 || height > 2.45) continue;
    if (raw["head"].z <= raw["pelvis"].z) continue;
    if (shoulders < 0.15) continue;
    if (!found || std::abs(height - 1.75) < std::abs(best.height - 1.75)) {
      best.mode = mode;
      best.scale = scale;
      best.height = height;
      best.position = raw;
      found = true;
    }
  }
  if (!found) {
    std::cerr << "Could not find a Z-up meter conversion for the Y-Bot skeleton.\n";
    return 1;
  }

  double minZ = 1e9;
  for (const auto& [name, position] : best.position) minZ = std::min(minZ, position.z);
  const Vec3 pelvis = best.position["pelvis"];
  for (auto& [name, position] : best.position) {
    position.x -= pelvis.x;
    position.y -= pelvis.y;
    position.z -= minZ;
  }

  Vec3 right = best.position["right_upper_arm"] - best.position["left_upper_arm"];
  right.z = 0;
  right = normalize(right);
  Vec3 up{0, 0, 1};
  Vec3 forward = normalize(cross(up, right));
  right = normalize(cross(forward, up));

  auto converted = [&](int index) {
    Vec3 point = convertPoint(loaded.nodes[index].translation, best.mode) * best.scale;
    const Vec3 rawPelvis = convertPoint(loaded.nodes[findKey(keys, "Hips")].translation, best.mode) * best.scale;
    double rawMinZ = 1e9;
    for (const pm::CanonicalBone& bone : pm::kCanonicalBones) {
      const Vec3 sample = convertPoint(loaded.nodes[findKey(keys, bone.sourceKey)].translation, best.mode) * best.scale;
      rawMinZ = std::min(rawMinZ, sample.z);
    }
    point.x -= rawPelvis.x;
    point.y -= rawPelvis.y;
    point.z -= rawMinZ;
    return point;
  };

  const int leftToe = findFirst(keys, {"LeftToe_End", "LeftToeBase"});
  const int rightToe = findFirst(keys, {"RightToe_End", "RightToeBase"});
  if (leftToe >= 0) {
    Vec3 toeDir = converted(leftToe) - best.position["left_foot"];
    toeDir.z = 0;
    if (length(toeDir) > 0.02 && dot(toeDir, forward) < 0) {
      forward = -forward;
      right = normalize(cross(forward, up));
      std::cout << "Flipped facing so the toes point forward.\n";
    }
  }

  auto endFrom = [&](Vec3 origin, Vec3 target, double minimum, double maximum) {
    const double raw = length(target - origin);
    const double used = std::clamp(raw < 1e-4 ? minimum : raw, minimum, maximum);
    const Vec3 dir = raw < 1e-4 ? forward : normalize(target - origin);
    return origin + dir * used;
  };

  std::map<std::string, Vec3> auxiliary;
  std::map<std::string, std::string> auxiliarySource;
  auto addAux = [&](const std::string& name, const std::string& source, Vec3 position) {
    auxiliary[name] = position;
    auxiliarySource[name] = source;
  };

  if (const int headTop = findFirst(keys, {"HeadTop_End"}); headTop >= 0) {
    addAux("head_top", loaded.nodes[headTop].name, endFrom(best.position["head"], converted(headTop), 0.12, 0.28));
  } else {
    addAux("head_top", "(synthesized)", best.position["head"] + Vec3{0, 0, 0.18});
  }

  auto addHand = [&](const char* side, std::initializer_list<const char*> fingers) {
    const std::string hand = std::string(side) + "_hand";
    const std::string elbow = std::string(side) + "_lower_arm";
    const int finger = findFirst(keys, fingers);
    Vec3 position;
    std::string source;
    if (finger >= 0) {
      position = endFrom(best.position[hand], converted(finger), 0.06, 0.11);
      source = loaded.nodes[finger].name;
    } else {
      const Vec3 dir = normalize(best.position[hand] - best.position[elbow]);
      position = best.position[hand] + (length(dir) > 0.5 ? dir : forward) * 0.09;
      source = "(synthesized)";
    }
    addAux(hand + "_end", source, position);
  };
  addHand("left", {"LeftHandMiddle1", "LeftHandMiddle2", "LeftHandIndex1", "LeftHandThumb1"});
  addHand("right", {"RightHandMiddle1", "RightHandMiddle2", "RightHandIndex1", "RightHandThumb1"});

  auto addToe = [&](const char* side, int index) {
    const std::string foot = std::string(side) + "_foot";
    const std::string name = std::string(side) + "_toe_end";
    if (index >= 0) {
      addAux(name, loaded.nodes[index].name, endFrom(best.position[foot], converted(index), 0.12, 0.28));
    } else {
      addAux(name, "(synthesized)", best.position[foot] + forward * 0.18 + Vec3{0, 0, -0.02});
    }
  };
  addToe("left", leftToe);
  addToe("right", rightToe);

  const Vec3 arm = normalize(best.position["left_lower_arm"] - best.position["left_upper_arm"]);
  const double dropDeg = std::atan2(-arm.z, std::sqrt(arm.x * arm.x + arm.y * arm.y)) * 180.0 / 3.141592653589793;
  std::cout << "Converted height " << best.height << " m, scale " << best.scale << ", up mode " << best.mode << "\n";
  std::cout << "Arm drop from horizontal: " << dropDeg << " deg\n";
  std::cout << "Axes right=(" << formatVec(right) << ") forward=(" << formatVec(forward) << ")\n";

  nlohmann::json json;
  json["version"] = 1;
  json["units"] = "meters";
  json["up_axis"] = "z";
  json["animation_curves_imported"] = false;
  json["mesh_imported"] = false;
  json["source_file"] = input.filename().string();
  json["source_scale_to_meters"] = best.scale;
  json["up_conversion_mode"] = best.mode;
  json["ignored_animation_count"] = loaded.scene->mNumAnimations;
  json["notes"] = "Rest pose only. Animation curves were not stored. The mesh is not used.";
  json["axes"]["right"] = {right.x, right.y, right.z};
  json["axes"]["forward"] = {forward.x, forward.y, forward.z};
  json["axes"]["up"] = {up.x, up.y, up.z};
  json["ignored_animation_names"] = nlohmann::json::array();
  for (unsigned i = 0; i < loaded.scene->mNumAnimations; ++i) {
    const aiAnimation* animation = loaded.scene->mAnimations[i];
    json["ignored_animation_names"].push_back(animation->mName.length ? animation->mName.C_Str() : "");
  }

  std::set<int> usedNodes;
  json["bones"] = nlohmann::json::array();
  for (const pm::CanonicalBone& bone : pm::kCanonicalBones) {
    const int index = findKey(keys, bone.sourceKey);
    usedNodes.insert(index);
    const SourceNode& node = loaded.nodes[index];
    std::string sourceParent;
    if (node.parent >= 0) sourceParent = loaded.nodes[node.parent].name;
    const Vec3 position = best.position[bone.name];
    json["bones"].push_back({
        {"name", bone.name},
        {"parent", bone.parent},
        {"source", node.name},
        {"source_key", node.key},
        {"source_parent", sourceParent},
        {"position", {position.x, position.y, position.z}},
    });
  }
  json["auxiliary"] = nlohmann::json::array();
  const char* auxOrder[] = {"head_top", "left_hand_end", "right_hand_end", "left_toe_end", "right_toe_end"};
  const char* auxParent[] = {"head", "left_hand", "right_hand", "left_foot", "right_foot"};
  const char* auxPurpose[] = {"head length", "palm length", "palm length", "foot length", "foot length"};
  for (int i = 0; i < 5; ++i) {
    const Vec3 position = auxiliary[auxOrder[i]];
    json["auxiliary"].push_back({
        {"name", auxOrder[i]},
        {"parent", auxParent[i]},
        {"source", auxiliarySource[auxOrder[i]]},
        {"source_parent", auxParent[i]},
        {"position", {position.x, position.y, position.z}},
        {"purpose", auxPurpose[i]},
    });
  }

  std::filesystem::create_directories(output.parent_path());
  const std::filesystem::path temporary = std::filesystem::path(output.string() + ".tmp");
  {
    std::ofstream stream(temporary);
    if (!stream) {
      std::cerr << "Could not write " << temporary.string() << "\n";
      return 1;
    }
    stream << json.dump(2) << "\n";
  }
  std::error_code removeError;
  std::filesystem::remove(output, removeError);
  std::filesystem::rename(temporary, output);

  std::ofstream hierarchy(output.parent_path() / "hierarchy.txt");
  hierarchy << "Meshes ignored: " << loaded.scene->mNumMeshes << "\n";
  hierarchy << "Animation clips ignored: " << loaded.scene->mNumAnimations << "\n\n";
  printHierarchy(loaded.nodes, hierarchy);

  std::ofstream mapping(output.parent_path() / "skeleton_mapping.md");
  mapping << "# Y-Bot to canonical skeleton\n\n";
  mapping << "The importer keeps rest-pose joint positions only. ";
  mapping << "Animation curves are not stored and the mesh is not rendered.\n\n";
  mapping << "Scale to meters: " << best.scale << ". Up conversion mode: " << best.mode << ".\n\n";
  mapping << "| Canonical | Parent | Source bone | Position (m) |\n";
  mapping << "| --- | --- | --- | --- |\n";
  for (const pm::CanonicalBone& bone : pm::kCanonicalBones) {
    const int index = findKey(keys, bone.sourceKey);
    mapping << "| " << bone.name << " | " << (bone.parent[0] ? bone.parent : "-") << " | "
            << loaded.nodes[index].name << " | " << formatVec(best.position[bone.name]) << " |\n";
  }
  mapping << "\n## Auxiliary points\n\n";
  mapping << "These are not simulated joints. They size the skull, palms, and feet.\n\n";
  for (int i = 0; i < 5; ++i) {
    mapping << "- `" << auxOrder[i] << "` from `" << auxiliarySource[auxOrder[i]] << "` at "
            << formatVec(auxiliary[auxOrder[i]]) << " (" << auxPurpose[i] << ")\n";
  }
  mapping << "\n## Not separate rigid bodies\n\n";
  mapping << "- `left_shoulder` / `right_shoulder` are clavicles. The physical shoulder joint is at the upper arm.\n";
  mapping << "- Fingers, facial bones, IK targets, and unused end sites are omitted.\n\n";
  mapping << "## Source bones under the hips that were not mapped\n\n";
  for (int i = 0; i < static_cast<int>(loaded.nodes.size()); ++i) {
    if (!underHips(loaded.nodes, i) || usedNodes.count(i)) continue;
    bool usedAux = false;
    for (const auto& entry : auxiliarySource) {
      if (entry.second == loaded.nodes[i].name) usedAux = true;
    }
    if (usedAux || loaded.nodes[i].key.empty()) continue;
    mapping << "- " << loaded.nodes[i].name << "\n";
  }
  mapping << "\n";

  std::cout << "Wrote " << output.string() << "\n";
  std::cout << "Wrote " << (output.parent_path() / "skeleton_mapping.md").string() << "\n";
  return 0;
}
