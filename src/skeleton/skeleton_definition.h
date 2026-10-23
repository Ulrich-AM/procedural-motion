#pragma once

#include "math_types.h"

#include <filesystem>
#include <string>
#include <vector>

namespace pm {

struct Bone {
  std::string name;
  std::string parent;
  std::string source;
  std::string sourceParent;
  Vec3 position;
};

// Rest-pose proportions loaded from skeleton.json.
// There are no keyframes and this type cannot play a clip.
struct SkeletonDefinition {
  std::vector<Bone> bones;
  std::vector<Bone> auxiliary;
  Vec3 right{1, 0, 0};
  Vec3 forward{0, 1, 0};
  Vec3 up{0, 0, 1};
  double sourceScale = 1;
  std::string sourceFile;
  int ignoredAnimations = 0;

  const Bone* findBone(const std::string& name) const;
  const Bone* findAny(const std::string& name) const;
  bool loadFromFile(const std::filesystem::path& path, std::string& error);
};

}  // namespace pm
