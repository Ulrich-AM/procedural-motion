#include "test_support.h"

#include "import/fbx_importer.h"

#include <filesystem>

namespace {

std::filesystem::path fixture(const char* name) {
  return std::filesystem::path(PROCMOTION_SOURCE_DIR) / "tests" / "fixtures" / name;
}

}  // namespace

int runImportTests() {
  TestStats stats;
  proc::SourceClip clip;
  const proc::Status status = proc::loadFbxFromFile(fixture("synthetic_source.fbx"), proc::identityConvention(), clip);
  stats.expect(status.ok(), status.ok() ? "fbx load" : status.text());
  if (status.ok()) {
    stats.expect(clip.format == "FBX", "format");
    stats.expect(clip.joints.size() >= 2, "fbx joints " + std::to_string(clip.joints.size()));
    int hips = -1;
    int leg = -1;
    for (int index = 0; index < static_cast<int>(clip.joints.size()); ++index) {
      if (clip.joints[index].name.find("Hips") != std::string::npos) hips = index;
      if (clip.joints[index].name.find("LeftUpLeg") != std::string::npos) leg = index;
    }
    stats.expect(hips >= 0 && leg >= 0, "Hips and LeftUpLeg imported");
    if (hips >= 0 && leg >= 0) {
      const glm::dvec3 delta = clip.joints[leg].rest_global_position - clip.joints[hips].rest_global_position;
      stats.expect(glm::length(delta) > 1e-4, "leg offset " + std::to_string(glm::length(delta)));
      stats.expect(!clip.frames.empty(), "fbx has a pose");
    }
  }
  std::cout << "import tests: " << (stats.failed == 0 ? "ok" : "failed") << '\n';
  return stats.failed;
}
