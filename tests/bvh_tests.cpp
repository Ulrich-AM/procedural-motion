#include "test_support.h"

#include "import/bvh_importer.h"
#include "import/coordinate_normalizer.h"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <sstream>

namespace {

std::filesystem::path fixture(const char* name) {
  return std::filesystem::path(PROCMOTION_SOURCE_DIR) / "tests" / "fixtures" / name;
}

double dot(const glm::dquat& a, const glm::dquat& b) { return std::abs(glm::dot(a, b)); }

}  // namespace

int runBvhTests() {
  TestStats stats;
  proc::SourceClip clip;
  const proc::Status status = proc::loadBvhFromFile(fixture("synthetic_source.bvh"), proc::identityConvention(), clip);
  stats.expect(status.ok(), status.ok() ? "bvh load" : status.text());
  if (!status.ok()) return stats.failed;
  stats.expect(clip.format == "BVH", "format");
  stats.expect(clip.joints.size() == 6, "joint count " + std::to_string(clip.joints.size()));
  stats.expect(clip.frames.size() == 2, "frame count");
  stats.expect(std::abs(clip.nominal_fps - 2.0) < 1e-9, "fps");
  stats.expect(clip.joints[0].name == "hips", "root name");
  stats.expect(clip.joints[1].name == "thigh", "thigh");
  stats.expect(clip.joints[2].name == "shin", "shin");
  stats.expect(clip.joints[3].name == "shin_EndSite", "end site");
  stats.expect(clip.joints[1].parent == 0 && clip.joints[2].parent == 1, "hierarchy");

  const glm::dquat root0 = proc::quatAxisAngle(glm::dvec3(0, 0, 1), glm::radians(90.0));
  stats.expect(dot(clip.frames[0].local_rotation[0], root0) > 0.9999, "frame 0 root is Z rotation");
  const glm::dquat root1 = proc::quatAxisAngle(glm::dvec3(1, 0, 0), glm::radians(30.0));
  stats.expect(dot(clip.frames[1].local_rotation[0], root1) > 0.9999, "frame 1 root is X rotation");
  stats.expect(glm::length(clip.frames[1].local_translation[0] - glm::dvec3(1, 2, 3)) < 1e-9, "root translation");

  const glm::dquat z90 = proc::quatAxisAngle(glm::dvec3(0, 0, 1), glm::radians(90.0));
  const glm::dquat x90 = proc::quatAxisAngle(glm::dvec3(1, 0, 0), glm::radians(90.0));
  const glm::dquat thigh = proc::quatNormalize(x90 * z90);
  stats.expect(dot(clip.frames[1].local_rotation[1], thigh) > 0.9999, "Z-then-X rotation order");
  const glm::dquat wrong_order = proc::quatNormalize(z90 * x90);
  stats.expect(dot(clip.frames[1].local_rotation[1], wrong_order) < 0.99, "XYZ order must not match");

  const glm::dquat shin = proc::quatAxisAngle(glm::dvec3(1, 0, 0), glm::radians(10.0));
  stats.expect(dot(clip.frames[1].local_rotation[2], shin) > 0.9999, "shin X rotation");
  stats.expect(glm::length(clip.frames[1].local_translation[4] - glm::dvec3(0.25, 0.3, 0.0)) < 1e-9,
               "non-root position channel");
  stats.expect(!clip.translation_notes.empty(), "non-root translation is reported");
  bool mentioned = false;
  for (const std::string& note : clip.translation_notes) {
    if (note.find("slide") != std::string::npos && note.find("not exported") != std::string::npos) mentioned = true;
  }
  stats.expect(mentioned, "translation note names slide");

  proc::AxisConvention detected = proc::detectHumanConvention(clip);
  stats.expect(detected.unit_to_meters == 1.0, "synthetic bvh stays in meters");
  std::cout << "bvh tests: " << (stats.failed == 0 ? "ok" : "failed") << '\n';
  return stats.failed;
}
