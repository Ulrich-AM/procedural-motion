#include "test_support.h"

#include "body/proc_body.h"

#include <nlohmann/json.hpp>

#include <filesystem>
#include <fstream>
#include <sstream>

namespace {

std::filesystem::path sourceRoot() { return std::filesystem::path(PROCMOTION_SOURCE_DIR); }

std::string readFile(const std::filesystem::path& path) {
  std::ifstream stream(path, std::ios::binary);
  std::ostringstream buffer;
  buffer << stream.rdbuf();
  return buffer.str();
}

nlohmann::json neutralJson() {
  return nlohmann::json::parse(readFile(sourceRoot() / "assets" / "bodies" / "neutral.procbody.json"));
}

}  // namespace

int runBodyTests() {
  TestStats stats;
  proc::ProcBody body;
  const proc::Status loaded =
      proc::loadProcBodyFromFile(sourceRoot() / "assets" / "bodies" / "neutral.procbody.json", body);
  stats.expect(loaded.ok(), "neutral body loads: " + loaded.text());
  stats.expect(body.skeleton == proc::kSkeletonId, "neutral skeleton id");
  stats.expect(body.joints.size() == static_cast<size_t>(proc::kJointCount), "joint count");
  stats.expect(body.bones.size() == static_cast<size_t>(proc::kBoneCount), "bone count");
  stats.expect(body.joints[proc::kRootJoint].name == "legrootj", "root is legrootj");
  stats.expect(body.joints[proc::kRootJoint].parent == -1, "root has no parent");
  bool linked = true;
  for (int index = 0; index < proc::kBoneCount; ++index) {
    const proc::BoneGeom& bone = body.bones[index];
    if (bone.parent_joint != proc::kCanonicalBones[index].parent ||
        bone.child_joint != proc::kCanonicalBones[index].child || !bone.locked || bone.length_m <= 0.0) {
      linked = false;
    }
    if (body.joints[bone.child_joint].parent != bone.parent_joint) linked = false;
  }
  stats.expect(linked, "canonical hierarchy is linked and every bone is locked");
  const double* stature = body.anthropometry.find("stature_m");
  const double* shoulders = body.anthropometry.find("shoulder_breadth_m");
  stats.expect(stature != nullptr && *stature == 1.75, "stature stays an anthropometric reference");
  stats.expect(shoulders != nullptr && *shoulders > body.joints[18].rest_position.x - body.joints[14].rest_position.x,
               "external shoulder breadth is not the joint-center span");

  {
    nlohmann::json doc = neutralJson();
    auto& bones = doc["bones"];
    for (auto it = bones.begin(); it != bones.end(); ++it) {
      if ((*it)["name"] == "lleguppb") {
        bones.erase(it);
        break;
      }
    }
    proc::ProcBody rejected;
    const proc::Status status = proc::loadProcBodyFromString(doc.dump(), rejected, "missing-bone");
    stats.expect(!status.ok(), "missing bone is rejected");
    stats.expect(status.text().find("missing bone") != std::string::npos, "missing bone error names the problem");
  }
  {
    nlohmann::json doc = neutralJson();
    for (auto& bone : doc["bones"]) {
      if (bone["name"] == "rleguppb") bone["length_m"] = 0.1;
    }
    proc::ProcBody rejected;
    const proc::Status status = proc::loadProcBodyFromString(doc.dump(), rejected, "bad-length");
    stats.expect(!status.ok(), "inconsistent bone length is rejected");
    stats.expect(status.text().find("does not match") != std::string::npos, "length error explains the mismatch");
  }
  {
    nlohmann::json doc = neutralJson();
    for (auto& bone : doc["bones"]) {
      if (bone["name"] == "lleglowb") bone["length_m"] = -0.2;
    }
    proc::ProcBody rejected;
    const proc::Status status = proc::loadProcBodyFromString(doc.dump(), rejected, "negative-length");
    stats.expect(!status.ok(), "negative length is rejected");
    stats.expect(status.text().find("zero or negative") != std::string::npos, "negative length is identified");
  }
  {
    nlohmann::json doc = neutralJson();
    for (auto& bone : doc["bones"]) {
      if (bone["name"] == "lleguppb") bone["parent_joint"] = "spine1j";
    }
    proc::ProcBody rejected;
    const proc::Status status = proc::loadProcBodyFromString(doc.dump(), rejected, "bad-hierarchy");
    stats.expect(!status.ok(), "invalid hierarchy is rejected");
    stats.expect(status.text().find("invalid hierarchy") != std::string::npos, "hierarchy error is explicit");
  }
  {
    proc::ProcBody missing;
    const proc::Status status = proc::loadProcBodyFromFile(sourceRoot() / "assets" / "bodies" / "no-such-body.json", missing);
    stats.expect(!status.ok(), "a missing file does not crash");
  }

  std::cout << "body tests: " << (stats.failed == 0 ? "ok" : "failed") << '\n';
  return stats.failed;
}
