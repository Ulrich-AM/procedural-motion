#include "test_support.h"

#include "body/joint_limits.h"
#include "body/proc_body.h"
#include "math/quat.h"
#include "motion/interpolation.h"
#include "motion/motion_player.h"
#include "motion/proc_motion.h"
#include "rig/forward_kinematics.h"
#include "validation/bone_length_monitor.h"
#include "validation/tolerances.h"

#include <glm/geometric.hpp>
#include <glm/gtc/quaternion.hpp>

#include <cmath>
#include <filesystem>
#include <string>

namespace {

std::filesystem::path sourceRoot() { return std::filesystem::path(PROCMOTION_SOURCE_DIR); }

const char* kMinimal = R"({
  "format": "PROC_MOTION",
  "version": 1,
  "skeleton": "proc_humanoid_v0",
  "source": {"type": "synthetic", "dataset": null, "clip": "unit"},
  "timing": {"fps": 1.0, "frame_count": 2, "duration_s": 2.0},
  "frames": [
    {"time": 0.0, "root": {"translation": [0, 0, 0], "rotation": [0, 0, 0, 1]}, "joints": {}},
    {"time": 1.0, "root": {"translation": [2, 0, 0], "rotation": [0, 0, 0.7071067812, 0.7071067812]},
     "joints": {"rarmuppj": [0, 0, 0.7071067812, 0.7071067812]},
     "contacts": {"left_heel": false, "left_toe": true}}
  ]
})";

bool loadMotion(const char* name, proc::ProcMotion& motion, TestStats& stats) {
  const proc::Status status =
      proc::loadProcMotionFromFile(sourceRoot() / "assets" / "motions" / name, motion);
  stats.expect(status.ok(), std::string(name) + " loads: " + status.text());
  return status.ok();
}

}  // namespace

int runMotionTests() {
  TestStats stats;

  proc::ProcMotion arm;
  proc::ProcMotion leg;
  proc::ProcMotion spine;
  proc::ProcMotion root_motion;
  proc::ProcMotion walk;
  const bool assets_ok = loadMotion("arm_test.procmo", arm, stats) && loadMotion("leg_test.procmo", leg, stats) &&
                         loadMotion("spine_test.procmo", spine, stats) &&
                         loadMotion("root_motion_test.procmo", root_motion, stats) &&
                         loadMotion("walkish_test.procmo", walk, stats);
  stats.expect(arm.source.clip == "arm_test", "arm_test source clip");
  stats.expect(!arm.source.dataset.has_value(), "synthetic dataset is null, not an empty contact");
  stats.expect(arm.fps == 30.0, "arm_test fps");
  stats.expect(!arm.frames.empty() && !arm.frames.front().contacts.block_present, "arm_test contacts are unknown");
  stats.expect(walk.frames.front().contacts.block_present, "walkish provides contacts");

  bool saw_true = false;
  bool saw_false = false;
  for (const proc::Pose& frame : walk.frames) {
    for (const std::optional<bool>& flag :
         {frame.contacts.left_heel, frame.contacts.left_toe, frame.contacts.right_heel, frame.contacts.right_toe}) {
      if (flag == true) saw_true = true;
      if (flag == false) saw_false = true;
    }
  }
  stats.expect(saw_true && saw_false, "walkish uses explicit true and explicit false contacts");

  proc::ProcMotion minimal;
  const proc::Status minimal_status = proc::loadProcMotionFromString(kMinimal, minimal, "unit");
  stats.expect(minimal_status.ok(), "minimal motion loads: " + minimal_status.text());
  if (minimal_status.ok()) {
    stats.expect(!minimal.frames[0].contacts.block_present, "missing contacts stay unknown");
    stats.expect(minimal.frames[1].contacts.left_heel.has_value() && *minimal.frames[1].contacts.left_heel == false,
                 "explicit false is stored");
    stats.expect(!minimal.frames[0].contacts.left_heel.has_value(), "unknown is not the same value as false");
    stats.expect(minimal.frames[0].contacts.left_heel != minimal.frames[1].contacts.left_heel,
                 "unknown and false compare differently");
    const proc::Pose mid = proc::sampleMotion(minimal, 0.5, false);
    stats.expect(std::abs(mid.root_translation.x - 1.0) < 1e-6, "root translation midpoint is linear");
    stats.expect(std::abs(proc::quatLength(mid.root_rotation) - 1.0) < 1e-8, "SLERP result is normalized");
    const proc::Pose at_frame = proc::sampleMotion(minimal, 1.0, false);
    stats.expect(std::abs(at_frame.root_translation.x - 2.0) < 1e-8, "sampling a source time hits that frame");
    stats.expect(at_frame.contacts.left_toe.has_value() && *at_frame.contacts.left_toe,
                 "source-frame contacts are kept");
    const double dot = glm::dot(mid.root_rotation, proc::quatAxisAngle(glm::dvec3(0.0, 0.0, 1.0), glm::radians(45.0)));
    stats.expect(std::abs(dot) > 0.9999, "quaternion midpoint is 45 degrees");
  }

  {
    std::string bad = kMinimal;
    const std::string needle = "[0, 0, 0, 1]";
    bad.replace(bad.find(needle), needle.size(), "[0, 0, 0, 0]");
    proc::ProcMotion rejected;
    const proc::Status status = proc::loadProcMotionFromString(bad, rejected, "zero-quat");
    stats.expect(!status.ok(), "a zero quaternion is rejected");
  }
  {
    std::string scaled = kMinimal;
    const std::string marker = "\"joints\": {}";
    scaled.replace(scaled.find(marker), marker.size(), "\"scale\": 1, \"joints\": {}");
    proc::ProcMotion rejected;
    const proc::Status status = proc::loadProcMotionFromString(scaled, rejected, "scale");
    stats.expect(!status.ok(), "animated scale is rejected");
    stats.expect(status.text().find("scale") != std::string::npos, "scale error names the field");
  }
  {
    std::string translated = kMinimal;
    const std::string marker = "\"rarmuppj\": [0, 0, 0.7071067812, 0.7071067812]";
    translated.replace(translated.find(marker), marker.size(), "\"rarmuppj\": {\"translation\": [1, 0, 0]}");
    proc::ProcMotion rejected;
    const proc::Status status = proc::loadProcMotionFromString(translated, rejected, "joint-translation");
    stats.expect(!status.ok(), "non-root joint translation is rejected");
  }
  {
    proc::ProcMotion normalized;
    std::string text = kMinimal;
    const std::string marker = "[0, 0, 0.7071067812, 0.7071067812]";
    text.replace(text.find(marker), marker.size(), "[1, 0, 0, 1]");
    const proc::Status status = proc::loadProcMotionFromString(text, normalized, "non-unit");
    stats.expect(status.ok(), "a non-unit quaternion is normalized: " + status.text());
    stats.expect(!status.warnings.empty(), "normalization is reported");
    if (status.ok()) {
      stats.expect(std::abs(proc::quatLength(normalized.frames[1].root_rotation) - 1.0) < 1e-8,
                   "stored quaternion is unit length");
    }
  }
  {
    std::string text = kMinimal;
    text.replace(text.find("\"time\": 1.0"), std::string("\"time\": 1.0").size(), "\"time\": 0.0");
    proc::ProcMotion rejected;
    const proc::Status status = proc::loadProcMotionFromString(text, rejected, "times");
    stats.expect(!status.ok(), "non-monotonic times are rejected");
  }

  proc::MotionPlayer player;
  player.setMotion(&minimal);
  player.setLoop(false);
  player.setSpeed(2.0);
  player.play();
  player.update(0.25);
  stats.expect(std::abs(player.time() - 0.5) < 1e-9, "playback speed scales delta time");
  player.jumpEnd();
  stats.expect(std::abs(player.time() - 1.0) < 1e-9, "jump to end uses the last source frame");
  player.jumpStart();
  player.nextFrame();
  stats.expect(std::abs(player.time() - 1.0) < 1e-9, "next frame steps to the next source time");
  player.setLoop(true);
  player.seekSeconds(0.0);
  player.play();
  player.setSpeed(1.0);
  player.update(2.25);
  stats.expect(std::abs(player.time() - 0.25) < 1e-6, "loop wraps inside duration");

  proc::ProcBody body;
  const proc::Status body_status =
      proc::loadProcBodyFromFile(sourceRoot() / "assets" / "bodies" / "neutral.procbody.json", body);
  stats.expect(body_status.ok(), "body loads for motion sweep");
  if (assets_ok && body_status.ok()) {
    const proc::ProcMotion* motions[] = {&arm, &leg, &spine, &root_motion, &walk};
    bool lengths_ok = true;
    double worst = 0.0;
    std::string worst_label;
    for (const proc::ProcMotion* motion : motions) {
      const int samples = motion->frame_count * 2;
      for (int sample = 0; sample < samples; ++sample) {
        const double time_s = motion->duration_s * static_cast<double>(sample) / static_cast<double>(samples);
        const proc::Pose raw = proc::sampleMotion(*motion, time_s, true);
        for (proc::LimitMode mode : {proc::LimitMode::Warn, proc::LimitMode::Clamp}) {
          const proc::LimitResult limited = proc::applyJointLimits(body, raw, mode);
          const proc::BoneLengthReport report =
              proc::measureBoneLengths(body, proc::forwardKinematics(body, limited.pose), proc::kRuntimeBoneToleranceM);
          if (report.max_error_m > worst) {
            worst = report.max_error_m;
            worst_label = motion->source.clip;
          }
          if (!report.ok) lengths_ok = false;
        }
      }
      const proc::Pose before = motion->frames[motion->frames.size() / 2];
      proc::applyJointLimits(body, before, proc::LimitMode::Clamp);
      const proc::Pose after = motion->frames[motion->frames.size() / 2];
      if (before.root_translation != after.root_translation || before.local_rotation[1].w != after.local_rotation[1].w) {
        lengths_ok = false;
      }
    }
    stats.expect(lengths_ok, "sample motions keep bone lengths under WARN and CLAMP, worst " + std::to_string(worst) +
                                 " in " + worst_label);

    proc::ProcBody again_body;
    proc::ProcMotion again_motion;
    proc::loadProcBodyFromFile(sourceRoot() / "assets" / "bodies" / "neutral.procbody.json", again_body);
    proc::loadProcMotionFromFile(sourceRoot() / "assets" / "motions" / "walkish_test.procmo", again_motion);
    bool identical = true;
    for (double time_s : {0.0, 0.15, 0.5, 1.0, 1.7}) {
      const proc::WorldPose a = proc::forwardKinematics(body, proc::sampleMotion(walk, time_s, true));
      const proc::WorldPose b = proc::forwardKinematics(again_body, proc::sampleMotion(again_motion, time_s, true));
      for (int index = 0; index < proc::kJointCount; ++index) {
        if (glm::length(a.joints[index].position - b.joints[index].position) > 1e-12) identical = false;
      }
    }
    stats.expect(identical, "loading the same body and motion twice is deterministic");
  }

  std::cout << "motion tests: " << (stats.failed == 0 ? "ok" : "failed") << '\n';
  return stats.failed;
}
