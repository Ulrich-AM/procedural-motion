#include "retarget/retargeter.h"

#include "rig/forward_kinematics.h"

#include <glm/geometric.hpp>

#include <cmath>

namespace proc {
namespace {

std::array<glm::dquat, kJointCount> desiredGlobals(const ProcBody& body, const SourceClip& clip, const SourceFrame& frame,
                                                  const std::array<int, kJointCount>& mapping,
                                                  const std::array<glm::dquat, kJointCount>& target_rest) {
  std::array<glm::dquat, kJointCount> desired;
  desired.fill(quatIdentity());
  for (int joint = 0; joint < kJointCount; ++joint) {
    const int parent = body.joints[joint].parent;
    const int source = mapping[joint];
    if (source >= 0) {
      const glm::dquat animated = quatNormalize(frame.global_rotation[source]);
      const glm::dquat rest = quatNormalize(clip.joints[source].rest_global_rotation);
      const glm::dquat delta = quatNormalize(animated * quatInverse(rest));
      desired[joint] = quatNormalize(delta * target_rest[joint]);
    } else if (parent >= 0) {
      desired[joint] = desired[parent];
    } else {
      desired[joint] = target_rest[joint];
    }
  }
  return desired;
}

SourceFrame restFrame(const SourceClip& clip) {
  SourceFrame frame;
  frame.time = 0.0;
  const int count = static_cast<int>(clip.joints.size());
  frame.global_rotation.resize(count);
  frame.global_position.resize(count);
  for (int index = 0; index < count; ++index) {
    frame.global_rotation[index] = clip.joints[index].rest_global_rotation;
    frame.global_position[index] = clip.joints[index].rest_global_position;
  }
  return frame;
}

double yawOf(const glm::dquat& rotation) {
  const glm::dvec3 forward = quatNormalize(rotation) * glm::dvec3(0.0, 1.0, 0.0);
  return std::atan2(forward.x, forward.y);
}

}  // namespace

Status retargetClip(const ProcBody& body, const SourceClip& clip, const std::array<int, kJointCount>& mapping,
                    const std::array<bool, kJointCount>& required, const RetargetOptions& options, ProcMotion& out,
                    RetargetReport& report) {
  Status status;
  report = {};
  if (clip.frames.empty() || clip.joints.empty()) {
    status.error("source clip has no frames");
    return status;
  }
  for (int joint = 0; joint < kJointCount; ++joint) {
    if (!required[joint]) continue;
    ++report.required;
    if (mapping[joint] >= 0) ++report.mapped;
  }
  if (mapping[kRootJoint] < 0) status.warn("legrootj is not mapped; root motion stays at the bind pose");

  Pose bind;
  const WorldPose rest_world = forwardKinematics(body, bind);
  std::array<glm::dquat, kJointCount> target_rest;
  for (int joint = 0; joint < kJointCount; ++joint) target_rest[joint] = rest_world.joints[joint].rotation;

  const std::array<glm::dquat, kJointCount> rest_desired = desiredGlobals(body, clip, restFrame(clip), mapping, target_rest);
  for (int joint = 0; joint < kJointCount; ++joint) {
    if (mapping[joint] < 0) {
      report.rest_error_deg[joint] = 0.0;
      continue;
    }
    const int parent = body.joints[joint].parent;
    const glm::dquat local = parent < 0 ? rest_desired[joint]
                                        : quatNormalize(quatInverse(rest_desired[parent]) * rest_desired[joint]);
    report.rest_error_deg[joint] = quatAngleDegrees(local);
    report.rest_max_error_deg = std::max(report.rest_max_error_deg, report.rest_error_deg[joint]);
  }

  ProcMotion motion;
  motion.skeleton = kSkeletonId;
  motion.source.type = "retarget";
  motion.source.dataset = std::nullopt;
  motion.source.clip = clip.name;
  motion.fps = clip.nominal_fps > 0.0 ? clip.nominal_fps : 30.0;
  motion.frame_count = static_cast<int>(clip.frames.size());
  motion.duration_s = std::max(clip.duration, static_cast<double>(motion.frame_count) / motion.fps);
  motion.frames.reserve(clip.frames.size());

  const int source_root = mapping[kRootJoint];
  glm::dvec3 horizontal_origin(0.0);
  glm::dquat facing = quatIdentity();
  if (source_root >= 0 && !clip.frames.empty()) {
    const glm::dvec3 first = clip.frames.front().global_position[source_root] - clip.joints[source_root].rest_global_position;
    horizontal_origin = glm::dvec3(first.x, first.y, 0.0);
    if (options.rebase_initial_facing) {
      const std::array<glm::dquat, kJointCount> first_desired =
          desiredGlobals(body, clip, clip.frames.front(), mapping, target_rest);
      facing = quatAxisAngle(glm::dvec3(0.0, 0.0, 1.0), -yawOf(first_desired[kRootJoint]));
    }
  }

  for (const SourceFrame& frame : clip.frames) {
    const std::array<glm::dquat, kJointCount> desired = desiredGlobals(body, clip, frame, mapping, target_rest);
    Pose pose;
    pose.time = frame.time;
    pose.root_rotation = quatNormalize(facing * desired[kRootJoint]);
    if (source_root >= 0) {
      glm::dvec3 delta = frame.global_position[source_root] - clip.joints[source_root].rest_global_position;
      if (options.rebase_horizontal_start) delta -= horizontal_origin;
      if (options.rebase_initial_facing) delta = facing * delta;
      pose.root_translation = delta;
    }
    for (int joint = 0; joint < kJointCount; ++joint) {
      if (joint == kRootJoint) continue;
      const int parent = body.joints[joint].parent;
      const glm::dquat parent_desired = parent < 0 ? quatIdentity() : desired[parent];
      pose.local_rotation[joint] = quatNormalize(quatInverse(parent_desired) * desired[joint]);
    }
    motion.frames.push_back(pose);
  }

  if (!motion.frames.empty() && motion.frames.back().time > motion.duration_s) {
    motion.duration_s = motion.frames.back().time;
  }
  report.translation_notes = clip.translation_notes;
  report.non_root_translations_exported = 0;
  out = std::move(motion);
  return status;
}

double segmentDirectionError(const SourceClip& clip, double time_s, const ProcBody& body, const WorldPose& target,
                             const std::array<int, kJointCount>& mapping, std::string& worst_bone) {
  if (clip.frames.empty()) return 0.0;
  std::vector<glm::dvec3> translation;
  std::vector<glm::dquat> rotation;
  sampleSourceLocals(clip, time_s, translation, rotation);
  std::vector<glm::dvec3> position(clip.joints.size());
  std::vector<glm::dquat> global(clip.joints.size());
  for (int index = 0; index < static_cast<int>(clip.joints.size()); ++index) {
    const int parent = clip.joints[index].parent;
    if (parent < 0) {
      global[index] = quatNormalize(rotation[index]);
      position[index] = translation[index];
    } else {
      global[index] = quatNormalize(global[parent] * rotation[index]);
      position[index] = position[parent] + global[parent] * translation[index];
    }
  }
  (void)global;
  double worst = 0.0;
  worst_bone.clear();
  for (const CanonicalBone& bone : kCanonicalBones) {
    if (mapping[bone.parent] < 0 || mapping[bone.child] < 0) continue;
    const glm::dvec3 source_delta = position[mapping[bone.child]] - position[mapping[bone.parent]];
    const glm::dvec3 target_delta = target.joints[bone.child].position - target.joints[bone.parent].position;
    if (glm::length(source_delta) < 1e-6 || glm::length(target_delta) < 1e-6) continue;
    const double cosine = std::clamp(glm::dot(glm::normalize(source_delta), glm::normalize(target_delta)), -1.0, 1.0);
    const double degrees = glm::degrees(std::acos(cosine));
    if (degrees >= worst) {
      worst = degrees;
      worst_bone = bone.name;
    }
  }
  return worst;
}

}  // namespace proc
