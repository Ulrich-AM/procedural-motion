#include "import/source_motion.h"

#include <algorithm>
#include <cmath>

namespace proc {
namespace {

void globalsFromLocals(const SourceClip& clip, const std::vector<glm::dvec3>& local_translation,
                       const std::vector<glm::dquat>& local_rotation, std::vector<glm::dvec3>& global_position,
                       std::vector<glm::dquat>& global_rotation) {
  const int count = static_cast<int>(clip.joints.size());
  global_position.resize(count);
  global_rotation.resize(count);
  for (int index = 0; index < count; ++index) {
    const int parent = clip.joints[index].parent;
    const glm::dquat local = quatNormalize(local_rotation[index]);
    if (parent < 0) {
      global_rotation[index] = local;
      global_position[index] = local_translation[index];
    } else {
      global_rotation[index] = quatNormalize(global_rotation[parent] * local);
      global_position[index] = global_position[parent] + global_rotation[parent] * local_translation[index];
    }
  }
}

}  // namespace

void rebuildSourceTopology(SourceClip& clip) {
  for (SourceJoint& joint : clip.joints) joint.children.clear();
  clip.root = 0;
  for (int index = 0; index < static_cast<int>(clip.joints.size()); ++index) {
    const int parent = clip.joints[index].parent;
    if (parent < 0) clip.root = index;
    else if (parent < static_cast<int>(clip.joints.size())) clip.joints[parent].children.push_back(index);
  }
}

void rebuildSourceGlobals(SourceClip& clip) {
  const int count = static_cast<int>(clip.joints.size());
  std::vector<glm::dvec3> rest_translation(count);
  std::vector<glm::dquat> rest_rotation(count);
  for (int index = 0; index < count; ++index) {
    rest_translation[index] = clip.joints[index].rest_local_translation;
    rest_rotation[index] = clip.joints[index].rest_local_rotation;
  }
  std::vector<glm::dvec3> rest_position;
  std::vector<glm::dquat> rest_global;
  globalsFromLocals(clip, rest_translation, rest_rotation, rest_position, rest_global);
  for (int index = 0; index < count; ++index) {
    clip.joints[index].rest_global_position = rest_position[index];
    clip.joints[index].rest_global_rotation = rest_global[index];
  }
  for (SourceFrame& frame : clip.frames) {
    globalsFromLocals(clip, frame.local_translation, frame.local_rotation, frame.global_position, frame.global_rotation);
  }
}

int findSourceJoint(const SourceClip& clip, const std::string& name) {
  for (int index = 0; index < static_cast<int>(clip.joints.size()); ++index) {
    if (clip.joints[index].name == name) return index;
  }
  return -1;
}

void sampleSourceLocals(const SourceClip& clip, double time_s, std::vector<glm::dvec3>& translation,
                        std::vector<glm::dquat>& rotation) {
  const int count = static_cast<int>(clip.joints.size());
  translation.assign(count, glm::dvec3(0.0));
  rotation.assign(count, quatIdentity());
  if (clip.frames.empty() || count == 0) return;
  if (clip.frames.size() == 1 || time_s <= clip.frames.front().time) {
    translation = clip.frames.front().local_translation;
    rotation = clip.frames.front().local_rotation;
    return;
  }
  if (time_s >= clip.frames.back().time) {
    translation = clip.frames.back().local_translation;
    rotation = clip.frames.back().local_rotation;
    return;
  }
  int upper = 1;
  while (upper < static_cast<int>(clip.frames.size()) && clip.frames[upper].time < time_s) ++upper;
  const SourceFrame& a = clip.frames[upper - 1];
  const SourceFrame& b = clip.frames[upper];
  const double span = b.time - a.time;
  const double t = span > 1e-12 ? (time_s - a.time) / span : 0.0;
  for (int index = 0; index < count; ++index) {
    translation[index] = a.local_translation[index] * (1.0 - t) + b.local_translation[index] * t;
    rotation[index] = quatSlerp(a.local_rotation[index], b.local_rotation[index], t);
  }
}

}  // namespace proc
