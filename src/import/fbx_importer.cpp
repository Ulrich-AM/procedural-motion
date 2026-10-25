#include "import/fbx_importer.h"

#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable : 4100 4127 4189 4244 4267 4996)
#endif
#include <assimp/Importer.hpp>
#include <assimp/config.h>
#include <assimp/postprocess.h>
#include <assimp/scene.h>
#if defined(_MSC_VER)
#pragma warning(pop)
#endif

#include <algorithm>
#include <cmath>
#include <map>
#include <vector>

namespace proc {
namespace {

glm::dvec3 fromAi(const aiVector3D& value) { return glm::dvec3(value.x, value.y, value.z); }

glm::dquat fromAi(const aiQuaternion& value) { return quatNormalize(glm::dquat(value.w, value.x, value.y, value.z)); }

void decompose(const aiMatrix4x4& matrix, glm::dvec3& translation, glm::dquat& rotation) {
  aiVector3D scale;
  aiQuaternion quat;
  aiVector3D position;
  matrix.Decompose(scale, quat, position);
  translation = fromAi(position);
  rotation = fromAi(quat);
}

const aiNodeAnim* findChannel(const aiAnimation* animation, const std::string& name) {
  if (animation == nullptr) return nullptr;
  for (unsigned index = 0; index < animation->mNumChannels; ++index) {
    if (animation->mChannels[index] != nullptr && animation->mChannels[index]->mNodeName.C_Str() == name) {
      return animation->mChannels[index];
    }
  }
  return nullptr;
}

glm::dvec3 samplePosition(const aiNodeAnim* channel, double ticks, const glm::dvec3& rest) {
  if (channel == nullptr || channel->mNumPositionKeys == 0) return rest;
  if (channel->mNumPositionKeys == 1 || ticks <= channel->mPositionKeys[0].mTime) return fromAi(channel->mPositionKeys[0].mValue);
  const unsigned last = channel->mNumPositionKeys - 1;
  if (ticks >= channel->mPositionKeys[last].mTime) return fromAi(channel->mPositionKeys[last].mValue);
  unsigned upper = 1;
  while (upper < channel->mNumPositionKeys && channel->mPositionKeys[upper].mTime < ticks) ++upper;
  const auto& a = channel->mPositionKeys[upper - 1];
  const auto& b = channel->mPositionKeys[upper];
  const double span = b.mTime - a.mTime;
  const double t = span > 1e-12 ? (ticks - a.mTime) / span : 0.0;
  return fromAi(a.mValue) * (1.0 - t) + fromAi(b.mValue) * t;
}

glm::dquat sampleRotation(const aiNodeAnim* channel, double ticks, const glm::dquat& rest) {
  if (channel == nullptr || channel->mNumRotationKeys == 0) return rest;
  if (channel->mNumRotationKeys == 1 || ticks <= channel->mRotationKeys[0].mTime) return fromAi(channel->mRotationKeys[0].mValue);
  const unsigned last = channel->mNumRotationKeys - 1;
  if (ticks >= channel->mRotationKeys[last].mTime) return fromAi(channel->mRotationKeys[last].mValue);
  unsigned upper = 1;
  while (upper < channel->mNumRotationKeys && channel->mRotationKeys[upper].mTime < ticks) ++upper;
  const auto& a = channel->mRotationKeys[upper - 1];
  const auto& b = channel->mRotationKeys[upper];
  const double span = b.mTime - a.mTime;
  const double t = span > 1e-12 ? (ticks - a.mTime) / span : 0.0;
  return quatSlerp(fromAi(a.mValue), fromAi(b.mValue), t);
}

void collectNodes(const aiNode* node, int parent, std::vector<const aiNode*>& nodes, std::vector<int>& parents) {
  if (node == nullptr) return;
  const int index = static_cast<int>(nodes.size());
  nodes.push_back(node);
  parents.push_back(parent);
  for (unsigned child = 0; child < node->mNumChildren; ++child) collectNodes(node->mChildren[child], index, nodes, parents);
}

}  // namespace

Status loadFbxFromFile(const std::filesystem::path& path, const AxisConvention& convention, SourceClip& out) {
  Status status;
  Assimp::Importer importer;
  importer.SetPropertyBool(AI_CONFIG_IMPORT_FBX_PRESERVE_PIVOTS, false);
  const aiScene* scene = importer.ReadFile(path.string(), 0);
  if (scene == nullptr || scene->mRootNode == nullptr) {
    status.error("FBX: " + std::string(importer.GetErrorString()));
    return status;
  }
  std::vector<const aiNode*> nodes;
  std::vector<int> parents;
  collectNodes(scene->mRootNode, -1, nodes, parents);
  if (nodes.empty()) {
    status.error("FBX: no nodes");
    return status;
  }

  const aiAnimation* animation = scene->mNumAnimations > 0 ? scene->mAnimations[0] : nullptr;
  double ticks_per_second = 30.0;
  double duration_ticks = 0.0;
  if (animation != nullptr) {
    if (animation->mTicksPerSecond > 1e-6) ticks_per_second = animation->mTicksPerSecond;
    duration_ticks = animation->mDuration;
  }
  std::vector<double> times;
  times.push_back(0.0);
  if (animation != nullptr) {
    times.push_back(duration_ticks);
    for (unsigned channel_index = 0; channel_index < animation->mNumChannels; ++channel_index) {
      const aiNodeAnim* channel = animation->mChannels[channel_index];
      if (channel == nullptr) continue;
      for (unsigned key = 0; key < channel->mNumRotationKeys; ++key) times.push_back(channel->mRotationKeys[key].mTime);
      for (unsigned key = 0; key < channel->mNumPositionKeys; ++key) times.push_back(channel->mPositionKeys[key].mTime);
    }
  }
  std::sort(times.begin(), times.end());
  times.erase(std::unique(times.begin(), times.end(), [](double a, double b) { return std::abs(a - b) < 1e-6; }), times.end());

  SourceClip clip;
  clip.format = "FBX";
  clip.path = path.string();
  clip.name = animation != nullptr && animation->mName.length > 0 ? animation->mName.C_Str() : path.stem().string();
  clip.nominal_fps = ticks_per_second;
  clip.duration = times.size() <= 1 ? 1.0 / clip.nominal_fps : (times.back() - times.front()) / ticks_per_second;
  clip.joints.resize(nodes.size());
  for (int index = 0; index < static_cast<int>(nodes.size()); ++index) {
    glm::dvec3 translation;
    glm::dquat rotation;
    decompose(nodes[index]->mTransformation, translation, rotation);
    clip.joints[index].name = nodes[index]->mName.C_Str();
    clip.joints[index].parent = parents[index];
    clip.joints[index].file_rest_translation = translation;
    clip.joints[index].file_rest_rotation = rotation;
  }
  clip.frames.resize(times.size());
  for (int frame_index = 0; frame_index < static_cast<int>(times.size()); ++frame_index) {
    SourceFrame& frame = clip.frames[frame_index];
    frame.time = (times[frame_index] - times.front()) / ticks_per_second;
    frame.file_translation.resize(nodes.size());
    frame.file_rotation.resize(nodes.size());
    for (int index = 0; index < static_cast<int>(nodes.size()); ++index) {
      const aiNodeAnim* channel = findChannel(animation, clip.joints[index].name);
      frame.file_translation[index] = samplePosition(channel, times[frame_index], clip.joints[index].file_rest_translation);
      frame.file_rotation[index] = sampleRotation(channel, times[frame_index], clip.joints[index].file_rest_rotation);
    }
  }
  for (int index = 0; index < static_cast<int>(nodes.size()); ++index) {
    clip.joints[index].rest_local_translation = clip.joints[index].file_rest_translation;
    clip.joints[index].rest_local_rotation = clip.joints[index].file_rest_rotation;
  }
  for (SourceFrame& frame : clip.frames) {
    frame.local_translation = frame.file_translation;
    frame.local_rotation = frame.file_rotation;
  }
  rebuildSourceTopology(clip);
  rebuildSourceGlobals(clip);
  double height = 0.0;
  for (const SourceJoint& joint : clip.joints) {
    height = std::max(height, std::abs(joint.rest_global_position.x));
    height = std::max(height, std::abs(joint.rest_global_position.y));
    height = std::max(height, std::abs(joint.rest_global_position.z));
  }
  if (height > 10.0) {
    for (SourceJoint& joint : clip.joints) joint.file_rest_translation *= 0.01;
    for (SourceFrame& frame : clip.frames) {
      for (glm::dvec3& translation : frame.file_translation) translation *= 0.01;
    }
    clip.units_detected = "FBX centimeters converted to meters";
  } else {
    clip.units_detected = "FBX meters";
  }
  applyAxisConvention(clip, convention);
  out = std::move(clip);
  return status;
}

}  // namespace proc
