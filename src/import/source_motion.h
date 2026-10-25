#pragma once

#include "math/quat.h"

#include <string>
#include <vector>

namespace proc {

struct SourceJoint {
  std::string name;
  int parent = -1;
  std::vector<int> children;

  glm::dvec3 file_rest_translation{0.0, 0.0, 0.0};
  glm::dquat file_rest_rotation{1.0, 0.0, 0.0, 0.0};

  glm::dvec3 rest_local_translation{0.0, 0.0, 0.0};
  glm::dquat rest_local_rotation{1.0, 0.0, 0.0, 0.0};
  glm::dvec3 rest_global_position{0.0, 0.0, 0.0};
  glm::dquat rest_global_rotation{1.0, 0.0, 0.0, 0.0};
};

struct SourceFrame {
  double time = 0.0;
  std::vector<glm::dvec3> file_translation;
  std::vector<glm::dquat> file_rotation;
  std::vector<glm::dvec3> local_translation;
  std::vector<glm::dquat> local_rotation;
  std::vector<glm::dvec3> global_position;
  std::vector<glm::dquat> global_rotation;
};

// Canonical clip: +X right, +Y forward, +Z up, meters, quaternions.
// file_* fields keep the importer's pre-conversion values so a profile can reapply a basis.
struct SourceClip {
  std::string name;
  std::string format;
  std::string path;
  double nominal_fps = 30.0;
  double duration = 0.0;
  int root = 0;
  std::string units_detected;
  std::string coordinate_conversion;
  std::vector<SourceJoint> joints;
  std::vector<SourceFrame> frames;
  std::vector<std::string> translation_notes;
};

void rebuildSourceTopology(SourceClip& clip);
void rebuildSourceGlobals(SourceClip& clip);
int findSourceJoint(const SourceClip& clip, const std::string& name);

// Sample canonical locals. Contacts are not part of a source clip.
void sampleSourceLocals(const SourceClip& clip, double time_s, std::vector<glm::dvec3>& translation,
                        std::vector<glm::dquat>& rotation);

}  // namespace proc
