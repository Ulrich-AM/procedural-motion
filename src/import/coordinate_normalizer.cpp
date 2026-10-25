#include "import/coordinate_normalizer.h"

#include <glm/common.hpp>
#include <glm/geometric.hpp>
#include <glm/gtc/quaternion.hpp>

#include <algorithm>
#include <cctype>
#include <cmath>

namespace proc {
namespace {

std::string lower(std::string text) {
  for (char& character : text) character = static_cast<char>(std::tolower(static_cast<unsigned char>(character)));
  return text;
}

bool contains(const std::string& name, const char* needle) {
  return lower(name).find(needle) != std::string::npos;
}

bool parseSignedAxis(const std::string& text, glm::dvec3& out) {
  const std::string value = lower(text);
  int sign = 1;
  std::size_t index = 0;
  if (!value.empty() && (value[0] == '+' || value[0] == '-')) {
    sign = value[0] == '-' ? -1 : 1;
    index = 1;
  }
  if (index >= value.size()) return false;
  out = glm::dvec3(0.0);
  if (value[index] == 'x') out.x = sign;
  else if (value[index] == 'y') out.y = sign;
  else if (value[index] == 'z') out.z = sign;
  else return false;
  return true;
}

glm::dquat basisQuat(const glm::dmat3& basis) {
  return quatNormalize(glm::quat_cast(basis));
}

glm::dvec3 mapVector(const AxisConvention& convention, const glm::dvec3& value, bool scale) {
  glm::dvec3 mapped = convention.basis * value;
  if (scale) mapped *= convention.unit_to_meters;
  return mapped;
}

glm::dquat mapQuat(const glm::dquat& basis, const glm::dquat& value) {
  return quatNormalize(basis * quatNormalize(value) * quatInverse(basis));
}

}  // namespace

AxisConvention identityConvention() {
  AxisConvention convention;
  convention.description = "identity (+X right, +Y forward, +Z up, meters)";
  return convention;
}

AxisConvention conventionFromAxes(const std::string& up, const std::string& forward, double unit_to_meters, std::string& error) {
  glm::dvec3 file_up(0.0);
  glm::dvec3 file_forward(0.0);
  if (!parseSignedAxis(up, file_up) || !parseSignedAxis(forward, file_forward)) {
    error = "coordinate axes must look like +X, -Y, or +Z";
    return identityConvention();
  }
  if (std::abs(glm::dot(file_up, file_forward)) > 0.5) {
    error = "up and forward axes must be different";
    return identityConvention();
  }
  // PROC is right-handed: right x forward = up. The file right axis is the one
  // completion of (forward, up) that keeps that relation. For a Y-up, +Z-forward
  // source this is -X, which is where Mixamo's character right lands after Assimp.
  const glm::dvec3 file_right = glm::cross(file_forward, file_up);
  AxisConvention convention;
  convention.unit_to_meters = unit_to_meters;
  for (int column = 0; column < 3; ++column) {
    convention.basis[column][0] = file_right[column];
    convention.basis[column][1] = file_forward[column];
    convention.basis[column][2] = file_up[column];
  }
  convention.description = "file up " + up + ", forward " + forward + ", scale " + std::to_string(unit_to_meters);
  return convention;
}

AxisConvention detectHumanConvention(const SourceClip& clip) {
  AxisConvention convention = identityConvention();
  if (clip.joints.empty()) return convention;
  const int root = clip.root;
  glm::dvec3 minimum(1.0e9);
  glm::dvec3 maximum(-1.0e9);
  int head = -1;
  int left = -1;
  int forward_joint = -1;
  for (int index = 0; index < static_cast<int>(clip.joints.size()); ++index) {
    const glm::dvec3 position = clip.joints[index].rest_global_position;
    minimum = glm::min(minimum, position);
    maximum = glm::max(maximum, position);
    const std::string& name = clip.joints[index].name;
    if (head < 0 && contains(name, "head")) head = index;
    if (left < 0 && contains(name, "left")) left = index;
    if (forward_joint < 0 && (contains(name, "toe") || contains(name, "foot"))) forward_joint = index;
  }
  const glm::dvec3 span = maximum - minimum;
  int up_axis = 0;
  if (span.y > span[up_axis]) up_axis = 1;
  if (span.z > span[up_axis]) up_axis = 2;
  int up_sign = 1;
  if (head >= 0) {
    const double delta = clip.joints[head].rest_global_position[up_axis] - clip.joints[root].rest_global_position[up_axis];
    if (delta < 0.0) up_sign = -1;
  }
  const double height = span[up_axis];
  convention.unit_to_meters = height > 10.0 ? 0.01 : 1.0;

  int forward_axis = (up_axis + 1) % 3;
  int other_axis = (up_axis + 2) % 3;
  if (span[other_axis] < span[forward_axis]) std::swap(forward_axis, other_axis);
  int forward_sign = 1;
  if (forward_joint >= 0) {
    const double delta =
        clip.joints[forward_joint].rest_global_position[forward_axis] - clip.joints[root].rest_global_position[forward_axis];
    if (std::abs(delta) > 1e-8) forward_sign = delta > 0.0 ? 1 : -1;
  }

  const char* names = "XYZ";
  const std::string up = std::string(up_sign < 0 ? "-" : "+") + names[up_axis];
  const std::string forward = std::string(forward_sign < 0 ? "-" : "+") + names[forward_axis];
  std::string error;
  convention = conventionFromAxes(up, forward, convention.unit_to_meters, error);
  if (left >= 0) {
    const glm::dvec3 left_offset =
        clip.joints[left].rest_global_position - clip.joints[root].rest_global_position;
    const double proc_x = (convention.basis * left_offset).x;
    if (proc_x > 0.0) {
      forward_sign = -forward_sign;
      const std::string flipped = std::string(forward_sign < 0 ? "-" : "+") + names[forward_axis];
      convention = conventionFromAxes(up, flipped, convention.unit_to_meters, error);
      convention.description += "; facing flipped so a Left* joint lands on -X";
    }
  }
  convention.description = "detected " + convention.description;
  return convention;
}

void applyAxisConvention(SourceClip& clip, const AxisConvention& convention) {
  const glm::dquat rotation = basisQuat(convention.basis);
  const int count = static_cast<int>(clip.joints.size());
  for (int index = 0; index < count; ++index) {
    clip.joints[index].rest_local_translation = mapVector(convention, clip.joints[index].file_rest_translation, true);
    clip.joints[index].rest_local_rotation = mapQuat(rotation, clip.joints[index].file_rest_rotation);
  }
  for (SourceFrame& frame : clip.frames) {
    frame.local_translation.resize(count);
    frame.local_rotation.resize(count);
    for (int index = 0; index < count; ++index) {
      frame.local_translation[index] = mapVector(convention, frame.file_translation[index], true);
      frame.local_rotation[index] = mapQuat(rotation, frame.file_rotation[index]);
    }
  }
  clip.coordinate_conversion = convention.description;
  if (convention.unit_to_meters == 1.0) clip.units_detected = "meters";
  else if (std::abs(convention.unit_to_meters - 0.01) < 1e-12) clip.units_detected = "centimeters converted to meters";
  else clip.units_detected = "scaled by " + std::to_string(convention.unit_to_meters);
  rebuildSourceGlobals(clip);

  clip.translation_notes.clear();
  std::vector<int> noted(count, 0);
  for (const SourceFrame& frame : clip.frames) {
    for (int index = 0; index < count; ++index) {
      if (clip.joints[index].parent < 0 || noted[index]) continue;
      const glm::dvec3 delta = frame.local_translation[index] - clip.joints[index].rest_local_translation;
      if (glm::length(delta) > 1e-5) {
        noted[index] = 1;
        clip.translation_notes.push_back(clip.joints[index].name +
                                          ": animated source translation detected; preserved for source preview; not exported to PROC MOTION");
      }
    }
  }
}

}  // namespace proc
