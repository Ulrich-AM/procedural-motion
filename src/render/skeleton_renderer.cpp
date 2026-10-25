#include "render/skeleton_renderer.h"

#include "body/proc_body.h"

#include <glm/gtc/constants.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>

#include <cmath>
#include <string>

namespace proc {
namespace {

glm::vec3 gray(float value) { return glm::vec3(value); }

glm::vec3 boneColor(const std::string& name) {
  if (!name.empty() && name[0] == 'l') return gray(0.78f);
  if (!name.empty() && name[0] == 'r') return gray(0.46f);
  return gray(0.64f);
}

glm::mat4 spanCylinder(const glm::dvec3& from, const glm::dvec3& to, float radius) {
  const glm::vec3 start(static_cast<float>(from.x), static_cast<float>(from.y), static_cast<float>(from.z));
  const glm::vec3 end(static_cast<float>(to.x), static_cast<float>(to.y), static_cast<float>(to.z));
  const glm::vec3 delta = end - start;
  const float length = glm::length(delta);
  glm::vec3 direction = length > 1e-6f ? delta / length : glm::vec3(0.0f, 0.0f, 1.0f);
  const glm::vec3 up(0.0f, 0.0f, 1.0f);
  const float cosine = glm::dot(up, direction);
  glm::quat rotation(1.0f, 0.0f, 0.0f, 0.0f);
  if (cosine < -0.9999f) {
    rotation = glm::angleAxis(glm::pi<float>(), glm::vec3(1.0f, 0.0f, 0.0f));
  } else if (cosine < 0.9999f) {
    rotation = glm::angleAxis(std::acos(cosine), glm::normalize(glm::cross(up, direction)));
  }
  return glm::translate(glm::mat4(1.0f), start) * glm::mat4_cast(rotation) *
         glm::scale(glm::mat4(1.0f), glm::vec3(radius, radius, std::max(length, 1e-4f)));
}

void drawMarker(Renderer& renderer, const glm::dvec3& position, float radius, const glm::vec3& color) {
  const glm::mat4 model =
      glm::translate(glm::mat4(1.0f), glm::vec3(static_cast<float>(position.x), static_cast<float>(position.y), 0.008f)) *
      glm::scale(glm::mat4(1.0f), glm::vec3(radius, radius, 1.0f));
  renderer.drawMesh(renderer.disc(), model, color, false);
}

}  // namespace

void drawGroundGrid(Renderer& renderer) {
  std::vector<glm::vec3> minor;
  std::vector<glm::vec3> major;
  constexpr int kExtent = 8;
  for (int i = -kExtent; i <= kExtent; ++i) {
    const float coordinate = static_cast<float>(i) * 0.5f;
    std::vector<glm::vec3>& bucket = (i % 2 == 0) ? major : minor;
    bucket.push_back(glm::vec3(-4.0f, coordinate, 0.0f));
    bucket.push_back(glm::vec3(4.0f, coordinate, 0.0f));
    bucket.push_back(glm::vec3(coordinate, -4.0f, 0.0f));
    bucket.push_back(glm::vec3(coordinate, 4.0f, 0.0f));
  }
  renderer.drawLines(minor, gray(0.18f), false);
  renderer.drawLines(major, gray(0.28f), false);
  renderer.drawLines({glm::vec3(0.0f, -4.0f, 0.001f), glm::vec3(0.0f, 4.0f, 0.001f)}, gray(0.42f), false);
  renderer.drawLines({glm::vec3(-4.0f, 0.0f, 0.001f), glm::vec3(4.0f, 0.0f, 0.001f)}, gray(0.42f), false);
}

void drawSkeleton(Renderer& renderer, const ProcBody& body, const WorldPose& world, const SkeletonDrawOptions& options) {
  for (const BoneGeom& bone : body.bones) {
    const glm::mat4 model = spanCylinder(world.joints[bone.parent_joint].position, world.joints[bone.child_joint].position,
                                         bone.name.find("hand") != std::string::npos || bone.name.find("foot") != std::string::npos
                                             ? 0.010f
                                             : 0.013f);
    renderer.drawMesh(renderer.cylinder(), model, boneColor(bone.name), true);
  }
  for (int index = 0; index < kJointCount; ++index) {
    const glm::dvec3& position = world.joints[index].position;
    const float radius = index == kRootJoint ? 0.030f : 0.020f;
    const glm::mat4 model =
        glm::translate(glm::mat4(1.0f), glm::vec3(static_cast<float>(position.x), static_cast<float>(position.y),
                                                   static_cast<float>(position.z))) *
        glm::scale(glm::mat4(1.0f), glm::vec3(radius));
    renderer.drawMesh(renderer.sphere(), model, index == kRootJoint ? gray(0.95f) : gray(0.88f), true);
  }

  if (options.draw_facing_marker) {
    const int head = *findJoint("headj");
    const glm::dvec3 start = world.joints[head].position;
    const glm::dvec3 end = start + world.joints[head].rotation * glm::dvec3(0.0, 0.08, 0.0);
    renderer.drawMesh(renderer.cylinder(), spanCylinder(start, end, 0.006f), gray(0.92f), true);
  }

  if (options.trajectory != nullptr && options.trajectory->size() >= 2) {
    renderer.drawLines(*options.trajectory, gray(0.72f), true);
  }

  if (options.contacts != nullptr && options.contacts->block_present) {
    auto marker = [&](const char* joint, const std::optional<bool>& flag, float radius) {
      if (!flag.has_value()) return;
      const int index = *findJoint(joint);
      drawMarker(renderer, world.joints[index].position, radius, *flag ? gray(0.92f) : gray(0.22f));
    };
    marker("lfootj", options.contacts->left_heel, 0.055f);
    marker("rfootj", options.contacts->right_heel, 0.055f);
    marker("ltoesj", options.contacts->left_toe, 0.040f);
    marker("rtoesj", options.contacts->right_toe, 0.040f);
  }
}

}  // namespace proc
