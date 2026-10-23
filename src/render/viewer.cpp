#include "render/viewer.h"

#ifndef NOMINMAX
#define NOMINMAX
#endif

#include <GLFW/glfw3.h>

#ifdef _WIN32
#include <windows.h>
#endif
#include <GL/gl.h>

#include <cmath>
#include <cstdint>
#include <fstream>

namespace pm {
namespace {

void setRgba(mjvGeom* geom, const float rgba[4]) {
  geom->rgba[0] = rgba[0];
  geom->rgba[1] = rgba[1];
  geom->rgba[2] = rgba[2];
  geom->rgba[3] = rgba[3];
}

}  // namespace

bool Viewer::init(GLFWwindow* window, mjModel* model, std::string& error) {
  window_ = window;
  model_ = model;
  glfwMakeContextCurrent(window_);
  mjv_defaultCamera(&camera_);
  mjv_defaultOption(&option_);
  mjv_defaultScene(&scene_);
  mjr_defaultContext(&context_);
  mjv_makeScene(model_, &scene_, 6000);
  mjr_makeContext(model_, &context_, mjFONTSCALE_100);
  option_.geomgroup[0] = 1;
  option_.geomgroup[1] = 1;
  option_.geomgroup[2] = 1;
  option_.geomgroup[3] = 0;
  scene_.flags[mjRND_SHADOW] = 0;
  scene_.flags[mjRND_REFLECTION] = 0;
  ready_ = true;
  error.clear();
  return true;
}

void Viewer::shutdown() {
  if (!ready_) return;
  mjr_freeContext(&context_);
  mjv_freeScene(&scene_);
  ready_ = false;
  model_ = nullptr;
  window_ = nullptr;
}

void Viewer::setInitialTarget(Vec3 target) {
  mjv_defaultCamera(&camera_);
  camera_.type = mjCAMERA_FREE;
  camera_.lookat[0] = target.x;
  camera_.lookat[1] = target.y;
  camera_.lookat[2] = target.z;
  camera_.distance = 3.6;
  camera_.azimuth = 135;
  camera_.elevation = -18;
  initialCamera_ = camera_;
}

void Viewer::resetCamera() { camera_ = initialCamera_; }

void Viewer::rotate(double dx, double dy) {
  int height = 1;
  if (window_) glfwGetWindowSize(window_, nullptr, &height);
  if (height < 1) height = 1;
  mjv_moveCamera(model_, mjMOUSE_ROTATE_V, dx / height, dy / height, &camera_);
}

void Viewer::pan(double dx, double dy) {
  int height = 1;
  if (window_) glfwGetWindowSize(window_, nullptr, &height);
  if (height < 1) height = 1;
  mjv_moveCamera(model_, mjMOUSE_MOVE_V, dx / height, dy / height, &camera_);
}

void Viewer::zoom(double scrollY) {
  mjv_moveCamera(model_, mjMOUSE_ZOOM, 0, -0.05 * scrollY, &camera_);
}

void Viewer::addCapsule(Vec3 from, Vec3 to, double radius, const float rgba[4]) {
  if (scene_.ngeom >= scene_.maxgeom) return;
  mjvGeom* geom = scene_.geoms + scene_.ngeom;
  mjtNum size[3] = {radius, radius, radius};
  mjtNum pos[3] = {0, 0, 0};
  mjtNum mat[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  mjtNum a[3] = {from.x, from.y, from.z};
  mjtNum b[3] = {to.x, to.y, to.z};
  mjv_initGeom(geom, mjGEOM_CAPSULE, size, pos, mat, rgba);
  mjv_connector(geom, mjGEOM_CAPSULE, static_cast<mjtNum>(radius), a, b);
  setRgba(geom, rgba);
  geom->category = mjCAT_DECOR;
  ++scene_.ngeom;
}

void Viewer::addSphere(Vec3 center, double radius, const float rgba[4]) {
  if (scene_.ngeom >= scene_.maxgeom) return;
  mjvGeom* geom = scene_.geoms + scene_.ngeom;
  mjtNum size[3] = {radius, radius, radius};
  mjtNum pos[3] = {center.x, center.y, center.z};
  mjtNum mat[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  mjv_initGeom(geom, mjGEOM_SPHERE, size, pos, mat, rgba);
  geom->category = mjCAT_DECOR;
  ++scene_.ngeom;
}

void Viewer::addGrid() {
  constexpr double kExtent = 12.0;
  constexpr double kCell = 0.5;
  const int count = static_cast<int>(kExtent / kCell);
  for (int i = -count; i <= count; ++i) {
    const double at = i * kCell;
    const bool major = (i % 4) == 0;
    const float minor[4] = {0.55f, 0.55f, 0.55f, 1.f};
    const float majorRgba[4] = {0.38f, 0.38f, 0.38f, 1.f};
    const float* rgba = major ? majorRgba : minor;
    const double radius = major ? 0.008 : 0.0045;
    const double z = 0.004;
    addCapsule({-kExtent, at, z}, {kExtent, at, z}, radius, rgba);
    addCapsule({at, -kExtent, z}, {at, kExtent, z}, radius, rgba);
  }
}

void Viewer::addComMarker(Vec3 com) {
  const float marker[4] = {0.08f, 0.08f, 0.08f, 1.f};
  const float pole[4] = {0.22f, 0.22f, 0.22f, 1.f};
  addSphere(com, 0.035, marker);
  addSphere({com.x, com.y, 0.012}, 0.02, marker);
  if (com.z > 0.03) addCapsule({com.x, com.y, 0.012}, com, 0.006, pole);
}

void Viewer::addSupportMarker(Vec3 support) {
  const float marker[4] = {0.28f, 0.28f, 0.28f, 1.f};
  addSphere({support.x, support.y, 0.014}, 0.028, marker);
}

void Viewer::addCross(Vec3 point, const float rgba[4]) {
  const double z = 0.02;
  addCapsule({point.x - 0.045, point.y, z}, {point.x + 0.045, point.y, z}, 0.004, rgba);
  addCapsule({point.x, point.y - 0.045, z}, {point.x, point.y + 0.045, z}, 0.004, rgba);
}

void Viewer::addRing(Vec3 point, const float rgba[4]) {
  constexpr int count = 12;
  constexpr double radius = 0.045;
  const double z = 0.022;
  for (int i = 0; i < count; ++i) {
    const double a0 = 6.283185307179586 * static_cast<double>(i) / count;
    const double a1 = 6.283185307179586 * static_cast<double>(i + 1) / count;
    addCapsule({point.x + std::cos(a0) * radius, point.y + std::sin(a0) * radius, z},
               {point.x + std::cos(a1) * radius, point.y + std::sin(a1) * radius, z}, 0.0035, rgba);
  }
}

void Viewer::addPath(const Vec3* points, int count, const float rgba[4]) {
  for (int i = 1; i < count; ++i) addCapsule(points[i - 1], points[i], 0.004, rgba);
}

void Viewer::addJointAxes(const mjData* data) {
  if (!model_ || !data) return;
  for (int joint = 0; joint < model_->njnt; ++joint) {
    if (model_->jnt_type[joint] != mjJNT_HINGE) continue;
    Vec3 anchor{data->xanchor[3 * joint], data->xanchor[3 * joint + 1], data->xanchor[3 * joint + 2]};
    Vec3 axis{data->xaxis[3 * joint], data->xaxis[3 * joint + 1], data->xaxis[3 * joint + 2]};
    if (length(axis) < 1e-6) continue;
    axis = normalize(axis);
    const char* name = mj_id2name(model_, mjOBJ_JOINT, joint);
    const std::string label = name ? name : "";
    float rgba[4] = {0.12f, 0.12f, 0.12f, 1.f};
    if (label.find("twist") != std::string::npos) {
      rgba[0] = rgba[1] = rgba[2] = 0.62f;
    } else if (label.find("abd") != std::string::npos || label.find("roll") != std::string::npos ||
               label.find("yaw") != std::string::npos) {
      rgba[0] = rgba[1] = rgba[2] = 0.40f;
    }
    addCapsule(anchor - axis * 0.07, anchor + axis * 0.07, 0.006, rgba);
  }
}

void Viewer::render(mjData* data, const RenderFrame& frame) {
  if (!ready_ || !data) return;
  int width = 1;
  int height = 1;
  glfwGetFramebufferSize(window_, &width, &height);
  if (width < 1) width = 1;
  if (height < 1) height = 1;

  option_.geomgroup[3] = frame.drawColliders ? 1 : 0;
  mjv_updateScene(model_, data, &option_, nullptr, &camera_, mjCAT_ALL, &scene_);
  scene_.flags[mjRND_SHADOW] = 0;
  scene_.flags[mjRND_REFLECTION] = 0;
  scene_.flags[mjRND_SKYBOX] = 0;
  if (model_) {
    model_->vis.rgba.fog[0] = 0.93f;
    model_->vis.rgba.fog[1] = 0.93f;
    model_->vis.rgba.fog[2] = 0.93f;
    model_->vis.rgba.fog[3] = 1.f;
  }
  addGrid();
  if (frame.drawCom) addComMarker(frame.com);
  if (frame.drawSupport) addSupportMarker(frame.support);
  if (frame.drawCapture) {
    const float capture[4] = {0.15f, 0.15f, 0.15f, 1.f};
    addCross(frame.capture, capture);
  }
  if (frame.drawLanding) {
    const float landing[4] = {0.05f, 0.05f, 0.05f, 1.f};
    addRing(frame.landing, landing);
  }
  if (frame.swingPathCount > 1) {
    const float path[4] = {0.2f, 0.2f, 0.2f, 1.f};
    addPath(frame.swingPath, frame.swingPathCount, path);
  }
  if (frame.drawJointAxes) addJointAxes(data);

  mjrRect viewport{0, 0, width, height};
  mjr_render(viewport, &scene_, &context_);
}

bool Viewer::readPixels(std::vector<unsigned char>& rgb, int& width, int& height) {
  glfwGetFramebufferSize(window_, &width, &height);
  if (width < 1 || height < 1) return false;
  rgb.resize(static_cast<size_t>(width) * static_cast<size_t>(height) * 3);
  glDisable(GL_SCISSOR_TEST);
  mjrRect viewport{0, 0, width, height};
  mjr_readPixels(rgb.data(), nullptr, viewport, &context_);
  return true;
}

bool writeBmp(const std::filesystem::path& path, int width, int height, const unsigned char* rgbBottomUp) {
  if (!rgbBottomUp || width < 1 || height < 1) return false;
  const int rowStride = (width * 3 + 3) & ~3;
  const uint32_t pixelBytes = static_cast<uint32_t>(rowStride * height);
  const uint32_t fileSize = 54u + pixelBytes;
  std::vector<unsigned char> file(fileSize, 0);
  file[0] = 'B';
  file[1] = 'M';
  auto put32 = [&file](int offset, uint32_t value) {
    file[offset] = static_cast<unsigned char>(value & 255u);
    file[offset + 1] = static_cast<unsigned char>((value >> 8) & 255u);
    file[offset + 2] = static_cast<unsigned char>((value >> 16) & 255u);
    file[offset + 3] = static_cast<unsigned char>((value >> 24) & 255u);
  };
  put32(2, fileSize);
  put32(10, 54);
  put32(14, 40);
  put32(18, static_cast<uint32_t>(width));
  put32(22, static_cast<uint32_t>(height));
  file[26] = 1;
  file[28] = 24;
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      const unsigned char* source = rgbBottomUp + (static_cast<size_t>(y) * width + x) * 3;
      unsigned char* dest = file.data() + 54 + static_cast<size_t>(y) * rowStride + static_cast<size_t>(x) * 3;
      dest[0] = source[2];
      dest[1] = source[1];
      dest[2] = source[0];
    }
  }
  std::ofstream output(path, std::ios::binary);
  if (!output) return false;
  output.write(reinterpret_cast<const char*>(file.data()), static_cast<std::streamsize>(file.size()));
  return output.good();
}

}  // namespace pm
