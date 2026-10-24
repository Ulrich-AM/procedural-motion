#pragma once

#include "math_types.h"

#include <mujoco/mujoco.h>

#include <filesystem>
#include <string>
#include <vector>

struct GLFWwindow;

namespace pm {

struct RenderFrame {
  Vec3 com;
  bool drawCom = true;
  bool drawSupport = false;
  Vec3 support;
  bool drawStance = false;
  Vec3 stance;
  bool drawCapture = false;
  Vec3 capture;
  bool drawLanding = false;
  Vec3 landing;
  Vec3 swingPath[8]{};
  int swingPathCount = 0;
  bool drawVelocity = false;
  Vec3 desiredVelocity;
  Vec3 comVelocity;
  bool drawColliders = false;
  bool drawJointAxes = false;
};

class Viewer {
 public:
  bool init(GLFWwindow* window, mjModel* model, std::string& error);
  void shutdown();
  void setInitialTarget(Vec3 target);
  void resetCamera();
  void rotate(double dx, double dy);
  void pan(double dx, double dy);
  void zoom(double scrollY);
  void render(mjData* data, const RenderFrame& frame);
  bool readPixels(std::vector<unsigned char>& rgb, int& width, int& height);

  mjvCamera camera() const { return camera_; }

 private:
  void addCapsule(Vec3 from, Vec3 to, double radius, const float rgba[4]);
  void addSphere(Vec3 center, double radius, const float rgba[4]);
  void addGrid();
  void addComMarker(Vec3 com);
  void addSupportMarker(Vec3 support);
  void addCross(Vec3 point, const float rgba[4]);
  void addRing(Vec3 point, const float rgba[4]);
  void addPath(const Vec3* points, int count, const float rgba[4]);
  void addJointAxes(const mjData* data);

  GLFWwindow* window_ = nullptr;
  mjModel* model_ = nullptr;
  mjvCamera camera_{};
  mjvCamera initialCamera_{};
  mjvOption option_{};
  mjvScene scene_{};
  mjrContext context_{};
  bool ready_ = false;
};

bool writeBmp(const std::filesystem::path& path, int width, int height, const unsigned char* rgbBottomUp);

}  // namespace pm
