#pragma once

#include "app/sim_config.h"
#include "physics/physical_skeleton.h"
#include "render/viewer.h"
#include "skeleton/skeleton_definition.h"

#include <filesystem>
#include <string>

struct GLFWwindow;

namespace pm {

class Application {
 public:
  Application() = default;
  ~Application();

  Application(const Application&) = delete;
  Application& operator=(const Application&) = delete;

  int run(int argc, char** argv);

 private:
  bool loadAssets(std::string& error);
  bool openWindow(std::string& error);
  void shutdownWindow();
  void shutdown();
  int runLoop(double maxSeconds);
  int runSelfTest();
  int runScreenshot(const std::filesystem::path& prefix);
  void drawFrame();
  void drawUi(const BodyState& state);
  void resetSimulation();
  void setPaused(bool paused);
  void setGravity(bool enabled);

  static void onMouseButton(GLFWwindow* window, int button, int action, int mods);
  static void onCursor(GLFWwindow* window, double x, double y);
  static void onScroll(GLFWwindow* window, double xOffset, double yOffset);
  static void onKey(GLFWwindow* window, int key, int scancode, int action, int mods);

  SkeletonDefinition skeleton_;
  SimConfig config_;
  PhysicalSkeleton physics_;
  Viewer viewer_;
  GLFWwindow* window_ = nullptr;
  bool glfwReady_ = false;
  bool imguiReady_ = false;
  bool paused_ = false;
  bool singleStep_ = false;
  bool drawColliders_ = false;
  bool drawAxes_ = false;
  bool gravity_ = true;
  double accumulator_ = 0;
  double cursorX_ = 0;
  double cursorY_ = 0;
  bool cursorReady_ = false;
  bool rotating_ = false;
  bool panning_ = false;
  std::filesystem::path skeletonPath_;
};

}  // namespace pm
