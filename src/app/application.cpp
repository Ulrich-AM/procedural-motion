#include "app/application.h"

#ifndef NOMINMAX
#define NOMINMAX
#endif

#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

#include <windows.h>

#include "tests/self_test.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <vector>

namespace pm {
namespace {

void mujocoWarning(const char* message) { std::cerr << "MuJoCo: " << message; }

std::filesystem::path executableDirectory() {
  wchar_t buffer[32768];
  const DWORD count = GetModuleFileNameW(nullptr, buffer, 32768);
  if (count == 0 || count >= 32768) return std::filesystem::current_path();
  return std::filesystem::path(buffer).parent_path();
}

std::filesystem::path findAsset(const std::filesystem::path& relative) {
  std::filesystem::path dir = executableDirectory();
  for (int i = 0; i < 6; ++i) {
    const std::filesystem::path candidate = dir / relative;
    if (std::filesystem::exists(candidate)) return candidate;
    if (!dir.has_parent_path() || dir.parent_path() == dir) break;
    dir = dir.parent_path();
  }
  const std::filesystem::path fromCwd = std::filesystem::current_path() / relative;
  if (std::filesystem::exists(fromCwd)) return fromCwd;
  return {};
}

void applyPlainStyle() {
  ImGuiStyle& style = ImGui::GetStyle();
  style.WindowRounding = 0;
  style.ChildRounding = 0;
  style.FrameRounding = 0;
  style.PopupRounding = 0;
  style.ScrollbarRounding = 0;
  style.GrabRounding = 0;
  style.TabRounding = 0;
  style.WindowBorderSize = 1;
  style.FrameBorderSize = 1;
  style.WindowPadding = ImVec2(8, 6);
  style.FramePadding = ImVec2(6, 3);
  style.ItemSpacing = ImVec2(8, 4);
  ImVec4* colors = style.Colors;
  colors[ImGuiCol_Text] = ImVec4(0.08f, 0.08f, 0.08f, 1);
  colors[ImGuiCol_WindowBg] = ImVec4(0.95f, 0.95f, 0.95f, 0.96f);
  colors[ImGuiCol_Border] = ImVec4(0.35f, 0.35f, 0.35f, 1);
  colors[ImGuiCol_FrameBg] = ImVec4(0.90f, 0.90f, 0.90f, 1);
  colors[ImGuiCol_FrameBgHovered] = ImVec4(0.84f, 0.84f, 0.84f, 1);
  colors[ImGuiCol_FrameBgActive] = ImVec4(0.78f, 0.78f, 0.78f, 1);
  colors[ImGuiCol_Button] = ImVec4(0.86f, 0.86f, 0.86f, 1);
  colors[ImGuiCol_ButtonHovered] = ImVec4(0.78f, 0.78f, 0.78f, 1);
  colors[ImGuiCol_ButtonActive] = ImVec4(0.70f, 0.70f, 0.70f, 1);
  colors[ImGuiCol_CheckMark] = ImVec4(0.10f, 0.10f, 0.10f, 1);
  colors[ImGuiCol_TitleBg] = colors[ImGuiCol_WindowBg];
  colors[ImGuiCol_TitleBgActive] = colors[ImGuiCol_WindowBg];
}

bool loadUiFont() {
  ImGuiIO& io = ImGui::GetIO();
  const char* fonts[] = {
      "C:\\Windows\\Fonts\\arial.ttf",
      "C:\\Windows\\Fonts\\calibri.ttf",
      "C:\\Windows\\Fonts\\segoeui.ttf",
  };
  for (const char* font : fonts) {
    if (std::filesystem::exists(font) && io.Fonts->AddFontFromFileTTF(font, 16.0f)) {
      std::cout << "UI font: " << font << "\n";
      return true;
    }
  }
  io.Fonts->AddFontDefault();
  std::cout << "UI font: ImGui default (Arial was not found)\n";
  return false;
}

int articulationCount(const mjModel* model) {
  int count = 0;
  for (int i = 1; i < model->nbody; ++i) {
    if (model->body_parentid[i] > 0) ++count;
  }
  return count;
}

}  // namespace

Application::~Application() { shutdown(); }

int Application::run(int argc, char** argv) {
  enum class Mode { Live, Test, Shot };
  Mode mode = Mode::Live;
  double runSeconds = 0;
  std::filesystem::path shotPrefix;

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--self-test") {
      mode = Mode::Test;
    } else if (arg == "--screenshot" && i + 1 < argc) {
      mode = Mode::Shot;
      shotPrefix = argv[++i];
    } else if (arg == "--run-seconds" && i + 1 < argc) {
      runSeconds = std::atof(argv[++i]);
    } else if (arg == "--help" || arg == "-h") {
      std::cout << "procedural_motion [--self-test] [--screenshot prefix] [--run-seconds n]\n";
      return 0;
    } else {
      std::cerr << "Unknown argument: " << arg << "\n";
      return 2;
    }
  }

  mju_user_warning = mujocoWarning;
  std::string error;
  if (!loadAssets(error)) {
    std::cerr << error << "\n";
    return 1;
  }
  if (mode == Mode::Test) return runSelfTest();
  if (!openWindow(error)) {
    std::cerr << error << "\n";
    return 1;
  }
  const int code = mode == Mode::Shot ? runScreenshot(shotPrefix) : runLoop(runSeconds);
  shutdownWindow();
  return code;
}

bool Application::loadAssets(std::string& error) {
  skeletonPath_ = findAsset("assets/generated/skeleton.json");
  if (skeletonPath_.empty()) {
    error = "Could not find assets/generated/skeleton.json. Build the project so the FBX importer runs.";
    return false;
  }
  const std::filesystem::path configPath = findAsset("config/lab.json");
  if (!configPath.empty()) {
    if (!loadSimConfig(configPath, config_, error)) return false;
    std::cout << "Config: " << configPath.string() << "\n";
  }
  if (!skeleton_.loadFromFile(skeletonPath_, error)) return false;

  std::cout << "Canonical skeleton (" << skeleton_.bones.size() << " bones, no animation curves):\n";
  for (const Bone& bone : skeleton_.bones) {
    std::cout << "  " << bone.name << "  parent=" << (bone.parent.empty() ? "-" : bone.parent)
              << "  source=" << bone.source << "  pos=(" << formatVec(bone.position) << ")\n";
  }
  std::cout << "Source hierarchy: " << (skeletonPath_.parent_path() / "hierarchy.txt").string() << "\n";
  std::cout << "Fixed timestep: " << config_.timestep << " s\n";

  if (!physics_.load(skeleton_, config_, skeletonPath_.parent_path() / "humanoid.xml")) {
    error = physics_.error();
    return false;
  }
  controller_.capture(skeleton_, physics_.sample());
  recoveryStep_.capture(skeleton_, physics_.sample());
  locomotion_.capture(skeleton_, physics_.sample());
  return true;
}

bool Application::openWindow(std::string& error) {
  glfwSetErrorCallback([](int code, const char* description) {
    std::cerr << "GLFW " << code << ": " << description << "\n";
  });
  if (!glfwInit()) {
    error = "glfwInit failed";
    return false;
  }
  glfwReady_ = true;
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_COMPAT_PROFILE);
  glfwWindowHint(GLFW_SAMPLES, 0);
  window_ = glfwCreateWindow(1440, 900, "Procedural Motion Lab", nullptr, nullptr);
  if (!window_) {
    error = "Could not create an OpenGL window";
    return false;
  }
  glfwMakeContextCurrent(window_);
  glfwSwapInterval(1);
  glfwSetWindowUserPointer(window_, this);
  glfwSetMouseButtonCallback(window_, &Application::onMouseButton);
  glfwSetCursorPosCallback(window_, &Application::onCursor);
  glfwSetScrollCallback(window_, &Application::onScroll);
  glfwSetKeyCallback(window_, &Application::onKey);

  if (!viewer_.init(window_, physics_.model(), error)) return false;

  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGuiIO& io = ImGui::GetIO();
  io.IniFilename = nullptr;
  io.ConfigFlags &= ~ImGuiConfigFlags_NavEnableKeyboard;
  loadUiFont();
  applyPlainStyle();
  ImGui_ImplGlfw_InitForOpenGL(window_, true);
  ImGui_ImplOpenGL3_Init("#version 150");
  imguiReady_ = true;

  const BodyState state = physics_.sample();
  Vec3 look{0, 0, 1};
  bool havePelvis = false;
  bool haveChest = false;
  Vec3 pelvis;
  Vec3 chest;
  for (const BodyKinematics& body : state.bodies) {
    if (body.name == "pelvis") {
      pelvis = body.position;
      havePelvis = true;
    }
    if (body.name == "chest") {
      chest = body.position;
      haveChest = true;
    }
  }
  if (havePelvis && haveChest) look = (pelvis + chest) * 0.5;
  else if (havePelvis) look = pelvis;
  viewer_.setInitialTarget(look);

  std::cout << "Controls: WASD walk, T active, Y recovery, V walking, R reset, Space pause, N step, P push\n";
  std::cout << "Mouse: left orbit, right pan, wheel zoom\n";
  return true;
}

void Application::shutdownWindow() { shutdown(); }

void Application::shutdown() {
  if (imguiReady_) {
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    imguiReady_ = false;
  }
  if (window_) glfwMakeContextCurrent(window_);
  viewer_.shutdown();
  if (window_) {
    glfwDestroyWindow(window_);
    window_ = nullptr;
  }
  if (glfwReady_) {
    glfwTerminate();
    glfwReady_ = false;
  }
}

void Application::drawUi(const BodyState& state) {
  const mjModel* model = physics_.model();
  ImGui::SetNextWindowPos(ImVec2(10, 10), ImGuiCond_Once);
  const ImGuiWindowFlags flags = ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_AlwaysAutoResize |
                                 ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoScrollbar |
                                 ImGuiWindowFlags_NoCollapse;
  ImGui::Begin("lab", nullptr, flags);
  ImGui::TextUnformatted("PROCEDURAL MOTION LAB");
  ImGui::TextUnformatted("Phase: 3 - Continuous Walking");
  ImGui::Text("Simulation: %s", paused_ ? "Paused" : "Running");
  ImGui::Text("Time: %.2f s", state.time);
  ImGui::Separator();
  ImGui::Text("Bodies: %d", model ? model->nbody - 1 : 0);
  ImGui::Text("Joints: %d", model ? articulationCount(model) : 0);
  ImGui::Text("Degrees of freedom: %d", model ? model->nv : 0);
  ImGui::Text("Contacts: %d", static_cast<int>(state.contacts.size()));
  ImGui::Text("Mass: %.1f kg", state.mass);
  ImGui::Text("COM: %.2f  %.2f  %.2f", state.centerOfMass.x, state.centerOfMass.y, state.centerOfMass.z);
  ImGui::Separator();
  if (ImGui::Button("Reset")) resetSimulation();
  ImGui::SameLine();
  if (ImGui::Button(paused_ ? "Resume" : "Pause")) setPaused(!paused_);
  ImGui::SameLine();
  if (ImGui::Button("Reset View")) viewer_.resetCamera();
  ImGui::Separator();
  if (ImGui::Checkbox("Gravity", &gravity_)) setGravity(gravity_);
  ImGui::SameLine();
  ImGui::TextUnformatted(gravity_ ? "On" : "Off");
  if (ImGui::Checkbox("Debug Colliders", &drawColliders_)) {
  }
  ImGui::SameLine();
  ImGui::TextUnformatted(drawColliders_ ? "On" : "Off");
  if (ImGui::Checkbox("Joint Axes", &drawAxes_)) {
  }
  ImGui::SameLine();
  ImGui::TextUnformatted(drawAxes_ ? "On" : "Off");
  if (ImGui::Checkbox("Active Control", &active_)) {
    if (active_) controller_.resetFilters();
    else physics_.clearJointTorques();
  }
  ImGui::SameLine();
  ImGui::TextUnformatted(active_ ? "On" : "Off");
  ImGui::SameLine();
  if (ImGui::Checkbox("Recovery Step", &recovery_)) {
    if (!recovery_) recoveryStep_.reset();
  }
  if (ImGui::Checkbox("Walking", &walking_)) {
    if (walking_) {
      active_ = true;
      controller_.resetFilters();
      locomotion_.capture(skeleton_, physics_.sample());
    } else {
      locomotion_.reset();
    }
  }
  ImGui::Text("Push %.0f N", pushForce_);
  ImGui::SliderFloat("##push", &pushForce_, 20.f, 250.f, "%.0f N");
  if (ImGui::Button("Forward")) requestPush(0);
  ImGui::SameLine();
  if (ImGui::Button("Back")) requestPush(1);
  ImGui::SameLine();
  if (ImGui::Button("Left")) requestPush(2);
  ImGui::SameLine();
  if (ImGui::Button("Right")) requestPush(3);
  ImGui::Separator();
  Vec3 forward = skeleton_.forward;
  Vec3 right = skeleton_.right;
  forward.z = 0;
  right.z = 0;
  forward = normalize(forward);
  right = normalize(right);
  const Vec3 comVelocity = state.centerOfMassVelocity;
  const double desiredX = dot(desiredVelocity_, right);
  const double desiredY = dot(desiredVelocity_, forward);
  const double actualX = dot(comVelocity, right);
  const double actualY = dot(comVelocity, forward);
  const char* loco = walking_ ? locoStateLabel(locoReport_.state) : (recovery_ ? stepStateLabel(recoveryReport_.state) : "Standing");
  const std::string& swing = walking_ ? locoReport_.swingFoot : recoveryReport_.swingFoot;
  const double phase = walking_ ? locoReport_.phase : recoveryReport_.phase;
  const double distance = walking_ ? locoReport_.stepDistance : recoveryReport_.stepDistance;
  const double duration = walking_ ? locoReport_.duration : recoveryReport_.duration;
  const double captureError = walking_ ? locoReport_.captureError : recoveryReport_.captureError;
  ImGui::Text("Active Control: %s", active_ ? "On" : "Off");
  ImGui::Text("Walking: %s", walking_ ? "On" : "Off");
  ImGui::Text("Locomotion State: %s", loco);
  ImGui::Text("Desired Velocity X/Y: %.2f  %.2f", desiredX, desiredY);
  ImGui::Text("Actual COM Velocity X/Y: %.2f  %.2f", actualX, actualY);
  ImGui::Text("Swing Foot: %s", swing.empty() ? "None" : swing.c_str());
  ImGui::Text("Step Phase: %.2f", phase);
  ImGui::Text("Step Distance: %.3f", distance);
  ImGui::Text("Step Duration: %.2f", duration);
  ImGui::Text("Capture Error: %.3f", captureError);
  ImGui::Text("Left Foot Contact State: %s", footContactLabel(contacts_.left().mode));
  ImGui::Text("Right Foot Contact State: %s", footContactLabel(contacts_.right().mode));
  ImGui::Text("Peak Controller Torque: %.1f Nm", active_ ? report_.peakTorque : 0.0);
  ImGui::Separator();
  ImGui::TextUnformatted("WASD walk   V walking   T active   Y recovery");
  ImGui::TextUnformatted("R reset   Space pause   N step   P push");
  ImGui::TextUnformatted("G gravity   C colliders   J axes   Home view");
  ImGui::TextUnformatted("Mouse: orbit, right-pan, wheel zoom");
  ImGui::End();
}

void Application::drawFrame() {
  const BodyState state = physics_.sample();
  report_ = controller_.diagnose(state);
  report_.active = active_;
  report_.peakTorque = active_ ? controller_.peakTorque() : 0;
  RenderFrame frame;
  frame.com = state.centerOfMass;
  frame.drawCom = true;
  frame.drawSupport = report_.supportValid;
  frame.support = report_.supportCenter;
  const bool showWalk = walking_ && active_;
  const bool showRecovery = recovery_ && active_ && !walking_;
  frame.drawCapture = showWalk || showRecovery;
  frame.capture = showWalk ? locoReport_.capture : recoveryReport_.capture;
  frame.drawLanding = showWalk ? locoReport_.landingValid : (showRecovery && recoveryReport_.landingValid);
  frame.landing = showWalk ? locoReport_.landing : recoveryReport_.landing;
  frame.swingPathCount = showWalk ? locoReport_.pathCount : (showRecovery ? recoveryReport_.pathCount : 0);
  for (int i = 0; i < frame.swingPathCount; ++i) {
    frame.swingPath[i] = showWalk ? locoReport_.path[i] : recoveryReport_.path[i];
  }
  frame.drawVelocity = active_;
  frame.desiredVelocity = desiredVelocity_;
  frame.comVelocity = state.centerOfMassVelocity;
  frame.drawColliders = drawColliders_;
  frame.drawJointAxes = drawAxes_;
  viewer_.render(physics_.data(), frame);

  ImGui_ImplOpenGL3_NewFrame();
  ImGui_ImplGlfw_NewFrame();
  ImGui::NewFrame();
  drawUi(state);
  ImGui::Render();
  ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}

int Application::runLoop(double maxSeconds) {
  const double start = glfwGetTime();
  double previous = start;
  while (!glfwWindowShouldClose(window_)) {
    if (maxSeconds > 0 && glfwGetTime() - start >= maxSeconds) break;
    glfwPollEvents();
    const double now = glfwGetTime();
    double frame = now - previous;
    previous = now;
    if (frame < 0) frame = 0;
    if (frame > 0.1) frame = 0.1;

    if (!paused_) {
      accumulator_ += frame;
      int steps = 0;
      while (accumulator_ >= config_.timestep && steps < config_.maxSubsteps) {
        stepPhysics();
        accumulator_ -= config_.timestep;
        ++steps;
      }
    } else {
      accumulator_ = 0;
      if (singleStep_) {
        stepPhysics();
        singleStep_ = false;
      }
    }

    drawFrame();
    glfwSwapBuffers(window_);
  }
  return 0;
}

int Application::runScreenshot(const std::filesystem::path& prefix) {
  auto save = [&](const char* suffix) {
    drawFrame();
    std::vector<unsigned char> rgb;
    int width = 0;
    int height = 0;
    if (!viewer_.readPixels(rgb, width, height)) return false;
    const std::filesystem::path path = prefix.string() + suffix;
    if (!writeBmp(path, width, height, rgb.data())) {
      std::cerr << "Could not write " << path.string() << "\n";
      return false;
    }
    std::cout << "Wrote " << path.string() << "\n";
    glfwSwapBuffers(window_);
    return true;
  };
  glfwPollEvents();
  if (!save("_rest.bmp")) return 1;
  const int steps = std::max(1, static_cast<int>(std::lround(3.0 / config_.timestep)));
  for (int i = 0; i < steps; ++i) physics_.step();
  if (!save("_settled.bmp")) return 1;
  return 0;
}

int Application::runSelfTest() {
  return runProceduralSelfTest(physics_, controller_, recoveryStep_, locomotion_, contacts_, servo_, skeleton_);
}

void Application::updateDesired() {
  Vec3 forward = skeleton_.forward;
  Vec3 right = skeleton_.right;
  forward.z = 0;
  right.z = 0;
  forward = normalize(forward);
  right = normalize(right);
  Vec3 desired{};
  if (keyW_) desired = desired + forward;
  if (keyS_) desired = desired - forward;
  if (keyD_) desired = desired + right;
  if (keyA_) desired = desired - right;
  if (length(desired) > 1.0) desired = normalize(desired);
  desiredVelocity_ = walking_ ? desired * 1.4 : Vec3{};
}

void Application::stepPhysics() {
  updateDesired();
  ControlInput input;
  input.active = active_;
  input.recovery = recovery_ && !walking_;
  input.walking = walking_;
  input.desired = desiredVelocity_;
  input.dt = physics_.timestep() > 0 ? physics_.timestep() : config_.timestep;
  const ControlOutput output = applyControl(input, controller_, recoveryStep_, locomotion_, contacts_, servo_, physics_);
  report_ = output.standing;
  recoveryReport_ = output.recovery;
  locoReport_ = output.locomotion;
  report_.peakTorque = output.peakTorque;
  physics_.step();
}

void Application::requestPush(int direction) {
  const BodyState state = physics_.sample();
  Vec3 forward = skeleton_.forward;
  Vec3 right = skeleton_.right;
  for (const BodyKinematics& body : state.bodies) {
    if (body.name != "pelvis") continue;
    const double* rotation = body.rotation;
    auto rotate = [&](Vec3 local) {
      return Vec3{rotation[0] * local.x + rotation[1] * local.y + rotation[2] * local.z,
                  rotation[3] * local.x + rotation[4] * local.y + rotation[5] * local.z, 0};
    };
    forward = rotate(skeleton_.forward);
    right = rotate(skeleton_.right);
    break;
  }
  forward.z = 0;
  right.z = 0;
  forward = normalize(forward);
  right = normalize(right);
  if (length(forward) < 0.5) forward = normalize(skeleton_.forward);
  if (length(right) < 0.5) right = normalize(skeleton_.right);
  Vec3 axis = forward;
  if (direction == 1) axis = forward * -1;
  if (direction == 2) axis = right * -1;
  if (direction == 3) axis = right;
  const double force = std::clamp(static_cast<double>(pushForce_), 20.0, 250.0);
  physics_.startBodyForcePulse("chest", axis * force, 0.12);
}

void Application::resetSimulation() {
  physics_.reset();
  controller_.capture(skeleton_, physics_.sample());
  recoveryStep_.capture(skeleton_, physics_.sample());
  locomotion_.capture(skeleton_, physics_.sample());
  contacts_.reset();
  recoveryReport_ = {};
  locoReport_ = {};
  accumulator_ = 0;
}

void Application::setPaused(bool paused) {
  paused_ = paused;
  accumulator_ = 0;
}

void Application::setGravity(bool enabled) {
  gravity_ = enabled;
  physics_.setGravityEnabled(enabled);
}

void Application::onMouseButton(GLFWwindow* window, int button, int action, int) {
  auto* app = static_cast<Application*>(glfwGetWindowUserPointer(window));
  if (!app) return;
  if (ImGui::GetIO().WantCaptureMouse) {
    app->rotating_ = false;
    app->panning_ = false;
    return;
  }
  const bool pressed = action == GLFW_PRESS;
  if (button == GLFW_MOUSE_BUTTON_LEFT) app->rotating_ = pressed;
  if (button == GLFW_MOUSE_BUTTON_RIGHT || button == GLFW_MOUSE_BUTTON_MIDDLE) app->panning_ = pressed;
}

void Application::onCursor(GLFWwindow* window, double x, double y) {
  auto* app = static_cast<Application*>(glfwGetWindowUserPointer(window));
  if (!app) return;
  if (!app->cursorReady_) {
    app->cursorX_ = x;
    app->cursorY_ = y;
    app->cursorReady_ = true;
    return;
  }
  const double dx = x - app->cursorX_;
  const double dy = y - app->cursorY_;
  app->cursorX_ = x;
  app->cursorY_ = y;
  if (ImGui::GetIO().WantCaptureMouse) return;
  if (app->rotating_) app->viewer_.rotate(dx, dy);
  if (app->panning_) app->viewer_.pan(dx, dy);
}

void Application::onScroll(GLFWwindow* window, double, double yOffset) {
  auto* app = static_cast<Application*>(glfwGetWindowUserPointer(window));
  if (!app || ImGui::GetIO().WantCaptureMouse) return;
  app->viewer_.zoom(yOffset);
}

void Application::onKey(GLFWwindow* window, int key, int, int action, int) {
  auto* app = static_cast<Application*>(glfwGetWindowUserPointer(window));
  if (!app || action == GLFW_REPEAT) return;
  if (ImGui::GetIO().WantCaptureKeyboard) return;
  if (key == GLFW_KEY_W) app->keyW_ = action != GLFW_RELEASE;
  if (key == GLFW_KEY_A) app->keyA_ = action != GLFW_RELEASE;
  if (key == GLFW_KEY_S) app->keyS_ = action != GLFW_RELEASE;
  if (key == GLFW_KEY_D) app->keyD_ = action != GLFW_RELEASE;
  if (action != GLFW_PRESS) return;
  if (key == GLFW_KEY_R) app->resetSimulation();
  if (key == GLFW_KEY_SPACE) app->setPaused(!app->paused_);
  if (key == GLFW_KEY_N && app->paused_) app->singleStep_ = true;
  if (key == GLFW_KEY_G) app->setGravity(!app->gravity_);
  if (key == GLFW_KEY_C) app->drawColliders_ = !app->drawColliders_;
  if (key == GLFW_KEY_J) app->drawAxes_ = !app->drawAxes_;
  if (key == GLFW_KEY_HOME) app->viewer_.resetCamera();
  if (key == GLFW_KEY_T) {
    app->active_ = !app->active_;
    if (app->active_) app->controller_.resetFilters();
    else app->physics_.clearJointTorques();
  }
  if (key == GLFW_KEY_Y) {
    app->recovery_ = !app->recovery_;
    if (!app->recovery_) app->recoveryStep_.reset();
  }
  if (key == GLFW_KEY_V) {
    app->walking_ = !app->walking_;
    if (app->walking_) {
      app->active_ = true;
      app->controller_.resetFilters();
      app->locomotion_.capture(app->skeleton_, app->physics_.sample());
    } else {
      app->locomotion_.reset();
    }
  }
  if (key == GLFW_KEY_P) app->requestPush(0);
}

}  // namespace pm
