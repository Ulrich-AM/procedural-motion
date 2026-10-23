#include "app/application.h"

#ifndef NOMINMAX
#define NOMINMAX
#endif

#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

#include <windows.h>

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

bool finiteArray(const mjtNum* values, int count) {
  for (int i = 0; i < count; ++i) {
    if (!std::isfinite(values[i])) return false;
  }
  return true;
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

  std::cout << "Controls: R reset, Space pause, N step, G gravity, C colliders, J axes, Home reset view\n";
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
  ImGui::TextUnformatted("Phase: 0 - Physical Humanoid");
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
  ImGui::Separator();
  ImGui::TextUnformatted("R reset   Space pause   N step");
  ImGui::TextUnformatted("G gravity   C colliders   J axes");
  ImGui::TextUnformatted("Home reset view");
  ImGui::TextUnformatted("Mouse: orbit, right-pan, wheel zoom");
  ImGui::End();
}

void Application::drawFrame() {
  const BodyState state = physics_.sample();
  RenderFrame frame;
  frame.com = state.centerOfMass;
  frame.drawCom = true;
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
        physics_.step();
        accumulator_ -= config_.timestep;
        ++steps;
      }
    } else {
      accumulator_ = 0;
      if (singleStep_) {
        physics_.step();
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
  mjModel* model = physics_.model();
  mjData* data = physics_.data();
  auto fail = [](const std::string& message) {
    std::cerr << "SELF-TEST FAIL: " << message << "\n";
    return 1;
  };

  if (model->nbody - 1 != 16) return fail("expected 16 bodies, got " + std::to_string(model->nbody - 1));
  if (articulationCount(model) != 15) return fail("expected 15 body joints");
  if (model->nu != 0) return fail("model has actuators");
  const int root = mj_name2id(model, mjOBJ_JOINT, "root");
  if (root < 0 || model->jnt_type[root] != mjJNT_FREE) return fail("root is not free");

  const double mass = mj_getTotalmass(model);
  std::cout << "Self-test total mass " << mass << " kg\n";
  if (mass < 68.0 || mass > 82.0) return fail("total mass outside 68-82 kg");

  const char* limitedJoints[] = {"left_knee", "right_knee", "left_elbow", "right_elbow"};
  for (const char* name : limitedJoints) {
    const int joint = mj_name2id(model, mjOBJ_JOINT, name);
    if (joint < 0 || !model->jnt_limited[joint]) return fail(std::string("missing limited joint ") + name);
    const double lo = std::abs(model->jnt_range[2 * joint]);
    const double hi = std::abs(model->jnt_range[2 * joint + 1]);
    const double extension = std::min(lo, hi);
    const double flexion = std::max(lo, hi);
    if (extension > 25.0 * 3.141592653589793 / 180.0 || flexion < 90.0 * 3.141592653589793 / 180.0) {
      return fail(std::string("joint range is not hinge-like: ") + name);
    }
  }

  physics_.reset();
  std::vector<mjtNum> initial(static_cast<size_t>(model->nq));
  std::copy(data->qpos, data->qpos + model->nq, initial.begin());
  const double timeStep = model->opt.timestep;
  physics_.step();
  if (std::abs(data->time - timeStep) > 1e-8) return fail("single step did not advance one timestep");
  physics_.reset();
  if (std::abs(data->time) > 1e-12) return fail("reset did not clear time");
  for (int i = 0; i < model->nq; ++i) {
    if (std::abs(data->qpos[i] - initial[static_cast<size_t>(i)]) > 1e-8) return fail("reset pose mismatch");
  }
  for (int i = 0; i < model->nv; ++i) {
    if (std::abs(data->qvel[i]) > 1e-10) return fail("reset left a nonzero velocity");
  }

  const int pelvis = mj_name2id(model, mjOBJ_BODY, "pelvis");
  const double startZ = data->xpos[3 * pelvis + 2];
  const int airStep = std::max(1, static_cast<int>(std::lround(0.08 / timeStep)));
  const int totalSteps = std::max(airStep + 1, static_cast<int>(std::lround(4.0 / timeStep)));
  bool falling = false;
  for (int step = 1; step <= totalSteps; ++step) {
    physics_.step();
    if (!finiteArray(data->qpos, model->nq) || !finiteArray(data->qvel, model->nv)) return fail("non-finite state");
    for (int b = 1; b < model->nbody; ++b) {
      const double x = data->xpos[3 * b];
      const double y = data->xpos[3 * b + 1];
      const double z = data->xpos[3 * b + 2];
      if (std::abs(x) > 4 || std::abs(y) > 4 || z < -0.25 || z > 4) {
        return fail(std::string("body left the lab: ") + mj_id2name(model, mjOBJ_BODY, b));
      }
    }
    double peak = 0;
    for (int i = 0; i < model->nv; ++i) peak = std::max(peak, std::abs(data->qvel[i]));
    if (peak > 200) return fail("velocity exploded");
    if (step == airStep) {
      mjtNum velocity[6] = {};
      mj_objectVelocity(model, data, mjOBJ_BODY, pelvis, velocity, 0);
      std::cout << "t=" << data->time << " pelvis angular (" << velocity[0] << " " << velocity[1] << " "
                << velocity[2] << ") linear (" << velocity[3] << " " << velocity[4] << " " << velocity[5] << ")\n";
      if (velocity[5] < -0.35 && velocity[5] > -4.0) falling = true;
    }
  }
  if (!falling) return fail("pelvis linear velocity did not show a fall");
  const double endZ = data->xpos[3 * pelvis + 2];
  std::cout << "Pelvis z " << startZ << " -> " << endZ << ", contacts " << data->ncon << "\n";
  if (endZ > startZ - 0.30) return fail("pelvis did not drop under gravity");
  if (data->ncon <= 0) return fail("no floor contacts after the fall");
  double finalSpeed = 0;
  for (int i = 0; i < model->nv; ++i) finalSpeed = std::max(finalSpeed, std::abs(data->qvel[i]));
  if (finalSpeed > 40) return fail("simulation still unstable after 4s");

  for (int i = 0; i < mjNWARNING; ++i) {
    if (data->warning[i].number > 0) {
      std::cout << "MuJoCo warning " << i << " count " << data->warning[i].number << "\n";
    }
  }

  const BodyState state = physics_.sample();
  const Vec3 subtree{data->subtree_com[3 * pelvis], data->subtree_com[3 * pelvis + 1],
                     data->subtree_com[3 * pelvis + 2]};
  if (length(state.centerOfMass - subtree) > 1e-3) return fail("center of mass does not match the pelvis subtree");

  physics_.reset();
  for (int i = 0; i < model->nq; ++i) {
    if (std::abs(data->qpos[i] - initial[static_cast<size_t>(i)]) > 1e-8) return fail("final reset pose mismatch");
  }
  for (int i = 0; i < model->nv; ++i) {
    if (std::abs(data->qvel[i]) > 1e-10) return fail("final reset velocity mismatch");
  }
  std::cout << "SELF-TEST PASS\n";
  return 0;
}

void Application::resetSimulation() {
  physics_.reset();
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
  if (!app || action == GLFW_RELEASE) return;
  if (ImGui::GetIO().WantCaptureKeyboard) return;
  if (key == GLFW_KEY_R && action == GLFW_PRESS) app->resetSimulation();
  if (key == GLFW_KEY_SPACE && action == GLFW_PRESS) app->setPaused(!app->paused_);
  if (key == GLFW_KEY_N && app->paused_) app->singleStep_ = true;
  if (key == GLFW_KEY_G && action == GLFW_PRESS) app->setGravity(!app->gravity_);
  if (key == GLFW_KEY_C && action == GLFW_PRESS) app->drawColliders_ = !app->drawColliders_;
  if (key == GLFW_KEY_J && action == GLFW_PRESS) app->drawAxes_ = !app->drawAxes_;
  if (key == GLFW_KEY_HOME && action == GLFW_PRESS) app->viewer_.resetCamera();
}

}  // namespace pm
