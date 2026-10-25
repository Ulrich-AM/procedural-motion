#include "app/application.h"

#include "body/joint_limits.h"
#include "body/proc_body.h"
#include "motion/motion_player.h"
#include "render/renderer.h"
#include "render/skeleton_renderer.h"
#include "rig/forward_kinematics.h"
#include "app/analysis_panel.h"
#include "app/import_panel.h"
#include "retarget/retargeter.h"
#include "validation/bone_length_monitor.h"
#include "validation/tolerances.h"

#define NOMINMAX
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <commdlg.h>
#include <shellapi.h>

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>
#define GLFW_EXPOSE_NATIVE_WIN32
#include <GLFW/glfw3native.h>

#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

#include <glm/gtc/matrix_transform.hpp>

#include <GL/gl.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <string>
#include <vector>

namespace proc {
namespace {

constexpr ImGuiWindowFlags kPanel = ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
                                    ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoSavedSettings;

std::filesystem::path pathFromUtf8(const char* text) {
  return std::filesystem::path(std::u8string(reinterpret_cast<const char8_t*>(text)));
}

std::filesystem::path findAssetRoot() {
  std::vector<std::filesystem::path> seeds;
  seeds.push_back(std::filesystem::current_path());
  wchar_t executable[MAX_PATH];
  const DWORD length = GetModuleFileNameW(nullptr, executable, MAX_PATH);
  if (length > 0) seeds.push_back(std::filesystem::path(executable).parent_path());
  for (std::filesystem::path seed : seeds) {
    for (int step = 0; step < 6 && !seed.empty(); ++step) {
      const std::filesystem::path candidate = seed / "assets" / "bodies" / "neutral.procbody.json";
      if (std::filesystem::exists(candidate)) return seed;
      seed = seed.parent_path();
    }
  }
  return {};
}

void applyTheme(float scale) {
  ImGuiStyle& style = ImGui::GetStyle();
  ImGui::StyleColorsDark();
  style.WindowRounding = 0.0f;
  style.ChildRounding = 0.0f;
  style.FrameRounding = 0.0f;
  style.PopupRounding = 0.0f;
  style.ScrollbarRounding = 0.0f;
  style.GrabRounding = 0.0f;
  style.TabRounding = 0.0f;
  style.WindowBorderSize = 1.0f;
  style.FrameBorderSize = 1.0f;
  style.WindowPadding = ImVec2(8.0f, 8.0f);
  style.FramePadding = ImVec2(6.0f, 3.0f);
  style.ItemSpacing = ImVec2(6.0f, 4.0f);
  style.ScaleAllSizes(scale);
  ImVec4* colors = style.Colors;
  const auto gray = [](float value, float alpha = 1.0f) { return ImVec4(value, value, value, alpha); };
  colors[ImGuiCol_Text] = gray(0.90f);
  colors[ImGuiCol_TextDisabled] = gray(0.50f);
  colors[ImGuiCol_WindowBg] = gray(0.18f);
  colors[ImGuiCol_ChildBg] = gray(0.13f);
  colors[ImGuiCol_PopupBg] = gray(0.16f);
  colors[ImGuiCol_Border] = gray(0.40f);
  colors[ImGuiCol_FrameBg] = gray(0.10f);
  colors[ImGuiCol_FrameBgHovered] = gray(0.22f);
  colors[ImGuiCol_FrameBgActive] = gray(0.28f);
  colors[ImGuiCol_TitleBg] = gray(0.12f);
  colors[ImGuiCol_TitleBgActive] = gray(0.16f);
  colors[ImGuiCol_Button] = gray(0.24f);
  colors[ImGuiCol_ButtonHovered] = gray(0.36f);
  colors[ImGuiCol_ButtonActive] = gray(0.50f);
  colors[ImGuiCol_Header] = gray(0.28f);
  colors[ImGuiCol_HeaderHovered] = gray(0.36f);
  colors[ImGuiCol_HeaderActive] = gray(0.46f);
  colors[ImGuiCol_SliderGrab] = gray(0.74f);
  colors[ImGuiCol_SliderGrabActive] = gray(0.90f);
  colors[ImGuiCol_CheckMark] = gray(0.92f);
  colors[ImGuiCol_ScrollbarBg] = gray(0.10f);
  colors[ImGuiCol_ScrollbarGrab] = gray(0.34f);
  colors[ImGuiCol_Separator] = gray(0.42f);
}

}  // namespace

struct LabState {
  GLFWwindow* window = nullptr;
  Renderer renderer;
  ProcBody body;
  ProcMotion motion;
  MotionPlayer player;
  Status body_status;
  Status motion_status;
  bool has_body = false;
  bool has_motion = false;
  LimitMode limit_mode = LimitMode::Warn;
  std::filesystem::path asset_root;
  std::filesystem::path body_path;
  std::filesystem::path motion_path;
  bool show_trajectory = false;
  bool show_joint_labels = false;
  bool show_bone_labels = false;
  double yaw = 0.85;
  double pitch = 0.42;
  double distance = 3.8;
  glm::dvec3 target{0.0, 0.0, 0.95};
  bool dragging = false;
  double last_x = 0.0;
  double last_y = 0.0;
  double scroll = 0.0;
  float view_x = 0;
  float view_y = 0;
  float view_w = 1;
  float view_h = 1;
  int speed_index = 2;
  std::vector<glm::vec3> trajectory;
  WorldPose world;
  BoneLengthReport bones;
  LimitResult limits;
  bool pose_ready = false;
  ImportSession import;
  AnalysisUi analysis_ui;
  glm::mat4 view{1.0f};
  glm::mat4 projection{1.0f};
};

LabState* labFrom(GLFWwindow* window) { return static_cast<LabState*>(glfwGetWindowUserPointer(window)); }

void rebuildTrajectory(LabState& lab) {
  lab.trajectory.clear();
  if (!lab.show_trajectory || !lab.has_body || !lab.has_motion || lab.motion.skeleton != lab.body.skeleton) return;
  const int steps = std::max(2, lab.motion.frame_count * 2);
  for (int step = 0; step <= steps; ++step) {
    const double time_s = lab.motion.duration_s * static_cast<double>(step) / static_cast<double>(steps);
    const Pose pose = sampleMotion(lab.motion, time_s, false);
    const glm::dvec3 position = lab.body.joints[kRootJoint].rest_position + pose.root_translation;
    lab.trajectory.emplace_back(static_cast<float>(position.x), static_cast<float>(position.y), static_cast<float>(position.z));
  }
}

void loadBody(LabState& lab, const std::filesystem::path& path) {
  ProcBody body;
  Status status = loadProcBodyFromFile(path, body);
  lab.body_status = status;
  lab.body_path = path;
  if (!status.ok()) {
    lab.has_body = false;
    lab.pose_ready = false;
    return;
  }
  lab.body = std::move(body);
  lab.has_body = true;
  lab.analysis_ui.valid = false;
  lab.target = lab.body.joints[kRootJoint].rest_position;
  if (lab.import.has_source) {
    retargetCurrent(lab.import, lab.body, lab.motion, lab.has_motion, lab.motion_path, lab.motion_status, lab.player);
  } else if (lab.has_motion && lab.motion.skeleton != lab.body.skeleton) {
    Status mismatch;
    mismatch.error("motion skeleton does not match the loaded body");
    lab.motion_status = mismatch;
  }
  rebuildTrajectory(lab);
}

void loadMotion(LabState& lab, const std::filesystem::path& path) {
  ProcMotion motion;
  Status status = loadProcMotionFromFile(path, motion);
  lab.motion_status = status;
  lab.motion_path = path;
  lab.analysis_ui.valid = false;
  if (!status.ok()) {
    lab.has_motion = false;
    lab.player.setMotion(nullptr);
    return;
  }
  if (lab.has_body && motion.skeleton != lab.body.skeleton) {
    status.error("motion skeleton " + motion.skeleton + " does not match body skeleton " + lab.body.skeleton);
    lab.motion_status = status;
    lab.has_motion = false;
    lab.player.setMotion(nullptr);
    return;
  }
  lab.motion = std::move(motion);
  lab.has_motion = true;
  lab.player.setMotion(&lab.motion);
  lab.player.setLoop(lab.player.loop());
  rebuildTrajectory(lab);
}

std::filesystem::path openDialog(GLFWwindow* window, const wchar_t* filter, const wchar_t* title,
                                 const std::filesystem::path& initial) {
  wchar_t buffer[1024] = {};
  OPENFILENAMEW dialog{};
  dialog.lStructSize = sizeof(dialog);
  dialog.hwndOwner = glfwGetWin32Window(window);
  dialog.lpstrFilter = filter;
  dialog.lpstrFile = buffer;
  dialog.nMaxFile = 1024;
  dialog.lpstrTitle = title;
  const std::wstring directory = initial.wstring();
  dialog.lpstrInitialDir = directory.c_str();
  dialog.Flags = OFN_FILEMUSTEXIST | OFN_PATHMUSTEXIST | OFN_NOCHANGEDIR;
  if (!GetOpenFileNameW(&dialog)) return {};
  return std::filesystem::path(buffer);
}

glm::mat4 cameraView(const LabState& lab) {
  const double cp = std::cos(lab.pitch);
  const double sp = std::sin(lab.pitch);
  const double cy = std::cos(lab.yaw);
  const double sy = std::sin(lab.yaw);
  const glm::dvec3 eye = lab.target + lab.distance * glm::dvec3(cp * cy, cp * sy, sp);
  return glm::lookAt(glm::vec3(eye), glm::vec3(lab.target), glm::vec3(0.0f, 0.0f, 1.0f));
}

void setCameraPreset(LabState& lab, double yaw, double pitch) {
  lab.yaw = yaw;
  lab.pitch = pitch;
  lab.distance = 3.8;
  if (lab.has_body) lab.target = lab.body.joints[kRootJoint].rest_position;
}

bool cursorInViewport(const LabState& lab) {
  double x = 0.0;
  double y = 0.0;
  glfwGetCursorPos(lab.window, &x, &y);
  return x >= lab.view_x && x < lab.view_x + lab.view_w && y >= lab.view_y && y < lab.view_y + lab.view_h;
}

void updatePose(LabState& lab) {
  lab.pose_ready = false;
  if (!lab.has_body) return;
  Pose pose;
  if (lab.has_motion && lab.motion.skeleton == lab.body.skeleton) pose = lab.player.sample();
  lab.limits = applyJointLimits(lab.body, pose, lab.limit_mode);
  lab.world = forwardKinematics(lab.body, lab.limits.pose);
  lab.bones = measureBoneLengths(lab.body, lab.world, kRuntimeBoneToleranceM);
  lab.pose_ready = true;
  if (lab.import.has_source) {
    lab.import.report.direction_max_error_deg =
        segmentDirectionError(lab.import.clip, lab.player.time(), lab.body, lab.world, lab.import.mapping,
                              lab.import.report.direction_worst_bone);
  }
}

void section(const char* title) {
  ImGui::Spacing();
  ImGui::TextUnformatted(title);
  ImGui::Separator();
}

void drawSidePanel(LabState& lab) {
  section("BODY");
  ImGui::TextWrapped("%s", lab.body_path.empty() ? "no file" : lab.body_path.filename().string().c_str());
  if (ImGui::Button("Load body")) {
    const std::filesystem::path path =
        openDialog(lab.window, L"PROC body\0*.procbody.json;*.json\0\0", L"Load PROC BODY", lab.asset_root / "assets" / "bodies");
    if (!path.empty()) loadBody(lab, path);
  }
  ImGui::SameLine();
  if (ImGui::Button("Reload body") && !lab.body_path.empty()) loadBody(lab, lab.body_path);
  if (lab.has_body) {
    ImGui::Text("skeleton %s", lab.body.skeleton.c_str());
    ImGui::Text("joints %d", kJointCount);
    ImGui::Text("bones %d", kBoneCount);
    ImGui::Text("validation %s", lab.body_status.ok() ? "PASS" : "FAIL");
    const int head = *findJoint("headj");
    const int left_toe = *findJoint("ltoesj");
    const int right_toe = *findJoint("rtoesj");
    const int left_shoulder = *findJoint("larmuppj");
    const int right_shoulder = *findJoint("rarmuppj");
    const int left_hip = *findJoint("lleguppj");
    const int right_hip = *findJoint("rleguppj");
    const double height = lab.body.joints[head].rest_position.z -
                          std::min(lab.body.joints[left_toe].rest_position.z, lab.body.joints[right_toe].rest_position.z);
    const double shoulder = glm::length(lab.body.joints[right_shoulder].rest_position - lab.body.joints[left_shoulder].rest_position);
    const double hip = glm::length(lab.body.joints[right_hip].rest_position - lab.body.joints[left_hip].rest_position);
    ImGui::Text("joint-center height %.3f m", height);
    ImGui::Text("shoulder joint span %.3f m", shoulder);
    ImGui::Text("hip joint span %.3f m", hip);
    ImGui::TextWrapped("Anthropometry is reference only. It is not joint-center spacing.");
    for (const auto& measurement : lab.body.anthropometry.measurements) {
      ImGui::Text("  %s %.3f", measurement.first.c_str(), measurement.second);
    }
  } else {
    ImGui::TextUnformatted("validation FAIL");
  }
  if (!lab.body_status.ok() || !lab.body_status.warnings.empty()) {
    ImGui::TextWrapped("%s", lab.body_status.text().c_str());
  }

  section("MOTION");
  ImGui::TextWrapped("%s", lab.motion_path.empty() ? "no file" : lab.motion_path.filename().string().c_str());
  if (ImGui::Button("Load motion")) {
    const std::filesystem::path path =
        openDialog(lab.window, L"PROC motion\0*.procmo\0\0", L"Load .procmo", lab.asset_root / "assets" / "motions");
    if (!path.empty()) loadMotion(lab, path);
  }
  ImGui::SameLine();
  if (ImGui::Button("Reload motion") && !lab.motion_path.empty()) loadMotion(lab, lab.motion_path);
  if (!lab.asset_root.empty()) {
    std::vector<std::filesystem::path> clips;
    const std::filesystem::path directory = lab.asset_root / "assets" / "motions";
    if (std::filesystem::exists(directory)) {
      for (const std::filesystem::directory_entry& entry : std::filesystem::directory_iterator(directory)) {
        if (entry.path().extension() == ".procmo") clips.push_back(entry.path());
      }
    }
    std::sort(clips.begin(), clips.end());
    for (const std::filesystem::path& clip : clips) {
      if (ImGui::Button(clip.stem().string().c_str())) loadMotion(lab, clip);
    }
  }
  if (lab.has_motion) {
    ImGui::Text("fps %.3f", lab.motion.fps);
    ImGui::Text("duration %.3f s", lab.motion.duration_s);
    ImGui::Text("frames %d", lab.motion.frame_count);
    ImGui::Text("source %s / %s", lab.motion.source.type.c_str(), lab.motion.source.clip.c_str());
    ImGui::Text("dataset %s", lab.motion.source.dataset ? lab.motion.source.dataset->c_str() : "null");
    const ContactSample& contacts = lab.limits.pose.contacts;
    auto contactText = [](const std::optional<bool>& value) { return !value ? "unknown" : (*value ? "TRUE" : "FALSE"); };
    ImGui::Text("L heel %s", contactText(contacts.left_heel));
    ImGui::Text("L toe %s", contactText(contacts.left_toe));
    ImGui::Text("R heel %s", contactText(contacts.right_heel));
    ImGui::Text("R toe %s", contactText(contacts.right_toe));
    ImGui::TextWrapped("A missing contacts field is unknown. FALSE is an explicit open contact.");
    ImGui::TextWrapped("Heel markers are drawn at the foot joint. proc_humanoid_v0 has no heel bone.");
  }
  if (!lab.motion_status.ok() || !lab.motion_status.warnings.empty()) {
    ImGui::TextWrapped("%s", lab.motion_status.text().c_str());
  }
  section("LIMITS");
  if (ImGui::RadioButton("WARN", lab.limit_mode == LimitMode::Warn)) lab.limit_mode = LimitMode::Warn;
  ImGui::SameLine();
  if (ImGui::RadioButton("CLAMP", lab.limit_mode == LimitMode::Clamp)) lab.limit_mode = LimitMode::Clamp;
  ImGui::Text("violations %d", static_cast<int>(lab.limits.violations.size()));
  ImGui::BeginChild("violations", ImVec2(0, 70), ImGuiChildFlags_Border);
  if (lab.limits.violations.empty()) ImGui::TextUnformatted("none");
  for (const JointViolation& violation : lab.limits.violations) ImGui::TextWrapped("%s", violation.message.c_str());
  ImGui::EndChild();

  section("BONE LENGTHS");
  ImGui::Text("MAX BONE LENGTH ERROR %.8f m", lab.pose_ready ? lab.bones.max_error_m : 0.0);
  ImGui::TextUnformatted(!lab.pose_ready ? "BONE LENGTH VALIDATION: NO POSE"
                                        : (lab.bones.ok ? "BONE LENGTH VALIDATION: PASS" : "BONE LENGTH VALIDATION: FAIL"));
  if (lab.pose_ready && !lab.bones.ok && lab.bones.worst_index >= 0) {
    ImGui::Text("worst %s", lab.bones.samples[lab.bones.worst_index].bone.c_str());
  }
  ImGui::BeginChild("bone_errors", ImVec2(0, 180), ImGuiChildFlags_Border);
  for (const BoneLengthSample& sample : lab.bones.samples) {
    ImGui::Text("%s", sample.bone.c_str());
    ImGui::Text("  expected %.6f m", sample.expected_m);
    ImGui::Text("  actual   %.6f m", sample.actual_m);
    ImGui::Text("  error    %.6f m", sample.error_m);
  }
  ImGui::EndChild();

  section("VIEW");
  if (ImGui::Checkbox("root trajectory", &lab.show_trajectory)) rebuildTrajectory(lab);
  ImGui::Checkbox("joint labels", &lab.show_joint_labels);
  ImGui::Checkbox("bone names", &lab.show_bone_labels);
  ImGui::TextWrapped("Head peg is a facing marker, not a bone. LMB orbits. Wheel zooms.");
  const ImportResult imported = drawImportPanel(lab.import, lab.window, lab.has_body ? &lab.body : nullptr, lab.has_body,
                                                lab.asset_root, lab.motion, lab.has_motion, lab.motion_path,
                                                lab.motion_status, lab.player);
  if (imported.replaced_motion) {
    rebuildTrajectory(lab);
    lab.analysis_ui.valid = false;
  }
  drawAnalysisSection(lab.analysis_ui, lab.import.has_source ? &lab.import.clip : nullptr, lab.has_motion ? &lab.motion : nullptr,
                      lab.import.mapping, lab.player, lab.asset_root);
}

void drawTransport(LabState& lab) {
  const bool enabled = lab.has_motion;
  if (!enabled) ImGui::BeginDisabled();
  if (ImGui::Button("|<")) lab.player.jumpStart();
  ImGui::SameLine();
  if (ImGui::Button("<")) lab.player.previousFrame();
  ImGui::SameLine();
  if (ImGui::Button(lab.player.playing() ? "PAUSE" : "PLAY")) lab.player.togglePlay();
  ImGui::SameLine();
  if (ImGui::Button(">")) lab.player.nextFrame();
  ImGui::SameLine();
  if (ImGui::Button(">|")) lab.player.jumpEnd();
  ImGui::SameLine();
  const char* speeds[] = {"0.25x", "0.5x", "1x", "2x"};
  ImGui::SetNextItemWidth(80.0f);
  if (ImGui::Combo("speed", &lab.speed_index, speeds, 4)) {
    const double values[] = {0.25, 0.5, 1.0, 2.0};
    lab.player.setSpeed(values[lab.speed_index]);
  }
  ImGui::SameLine();
  bool loop = lab.player.loop();
  if (ImGui::Checkbox("loop", &loop)) lab.player.setLoop(loop);
  float time_s = static_cast<float>(lab.player.time());
  const float start = lab.has_motion && !lab.motion.frames.empty() ? static_cast<float>(lab.motion.frames.front().time) : 0.0f;
  const float end = lab.has_motion && !lab.motion.frames.empty() ? static_cast<float>(lab.motion.frames.back().time) : 1.0f;
  ImGui::SetNextItemWidth(-1.0f);
  if (ImGui::SliderFloat("##timeline", &time_s, start, std::max(start + 0.001f, end), "time %.3f s")) {
    lab.player.seekSeconds(time_s);
  }
  const ImVec2 timeline_min = ImGui::GetItemRectMin();
  const ImVec2 timeline_max = ImGui::GetItemRectMax();
  drawAnalysisOverlay(lab.analysis_ui, lab.player, timeline_min.x, timeline_min.y, timeline_max.x, timeline_max.y);
  if (!enabled) ImGui::EndDisabled();
}

void drawLabels(LabState& lab, int framebuffer_x, int framebuffer_y, int framebuffer_w, int framebuffer_h, float scale_x,
                float scale_y) {
  if (!lab.pose_ready || (!lab.show_joint_labels && !lab.show_bone_labels)) return;
  const glm::mat4 view_projection = lab.projection * lab.view;
  ImDrawList* draw_list = ImGui::GetForegroundDrawList();
  auto textAt = [&](const glm::dvec3& position, const char* text) {
    const glm::vec4 clip = view_projection * glm::vec4(static_cast<float>(position.x), static_cast<float>(position.y),
                                                       static_cast<float>(position.z), 1.0f);
    if (clip.w <= 0.05f) return;
    const glm::vec3 ndc = glm::vec3(clip) / clip.w;
    if (ndc.z < -1.0f || ndc.z > 1.0f) return;
    const float pixel_x = static_cast<float>(framebuffer_x) + (ndc.x * 0.5f + 0.5f) * static_cast<float>(framebuffer_w);
    const float pixel_y = static_cast<float>(framebuffer_y) + (ndc.y * 0.5f + 0.5f) * static_cast<float>(framebuffer_h);
    const ImVec2 point(pixel_x / scale_x, (static_cast<float>(ImGui::GetIO().DisplaySize.y) * scale_y - pixel_y) / scale_y);
    draw_list->AddText(point, IM_COL32(220, 220, 220, 255), text);
  };
  if (lab.show_joint_labels) {
    for (int index = 0; index < kJointCount; ++index) textAt(lab.world.joints[index].position, kJointNames[index]);
  }
  if (lab.show_bone_labels) {
    for (const BoneGeom& bone : lab.body.bones) {
      const glm::dvec3 middle = (lab.world.joints[bone.parent_joint].position + lab.world.joints[bone.child_joint].position) * 0.5;
      textAt(middle, bone.name.c_str());
    }
  }
}

int Application::run() {
  LabState lab;
  if (!glfwInit()) {
    std::fprintf(stderr, "GLFW failed to initialize.\n");
    return 1;
  }
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
  glfwWindowHint(GLFW_SAMPLES, 4);
  lab.window = glfwCreateWindow(1480, 920, "PROC MOTION LAB", nullptr, nullptr);
  if (lab.window == nullptr) {
    std::fprintf(stderr, "Could not create an OpenGL 3.3 window.\n");
    glfwTerminate();
    return 1;
  }
  glfwMakeContextCurrent(lab.window);
  glfwSwapInterval(1);
  glfwSetWindowUserPointer(lab.window, &lab);
  std::string gl_error;
  if (!lab.renderer.init(gl_error)) {
    std::fprintf(stderr, "OpenGL setup failed: %s\n", gl_error.c_str());
    glfwDestroyWindow(lab.window);
    glfwTerminate();
    return 1;
  }

  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGuiIO& io = ImGui::GetIO();
  io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
  float content_scale = 1.0f;
  glfwGetWindowContentScale(lab.window, &content_scale, nullptr);
  if (content_scale < 1.0f) content_scale = 1.0f;
  if (io.Fonts->AddFontFromFileTTF("C:\\Windows\\Fonts\\arial.ttf", 16.0f * content_scale) == nullptr) {
    io.Fonts->AddFontDefault();
  }
  applyTheme(content_scale);
  ImGui_ImplGlfw_InitForOpenGL(lab.window, true);
  ImGui_ImplOpenGL3_Init("#version 330");

  glfwSetScrollCallback(lab.window, [](GLFWwindow* window, double, double y) {
    labFrom(window)->scroll += y;
  });
  glfwSetDropCallback(lab.window, [](GLFWwindow* window, int count, const char** paths) {
    if (count <= 0) return;
    LabState* state = labFrom(window);
    const std::filesystem::path path = pathFromUtf8(paths[0]);
    std::string extension = path.extension().string();
    for (char& character : extension) {
      if (character >= 'A' && character <= 'Z') character = static_cast<char>(character - 'A' + 'a');
    }
    if (extension == ".procmo") loadMotion(*state, path);
    else if (extension == ".bvh" || extension == ".fbx") {
      loadDroppedSource(state->import, path, state->asset_root, state->has_body ? &state->body : nullptr, state->has_body,
                        state->motion, state->has_motion, state->motion_path, state->motion_status, state->player);
    } else loadBody(*state, path);
  });

  lab.asset_root = findAssetRoot();
  if (!lab.asset_root.empty()) loadBody(lab, lab.asset_root / "assets" / "bodies" / "neutral.procbody.json");
  int command_count = 0;
  LPWSTR* command = CommandLineToArgvW(GetCommandLineW(), &command_count);
  if (command != nullptr && command_count > 1) {
    const std::filesystem::path path(command[1]);
    std::string extension = path.extension().string();
    for (char& character : extension) {
      if (character >= 'A' && character <= 'Z') character = static_cast<char>(character - 'A' + 'a');
    }
    if (extension == ".fbx" || extension == ".bvh") {
      loadDroppedSource(lab.import, path, lab.asset_root, lab.has_body ? &lab.body : nullptr, lab.has_body, lab.motion,
                        lab.has_motion, lab.motion_path, lab.motion_status, lab.player);
      lab.import.view = ViewportMode::Overlay;
      rebuildTrajectory(lab);
    } else if (extension == ".procmo") {
      loadMotion(lab, path);
    } else {
      loadBody(lab, path);
    }
  }
  if (command != nullptr) LocalFree(command);
  lab.player.setSpeed(1.0);
  lab.player.setLoop(true);

  double previous = glfwGetTime();
  while (!glfwWindowShouldClose(lab.window)) {
    glfwPollEvents();
    const double now = glfwGetTime();
    double dt = now - previous;
    previous = now;
    if (dt < 0.0) dt = 0.0;
    if (dt > 0.1) dt = 0.1;
    lab.player.update(dt);
    updatePose(lab);

    int window_w = 1;
    int window_h = 1;
    glfwGetWindowSize(lab.window, &window_w, &window_h);
    int framebuffer_w = 1;
    int framebuffer_h = 1;
    glfwGetFramebufferSize(lab.window, &framebuffer_w, &framebuffer_h);
    const float scale_x = window_w > 0 ? static_cast<float>(framebuffer_w) / static_cast<float>(window_w) : 1.0f;
    const float scale_y = window_h > 0 ? static_cast<float>(framebuffer_h) / static_cast<float>(window_h) : 1.0f;
    const float title_h = 34.0f;
    const float status_h = 68.0f;
    const float transport_h = 88.0f;
    const float side_w = std::min(420.0f, static_cast<float>(window_w) * 0.40f);
    lab.view_x = side_w;
    lab.view_y = title_h;
    lab.view_w = std::max(1.0f, static_cast<float>(window_w) - side_w);
    lab.view_h = std::max(1.0f, static_cast<float>(window_h) - title_h - transport_h - status_h);
    const int viewport_x = static_cast<int>(std::lround(lab.view_x * scale_x));
    const int viewport_y = static_cast<int>(std::lround((static_cast<float>(window_h) - (lab.view_y + lab.view_h)) * scale_y));
    const int viewport_w = std::max(1, static_cast<int>(std::lround(lab.view_w * scale_x)));
    const int viewport_h = std::max(1, static_cast<int>(std::lround(lab.view_h * scale_y)));

    glViewport(0, 0, framebuffer_w, framebuffer_h);
    glClearColor(0.16f, 0.16f, 0.16f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_SCISSOR_TEST);
    glScissor(viewport_x, viewport_y, viewport_w, viewport_h);
    glClearColor(0.07f, 0.07f, 0.07f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glDisable(GL_SCISSOR_TEST);
    glViewport(viewport_x, viewport_y, viewport_w, viewport_h);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);

    const float aspect = lab.view_w / lab.view_h;
    lab.view = cameraView(lab);
    lab.projection = glm::perspective(glm::radians(38.0f), aspect, 0.05f, 80.0f);
    lab.renderer.setViewport(viewport_x, viewport_y, viewport_w, viewport_h);
    lab.renderer.setCamera(lab.view, lab.projection);
    drawGroundGrid(lab.renderer);
    const bool show_target = lab.pose_ready && (!lab.import.has_source || lab.import.view != ViewportMode::Source);
    const bool show_source = lab.import.has_source && lab.import.view != ViewportMode::Target;
    if (show_target) {
      SkeletonDrawOptions options;
      options.contacts = lab.has_motion ? &lab.limits.pose.contacts : nullptr;
      options.trajectory = lab.show_trajectory ? &lab.trajectory : nullptr;
      drawSkeleton(lab.renderer, lab.body, lab.world, options);
    }
    if (show_source) {
      std::vector<glm::dvec3> source_positions;
      std::vector<int> source_parents;
      sourcePositionsAt(lab.import, lab.has_motion ? lab.player.time() : 0.0, source_positions, source_parents);
      if (lab.import.view == ViewportMode::Overlay && lab.pose_ready && !source_positions.empty()) {
        const glm::dvec3 shift = lab.world.joints[kRootJoint].position - source_positions[lab.import.clip.root];
        for (glm::dvec3& position : source_positions) position += shift;
      }
      drawSourceChain(lab.renderer, source_positions, source_parents, glm::vec3(0.28f, 0.28f, 0.28f), 0.006f);
    }

    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();
    if (ImGui::IsKeyPressed(ImGuiKey_Space) && !io.WantCaptureKeyboard) lab.player.togglePlay();
    double mouse_x = 0.0;
    double mouse_y = 0.0;
    glfwGetCursorPos(lab.window, &mouse_x, &mouse_y);
    const bool inside = cursorInViewport(lab);
    const bool down = glfwGetMouseButton(lab.window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
    if (down && !lab.dragging && inside && !io.WantCaptureMouse) {
      lab.dragging = true;
      lab.last_x = mouse_x;
      lab.last_y = mouse_y;
    }
    if (!down) lab.dragging = false;
    if (lab.dragging) {
      lab.yaw -= (mouse_x - lab.last_x) * 0.008;
      lab.pitch -= (mouse_y - lab.last_y) * 0.008;
      lab.pitch = std::clamp(lab.pitch, -1.15, 1.15);
    }
    lab.last_x = mouse_x;
    lab.last_y = mouse_y;
    if (std::abs(lab.scroll) > 0.0 && inside && !io.WantCaptureMouse) {
      lab.distance *= std::exp(-lab.scroll * 0.12);
      lab.distance = std::clamp(lab.distance, 0.45, 30.0);
    }
    lab.scroll = 0.0;

    ImGui::SetNextWindowPos(ImVec2(0, 0));
    ImGui::SetNextWindowSize(ImVec2(static_cast<float>(window_w), title_h));
    ImGui::Begin("title", nullptr, kPanel | ImGuiWindowFlags_NoScrollbar);
    ImGui::TextUnformatted("PROC MOTION LAB");
    ImGui::End();

    ImGui::SetNextWindowPos(ImVec2(0, title_h));
    ImGui::SetNextWindowSize(ImVec2(side_w, static_cast<float>(window_h) - title_h - transport_h - status_h));
    ImGui::Begin("side", nullptr, kPanel);
    drawSidePanel(lab);
    ImGui::End();

    ImGui::SetNextWindowPos(ImVec2(0, static_cast<float>(window_h) - status_h - transport_h));
    ImGui::SetNextWindowSize(ImVec2(static_cast<float>(window_w), transport_h));
    ImGui::Begin("transport", nullptr, kPanel);
    drawTransport(lab);
    ImGui::End();

    ImGui::SetNextWindowPos(ImVec2(0, static_cast<float>(window_h) - status_h));
    ImGui::SetNextWindowSize(ImVec2(static_cast<float>(window_w), status_h));
    ImGui::Begin("status", nullptr, kPanel | ImGuiWindowFlags_NoScrollbar);
    const int frame = lab.has_motion ? lab.player.frameIndex() : 0;
    const char* bone_state = !lab.pose_ready ? "BONE LENGTH VALIDATION: NO POSE"
                                             : (lab.bones.ok ? "BONE LENGTH VALIDATION: PASS" : "BONE LENGTH VALIDATION: FAIL");
    const std::string violation = lab.limits.violations.empty()
                                      ? "none"
                                      : lab.limits.violations.front().joint + " (" + std::to_string(lab.limits.violations.size()) + ")";
    const char* source_name = lab.import.has_source ? lab.import.clip.name.c_str() : "none";
    const std::string target_name = lab.has_body ? lab.body_path.filename().string() : std::string("none");
    ImGui::Text("source %s | target %s | frame %d | bone error %.8f m | rest error %.3f deg | warnings %d", source_name,
                target_name.c_str(), frame, lab.pose_ready ? lab.bones.max_error_m : 0.0, lab.import.report.rest_max_error_deg,
                static_cast<int>(lab.limits.violations.size()));
    ImGui::Text("joint violation %s | %s%s", violation.c_str(), bone_state,
                lab.limit_mode == LimitMode::Clamp && !lab.limits.violations.empty() ? " | CLAMPED" : "");
    ImGui::End();

    ImGui::SetNextWindowPos(ImVec2(lab.view_x + 8.0f, lab.view_y + 8.0f));
    ImGui::SetNextWindowBgAlpha(0.85f);
    ImGui::Begin("camera", nullptr, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
                                       ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoSavedSettings);
    if (ImGui::Button("Front")) setCameraPreset(lab, 1.5707963, 0.22);
    ImGui::SameLine();
    if (ImGui::Button("Side")) setCameraPreset(lab, 0.0, 0.22);
    ImGui::SameLine();
    if (ImGui::Button("3/4")) setCameraPreset(lab, 0.78539816, 0.36);
    ImGui::SameLine();
    if (ImGui::Button("Reset camera")) setCameraPreset(lab, 0.85, 0.42);
    ImGui::End();

    drawLabels(lab, viewport_x, viewport_y, viewport_w, viewport_h, scale_x, scale_y);
    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    glfwSwapBuffers(lab.window);
  }

  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImGui::DestroyContext();
  glfwDestroyWindow(lab.window);
  glfwTerminate();
  return 0;
}

}  // namespace proc
