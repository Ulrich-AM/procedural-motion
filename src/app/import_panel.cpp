#include "app/import_panel.h"

#include "import/bvh_importer.h"
#include "import/fbx_importer.h"
#include "retarget/retarget_diagnostics.h"

#define NOMINMAX
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <commdlg.h>

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>
#define GLFW_EXPOSE_NATIVE_WIN32
#include <GLFW/glfw3native.h>

#include <imgui.h>

#include <algorithm>
#include <cctype>

namespace proc {
namespace {

std::filesystem::path openDialog(GLFWwindow* window, const wchar_t* filter, const wchar_t* title) {
  wchar_t buffer[1024] = {};
  OPENFILENAMEW dialog{};
  dialog.lStructSize = sizeof(dialog);
  dialog.hwndOwner = glfwGetWin32Window(window);
  dialog.lpstrFilter = filter;
  dialog.lpstrFile = buffer;
  dialog.nMaxFile = 1024;
  dialog.lpstrTitle = title;
  dialog.Flags = OFN_FILEMUSTEXIST | OFN_PATHMUSTEXIST | OFN_NOCHANGEDIR;
  if (!GetOpenFileNameW(&dialog)) return {};
  return std::filesystem::path(buffer);
}

std::filesystem::path saveDialog(GLFWwindow* window, const wchar_t* filter, const wchar_t* title, const wchar_t* name) {
  wchar_t buffer[1024] = {};
  if (name != nullptr) wcsncpy_s(buffer, name, _TRUNCATE);
  OPENFILENAMEW dialog{};
  dialog.lStructSize = sizeof(dialog);
  dialog.hwndOwner = glfwGetWin32Window(window);
  dialog.lpstrFilter = filter;
  dialog.lpstrFile = buffer;
  dialog.nMaxFile = 1024;
  dialog.lpstrTitle = title;
  dialog.Flags = OFN_OVERWRITEPROMPT | OFN_NOCHANGEDIR;
  if (!GetSaveFileNameW(&dialog)) return {};
  return std::filesystem::path(buffer);
}

std::string extensionOf(const std::filesystem::path& path) {
  std::string extension = path.extension().string();
  for (char& character : extension) character = static_cast<char>(std::tolower(static_cast<unsigned char>(character)));
  return extension;
}

void ensureProfiles(ImportSession& session, const std::filesystem::path& asset_root) {
  if (session.profiles_loaded || asset_root.empty()) return;
  loadRetargetProfiles(asset_root / "config" / "retarget", session.profiles);
  session.profiles_loaded = true;
}

std::vector<std::string> sourceNames(const SourceClip& clip) {
  std::vector<std::string> names;
  names.reserve(clip.joints.size());
  for (const SourceJoint& joint : clip.joints) names.push_back(joint.name);
  return names;
}

void applyDetectedConvention(ImportSession& session) {
  if (session.profile_index >= 0 && session.profiles[session.profile_index].has_coordinates) {
    applyAxisConvention(session.clip, session.profiles[session.profile_index].coordinates);
  } else {
    applyAxisConvention(session.clip, detectHumanConvention(session.clip));
  }
}

void assignMapping(ImportSession& session, const std::array<int, kJointCount>& mapping, const std::array<bool, kJointCount>& required) {
  session.mapping = mapping;
  session.required = required;
}

bool pushRetarget(ImportSession& session, const ProcBody& body, ProcMotion& motion, bool& has_motion,
                  std::filesystem::path& motion_path, Status& motion_status, MotionPlayer& player) {
  ProcMotion retargeted;
  Status status = retargetClip(body, session.clip, session.mapping, session.required,
                               RetargetOptions{session.rebase_horizontal, session.rebase_facing}, retargeted, session.report);
  motion_status = status;
  if (!status.ok()) return false;
  const double time_s = player.time();
  const bool was_playing = player.playing();
  motion = std::move(retargeted);
  has_motion = true;
  motion_path = session.path.filename().wstring().empty() ? std::filesystem::path("retarget") : session.path.stem();
  motion_path += ".procmo";
  player.setMotion(&motion);
  if (time_s > 0.0) player.seekSeconds(time_s);
  if (was_playing) player.play();
  return true;
}

void loadSource(ImportSession& session, const std::filesystem::path& path, const std::filesystem::path& asset_root,
                const ProcBody* body, bool has_body, ProcMotion& motion, bool& has_motion, std::filesystem::path& motion_path,
                Status& motion_status, MotionPlayer& player) {
  ensureProfiles(session, asset_root);
  SourceClip clip;
  const std::string extension = extensionOf(path);
  Status status;
  if (extension == ".bvh") status = loadBvhFromFile(path, identityConvention(), clip);
  else if (extension == ".fbx") status = loadFbxFromFile(path, identityConvention(), clip);
  else status.error("source files must be .bvh or .fbx");
  session.status = status;
  if (!status.ok()) {
    session.has_source = false;
    return;
  }
  session.clip = std::move(clip);
  session.path = path;
  session.has_source = true;
  const std::vector<std::string> names = sourceNames(session.clip);
  const ProfileMatch match = matchProfile(session.profiles, names);
  session.mapping.fill(-1);
  session.required.fill(false);
  if (match.confidence == "HIGH" && match.profile_index >= 0) {
    session.profile_index = match.profile_index;
    session.detection = "Detected " + session.profiles[match.profile_index].name + "  " + std::to_string(match.matched) + "/" +
                        std::to_string(match.required) + "  HIGH";
    assignMapping(session, match.mapping, session.profiles[match.profile_index].required);
  } else {
    session.profile_index = -1;
    session.detection = "No complete profile  best " + std::to_string(match.matched) + "/" + std::to_string(match.required);
  }
  applyDetectedConvention(session);
  if (has_body && body != nullptr) pushRetarget(session, *body, motion, has_motion, motion_path, motion_status, player);
}

void drawHierarchy(const SourceClip& clip, int joint) {
  const std::string label = clip.joints[joint].name + "##" + std::to_string(joint);
  if (clip.joints[joint].children.empty()) {
    ImGui::TreeNodeEx(label.c_str(), ImGuiTreeNodeFlags_Leaf | ImGuiTreeNodeFlags_NoTreePushOnOpen);
    return;
  }
  if (ImGui::TreeNode(label.c_str())) {
    for (int child : clip.joints[joint].children) drawHierarchy(clip, child);
    ImGui::TreePop();
  }
}

}  // namespace

void sourcePositionsAt(const ImportSession& session, double time_s, std::vector<glm::dvec3>& positions, std::vector<int>& parents) {
  positions.clear();
  parents.clear();
  if (!session.has_source) return;
  std::vector<glm::dvec3> translation;
  std::vector<glm::dquat> rotation;
  sampleSourceLocals(session.clip, time_s, translation, rotation);
  const int count = static_cast<int>(session.clip.joints.size());
  positions.resize(count);
  parents.resize(count);
  std::vector<glm::dquat> global(count);
  for (int index = 0; index < count; ++index) {
    const int parent = session.clip.joints[index].parent;
    parents[index] = parent;
    if (parent < 0) {
      global[index] = quatNormalize(rotation[index]);
      positions[index] = translation[index];
    } else {
      global[index] = quatNormalize(global[parent] * rotation[index]);
      positions[index] = positions[parent] + global[parent] * translation[index];
    }
  }
}

ImportResult drawImportPanel(ImportSession& session, GLFWwindow* window, const ProcBody* body, bool has_body,
                             const std::filesystem::path& asset_root, ProcMotion& motion, bool& has_motion,
                             std::filesystem::path& motion_path, Status& motion_status, MotionPlayer& player) {
  ImportResult result;
  ensureProfiles(session, asset_root);
  ImGui::Spacing();
  ImGui::TextUnformatted("SOURCE");
  ImGui::Separator();
  ImGui::TextWrapped("%s", session.path.empty() ? "no file" : session.path.filename().string().c_str());
  if (ImGui::Button("Load FBX/BVH")) {
    const std::filesystem::path path = openDialog(window, L"Source motion\0*.fbx;*.bvh\0\0", L"Load source motion");
    if (!path.empty()) {
      loadSource(session, path, asset_root, body, has_body, motion, has_motion, motion_path, motion_status, player);
      result.replaced_motion = has_motion;
    }
  }
  if (!session.status.ok() || !session.status.warnings.empty()) ImGui::TextWrapped("%s", session.status.text().c_str());
  if (!session.has_source) return result;

  ImGui::Text("format %s", session.clip.format.c_str());
  ImGui::Text("clip %s", session.clip.name.c_str());
  ImGui::Text("duration %.3f s", session.clip.duration);
  ImGui::Text("fps %.3f", session.clip.nominal_fps);
  ImGui::Text("joints %d", static_cast<int>(session.clip.joints.size()));
  ImGui::Text("root %s", session.clip.joints[session.clip.root].name.c_str());
  ImGui::TextWrapped("units %s", session.clip.units_detected.c_str());
  ImGui::TextWrapped("coordinates %s", session.clip.coordinate_conversion.c_str());
  ImGui::TextWrapped("%s", session.detection.c_str());
  ImGui::Text("mapped %d / %d", session.report.mapped, session.report.required);

  if (ImGui::Button("Auto Detect")) {
    const ProfileMatch match = matchProfile(session.profiles, sourceNames(session.clip));
    if (match.confidence == "HIGH" && match.profile_index >= 0) {
      session.profile_index = match.profile_index;
      session.detection = "Detected " + session.profiles[match.profile_index].name + "  HIGH";
      assignMapping(session, match.mapping, session.profiles[match.profile_index].required);
      applyDetectedConvention(session);
      if (has_body && body != nullptr &&
          pushRetarget(session, *body, motion, has_motion, motion_path, motion_status, player)) {
        result.replaced_motion = true;
      }
    } else {
      session.detection = "No complete profile. Required mapping is incomplete.";
      session.profile_index = -1;
    }
  }
  ImGui::SameLine();
  if (ImGui::Button("Load Profile")) {
    const std::filesystem::path path = openDialog(window, L"Retarget profile\0*.json\0\0", L"Load retarget profile");
    if (!path.empty()) {
      RetargetProfile profile;
      Status loaded = loadRetargetProfileFromFile(path, profile);
      if (!loaded.ok()) session.status = loaded;
      else {
        session.profiles.push_back(profile);
        session.profile_index = static_cast<int>(session.profiles.size()) - 1;
        assignMapping(session, mappingFromProfile(profile, sourceNames(session.clip)), profile.required);
        applyDetectedConvention(session);
        session.detection = "Loaded " + profile.name;
        if (has_body && body != nullptr &&
            pushRetarget(session, *body, motion, has_motion, motion_path, motion_status, player)) {
          result.replaced_motion = true;
        }
      }
    }
  }
  ImGui::SameLine();
  if (ImGui::Button("Save Mapping")) {
    std::filesystem::path path = saveDialog(window, L"Retarget mapping\0*.json\0\0", L"Save mapping", L"mapping.json");
    if (!path.empty()) {
      if (path.extension().empty()) path += ".json";
      session.status = saveMappingFile(path, session.mapping, sourceNames(session.clip));
    }
  }
  ImGui::SameLine();
  if (ImGui::Button("Reset Mapping")) {
    session.mapping.fill(-1);
    session.required.fill(false);
    session.profile_index = -1;
    session.detection = "Mapping cleared";
  }

  const char* modes[] = {"TARGET", "SOURCE", "OVERLAY"};
  int mode = static_cast<int>(session.view);
  if (ImGui::Combo("view", &mode, modes, 3)) session.view = static_cast<ViewportMode>(mode);
  if (ImGui::Checkbox("Rebase horizontal start to origin", &session.rebase_horizontal) ||
      ImGui::Checkbox("Rebase initial facing to +Y", &session.rebase_facing)) {
    if (has_body && body != nullptr &&
        pushRetarget(session, *body, motion, has_motion, motion_path, motion_status, player)) {
      result.replaced_motion = true;
    }
  }

  ImGui::TextUnformatted("PROC joint");
  const std::vector<std::string> names = sourceNames(session.clip);
  for (int joint = 0; joint < kJointCount; ++joint) {
    const std::string preview = session.mapping[joint] >= 0 ? names[session.mapping[joint]] : "(none)";
    ImGui::SetNextItemWidth(180.0f);
    bool changed = false;
    if (ImGui::BeginCombo(kJointNames[joint], preview.c_str())) {
      if (ImGui::Selectable("(none)", session.mapping[joint] < 0)) {
        session.mapping[joint] = -1;
        changed = true;
      }
      for (int source = 0; source < static_cast<int>(names.size()); ++source) {
        const std::string id = names[source] + "##src" + std::to_string(source);
        if (ImGui::Selectable(id.c_str(), session.mapping[joint] == source)) {
          session.mapping[joint] = source;
          changed = true;
        }
      }
      ImGui::EndCombo();
      if (changed && has_body && body != nullptr &&
          pushRetarget(session, *body, motion, has_motion, motion_path, motion_status, player)) {
        result.replaced_motion = true;
      }
    }
  }

  if (ImGui::Button("Export .procmo") && has_motion) {
    std::filesystem::path path = saveDialog(window, L"PROC motion\0*.procmo\0\0", L"Export .procmo", L"retarget.procmo");
    if (!path.empty()) {
      if (path.extension().empty()) path += ".procmo";
      motion_status = writeProcMotionToFile(motion, path);
      if (motion_status.ok()) motion_path = path;
    }
  }
  ImGui::TextWrapped("%s", formatRetargetReport(session.report).c_str());
  if (!session.report.translation_notes.empty()) {
    ImGui::TextUnformatted("Source translations kept out of .procmo:");
    for (const std::string& note : session.report.translation_notes) ImGui::TextWrapped("%s", note.c_str());
  }
  if (ImGui::TreeNode("Source hierarchy")) {
    drawHierarchy(session.clip, session.clip.root);
    ImGui::TreePop();
  }
  return result;
}

void loadDroppedSource(ImportSession& session, const std::filesystem::path& path, const std::filesystem::path& asset_root,
                       const ProcBody* body, bool has_body, ProcMotion& motion, bool& has_motion,
                       std::filesystem::path& motion_path, Status& motion_status, MotionPlayer& player) {
  loadSource(session, path, asset_root, body, has_body, motion, has_motion, motion_path, motion_status, player);
}

bool retargetCurrent(ImportSession& session, const ProcBody& body, ProcMotion& motion, bool& has_motion,
                     std::filesystem::path& motion_path, Status& motion_status, MotionPlayer& player) {
  if (!session.has_source) return false;
  return pushRetarget(session, body, motion, has_motion, motion_path, motion_status, player);
}

}  // namespace proc
