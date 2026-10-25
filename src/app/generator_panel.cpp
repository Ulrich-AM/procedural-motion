#include "app/generator_panel.h"

#include "locomotion/body_scale.h"
#include "locomotion/locomotion_model_io.h"
#include "motion/proc_motion.h"

#include <imgui.h>

#define NOMINMAX
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <commdlg.h>

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>
#define GLFW_EXPOSE_NATIVE_WIN32
#include <GLFW/glfw3native.h>

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

namespace proc {
namespace {

std::filesystem::path openModelDialog(GLFWwindow* window, const std::filesystem::path& initial) {
  wchar_t buffer[1024] = {};
  OPENFILENAMEW dialog{};
  dialog.lStructSize = sizeof(dialog);
  dialog.hwndOwner = glfwGetWin32Window(window);
  dialog.lpstrFilter = L"PROC locomotion model\0*.procloco.json;*.json\0\0";
  dialog.lpstrFile = buffer;
  dialog.nMaxFile = 1024;
  dialog.lpstrTitle = L"Load locomotion model";
  const std::wstring directory = initial.wstring();
  dialog.lpstrInitialDir = directory.c_str();
  dialog.Flags = OFN_FILEMUSTEXIST | OFN_PATHMUSTEXIST | OFN_NOCHANGEDIR;
  if (!GetOpenFileNameW(&dialog)) return {};
  return std::filesystem::path(buffer);
}

std::filesystem::path saveMotionDialog(GLFWwindow* window) {
  wchar_t buffer[1024] = L"generated.procmo";
  OPENFILENAMEW dialog{};
  dialog.lStructSize = sizeof(dialog);
  dialog.hwndOwner = glfwGetWin32Window(window);
  dialog.lpstrFilter = L"PROC motion\0*.procmo\0\0";
  dialog.lpstrFile = buffer;
  dialog.nMaxFile = 1024;
  dialog.lpstrTitle = L"Bake .procmo";
  dialog.Flags = OFN_OVERWRITEPROMPT | OFN_NOCHANGEDIR;
  dialog.lpstrDefExt = L"procmo";
  if (!GetSaveFileNameW(&dialog)) return {};
  return std::filesystem::path(buffer);
}

void plotCurve(const char* label, const std::vector<double>& values) {
  ImDrawList* draw = ImGui::GetWindowDrawList();
  const ImVec2 origin = ImGui::GetCursorScreenPos();
  const float width = std::max(40.0f, ImGui::GetContentRegionAvail().x);
  const float height = 64.0f;
  ImGui::InvisibleButton(label, ImVec2(width, height));
  draw->AddRectFilled(origin, ImVec2(origin.x + width, origin.y + height), IM_COL32(18, 18, 18, 255));
  draw->AddRect(origin, ImVec2(origin.x + width, origin.y + height), IM_COL32(90, 90, 90, 255));
  if (values.size() >= 2) {
    double min_value = values.front();
    double max_value = values.front();
    for (double value : values) {
      min_value = std::min(min_value, value);
      max_value = std::max(max_value, value);
    }
    if (std::abs(max_value - min_value) < 1e-6) max_value = min_value + 1.0;
    for (std::size_t index = 1; index < values.size(); ++index) {
      const float x0 = origin.x + width * static_cast<float>(index - 1) / static_cast<float>(values.size() - 1);
      const float x1 = origin.x + width * static_cast<float>(index) / static_cast<float>(values.size() - 1);
      const float y0 = origin.y + height - static_cast<float>((values[index - 1] - min_value) / (max_value - min_value)) * (height - 4.0f) - 2.0f;
      const float y1 = origin.y + height - static_cast<float>((values[index] - min_value) / (max_value - min_value)) * (height - 4.0f) - 2.0f;
      draw->AddLine(ImVec2(x0, y0), ImVec2(x1, y1), IM_COL32(210, 210, 210, 255), 1.0f);
    }
    const float x_wrap = origin.x;
    const float y_wrap = origin.y + height - static_cast<float>((values.front() - min_value) / (max_value - min_value)) * (height - 4.0f) - 2.0f;
    const float x_last = origin.x + width;
    const float y_last = origin.y + height - static_cast<float>((values.back() - min_value) / (max_value - min_value)) * (height - 4.0f) - 2.0f;
    draw->AddLine(ImVec2(x_last, y_last), ImVec2(x_wrap + width, y_wrap), IM_COL32(140, 140, 140, 180), 1.0f);
  }
  ImGui::TextUnformatted(label);
}

const RegimeModel* inspected(const GeneratorUi& ui) {
  if (ui.inspect == 1) return &ui.model.run;
  if (ui.inspect == 2) return &ui.model.sprint;
  return &ui.model.walk;
}

void describeRegime(const char* name, const RegimeModel& regime) {
  if (!regime.present || regime.knots.empty()) {
    ImGui::Text("%s  absent", name);
    return;
  }
  int cycles = 0;
  for (const SpeedKnot& knot : regime.knots) cycles += knot.cycle_count;
  ImGui::Text("%s  %.3f..%.3f  knots %d  cycles %d  seam %.4f %s%s", name, regime.support_min, regime.support_max,
              static_cast<int>(regime.knots.size()), cycles, regime.seam_error, regime.seam_channel.c_str(),
              regime.transition_inferred ? "  inferred transition" : "");
  for (std::size_t index = 0; index < regime.knots.size(); ++index) {
    const SpeedKnot& knot = regime.knots[index];
    ImGui::Text("  knot %d  dim %.3f  %.2f m/s  n %d  cadence %.1f  quality %.2f%s", static_cast<int>(index),
                knot.dimensionless_speed, knot.median_speed_mps, knot.cycle_count, knot.cadence_spm, knot.quality_median,
                knot.low_confidence ? "  low confidence" : "");
  }
}

}  // namespace

void loadLocomotionModel(GeneratorUi& ui, const std::filesystem::path& path) {
  ProcLocomotionModel model;
  Status status = readLocomotionModelFromFile(path, model);
  ui.status = status;
  ui.path = path;
  ui.bake_status.clear();
  if (!status.ok()) {
    ui.has_model = false;
    return;
  }
  ui.model = std::move(model);
  ui.has_model = true;
  ui.state = {};
  ui.has_previous = false;
  ui.left_samples = ui.right_samples = 0;
  ui.left_mean_mps = ui.left_peak_mps = ui.right_mean_mps = ui.right_peak_mps = 0.0;
}

void accumulateGeneratorSlip(GeneratorUi& ui, const WorldPose& world, const Pose& pose, double dt) {
  const glm::dvec3 left = world.joints[jointIndex(JointId::LeftFoot)].position;
  const glm::dvec3 right = world.joints[jointIndex(JointId::RightFoot)].position;
  if (ui.has_previous && dt > 1e-8) {
    auto take = [&](bool down, const glm::dvec3& current, const glm::dvec3& previous, double& sum_mean, double& peak, int& count) {
      if (!down) return;
      const glm::dvec3 delta = current - previous;
      const double speed = std::sqrt(delta.x * delta.x + delta.y * delta.y) / dt;
      sum_mean += speed;
      peak = std::max(peak, speed);
      ++count;
    };
    const bool left_down = (pose.contacts.left_heel && *pose.contacts.left_heel) || (pose.contacts.left_toe && *pose.contacts.left_toe);
    const bool right_down = (pose.contacts.right_heel && *pose.contacts.right_heel) || (pose.contacts.right_toe && *pose.contacts.right_toe);
    double left_sum = ui.left_mean_mps * static_cast<double>(ui.left_samples);
    double right_sum = ui.right_mean_mps * static_cast<double>(ui.right_samples);
    take(left_down, left, ui.previous_left, left_sum, ui.left_peak_mps, ui.left_samples);
    take(right_down, right, ui.previous_right, right_sum, ui.right_peak_mps, ui.right_samples);
    if (ui.left_samples > 0) ui.left_mean_mps = left_sum / static_cast<double>(ui.left_samples);
    if (ui.right_samples > 0) ui.right_mean_mps = right_sum / static_cast<double>(ui.right_samples);
  }
  ui.previous_left = left;
  ui.previous_right = right;
  ui.has_previous = true;
}

void drawGeneratorPanel(GeneratorUi& ui, GLFWwindow* window, const ProcBody* body, bool has_body,
                        const std::filesystem::path& asset_root) {
  ImGui::TextWrapped("%s", ui.path.empty() ? "no model" : ui.path.filename().string().c_str());
  if (ImGui::Button("Load locomotion model")) {
    const std::filesystem::path path = openModelDialog(window, asset_root / "assets" / "models");
    if (!path.empty()) loadLocomotionModel(ui, path);
  }
  if (!ui.status.ok()) ImGui::TextWrapped("%s", ui.status.text().c_str());
  if (!ui.has_model) {
    ImGui::TextUnformatted("Load a PROC_LOCOMOTION_MODEL. Playback stays on the other mode.");
    return;
  }
  ImGui::Text("phase samples %d", ui.model.phase_samples);
  ImGui::Text("seam %.5f %s", ui.model.max_seam_error, ui.model.max_seam_channel.c_str());
  describeRegime("walk", ui.model.walk);
  describeRegime("run", ui.model.run);
  describeRegime("sprint", ui.model.sprint);

  ImGui::RadioButton("AUTO", &ui.force, 0);
  ImGui::SameLine();
  ImGui::RadioButton("WALK", &ui.force, 1);
  ImGui::SameLine();
  ImGui::RadioButton("RUN", &ui.force, 2);
  ImGui::SameLine();
  ImGui::RadioButton("SPRINT", &ui.force, 3);
  ImGui::SliderFloat("desired m/s", &ui.desired_speed, 0.0f, 8.0f, "%.2f");
  ImGui::SliderFloat("accel m/s2", &ui.acceleration, 0.1f, 8.0f, "%.2f");
  ImGui::SliderFloat("heading deg", &ui.heading_deg, -180.0f, 180.0f, "%.1f");
  if (ImGui::Button(ui.playing ? "Pause" : "Play")) ui.playing = !ui.playing;
  ImGui::SameLine();
  if (ImGui::Button("Reset phase")) {
    ui.state.phase = 0.0;
    ui.state.root_position = glm::dvec3(0.0);
    ui.has_previous = false;
    ui.left_samples = ui.right_samples = 0;
    ui.left_mean_mps = ui.left_peak_mps = ui.right_mean_mps = ui.right_peak_mps = 0.0;
  }
  const double leg = has_body && body != nullptr ? targetLegLength(*body) : 0.0;
  ImGui::Text("desired %.2f  actual %.2f  dim %.3f", ui.state.desired_speed_mps, ui.state.actual_speed_mps, ui.state.dimensionless_speed);
  ImGui::Text("regime %s  blend %.2f %s", regimeName(ui.state.regime), ui.state.blend, regimeName(ui.state.blend_regime));
  ImGui::Text("phase %.3f  cadence %.1f spm", ui.state.phase, ui.state.cadence_spm);
  ImGui::Text("target leg %.3f m", leg);
  ImGui::Text("knots %d -> %d  weight %.2f%s", ui.state.knot_a, ui.state.knot_b, ui.state.knot_weight, ui.state.clamped ? "  clamped" : "");
  ImGui::Text("stride ref %.3f  implied %.3f m", ui.state.reference_stride_m, ui.state.implied_stride_m);
  ImGui::Text("contact L %.2f  R %.2f", ui.state.contact_left, ui.state.contact_right);
  ImGui::Text("slip L mean %.3f peak %.3f m/s", ui.left_mean_mps, ui.left_peak_mps);
  ImGui::Text("slip R mean %.3f peak %.3f m/s", ui.right_mean_mps, ui.right_peak_mps);
  if (!ui.state.note.empty()) ImGui::TextWrapped("%s", ui.state.note.c_str());
  ImGui::TextWrapped("Straight gait only. Heading yaws the whole cycle. There is no turning step, bank, or foot IK. Slip is reported and not corrected.");

  ImGui::SliderFloat("bake seconds", &ui.bake_seconds, 1.0f, 20.0f, "%.1f");
  if (ImGui::Button("Bake .procmo") && has_body && body != nullptr) {
    const std::filesystem::path path = saveMotionDialog(window);
    if (!path.empty()) {
      LocomotionCommand command;
      command.desired_speed_mps = ui.desired_speed;
      command.heading = glm::radians(static_cast<double>(ui.heading_deg));
      command.max_accel_mps2 = ui.acceleration;
      command.force = static_cast<RegimeForce>(ui.force);
      const ProcMotion motion = bakeLocomotion(*body, ui.model, command, ui.bake_seconds, 30.0);
      const Status written = writeProcMotionToFile(motion, path);
      ui.bake_status = written.ok() ? path.filename().string() + " frames " + std::to_string(motion.frame_count) : written.text();
    }
  }
  if (!ui.bake_status.empty()) ImGui::TextWrapped("%s", ui.bake_status.c_str());

  ImGui::RadioButton("inspect walk", &ui.inspect, 0);
  ImGui::SameLine();
  ImGui::RadioButton("inspect run", &ui.inspect, 1);
  ImGui::SameLine();
  ImGui::RadioButton("inspect sprint", &ui.inspect, 2);
  const RegimeModel* regime = inspected(ui);
  if (regime != nullptr && !regime->knots.empty()) {
    const SpeedKnot& knot = regime->knots[regime->knots.size() / 2];
    std::vector<double> hip, knee, ankle, pelvis, contact, cadence, speeds;
    for (int sample = 0; sample < kLocoPhaseSamples; ++sample) {
      hip.push_back(knot.rotation[static_cast<int>(LocoChannel::LeftHip)][static_cast<std::size_t>(sample)].x);
      knee.push_back(knot.rotation[static_cast<int>(LocoChannel::LeftKnee)][static_cast<std::size_t>(sample)].x);
      ankle.push_back(knot.rotation[static_cast<int>(LocoChannel::LeftAnkle)][static_cast<std::size_t>(sample)].x);
      pelvis.push_back(knot.pelvis_vertical[static_cast<std::size_t>(sample)]);
      contact.push_back(knot.contact_left_heel[static_cast<std::size_t>(sample)]);
    }
    for (const SpeedKnot& speed_knot : regime->knots) {
      speeds.push_back(speed_knot.dimensionless_speed);
      cadence.push_back(speed_knot.cadence_spm);
    }
    plotCurve("hip log x", hip);
    plotCurve("knee log x", knee);
    plotCurve("ankle log x", ankle);
    plotCurve("pelvis vertical / leg", pelvis);
    plotCurve("left heel contact", contact);
    if (speeds.size() >= 2) plotCurve("cadence vs knot order", cadence);
    else ImGui::Text("cadence %.1f at dim %.3f", knot.cadence_spm, knot.dimensionless_speed);
  }
}

}  // namespace proc
