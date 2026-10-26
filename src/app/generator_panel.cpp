#include "app/generator_panel.h"
#include "app/generator_preview.h"
#include "analysis/gait_validation.h"

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

void plotSeries(const char* label, const std::vector<std::vector<double>>& series, const std::vector<ImU32>& colors) {
  ImDrawList* draw = ImGui::GetWindowDrawList();
  const ImVec2 origin = ImGui::GetCursorScreenPos();
  const float width = std::max(40.0f, ImGui::GetContentRegionAvail().x);
  const float height = 72.0f;
  ImGui::InvisibleButton(label, ImVec2(width, height));
  draw->AddRectFilled(origin, ImVec2(origin.x + width, origin.y + height), IM_COL32(18, 18, 18, 255));
  draw->AddRect(origin, ImVec2(origin.x + width, origin.y + height), IM_COL32(90, 90, 90, 255));
  double min_value = 0.0;
  double max_value = 1.0;
  bool any = false;
  for (const std::vector<double>& values : series) {
    for (double value : values) {
      if (!any) {
        min_value = max_value = value;
        any = true;
      }
      min_value = std::min(min_value, value);
      max_value = std::max(max_value, value);
    }
  }
  if (std::abs(max_value - min_value) < 1e-6) max_value = min_value + 1.0;
  for (std::size_t series_index = 0; series_index < series.size(); ++series_index) {
    const std::vector<double>& values = series[series_index];
    if (values.size() < 2) continue;
    const ImU32 color = series_index < colors.size() ? colors[series_index] : IM_COL32(210, 210, 210, 255);
    const float thickness = series_index == 0 ? 1.6f : 1.1f;
    for (std::size_t index = 1; index < values.size(); ++index) {
      const float x0 = origin.x + width * static_cast<float>(index - 1) / static_cast<float>(values.size() - 1);
      const float x1 = origin.x + width * static_cast<float>(index) / static_cast<float>(values.size() - 1);
      const float y0 = origin.y + height - static_cast<float>((values[index - 1] - min_value) / (max_value - min_value)) * (height - 4.0f) - 2.0f;
      const float y1 = origin.y + height - static_cast<float>((values[index] - min_value) / (max_value - min_value)) * (height - 4.0f) - 2.0f;
      if (series_index == 1 && (index % 3) == 0) continue;
      if (series_index == 2 && (index % 2) == 0) continue;
      draw->AddLine(ImVec2(x0, y0), ImVec2(x1, y1), color, thickness);
    }
  }
  ImGui::TextUnformatted(label);
}

void plotCurve(const char* label, const std::vector<double>& values) {
  plotSeries(label, {values}, {IM_COL32(210, 210, 210, 255)});
}

void supportBar(const char* label, const std::array<double, kLocoPhaseSamples>& values, ImU32 color) {
  ImDrawList* draw = ImGui::GetWindowDrawList();
  const ImVec2 origin = ImGui::GetCursorScreenPos();
  const float width = std::max(40.0f, ImGui::GetContentRegionAvail().x);
  const float height = 14.0f;
  ImGui::InvisibleButton(label, ImVec2(width, height));
  draw->AddRectFilled(origin, ImVec2(origin.x + width, origin.y + height), IM_COL32(18, 18, 18, 255));
  const float cell = width / static_cast<float>(kLocoPhaseSamples);
  for (int index = 0; index < kLocoPhaseSamples; ++index) {
    if (values[static_cast<std::size_t>(index)] < 0.5) continue;
    const float x = origin.x + cell * static_cast<float>(index);
    draw->AddRectFilled(ImVec2(x, origin.y + 1.0f), ImVec2(x + std::max(cell, 1.0f), origin.y + height - 1.0f), color);
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
  ui.audit_valid = false;
  ui.state = {};
  ui.has_previous = false;
  ui.left_samples = ui.right_samples = 0;
  ui.left_mean_mps = ui.left_peak_mps = ui.right_mean_mps = ui.right_peak_mps = 0.0;
}

void loadComparisonModel(GeneratorUi& ui, const std::filesystem::path& path) {
  ProcLocomotionModel model;
  Status status = readLocomotionModelFromFile(path, model);
  ui.status = status;
  if (!status.ok()) {
    ui.has_compare = false;
    return;
  }
  ui.compare = std::move(model);
  ui.compare_path = path;
  ui.has_compare = true;
}

void swapLocomotionModels(GeneratorUi& ui) {
  if (!ui.has_compare || !ui.has_model) return;
  std::swap(ui.model, ui.compare);
  std::swap(ui.path, ui.compare_path);
  ui.audit_valid = false;
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
                        const std::filesystem::path& asset_root, const std::filesystem::path& body_path, LimitMode* limit_mode,
                        const LimitResult* limits) {
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
  ImGui::Checkbox("IN-PLACE PREVIEW", &ui.in_place);
  ImGui::SameLine();
  ImGui::Checkbox("GHOST POSES", &ui.show_ghosts);
  if (ImGui::Button("Load comparison")) {
    const std::filesystem::path path = openModelDialog(window, asset_root / "assets" / "models");
    if (!path.empty()) loadComparisonModel(ui, path);
  }
  if (ui.has_compare) {
    ImGui::SameLine();
    if (ImGui::Button("Swap models")) swapLocomotionModels(ui);
    ImGui::TextWrapped("PRIMARY %s", ui.path.filename().string().c_str());
    ImGui::TextWrapped("COMPARE %s", ui.compare_path.filename().string().c_str());
  }
  if (!ui.playing) {
    float phase = static_cast<float>(ui.state.phase);
    if (ImGui::SliderFloat("PHASE", &phase, 0.0f, 1.0f, "%.3f")) ui.state.phase = wrapPhase(phase >= 0.999f ? 0.0 : phase);
    if (ImGui::Button("-0.01")) ui.state.phase = wrapPhase(ui.state.phase - 0.01);
    ImGui::SameLine();
    if (ImGui::Button("+0.01")) ui.state.phase = wrapPhase(ui.state.phase + 0.01);
    ImGui::SameLine();
    if (ImGui::Button("0.00")) ui.state.phase = 0.0;
    ImGui::SameLine();
    if (ImGui::Button("0.25")) ui.state.phase = 0.25;
    ImGui::SameLine();
    if (ImGui::Button("0.50")) ui.state.phase = 0.50;
    ImGui::SameLine();
    if (ImGui::Button("0.75")) ui.state.phase = 0.75;
  }
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
  ImGui::TextUnformatted("CONSTRAINT MODE");
  if (limit_mode != nullptr) {
    if (ImGui::RadioButton("WARN", *limit_mode == LimitMode::Warn)) *limit_mode = LimitMode::Warn;
    ImGui::SameLine();
    if (ImGui::RadioButton("CLAMP", *limit_mode == LimitMode::Clamp)) *limit_mode = LimitMode::Clamp;
  }
  const int raw_violations = limits != nullptr ? static_cast<int>(limits->violations.size()) : 0;
  ImGui::Text("raw violations: %d", raw_violations);
  ImGui::Text("clamped: %s", limits != nullptr && limits->clamped ? "YES" : "NO");
  if (limits != nullptr) {
    const int shown = std::min(4, raw_violations);
    for (int index = 0; index < shown; ++index) ImGui::TextWrapped("%s", limits->violations[static_cast<std::size_t>(index)].message.c_str());
  }
  ImGui::TextWrapped("WARN keeps the raw generator pose. CLAMP rewrites only this runtime pose. The locomotion model is not changed.");
  const std::string body_key = body_path.string();
  if (ui.audited_body != body_key) ui.audit_valid = false;
  if (has_body && body != nullptr && !ui.audit_valid) {
    ui.audit = auditLocomotionModel(*body, ui.model);
    ui.audit_valid = true;
    ui.audited_body = body_key;
  }
  if (ImGui::Button("Re-run audit") && has_body && body != nullptr) {
    ui.audit = auditLocomotionModel(*body, ui.model);
    ui.audit_valid = true;
    ui.audited_body = body_key;
  }
  if (ui.audit_valid) {
    ImGui::TextUnformatted("CONSTRAINT AUDIT");
    ImGui::Text("status      %s", ui.audit.ok ? "PASS" : "WARN");
    ImGui::Text("violations  %d", ui.audit.violations);
    ImGui::Text("worst       %s", ui.audit.worst_joint.empty() ? "none" : ui.audit.worst_joint.c_str());
    if (!ui.audit.ok) ImGui::Text("phase       %.2f  %s", ui.audit.worst_phase, ui.audit.worst_regime.c_str());
  }
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

  auto channelOf = [](const SpeedKnot& knot, LocoChannel channel) {
    std::vector<double> values;
    values.reserve(kLocoPhaseSamples);
    for (int sample = 0; sample < kLocoPhaseSamples; ++sample) {
      values.push_back(knot.rotation[static_cast<int>(channel)][static_cast<std::size_t>(sample)].x);
    }
    return values;
  };
  const SpeedKnot* walk_knot = ui.model.walk.knots.empty() ? nullptr : &ui.model.walk.knots[ui.model.walk.knots.size() / 2];
  const SpeedKnot* run_knot = ui.model.run.knots.empty() ? nullptr : &ui.model.run.knots[ui.model.run.knots.size() / 2];
  const SpeedKnot* sprint_knot = ui.model.sprint.knots.empty() ? nullptr : &ui.model.sprint.knots[ui.model.sprint.knots.size() / 2];
  auto compare = [&](const char* label, LocoChannel channel) {
    std::vector<std::vector<double>> series;
    if (walk_knot != nullptr) series.push_back(channelOf(*walk_knot, channel));
    if (run_knot != nullptr) series.push_back(channelOf(*run_knot, channel));
    if (sprint_knot != nullptr) series.push_back(channelOf(*sprint_knot, channel));
    plotSeries(label, series, {IM_COL32(230, 230, 230, 255), IM_COL32(170, 170, 170, 255), IM_COL32(110, 110, 110, 255)});
  };
  if (ui.has_compare) {
    const char* regime = regimeName(ui.state.regime);
    const std::string regime_key = regime == std::string("walk") || regime == std::string("run") || regime == std::string("sprint") ? regime : "walk";
    const CurveDifference matched = compareModelsAtSpeed(ui.model, ui.compare, regime_key, ui.state.dimensionless_speed);
    if (matched.available) {
      ImGui::Text("matched dim %.3f  hip rms %.3f  knee rms %.3f  ankle rms %.3f", matched.dimensionless_speed, matched.hip_rms,
                  matched.knee_rms, matched.ankle_rms);
      ImGui::Text("cadence delta %.1f  stance delta %.3f  flight delta %.3f", matched.cadence_difference, matched.stance_difference,
                  matched.flight_difference);
    } else {
      ImGui::TextUnformatted("comparison has no shared regime at this speed");
    }
  }
  ImGui::TextUnformatted("COMPARE  walk solid   run broken   sprint dotted");
  compare("hip log x  walk / run / sprint", LocoChannel::LeftHip);
  compare("knee log x  walk / run / sprint", LocoChannel::LeftKnee);
  compare("ankle log x  walk / run / sprint", LocoChannel::LeftAnkle);
  const SpeedKnot* support_knot = inspected(ui) != nullptr && !inspected(ui)->knots.empty()
                                      ? &inspected(ui)->knots[inspected(ui)->knots.size() / 2]
                                      : nullptr;
  if (support_knot != nullptr) {
    supportBar("LEFT support", support_knot->contact_left_heel, IM_COL32(210, 210, 210, 255));
    supportBar("RIGHT support", support_knot->contact_right_heel, IM_COL32(150, 150, 150, 255));
    std::array<double, kLocoPhaseSamples> flight{};
    int flight_count = 0;
    int double_count = 0;
    for (int sample = 0; sample < kLocoPhaseSamples; ++sample) {
      const bool left = support_knot->contact_left_heel[static_cast<std::size_t>(sample)] >= 0.5;
      const bool right = support_knot->contact_right_heel[static_cast<std::size_t>(sample)] >= 0.5;
      flight[static_cast<std::size_t>(sample)] = (!left && !right) ? 1.0 : 0.0;
      if (!left && !right) ++flight_count;
      if (left && right) ++double_count;
    }
    supportBar("FLIGHT", flight, IM_COL32(120, 120, 120, 255));
    ImGui::Text("single / double / flight samples  %d / %d / %d", kLocoPhaseSamples - flight_count - double_count, double_count,
                flight_count);
    int toe_off = -1;
    int opposite = -1;
    for (int sample = 1; sample < kLocoPhaseSamples; ++sample) {
      const double previous = support_knot->contact_left_heel[static_cast<std::size_t>(sample - 1)];
      const double current = support_knot->contact_left_heel[static_cast<std::size_t>(sample)];
      if (toe_off < 0 && previous >= 0.5 && current < 0.5) toe_off = sample;
      const double right_previous = support_knot->contact_right_heel[static_cast<std::size_t>(sample - 1)];
      const double right_current = support_knot->contact_right_heel[static_cast<std::size_t>(sample)];
      if (opposite < 0 && right_previous < 0.5 && right_current >= 0.5) opposite = sample;
    }
    if (support_knot->contact_left_heel[0] >= 0.5) {
      ImGui::Text("reference IC 0.00");
      if (toe_off >= 0) ImGui::Text("toe-off %.2f", static_cast<double>(toe_off) / static_cast<double>(kLocoPhaseSamples));
      if (opposite >= 0) ImGui::Text("opposite contact %.2f", static_cast<double>(opposite) / static_cast<double>(kLocoPhaseSamples));
    }
  }

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
