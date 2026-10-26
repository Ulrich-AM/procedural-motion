#include "app/reference_panel.h"

#include "locomotion/body_scale.h"
#include "motion/proc_motion.h"
#include "retarget/source_semantics.h"

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
#include <cfloat>
#include <cmath>

namespace proc {
namespace {

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

const GaitCycle* reviewCycle(const ProcAnalysis& analysis, int& slot) {
  int seen = 0;
  for (int index = 0; index < static_cast<int>(analysis.cycles.size()); ++index) {
    if (analysis.cycles[static_cast<std::size_t>(index)].mirrored) continue;
    if (seen == slot) {
      slot = index;
      return &analysis.cycles[static_cast<std::size_t>(index)];
    }
    ++seen;
  }
  slot = -1;
  return nullptr;
}

void syncInclude(ReferenceUi& ui, const ProcAnalysis& analysis) {
  if (ui.analysis_cycle_count == static_cast<int>(analysis.cycles.size()) && ui.include.size() == analysis.cycles.size()) return;
  ui.include.assign(analysis.cycles.size(), 0);
  for (int index = 0; index < static_cast<int>(analysis.cycles.size()); ++index) {
    const GaitCycle& cycle = analysis.cycles[static_cast<std::size_t>(index)];
    ui.include[static_cast<std::size_t>(index)] = !cycle.mirrored && cycle.accepted && cycle.classification == "straight";
  }
  ui.analysis_cycle_count = static_cast<int>(analysis.cycles.size());
  ui.regime = suggestRegime(analysis.source.clip, analysis.source.theme);
  if (ui.regime.empty()) ui.regime = "walk";
}

void plotSeries(const char* label, const std::vector<float>& source, const std::vector<float>& model, ReferenceCurves mode) {
  if (mode != ReferenceCurves::ModelResult && !source.empty()) ImGui::PlotLines((std::string(label) + " source").c_str(), source.data(), static_cast<int>(source.size()), 0, nullptr, FLT_MAX, FLT_MAX, ImVec2(0, 48));
  if (mode != ReferenceCurves::SourceCycle && !model.empty()) ImGui::PlotLines((std::string(label) + " model").c_str(), model.data(), static_cast<int>(model.size()), 0, nullptr, FLT_MAX, FLT_MAX, ImVec2(0, 48));
}

std::vector<float> asFloat(const std::vector<double>& values) {
  std::vector<float> out;
  out.reserve(values.size());
  for (double value : values) out.push_back(static_cast<float>(value));
  return out;
}

std::vector<float> modelDegrees(const SpeedKnot& knot, LocoChannel channel) {
  std::vector<float> out;
  out.reserve(kLocoPhaseSamples);
  for (const glm::dvec3& log_vector : knot.rotation[static_cast<int>(channel)]) {
    out.push_back(static_cast<float>(log_vector.x * 2.0 * 180.0 / 3.14159265358979323846));
  }
  return out;
}

const SpeedKnot* knotFor(const ProcLocomotionModel& model, const std::string& regime) {
  const RegimeModel* slot = regime == "run" ? &model.run : regime == "sprint" ? &model.sprint : &model.walk;
  if (!slot->present || slot->knots.empty()) return nullptr;
  return &slot->knots[slot->knots.size() / 2];
}

}  // namespace

void drawReferencePanel(ReferenceUi& ui, AnalysisUi& analysis, const ImportSession& import, const ProcMotion* motion, bool has_motion,
                        GLFWwindow* window, const ProcBody* body, bool has_body, const std::filesystem::path& asset_root) {
  ImGui::TextUnformatted("One clip is one reference, not a complete gait dataset.");
  if (ImGui::Button("Analyze for Reference") && import.has_source && has_motion && motion != nullptr) {
    AnalysisInput input;
    input.source = &import.clip;
    input.motion = motion;
    input.semantics = semanticsFromMapping(import.mapping);
    input.motion_path = import.path.string();
    analysis.analysis = analyzeClip(input);
    analysis.valid = true;
    analysis.message = "reference analysis";
    ui.analysis_cycle_count = -1;
  }
  if (ImGui::RadioButton("SOURCE", ui.view == ReferenceView::Source)) ui.view = ReferenceView::Source;
  ImGui::SameLine();
  if (ImGui::RadioButton("PROCEDURAL", ui.view == ReferenceView::Procedural)) ui.view = ReferenceView::Procedural;
  ImGui::SameLine();
  if (ImGui::RadioButton("OVERLAY", ui.view == ReferenceView::Overlay)) ui.view = ReferenceView::Overlay;
  ImGui::Checkbox("IN-PLACE PREVIEW", &ui.in_place);
  ImGui::SameLine();
  ImGui::Checkbox("CYCLE GHOSTS", &ui.show_ghosts);
  ImGui::SliderFloat("phase", &ui.phase, 0.0f, 1.0f, "%.2f");
  if (analysis.valid) {
    syncInclude(ui, analysis.analysis);
    if (sourceAppearsInPlace(analysis.analysis)) {
      ImGui::TextUnformatted("SOURCE APPEARS IN-PLACE");
      ImGui::Text("Measured root speed %.2f m/s", measuredRootSpeed(analysis.analysis));
      ImGui::TextWrapped("Measured root speed is near zero. Set a reference speed before building.");
    }
    if (ImGui::RadioButton("WALK", ui.regime == "walk")) ui.regime = "walk";
    ImGui::SameLine();
    if (ImGui::RadioButton("RUN", ui.regime == "run")) ui.regime = "run";
    ImGui::SameLine();
    if (ImGui::RadioButton("SPRINT", ui.regime == "sprint")) ui.regime = "sprint";
    const std::string suggested = suggestRegime(import.path.string(), analysis.analysis.source.theme);
    ImGui::Text("Suggested: %s", suggested.empty() ? "(none)" : suggested.c_str());
    ImGui::Checkbox("Reference speed override", &ui.manual_speed);
    if (ui.manual_speed) ImGui::SliderFloat("m/s", &ui.speed_override, 0.2f, 8.0f, "%.2f");
    if (ui.manual_speed) ImGui::TextWrapped("provenance: manual override for in-place source");
    else ImGui::TextUnformatted("provenance: measured root");

    ImGui::TextUnformatted("Cycles");
    for (int index = 0; index < static_cast<int>(analysis.analysis.cycles.size()); ++index) {
      const GaitCycle& cycle = analysis.analysis.cycles[static_cast<std::size_t>(index)];
      if (cycle.mirrored) continue;
      const std::string label = "cycle " + std::to_string(cycle.index) + " " + cycle.side + "##ref" + std::to_string(index);
      if (!cycle.accepted) ImGui::BeginDisabled();
      bool included = ui.include[static_cast<std::size_t>(index)] != 0;
      if (ImGui::Checkbox(label.c_str(), &included)) ui.include[static_cast<std::size_t>(index)] = included;
      if (!cycle.accepted) ImGui::EndDisabled();
      ImGui::SameLine();
      ImGui::Text("%.2f m/s  stance %.2f  %s", cycle.measurements.speed_mps, cycle.measurements.stance_fraction,
                  cycle.accepted ? "accepted" : cycle.rejection_reason.c_str());
    }
    if (ImGui::Button("Add to Reference Set")) {
      ReferenceClipRecord record;
      record.source_path = import.path.string();
      record.format = import.clip.format;
      record.profile = import.profile_index >= 0 ? import.profiles[static_cast<std::size_t>(import.profile_index)].name : "";
      record.regime = ui.regime;
      record.suggested_regime = suggested;
      record.analysis = analysis.analysis;
      record.measured_speed_mps = measuredRootSpeed(record.analysis);
      record.in_place = sourceAppearsInPlace(record.analysis);
      record.source_leg_length_m = record.analysis.source_leg_length_m;
      record.review = std::any_of(record.analysis.cycles.begin(), record.analysis.cycles.end(),
                                  [](const GaitCycle& cycle) { return cycle.manual_override; })
                          ? "manually reviewed"
                          : "automatic";
      if (ui.manual_speed) record.speed_override_mps = ui.speed_override;
      for (int index = 0; index < static_cast<int>(ui.include.size()); ++index) {
        if (ui.include[static_cast<std::size_t>(index)]) record.included_cycles.push_back(index);
      }
      if (addReferenceClip(ui.set, std::move(record))) ui.status = "added to the reference set";
    }
  } else if (has_motion) {
    ImGui::TextWrapped("Retarget looks correct, then analyze. IN-PLACE ANALYSIS keeps the measurement on the traveling motion.");
  }

  ImGui::Text("Reference set  %d", static_cast<int>(ui.set.clips.size()));
  for (int index = 0; index < static_cast<int>(ui.set.clips.size()); ++index) {
    const ReferenceClipRecord& clip = ui.set.clips[static_cast<std::size_t>(index)];
    ImGui::TextWrapped("%s  %s  cycles %d  %s", std::filesystem::path(clip.source_path).filename().string().c_str(), clip.regime.c_str(),
                       static_cast<int>(clip.included_cycles.size()), speedProvenanceText(speedProvenance(clip)));
    ImGui::SameLine();
    if (ImGui::Button(("Remove##" + std::to_string(index)).c_str())) removeReferenceClip(ui.set, index);
  }
  if (ImGui::Button("Build Reference Model") && has_body && body != nullptr) {
    ui.build = buildReferenceModel(ui.set);
    ui.has_model = ui.build.ok;
    ui.model = ui.build.model;
    ui.status = ui.build.ok ? "reference model built" : ui.build.error;
    if (ui.build.ok) {
      const ConstraintAudit audit = auditLocomotionModel(*body, ui.model);
      ui.audit_violations = audit.violations;
      if (!ui.build.coverage.empty()) ui.preview_speed = static_cast<float>(ui.build.coverage.front().speed_min_mps);
    }
  }
  if (!ui.status.empty()) ImGui::TextWrapped("%s", ui.status.c_str());
  if (ui.has_model) {
    ImGui::TextUnformatted("REFERENCE MODEL");
    for (const RegimeCoverage& coverage : ui.build.coverage) {
      ImGui::Text("%s  clips %d  cycles %d  knots %d", coverage.regime.c_str(), coverage.clip_count, coverage.cycle_count, coverage.knot_count);
      ImGui::Text("speed %.2f-%.2f m/s", coverage.speed_min_mps, coverage.speed_max_mps);
      if (coverage.single_speed) ImGui::TextUnformatted("SINGLE-SPEED REFERENCE");
      ImGui::TextWrapped("Outside this range the pose curve clamps. Root speed still follows the slider.");
    }
    ImGui::Text("seam %.5f %s", ui.model.max_seam_error, ui.model.max_seam_channel.c_str());
    ImGui::Text("constraint audit %s  violations %d", ui.audit_violations == 0 ? "PASS" : "CHECK", ui.audit_violations);
    if (ui.build.fidelity.available) {
      ImGui::Text("hip %.4f knee %.4f ankle %.4f", ui.build.fidelity.hip_rms, ui.build.fidelity.knee_rms, ui.build.fidelity.ankle_rms);
      ImGui::Text("pelvis %.4f shoulder %.4f elbow %.4f", ui.build.fidelity.pelvis_rms, ui.build.fidelity.shoulder_rms, ui.build.fidelity.elbow_rms);
      ImGui::Text("stance delta %.3f  flight delta %.3f", ui.build.fidelity.stance_difference, ui.build.fidelity.flight_difference);
    }
    const double leg = body != nullptr ? targetLegLength(*body) : 0.0;
    const double dimensionless = leg > 0.0 ? ui.preview_speed / std::sqrt(kLocomotionGravity * leg) : 0.0;
    if (poseCurveClamped(ui.model, ui.regime, dimensionless)) ImGui::TextUnformatted("POSE CURVE CLAMPED TO REFERENCE SPEED");
    if (ImGui::Button("Export .procloco.json")) {
      std::filesystem::path path = saveDialog(window, L"Locomotion model\0*.procloco.json;*.json\0\0", L"Export reference model", L"reference.procloco.json");
      if (!path.empty()) {
        if (path.extension().empty()) path += ".procloco.json";
        if (asset_root.empty()) {
          // The dialog path is already absolute.
        }
        const Status exported = exportReferenceModel(ui.set, ui.build, ui.audit_violations, path);
        ui.status = exported.ok() ? "exported locally" : exported.text();
      }
    }
    ImGui::SameLine();
    if (ImGui::Button("Save reference set")) {
      std::filesystem::path path = saveDialog(window, L"Reference set\0*.procref.json\0\0", L"Save reference set", L"reference.procref.json");
      if (!path.empty()) {
        if (path.extension().empty()) path += ".procref.json";
        const Status saved = writeReferenceSet(ui.set, path);
        ui.status = saved.ok() ? "saved reference set" : saved.text();
      }
    }
  }

  if (analysis.valid) {
    int slot = ui.review_cycle;
    const GaitCycle* cycle = reviewCycle(analysis.analysis, slot);
    ImGui::TextUnformatted("CURVES");
    if (ImGui::RadioButton("SOURCE CYCLE", ui.curves == ReferenceCurves::SourceCycle)) ui.curves = ReferenceCurves::SourceCycle;
    ImGui::SameLine();
    if (ImGui::RadioButton("MODEL RESULT", ui.curves == ReferenceCurves::ModelResult)) ui.curves = ReferenceCurves::ModelResult;
    ImGui::SameLine();
    if (ImGui::RadioButton("OVERLAY", ui.curves == ReferenceCurves::Overlay)) ui.curves = ReferenceCurves::Overlay;
    if (cycle != nullptr) {
      const SpeedKnot* knot = ui.has_model ? knotFor(ui.model, ui.regime) : nullptr;
      plotSeries("hip", asFloat(cycle->curves.hip_flex_deg), knot ? modelDegrees(*knot, LocoChannel::LeftHip) : std::vector<float>{}, ui.curves);
      plotSeries("knee", asFloat(cycle->curves.knee_flex_deg), knot ? modelDegrees(*knot, LocoChannel::LeftKnee) : std::vector<float>{}, ui.curves);
      plotSeries("ankle", asFloat(cycle->curves.ankle_flex_deg), knot ? modelDegrees(*knot, LocoChannel::LeftAnkle) : std::vector<float>{}, ui.curves);
      plotSeries("pelvis", asFloat(cycle->curves.pelvis_height_m), knot ? asFloat(std::vector<double>(knot->pelvis_vertical.begin(), knot->pelvis_vertical.end())) : std::vector<float>{}, ui.curves);
      plotSeries("shoulder", asFloat(cycle->curves.shoulder_flex_deg), knot ? modelDegrees(*knot, LocoChannel::LeftShoulder) : std::vector<float>{}, ui.curves);
      plotSeries("elbow", {}, knot ? modelDegrees(*knot, LocoChannel::LeftElbow) : std::vector<float>{}, ui.curves);
    }
  }
  (void)motion;
}

}  // namespace proc
