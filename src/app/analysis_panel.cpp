#include "app/analysis_panel.h"

#include "analysis/analysis_io.h"
#include "analysis/signal_processing.h"
#include "retarget/source_semantics.h"

#include <imgui.h>

#include <algorithm>
#include <cstdio>
#include <cmath>
#include <string>
#include <vector>

namespace proc {
namespace {

const char* kPlots[] = {"foot height", "foot speed", "contact confidence", "root speed", "hip", "knee", "ankle", "pelvis height"};
const char* kMarkers[] = {"left_heel", "left_toe", "right_heel", "right_toe"};

void plotSeries(const char* label, const std::vector<double>& values, double cursor_fraction) {
  ImDrawList* draw = ImGui::GetWindowDrawList();
  const ImVec2 origin = ImGui::GetCursorScreenPos();
  const float width = std::max(40.0f, ImGui::GetContentRegionAvail().x);
  const float height = 72.0f;
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
  }
  const float cursor = origin.x + width * static_cast<float>(std::clamp(cursor_fraction, 0.0, 1.0));
  draw->AddLine(ImVec2(cursor, origin.y), ImVec2(cursor, origin.y + height), IM_COL32(170, 170, 170, 180));
  ImGui::TextUnformatted(label);
}

std::vector<double> seriesFor(const ProcAnalysis& analysis, int plot) {
  std::vector<double> values;
  values.reserve(analysis.signals.size());
  for (const SignalFrame& frame : analysis.signals) {
    double value = 0.0;
    if (plot == 0) value = frame.left_foot_height;
    else if (plot == 1) value = frame.left_foot_speed;
    else if (plot == 2) {
      for (const ContactTrack& track : analysis.contacts) {
        if (track.marker == "left_heel" && values.size() < track.confidence.size()) value = track.confidence[values.size()];
      }
    } else if (plot == 3) value = frame.root_speed;
    else if (plot == 4) value = frame.sagittal_deg[static_cast<int>(RotChannel::LeftHip)];
    else if (plot == 5) value = frame.sagittal_deg[static_cast<int>(RotChannel::LeftKnee)];
    else if (plot == 6) value = frame.sagittal_deg[static_cast<int>(RotChannel::LeftAnkle)];
    else value = frame.root_z;
    values.push_back(value);
  }
  return values;
}

const ContactTrack* findTrack(const ProcAnalysis& analysis, const std::string& name) {
  for (const ContactTrack& track : analysis.contacts) {
    if (track.marker == name) return &track;
  }
  return nullptr;
}

}  // namespace

void drawAnalysisSection(AnalysisUi& ui, const SourceClip* source, const ProcMotion* motion,
                         const std::array<int, kJointCount>& mapping, MotionPlayer& player,
                         const std::filesystem::path& asset_root) {
  ImGui::Spacing();
  ImGui::TextUnformatted("ANALYSIS");
  ImGui::Separator();
  ImGui::TextWrapped("Observes the loaded source clip. It does not generate locomotion.");
  if (ImGui::Button("Analyze Current Clip")) {
    if (source == nullptr || source->frames.empty()) ui.message = "Load an FBX or BVH source first. Contact is measured on the source skeleton.";
    else {
      AnalysisInput input;
      input.source = source;
      input.motion = motion;
      input.semantics = semanticsFromMapping(mapping);
      input.motion_path = source->path;
      input.dataset = "";
      input.theme = "";
      ui.analysis = analyzeClip(input);
      ui.valid = true;
      ui.event_index = ui.analysis.events.empty() ? -1 : 0;
      ui.cycle_index = -1;
      for (int index = 0; index < static_cast<int>(ui.analysis.cycles.size()); ++index) {
        if (!ui.analysis.cycles[static_cast<std::size_t>(index)].mirrored) {
          ui.cycle_index = index;
          break;
        }
      }
      ui.message = "analyzed " + std::to_string(ui.analysis.signals.size()) + " frames";
    }
  }
  if (!ui.message.empty()) ImGui::TextWrapped("%s", ui.message.c_str());
  if (!ui.valid) return;

  const ProcAnalysis& analysis = ui.analysis;
  ImGui::Text("ground height %.4f m", analysis.ground.height_m);
  ImGui::Text("ground confidence %.2f%s", analysis.ground.confidence, analysis.ground.confidence < 0.45 ? "   GROUND CONFIDENCE: LOW" : "");
  ImGui::TextWrapped("heel markers are inferred/virtual. proc_humanoid_v0 has no heel bone.");
  ImGui::Text("source leg %.3f m", analysis.source_leg_length_m);
  std::vector<double> time;
  std::vector<double> speed;
  for (const SignalFrame& frame : analysis.signals) {
    time.push_back(frame.time);
    speed.push_back(frame.root_speed);
  }
  ImGui::Text("current speed %.3f m/s", sampleSeries(time, speed, player.time()));
  const int frame = nearestFrame(analysis, player.time());
  auto contactLabel = [&](const char* marker) {
    const ContactTrack* track = findTrack(analysis, marker);
    if (track == nullptr || frame < 0 || frame >= static_cast<int>(track->contact.size())) return std::string("missing");
    const bool on = track->contact[static_cast<std::size_t>(frame)] != 0;
    const double confidence = frame < static_cast<int>(track->confidence.size()) ? track->confidence[static_cast<std::size_t>(frame)] : 0.0;
    char text[64];
    std::snprintf(text, sizeof(text), "%s  %.2f", on ? "contact" : "swing", confidence);
    return std::string(text);
  };
  ImGui::Text("L heel %s", contactLabel("left_heel").c_str());
  ImGui::Text("L toe %s", contactLabel("left_toe").c_str());
  ImGui::Text("R heel %s", contactLabel("right_heel").c_str());
  ImGui::Text("R toe %s", contactLabel("right_toe").c_str());

  ImGui::TextUnformatted("Events");
  ImGui::BeginChild("analysis_events", ImVec2(0, 110), ImGuiChildFlags_Border);
  for (int index = 0; index < static_cast<int>(analysis.events.size()); ++index) {
    const GaitEvent& event = analysis.events[static_cast<std::size_t>(index)];
    char line[160];
    std::snprintf(line, sizeof(line), "%s %.3f  %s  %s  %.2f%s", event.manual_override ? "M" : " ", event.time, event.side.c_str(),
                  event.type.c_str(), event.confidence, event.manual_override ? "  manual" : "");
    if (ImGui::Selectable(line, ui.event_index == index)) {
      ui.event_index = index;
      player.seekSeconds(event.time);
    }
  }
  ImGui::EndChild();
  if (ui.event_index >= static_cast<int>(ui.analysis.events.size())) ui.event_index = ui.analysis.events.empty() ? -1 : 0;
  if (ui.event_index >= 0 && ui.event_index < static_cast<int>(ui.analysis.events.size())) {
    const int frame_now = ui.analysis.events[static_cast<std::size_t>(ui.event_index)].frame;
    const std::string type_now = ui.analysis.events[static_cast<std::size_t>(ui.event_index)].type;
    if (ImGui::Button("Event -1")) {
      const int next = std::max(0, frame_now - 1);
      ui.event_index = moveEvent(ui.analysis, ui.event_index, ui.analysis.signals[static_cast<std::size_t>(next)].time);
      if (ui.event_index >= 0) player.seekSeconds(ui.analysis.events[static_cast<std::size_t>(ui.event_index)].time);
    }
    ImGui::SameLine();
    if (ImGui::Button("Event +1")) {
      const int next = std::min(static_cast<int>(ui.analysis.signals.size()) - 1, frame_now + 1);
      const double time_s = ui.analysis.signals.empty() ? ui.analysis.events[static_cast<std::size_t>(ui.event_index)].time
                                                        : ui.analysis.signals[static_cast<std::size_t>(next)].time;
      ui.event_index = moveEvent(ui.analysis, ui.event_index, time_s);
      if (ui.event_index >= 0) player.seekSeconds(ui.analysis.events[static_cast<std::size_t>(ui.event_index)].time);
    }
    ImGui::SameLine();
    if (ImGui::Button("Toggle type")) {
      ui.event_index = setEventType(ui.analysis, ui.event_index, type_now == "initial_contact" ? "toe_off" : "initial_contact");
    }
    if (ui.event_index >= 0 && ui.event_index < static_cast<int>(ui.analysis.events.size())) {
      float event_time = static_cast<float>(ui.analysis.events[static_cast<std::size_t>(ui.event_index)].time);
      const float start = ui.analysis.signals.empty() ? 0.0f : static_cast<float>(ui.analysis.signals.front().time);
      const float end = ui.analysis.signals.empty() ? 1.0f : static_cast<float>(ui.analysis.signals.back().time);
      if (ImGui::SliderFloat("event time", &event_time, start, std::max(start + 0.001f, end))) {
        ui.event_index = moveEvent(ui.analysis, ui.event_index, event_time);
        if (ui.event_index >= 0) player.seekSeconds(ui.analysis.events[static_cast<std::size_t>(ui.event_index)].time);
      }
    }
  }

  ImGui::TextUnformatted("Cycles");
  ImGui::BeginChild("analysis_cycles", ImVec2(0, 130), ImGuiChildFlags_Border);
  for (int index = 0; index < static_cast<int>(ui.analysis.cycles.size()); ++index) {
    const GaitCycle& cycle = ui.analysis.cycles[static_cast<std::size_t>(index)];
    if (cycle.mirrored) continue;
    char line[192];
    std::snprintf(line, sizeof(line), "%02d %s  %.3fs  q %.2f  %s%s", cycle.index, cycle.side.c_str(), cycle.measurements.stride_time,
                  cycle.quality.overall, cycle.accepted ? "ACCEPT" : cycle.rejection_reason.c_str(), cycle.manual_override ? "  manual" : "");
    if (ImGui::Selectable(line, ui.cycle_index == index)) {
      ui.cycle_index = index;
      player.seekSeconds(cycle.start_time);
    }
  }
  ImGui::EndChild();
  if (ui.cycle_index >= 0 && ui.cycle_index < static_cast<int>(ui.analysis.cycles.size())) {
    const GaitCycle& cycle = ui.analysis.cycles[static_cast<std::size_t>(ui.cycle_index)];
    if (ImGui::Button("Accept cycle")) setCycleAccepted(ui.analysis, ui.cycle_index, true, "");
    ImGui::SameLine();
    if (ImGui::Button("Reject cycle")) setCycleAccepted(ui.analysis, ui.cycle_index, false, "manual");
    ImGui::Text("quality contact %.2f  timing %.2f  heading %.2f  overall %.2f", cycle.quality.contact, cycle.quality.timing,
                cycle.quality.heading, cycle.quality.overall);
    ImGui::Text("speed %.3f  stride %.3f  cadence %.1f", cycle.measurements.speed_mps, cycle.measurements.stride_length_m,
                cycle.measurements.cadence_spm);
  }

  ImGui::Combo("marker", &ui.marker, kMarkers, 4);
  if (ImGui::Button("Force contact")) overrideContactRange(ui.analysis, kMarkers[ui.marker], frame, frame, true);
  ImGui::SameLine();
  if (ImGui::Button("Clear contact")) overrideContactRange(ui.analysis, kMarkers[ui.marker], frame, frame, false);
  ImGui::TextWrapped("A contact edit rebuilds events and cycles and clears downstream manual edits. An event edit rebuilds cycles. Automatic values stay beside the edit.");
  if (!ui.analysis.derivation_note.empty()) ImGui::TextWrapped("%s", ui.analysis.derivation_note.c_str());

  ImGui::Combo("plot", &ui.plot, kPlots, 8);
  const std::vector<double> plotted = seriesFor(ui.analysis, ui.plot);
  const double span = time.empty() ? 1.0 : std::max(1e-6, time.back() - time.front());
  const double fraction = time.empty() ? 0.0 : (player.time() - time.front()) / span;
  plotSeries(kPlots[ui.plot], plotted, fraction);

  if (ImGui::Button("Save .procanalysis.json") && !asset_root.empty()) {
    const std::filesystem::path path = asset_root / "analysis_output" / "clips" / "current.procanalysis.json";
    const Status status = writeAnalysisToFile(ui.analysis, path);
    ui.message = status.ok() ? path.string() : status.text();
  }
  if (!analysis.warnings.empty()) ImGui::TextWrapped("%s", analysis.warnings.front().c_str());
}

void drawAnalysisOverlay(const AnalysisUi& ui, MotionPlayer& player, float x0, float y0, float x1, float y1) {
  if (!ui.valid || ui.analysis.signals.empty() || x1 <= x0) return;
  const double start = ui.analysis.signals.front().time;
  const double end = ui.analysis.signals.back().time;
  const double span = std::max(1e-6, end - start);
  auto xOf = [&](double time_s) { return x0 + static_cast<float>((time_s - start) / span) * (x1 - x0); };
  ImDrawList* draw = ImGui::GetForegroundDrawList();
  for (const GaitCycle& cycle : ui.analysis.cycles) {
    if (cycle.mirrored || cycle.accepted) continue;
    draw->AddRectFilled(ImVec2(xOf(cycle.start_time), y0 - 8.0f), ImVec2(xOf(cycle.end_time), y0 - 3.0f), IM_COL32(90, 90, 90, 160));
  }
  for (int index = 0; index < static_cast<int>(ui.analysis.events.size()); ++index) {
    const GaitEvent& event = ui.analysis.events[static_cast<std::size_t>(index)];
    const float x = xOf(event.time);
    const bool contact = event.type == "initial_contact";
    draw->AddLine(ImVec2(x, y0 - 10.0f), ImVec2(x, y1), contact ? IM_COL32(230, 230, 230, 220) : IM_COL32(140, 140, 140, 220), 1.0f);
  }
  if (ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
    const ImVec2 mouse = ImGui::GetMousePos();
    if (mouse.y >= y0 - 12.0f && mouse.y <= y0 && mouse.x >= x0 && mouse.x <= x1) {
      double best = 1e9;
      int chosen = -1;
      for (int index = 0; index < static_cast<int>(ui.analysis.events.size()); ++index) {
        const float distance = std::abs(xOf(ui.analysis.events[static_cast<std::size_t>(index)].time) - mouse.x);
        if (distance < best) {
          best = distance;
          chosen = index;
        }
      }
      if (chosen >= 0 && best < 8.0f) player.seekSeconds(ui.analysis.events[static_cast<std::size_t>(chosen)].time);
    }
  }
}

}  // namespace proc
