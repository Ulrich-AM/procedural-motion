#include "analysis/analysis_io.h"

#include <nlohmann/json.hpp>

#include <fstream>
#include <sstream>

namespace proc {
namespace {

nlohmann::json vec(const std::vector<double>& values) { return values; }

std::vector<double> readVec(const nlohmann::json& node) {
  std::vector<double> values;
  if (!node.is_array()) return values;
  values.reserve(node.size());
  for (const nlohmann::json& value : node) values.push_back(value.get<double>());
  return values;
}

std::vector<std::uint8_t> readFlags(const nlohmann::json& node) {
  std::vector<std::uint8_t> values;
  if (!node.is_array()) return values;
  for (const nlohmann::json& value : node) values.push_back(value.get<int>() != 0 ? 1 : 0);
  return values;
}

nlohmann::json flags(const std::vector<std::uint8_t>& values) {
  nlohmann::json node = nlohmann::json::array();
  for (std::uint8_t value : values) node.push_back(value ? 1 : 0);
  return node;
}

nlohmann::json settingsJson(const AnalysisSettings& settings) {
  return {{"algorithm_version", settings.algorithm_version},
          {"position_smooth_s", settings.position_smooth_s},
          {"velocity_smooth_s", settings.velocity_smooth_s},
          {"ground_low_percentile", settings.ground_low_percentile},
          {"ground_speed_mps", settings.ground_speed_mps},
          {"contact_height_m", settings.contact_height_m},
          {"contact_speed_mps", settings.contact_speed_mps},
          {"contact_vertical_speed_mps", settings.contact_vertical_speed_mps},
          {"contact_enter", settings.contact_enter},
          {"contact_exit", settings.contact_exit},
          {"stability_window_s", settings.stability_window_s},
          {"event_refine_s", settings.event_refine_s},
          {"min_cycle_s", settings.min_cycle_s},
          {"max_cycle_s", settings.max_cycle_s},
          {"min_travel_m", settings.min_travel_m},
          {"standing_travel_m", settings.standing_travel_m},
          {"max_heading_deg", settings.max_heading_deg},
          {"min_contact_confidence", settings.min_contact_confidence},
          {"phase_samples", settings.phase_samples},
          {"heel_fraction", settings.heel_fraction},
          {"gravity_mps2", settings.gravity_mps2}};
}

void readSettings(const nlohmann::json& node, AnalysisSettings& settings) {
  auto take = [&](const char* key, double& slot) {
    if (node.contains(key)) slot = node[key].get<double>();
  };
  if (node.contains("algorithm_version")) settings.algorithm_version = node["algorithm_version"].get<int>();
  take("position_smooth_s", settings.position_smooth_s);
  take("velocity_smooth_s", settings.velocity_smooth_s);
  take("ground_low_percentile", settings.ground_low_percentile);
  take("ground_speed_mps", settings.ground_speed_mps);
  take("contact_height_m", settings.contact_height_m);
  take("contact_speed_mps", settings.contact_speed_mps);
  take("contact_vertical_speed_mps", settings.contact_vertical_speed_mps);
  take("contact_enter", settings.contact_enter);
  take("contact_exit", settings.contact_exit);
  take("stability_window_s", settings.stability_window_s);
  take("event_refine_s", settings.event_refine_s);
  take("min_cycle_s", settings.min_cycle_s);
  take("max_cycle_s", settings.max_cycle_s);
  take("min_travel_m", settings.min_travel_m);
  take("standing_travel_m", settings.standing_travel_m);
  take("max_heading_deg", settings.max_heading_deg);
  take("min_contact_confidence", settings.min_contact_confidence);
  if (node.contains("phase_samples")) settings.phase_samples = node["phase_samples"].get<int>();
  take("heel_fraction", settings.heel_fraction);
  take("gravity_mps2", settings.gravity_mps2);
}

nlohmann::json measurementsJson(const CycleMeasurements& measured) {
  return {{"stride_time", measured.stride_time},
          {"step_time", measured.step_time},
          {"stance_time", measured.stance_time},
          {"swing_time", measured.swing_time},
          {"cadence_spm", measured.cadence_spm},
          {"speed_mps", measured.speed_mps},
          {"stride_length_m", measured.stride_length_m},
          {"step_length_m", measured.step_length_m},
          {"stance_fraction", measured.stance_fraction},
          {"swing_fraction", measured.swing_fraction},
          {"double_support_fraction", measured.double_support_fraction},
          {"flight_fraction", measured.flight_fraction},
          {"foot_clearance_m", measured.foot_clearance_m},
          {"toe_clearance_m", measured.toe_clearance_m},
          {"contact_velocity_mps", measured.contact_velocity_mps},
          {"pelvis_height_m", measured.pelvis_height_m},
          {"pelvis_vertical_excursion_m", measured.pelvis_vertical_excursion_m},
          {"pelvis_pitch_excursion_deg", measured.pelvis_pitch_excursion_deg},
          {"pelvis_roll_excursion_deg", measured.pelvis_roll_excursion_deg},
          {"hip_flex_min_deg", measured.hip_flex_min_deg},
          {"hip_flex_max_deg", measured.hip_flex_max_deg},
          {"hip_flex_min_phase", measured.hip_flex_min_phase},
          {"hip_flex_max_phase", measured.hip_flex_max_phase},
          {"knee_flex_min_deg", measured.knee_flex_min_deg},
          {"knee_flex_max_deg", measured.knee_flex_max_deg},
          {"knee_flex_min_phase", measured.knee_flex_min_phase},
          {"knee_flex_max_phase", measured.knee_flex_max_phase},
          {"ankle_flex_min_deg", measured.ankle_flex_min_deg},
          {"ankle_flex_max_deg", measured.ankle_flex_max_deg},
          {"ankle_flex_min_phase", measured.ankle_flex_min_phase},
          {"ankle_flex_max_phase", measured.ankle_flex_max_phase},
          {"shoulder_swing_deg", measured.shoulder_swing_deg},
          {"arm_leg_phase", measured.arm_leg_phase},
          {"travel_direction_deg", measured.travel_direction_deg},
          {"heading_variation_deg", measured.heading_variation_deg},
          {"normalized_stride_length", measured.normalized_stride_length},
          {"normalized_step_length", measured.normalized_step_length},
          {"normalized_foot_clearance", measured.normalized_foot_clearance},
          {"normalized_pelvis_height", measured.normalized_pelvis_height},
          {"froude_number", measured.froude_number}};
}

void readMeasurements(const nlohmann::json& node, CycleMeasurements& measured) {
  auto take = [&](const char* key, double& slot) {
    if (node.contains(key)) slot = node[key].get<double>();
  };
  take("stride_time", measured.stride_time);
  take("step_time", measured.step_time);
  take("stance_time", measured.stance_time);
  take("swing_time", measured.swing_time);
  take("cadence_spm", measured.cadence_spm);
  take("speed_mps", measured.speed_mps);
  take("stride_length_m", measured.stride_length_m);
  take("step_length_m", measured.step_length_m);
  take("stance_fraction", measured.stance_fraction);
  take("swing_fraction", measured.swing_fraction);
  take("double_support_fraction", measured.double_support_fraction);
  take("flight_fraction", measured.flight_fraction);
  take("foot_clearance_m", measured.foot_clearance_m);
  take("toe_clearance_m", measured.toe_clearance_m);
  take("contact_velocity_mps", measured.contact_velocity_mps);
  take("pelvis_height_m", measured.pelvis_height_m);
  take("pelvis_vertical_excursion_m", measured.pelvis_vertical_excursion_m);
  take("pelvis_pitch_excursion_deg", measured.pelvis_pitch_excursion_deg);
  take("pelvis_roll_excursion_deg", measured.pelvis_roll_excursion_deg);
  take("hip_flex_min_deg", measured.hip_flex_min_deg);
  take("hip_flex_max_deg", measured.hip_flex_max_deg);
  take("hip_flex_min_phase", measured.hip_flex_min_phase);
  take("hip_flex_max_phase", measured.hip_flex_max_phase);
  take("knee_flex_min_deg", measured.knee_flex_min_deg);
  take("knee_flex_max_deg", measured.knee_flex_max_deg);
  take("knee_flex_min_phase", measured.knee_flex_min_phase);
  take("knee_flex_max_phase", measured.knee_flex_max_phase);
  take("ankle_flex_min_deg", measured.ankle_flex_min_deg);
  take("ankle_flex_max_deg", measured.ankle_flex_max_deg);
  take("ankle_flex_min_phase", measured.ankle_flex_min_phase);
  take("ankle_flex_max_phase", measured.ankle_flex_max_phase);
  take("shoulder_swing_deg", measured.shoulder_swing_deg);
  take("arm_leg_phase", measured.arm_leg_phase);
  take("travel_direction_deg", measured.travel_direction_deg);
  take("heading_variation_deg", measured.heading_variation_deg);
  take("normalized_stride_length", measured.normalized_stride_length);
  take("normalized_step_length", measured.normalized_step_length);
  take("normalized_foot_clearance", measured.normalized_foot_clearance);
  take("normalized_pelvis_height", measured.normalized_pelvis_height);
  take("froude_number", measured.froude_number);
}

nlohmann::json curvesJson(const PhaseCurves& curves) {
  return {{"hip_flex_deg", vec(curves.hip_flex_deg)},
          {"knee_flex_deg", vec(curves.knee_flex_deg)},
          {"ankle_flex_deg", vec(curves.ankle_flex_deg)},
          {"opposite_hip_flex_deg", vec(curves.opposite_hip_flex_deg)},
          {"pelvis_height_m", vec(curves.pelvis_height_m)},
          {"pelvis_pitch_deg", vec(curves.pelvis_pitch_deg)},
          {"pelvis_roll_deg", vec(curves.pelvis_roll_deg)},
          {"foot_height_m", vec(curves.foot_height_m)},
          {"toe_height_m", vec(curves.toe_height_m)},
          {"contact_confidence", vec(curves.contact_confidence)},
          {"shoulder_flex_deg", vec(curves.shoulder_flex_deg)}};
}

void readCurves(const nlohmann::json& node, PhaseCurves& curves) {
  auto take = [&](const char* key, std::vector<double>& slot) {
    if (node.contains(key)) slot = readVec(node[key]);
  };
  take("hip_flex_deg", curves.hip_flex_deg);
  take("knee_flex_deg", curves.knee_flex_deg);
  take("ankle_flex_deg", curves.ankle_flex_deg);
  take("opposite_hip_flex_deg", curves.opposite_hip_flex_deg);
  take("pelvis_height_m", curves.pelvis_height_m);
  take("pelvis_pitch_deg", curves.pelvis_pitch_deg);
  take("pelvis_roll_deg", curves.pelvis_roll_deg);
  take("foot_height_m", curves.foot_height_m);
  take("toe_height_m", curves.toe_height_m);
  take("contact_confidence", curves.contact_confidence);
  take("shoulder_flex_deg", curves.shoulder_flex_deg);
}

std::vector<double> pull(const std::vector<SignalFrame>& frames, double SignalFrame::*member) {
  std::vector<double> values;
  values.reserve(frames.size());
  for (const SignalFrame& frame : frames) values.push_back(frame.*member);
  return values;
}

nlohmann::json signalsJson(const std::vector<SignalFrame>& frames) {
  nlohmann::json rotation = nlohmann::json::object();
  for (int channel = 0; channel < kRotChannelCount; ++channel) {
    nlohmann::json quat = nlohmann::json::array();
    nlohmann::json log = nlohmann::json::array();
    nlohmann::json sagittal = nlohmann::json::array();
    for (const SignalFrame& frame : frames) {
      const glm::dquat& value = frame.rotation[static_cast<std::size_t>(channel)];
      const glm::dvec3& axis = frame.log_vector[static_cast<std::size_t>(channel)];
      quat.push_back({value.x, value.y, value.z, value.w});
      log.push_back({axis.x, axis.y, axis.z});
      sagittal.push_back(frame.sagittal_deg[static_cast<std::size_t>(channel)]);
    }
    rotation[rotChannelName(static_cast<RotChannel>(channel))] = {{"quat", quat}, {"log", log}, {"sagittal_deg", sagittal}};
  }
  return {{"time", vec(pull(frames, &SignalFrame::time))},
          {"root_x", vec(pull(frames, &SignalFrame::root_x))},
          {"root_y", vec(pull(frames, &SignalFrame::root_y))},
          {"root_z", vec(pull(frames, &SignalFrame::root_z))},
          {"root_horizontal_speed", vec(pull(frames, &SignalFrame::root_horizontal_speed))},
          {"root_speed", vec(pull(frames, &SignalFrame::root_speed))},
          {"left_foot_z", vec(pull(frames, &SignalFrame::left_foot_z))},
          {"right_foot_z", vec(pull(frames, &SignalFrame::right_foot_z))},
          {"left_foot_height", vec(pull(frames, &SignalFrame::left_foot_height))},
          {"right_foot_height", vec(pull(frames, &SignalFrame::right_foot_height))},
          {"left_toe_height", vec(pull(frames, &SignalFrame::left_toe_height))},
          {"right_toe_height", vec(pull(frames, &SignalFrame::right_toe_height))},
          {"left_foot_speed", vec(pull(frames, &SignalFrame::left_foot_speed))},
          {"right_foot_speed", vec(pull(frames, &SignalFrame::right_foot_speed))},
          {"pelvis_pitch_deg", vec(pull(frames, &SignalFrame::pelvis_pitch_deg))},
          {"pelvis_roll_deg", vec(pull(frames, &SignalFrame::pelvis_roll_deg))},
          {"rotation", rotation}};
}

void readSignals(const nlohmann::json& node, std::vector<SignalFrame>& frames) {
  const std::vector<double> time = readVec(node.value("time", nlohmann::json::array()));
  frames.assign(time.size(), SignalFrame{});
  auto apply = [&](const char* key, double SignalFrame::*member) {
    if (!node.contains(key)) return;
    const std::vector<double> values = readVec(node[key]);
    for (std::size_t index = 0; index < frames.size() && index < values.size(); ++index) frames[index].*member = values[index];
  };
  for (std::size_t index = 0; index < frames.size(); ++index) {
    frames[index].time = time[index];
    frames[index].rotation.fill(quatIdentity());
  }
  apply("root_x", &SignalFrame::root_x);
  apply("root_y", &SignalFrame::root_y);
  apply("root_z", &SignalFrame::root_z);
  apply("root_horizontal_speed", &SignalFrame::root_horizontal_speed);
  apply("root_speed", &SignalFrame::root_speed);
  apply("left_foot_z", &SignalFrame::left_foot_z);
  apply("right_foot_z", &SignalFrame::right_foot_z);
  apply("left_foot_height", &SignalFrame::left_foot_height);
  apply("right_foot_height", &SignalFrame::right_foot_height);
  apply("left_toe_height", &SignalFrame::left_toe_height);
  apply("right_toe_height", &SignalFrame::right_toe_height);
  apply("left_foot_speed", &SignalFrame::left_foot_speed);
  apply("right_foot_speed", &SignalFrame::right_foot_speed);
  apply("pelvis_pitch_deg", &SignalFrame::pelvis_pitch_deg);
  apply("pelvis_roll_deg", &SignalFrame::pelvis_roll_deg);
  if (!node.contains("rotation")) return;
  for (int channel = 0; channel < kRotChannelCount; ++channel) {
    const char* name = rotChannelName(static_cast<RotChannel>(channel));
    if (!node["rotation"].contains(name)) continue;
    const nlohmann::json& packed = node["rotation"][name];
    if (packed.contains("quat")) {
      const nlohmann::json& quat = packed["quat"];
      for (std::size_t index = 0; index < frames.size() && index < quat.size(); ++index) {
        const nlohmann::json& value = quat[index];
        if (value.size() >= 4) frames[index].rotation[static_cast<std::size_t>(channel)] = quatFromXYZW(value[0], value[1], value[2], value[3]);
      }
    }
    if (packed.contains("log")) {
      const nlohmann::json& log = packed["log"];
      for (std::size_t index = 0; index < frames.size() && index < log.size(); ++index) {
        const nlohmann::json& value = log[index];
        if (value.size() >= 3) frames[index].log_vector[static_cast<std::size_t>(channel)] = glm::dvec3(value[0], value[1], value[2]);
      }
    }
    if (packed.contains("sagittal_deg")) {
      const std::vector<double> sagittal = readVec(packed["sagittal_deg"]);
      for (std::size_t index = 0; index < frames.size() && index < sagittal.size(); ++index)
        frames[index].sagittal_deg[static_cast<std::size_t>(channel)] = sagittal[index];
    }
  }
}

std::string csvEscape(const std::string& text) {
  if (text.find_first_of(",\"\n") == std::string::npos) return text;
  std::string escaped = "\"";
  for (char character : text) {
    if (character == '"') escaped += "\"\"";
    else escaped += character;
  }
  escaped += "\"";
  return escaped;
}

}  // namespace

std::string writeAnalysisToString(const ProcAnalysis& analysis) {
  nlohmann::json events = nlohmann::json::array();
  for (const GaitEvent& event : analysis.events) {
    events.push_back({{"time", event.time},
                      {"automatic_time", event.automatic_time},
                      {"frame", event.frame},
                      {"automatic_frame", event.automatic_frame},
                      {"side", event.side},
                      {"type", event.type},
                      {"automatic_type", event.automatic_type},
                      {"confidence", event.confidence},
                      {"manual_override", event.manual_override}});
  }
  nlohmann::json contacts = nlohmann::json::array();
  for (const ContactTrack& track : analysis.contacts) {
    contacts.push_back({{"marker", track.marker},
                        {"provenance", track.provenance},
                        {"confidence", vec(track.confidence)},
                        {"contact", flags(track.contact)},
                        {"automatic_contact", flags(track.automatic_contact)},
                        {"manual_override", track.manual_override}});
  }
  nlohmann::json overrides = nlohmann::json::array();
  for (const ContactOverride& range : analysis.contact_overrides) {
    overrides.push_back({{"marker", range.marker}, {"frame0", range.frame0}, {"frame1", range.frame1}, {"contact", range.contact}});
  }
  nlohmann::json cycles = nlohmann::json::array();
  for (const GaitCycle& cycle : analysis.cycles) {
    cycles.push_back({{"index", cycle.index},
                      {"side", cycle.side},
                      {"canonical_side", cycle.canonical_side},
                      {"mirrored", cycle.mirrored},
                      {"start_time", cycle.start_time},
                      {"end_time", cycle.end_time},
                      {"start_frame", cycle.start_frame},
                      {"end_frame", cycle.end_frame},
                      {"accepted", cycle.accepted},
                      {"automatic_accepted", cycle.automatic_accepted},
                      {"rejection_reason", cycle.rejection_reason},
                      {"automatic_rejection_reason", cycle.automatic_rejection_reason},
                      {"classification", cycle.classification},
                      {"observed_regime", cycle.observed_regime},
                      {"manual_override", cycle.manual_override},
                      {"quality", {{"contact", cycle.quality.contact},
                                   {"timing", cycle.quality.timing},
                                   {"heading", cycle.quality.heading},
                                   {"continuity", cycle.quality.continuity},
                                   {"overall", cycle.quality.overall}}},
                      {"measurements", measurementsJson(cycle.measurements)},
                      {"curves", curvesJson(cycle.curves)}});
  }
  nlohmann::json relationships = nlohmann::json::array();
  for (const RelationshipFit& fit : analysis.relationships) {
    nlohmann::json bins = nlohmann::json::array();
    for (const SpeedBin& bin : fit.bins) {
      bins.push_back({{"speed_center", bin.speed_center}, {"median", bin.median}, {"p25", bin.p25}, {"p75", bin.p75}, {"count", bin.count}});
    }
    relationships.push_back({{"name", fit.name},
                             {"regime", fit.regime},
                             {"sample_count", fit.sample_count},
                             {"speed_min", fit.speed_min},
                             {"speed_max", fit.speed_max},
                             {"median", fit.median},
                             {"p25", fit.p25},
                             {"p75", fit.p75},
                             {"intercept", fit.intercept},
                             {"slope", fit.slope},
                             {"r2", fit.has_linear_fit ? nlohmann::json(fit.r2) : nlohmann::json(nullptr)},
                             {"bins", bins}});
  }
  nlohmann::json document = {
      {"format", "PROC_ANALYSIS"},
      {"version", 1},
      {"source", {{"dataset", analysis.source.dataset},
                  {"clip", analysis.source.clip},
                  {"motion_path", analysis.source.motion_path},
                  {"theme", analysis.source.theme},
                  {"subject", analysis.source.subject},
                  {"take", analysis.source.take},
                  {"fps", analysis.source.fps},
                  {"frame_count", analysis.source.frame_count},
                  {"duration_s", analysis.source.duration_s}}},
      {"settings", settingsJson(analysis.settings)},
      {"ground", {{"height_m", analysis.ground.height_m}, {"confidence", analysis.ground.confidence}, {"method", analysis.ground.method}}},
      {"source_leg_length_m", analysis.source_leg_length_m},
      {"leg_length_definition", analysis.leg_length_definition},
      {"heel", {{"provenance", analysis.heel_provenance}}},
      {"warnings", analysis.warnings},
      {"signals", signalsJson(analysis.signals)},
      {"contacts", contacts},
      {"contact_overrides", overrides},
      {"events", events},
      {"cycles", cycles},
      {"relationships", relationships}};
  return document.dump(2);
}

Status readAnalysisFromString(std::string_view text, ProcAnalysis& out, std::string_view label) {
  Status status;
  nlohmann::json document;
  try {
    document = nlohmann::json::parse(text.begin(), text.end());
  } catch (const std::exception& error) {
    status.error(std::string(label) + ": " + error.what());
    return status;
  }
  if (!document.is_object() || document.value("format", "") != "PROC_ANALYSIS" || document.value("version", 0) != 1) {
    status.error(std::string(label) + ": expected PROC_ANALYSIS version 1");
    return status;
  }
  ProcAnalysis analysis;
  const nlohmann::json source = document.value("source", nlohmann::json::object());
  analysis.source.dataset = source.value("dataset", "");
  analysis.source.clip = source.value("clip", "");
  analysis.source.motion_path = source.value("motion_path", "");
  analysis.source.theme = source.value("theme", "");
  analysis.source.subject = source.value("subject", "");
  analysis.source.take = source.value("take", "");
  analysis.source.fps = source.value("fps", 0.0);
  analysis.source.frame_count = source.value("frame_count", 0);
  analysis.source.duration_s = source.value("duration_s", 0.0);
  if (document.contains("settings")) readSettings(document["settings"], analysis.settings);
  const nlohmann::json ground = document.value("ground", nlohmann::json::object());
  analysis.ground.height_m = ground.value("height_m", 0.0);
  analysis.ground.confidence = ground.value("confidence", 0.0);
  analysis.ground.method = ground.value("method", "");
  analysis.source_leg_length_m = document.value("source_leg_length_m", 0.0);
  analysis.leg_length_definition = document.value("leg_length_definition", "");
  analysis.heel_provenance = document.value("heel", nlohmann::json::object()).value("provenance", "inferred_virtual");
  if (document.contains("warnings")) {
    for (const nlohmann::json& warning : document["warnings"]) analysis.warnings.push_back(warning.get<std::string>());
  }
  if (document.contains("signals")) readSignals(document["signals"], analysis.signals);
  if (document.contains("contacts")) {
    for (const nlohmann::json& node : document["contacts"]) {
      ContactTrack track;
      track.marker = node.value("marker", "");
      track.provenance = node.value("provenance", "");
      track.confidence = readVec(node.value("confidence", nlohmann::json::array()));
      track.contact = readFlags(node.value("contact", nlohmann::json::array()));
      track.automatic_contact = readFlags(node.value("automatic_contact", nlohmann::json::array()));
      track.manual_override = node.value("manual_override", false);
      analysis.contacts.push_back(std::move(track));
    }
  }
  if (document.contains("contact_overrides")) {
    for (const nlohmann::json& node : document["contact_overrides"]) {
      ContactOverride range;
      range.marker = node.value("marker", "");
      range.frame0 = node.value("frame0", 0);
      range.frame1 = node.value("frame1", 0);
      range.contact = node.value("contact", false);
      analysis.contact_overrides.push_back(range);
    }
  }
  if (document.contains("events")) {
    for (const nlohmann::json& node : document["events"]) {
      GaitEvent event;
      event.time = node.value("time", 0.0);
      event.automatic_time = node.value("automatic_time", event.time);
      event.frame = node.value("frame", 0);
      event.automatic_frame = node.value("automatic_frame", event.frame);
      event.side = node.value("side", "");
      event.type = node.value("type", "");
      event.automatic_type = node.value("automatic_type", event.type);
      event.confidence = node.value("confidence", 0.0);
      event.manual_override = node.value("manual_override", false);
      analysis.events.push_back(event);
    }
  }
  if (document.contains("cycles")) {
    for (const nlohmann::json& node : document["cycles"]) {
      GaitCycle cycle;
      cycle.index = node.value("index", 0);
      cycle.side = node.value("side", "");
      cycle.canonical_side = node.value("canonical_side", cycle.side);
      cycle.mirrored = node.value("mirrored", false);
      cycle.start_time = node.value("start_time", 0.0);
      cycle.end_time = node.value("end_time", 0.0);
      cycle.start_frame = node.value("start_frame", 0);
      cycle.end_frame = node.value("end_frame", 0);
      cycle.accepted = node.value("accepted", false);
      cycle.automatic_accepted = node.value("automatic_accepted", cycle.accepted);
      cycle.rejection_reason = node.value("rejection_reason", "");
      cycle.automatic_rejection_reason = node.value("automatic_rejection_reason", cycle.rejection_reason);
      cycle.classification = node.value("classification", "");
      cycle.observed_regime = node.value("observed_regime", "");
      cycle.manual_override = node.value("manual_override", false);
      const nlohmann::json quality = node.value("quality", nlohmann::json::object());
      cycle.quality.contact = quality.value("contact", 0.0);
      cycle.quality.timing = quality.value("timing", 0.0);
      cycle.quality.heading = quality.value("heading", 0.0);
      cycle.quality.continuity = quality.value("continuity", 0.0);
      cycle.quality.overall = quality.value("overall", 0.0);
      if (node.contains("measurements")) readMeasurements(node["measurements"], cycle.measurements);
      if (node.contains("curves")) readCurves(node["curves"], cycle.curves);
      analysis.cycles.push_back(std::move(cycle));
    }
  }
  if (document.contains("relationships")) {
    for (const nlohmann::json& node : document["relationships"]) {
      RelationshipFit fit;
      fit.name = node.value("name", "");
      fit.regime = node.value("regime", "");
      fit.sample_count = node.value("sample_count", 0);
      fit.speed_min = node.value("speed_min", 0.0);
      fit.speed_max = node.value("speed_max", 0.0);
      fit.median = node.value("median", 0.0);
      fit.p25 = node.value("p25", 0.0);
      fit.p75 = node.value("p75", 0.0);
      fit.intercept = node.value("intercept", 0.0);
      fit.slope = node.value("slope", 0.0);
      fit.has_linear_fit = node.contains("r2") && !node["r2"].is_null();
      fit.r2 = fit.has_linear_fit ? node["r2"].get<double>() : 0.0;
      if (node.contains("bins")) {
        for (const nlohmann::json& bin_node : node["bins"]) {
          SpeedBin bin;
          bin.speed_center = bin_node.value("speed_center", 0.0);
          bin.median = bin_node.value("median", 0.0);
          bin.p25 = bin_node.value("p25", 0.0);
          bin.p75 = bin_node.value("p75", 0.0);
          bin.count = bin_node.value("count", 0);
          fit.bins.push_back(bin);
        }
      }
      analysis.relationships.push_back(std::move(fit));
    }
  }
  out = std::move(analysis);
  return status;
}

Status writeAnalysisToFile(const ProcAnalysis& analysis, const std::filesystem::path& path) {
  Status status;
  std::error_code error;
  if (path.has_parent_path()) std::filesystem::create_directories(path.parent_path(), error);
  std::ofstream stream(path);
  if (!stream) {
    status.error("could not write " + path.string());
    return status;
  }
  stream << writeAnalysisToString(analysis);
  return status;
}

Status readAnalysisFromFile(const std::filesystem::path& path, ProcAnalysis& out) {
  std::ifstream stream(path);
  if (!stream) {
    Status status;
    status.error("could not read " + path.string());
    return status;
  }
  std::stringstream buffer;
  buffer << stream.rdbuf();
  return readAnalysisFromString(buffer.str(), out, path.string());
}

std::string cyclesToCsv(const std::vector<ProcAnalysis>& analyses) {
  std::ostringstream csv;
  csv << "dataset,clip,subject,theme,cycle_index,side,start_time,end_time,stride_time,speed_mps,stride_length_m,step_length_m,"
         "cadence_spm,stance_fraction,swing_fraction,double_support_fraction,flight_fraction,leg_length_m,normalized_stride_length,"
         "hip_flex_min,hip_flex_max,knee_flex_min,knee_flex_max,foot_clearance_m,heading_variation_deg,quality_overall,accepted,"
         "rejection_reason,observed_regime\n";
  csv.setf(std::ios::fixed);
  csv.precision(6);
  for (const ProcAnalysis& analysis : analyses) {
    for (const GaitCycle& cycle : analysis.cycles) {
      if (cycle.mirrored) continue;
      csv << csvEscape(analysis.source.dataset) << ',' << csvEscape(analysis.source.clip) << ',' << csvEscape(analysis.source.subject) << ','
          << csvEscape(analysis.source.theme) << ',' << cycle.index << ',' << cycle.side << ',' << cycle.start_time << ',' << cycle.end_time
          << ',' << cycle.measurements.stride_time << ',' << cycle.measurements.speed_mps << ',' << cycle.measurements.stride_length_m << ','
          << cycle.measurements.step_length_m << ',' << cycle.measurements.cadence_spm << ',' << cycle.measurements.stance_fraction << ','
          << cycle.measurements.swing_fraction << ',' << cycle.measurements.double_support_fraction << ','
          << cycle.measurements.flight_fraction << ',' << analysis.source_leg_length_m << ','
          << cycle.measurements.normalized_stride_length << ',' << cycle.measurements.hip_flex_min_deg << ','
          << cycle.measurements.hip_flex_max_deg << ',' << cycle.measurements.knee_flex_min_deg << ','
          << cycle.measurements.knee_flex_max_deg << ',' << cycle.measurements.foot_clearance_m << ','
          << cycle.measurements.heading_variation_deg << ',' << cycle.quality.overall << ',' << (cycle.accepted ? "1" : "0") << ','
          << csvEscape(cycle.rejection_reason) << ',' << cycle.observed_regime << '\n';
    }
  }
  return csv.str();
}

Status writeRelationshipFile(const std::vector<RelationshipFit>& fits, const std::string& regime, const std::filesystem::path& path) {
  nlohmann::json relationships = nlohmann::json::array();
  for (const RelationshipFit& fit : fits) {
    if (fit.regime != regime) continue;
    nlohmann::json bins = nlohmann::json::array();
    for (const SpeedBin& bin : fit.bins) {
      bins.push_back({{"speed_center", bin.speed_center},
                      {"median", bin.median},
                      {"p25", bin.p25},
                      {"p75", bin.p75},
                      {"count", bin.count}});
    }
    relationships.push_back({{"name", fit.name},
                             {"sample_count", fit.sample_count},
                             {"speed_min", fit.speed_min},
                             {"speed_max", fit.speed_max},
                             {"median", fit.median},
                             {"spread_p25", fit.p25},
                             {"spread_p75", fit.p75},
                             {"intercept", fit.intercept},
                             {"slope", fit.slope},
                             {"r2", fit.has_linear_fit ? nlohmann::json(fit.r2) : nlohmann::json(nullptr)},
                             {"bins", bins}});
  }
  nlohmann::json document = {{"format", "PROC_RELATIONSHIP"}, {"version", 1}, {"regime", regime}, {"relationships", relationships}};
  Status status;
  std::error_code error;
  if (path.has_parent_path()) std::filesystem::create_directories(path.parent_path(), error);
  std::ofstream stream(path);
  if (!stream) {
    status.error("could not write " + path.string());
    return status;
  }
  stream << document.dump(2);
  return status;
}

}  // namespace proc
