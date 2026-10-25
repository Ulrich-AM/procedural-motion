#include "locomotion/locomotion_model_io.h"

#include <nlohmann/json.hpp>

#include <fstream>
#include <sstream>

namespace proc {
namespace {

using json = nlohmann::json;

json vec3Json(const glm::dvec3& value) { return {value.x, value.y, value.z}; }

json knotJson(const SpeedKnot& knot) {
  json rotation = json::object();
  for (int channel = 0; channel < kLocoChannels; ++channel) {
    json samples = json::array();
    for (const glm::dvec3& sample : knot.rotation[static_cast<std::size_t>(channel)]) samples.push_back(vec3Json(sample));
    rotation[locoChannelName(channel)] = std::move(samples);
  }
  auto series = [](const std::array<double, kLocoPhaseSamples>& values) {
    json node = json::array();
    for (double value : values) node.push_back(value);
    return node;
  };
  return {{"dimensionless_speed", knot.dimensionless_speed},
          {"median_speed_mps", knot.median_speed_mps},
          {"cycle_count", knot.cycle_count},
          {"quality_median", knot.quality_median},
          {"cadence_spm", knot.cadence_spm},
          {"normalized_stride_length", knot.normalized_stride_length},
          {"stance_fraction", knot.stance_fraction},
          {"flight_fraction", knot.flight_fraction},
          {"spread", knot.spread},
          {"low_confidence", knot.low_confidence},
          {"rotation", rotation},
          {"pelvis_vertical", series(knot.pelvis_vertical)},
          {"contact_left_heel", series(knot.contact_left_heel)},
          {"contact_left_toe", series(knot.contact_left_toe)},
          {"contact_right_heel", series(knot.contact_right_heel)},
          {"contact_right_toe", series(knot.contact_right_toe)}};
}

json regimeJson(const RegimeModel& regime) {
  json knots = json::array();
  for (const SpeedKnot& knot : regime.knots) knots.push_back(knotJson(knot));
  return {{"support_min", regime.support_min},
          {"support_max", regime.support_max},
          {"transition_inferred", regime.transition_inferred},
          {"seam_error", regime.seam_error},
          {"seam_channel", regime.seam_channel},
          {"knots", knots}};
}

bool readVec3(const json& node, glm::dvec3& out) {
  if (!node.is_array() || node.size() < 3) return false;
  out = glm::dvec3(node[0].get<double>(), node[1].get<double>(), node[2].get<double>());
  return true;
}

bool readSeries(const json& node, std::array<double, kLocoPhaseSamples>& out) {
  if (!node.is_array() || node.size() != kLocoPhaseSamples) return false;
  for (int index = 0; index < kLocoPhaseSamples; ++index) out[static_cast<std::size_t>(index)] = node[index].get<double>();
  return true;
}

bool readKnot(const json& node, SpeedKnot& knot, std::string& error) {
  knot.dimensionless_speed = node.value("dimensionless_speed", 0.0);
  knot.median_speed_mps = node.value("median_speed_mps", 0.0);
  knot.cycle_count = node.value("cycle_count", 0);
  knot.quality_median = node.value("quality_median", 0.0);
  knot.cadence_spm = node.value("cadence_spm", 0.0);
  knot.normalized_stride_length = node.value("normalized_stride_length", 0.0);
  knot.stance_fraction = node.value("stance_fraction", 0.0);
  knot.flight_fraction = node.value("flight_fraction", 0.0);
  knot.spread = node.value("spread", 0.0);
  knot.low_confidence = node.value("low_confidence", false);
  if (!node.contains("rotation") || !node["rotation"].is_object()) {
    error = "speed knot is missing rotation curves";
    return false;
  }
  for (int channel = 0; channel < kLocoChannels; ++channel) {
    const char* name = locoChannelName(channel);
    if (!node["rotation"].contains(name) || !node["rotation"][name].is_array() ||
        node["rotation"][name].size() != kLocoPhaseSamples) {
      error = std::string("rotation curve ") + name + " must have 100 samples";
      return false;
    }
    for (int sample = 0; sample < kLocoPhaseSamples; ++sample) {
      if (!readVec3(node["rotation"][name][sample], knot.rotation[static_cast<std::size_t>(channel)][static_cast<std::size_t>(sample)])) {
        error = std::string("bad log sample in ") + name;
        return false;
      }
    }
  }
  if (!readSeries(node.value("pelvis_vertical", json::array()), knot.pelvis_vertical) ||
      !readSeries(node.value("contact_left_heel", json::array()), knot.contact_left_heel) ||
      !readSeries(node.value("contact_left_toe", json::array()), knot.contact_left_toe) ||
      !readSeries(node.value("contact_right_heel", json::array()), knot.contact_right_heel) ||
      !readSeries(node.value("contact_right_toe", json::array()), knot.contact_right_toe)) {
    error = "scalar phase curves must have 100 samples";
    return false;
  }
  return true;
}

bool readRegime(const json& node, RegimeModel& regime, std::string& error) {
  regime = {};
  if (!node.is_object()) return true;
  regime.support_min = node.value("support_min", 0.0);
  regime.support_max = node.value("support_max", 0.0);
  regime.transition_inferred = node.value("transition_inferred", false);
  regime.seam_error = node.value("seam_error", 0.0);
  regime.seam_channel = node.value("seam_channel", "");
  if (!node.contains("knots") || !node["knots"].is_array()) return true;
  for (const json& knot_node : node["knots"]) {
    SpeedKnot knot;
    if (!readKnot(knot_node, knot, error)) return false;
    regime.knots.push_back(std::move(knot));
  }
  regime.present = !regime.knots.empty();
  return true;
}

}  // namespace

std::string writeLocomotionModelToString(const ProcLocomotionModel& model) {
  json document = {{"format", "PROC_LOCOMOTION_MODEL"},
                   {"version", 1},
                   {"skeleton", model.skeleton},
                   {"phase_samples", model.phase_samples},
                   {"speed_domain", model.speed_domain},
                   {"gravity_mps2", model.gravity_mps2},
                   {"bin_width", model.bin_width},
                   {"max_seam_error", model.max_seam_error},
                   {"max_seam_channel", model.max_seam_channel},
                   {"regimes",
                    {{"walk", regimeJson(model.walk)}, {"run", regimeJson(model.run)}, {"sprint", regimeJson(model.sprint)}}}};
  return document.dump(2) + "\n";
}

Status writeLocomotionModelToFile(const ProcLocomotionModel& model, const std::filesystem::path& path) {
  std::ofstream stream(path, std::ios::binary);
  if (!stream) {
    Status status;
    status.error("could not write " + path.string());
    return status;
  }
  stream << writeLocomotionModelToString(model);
  return {};
}

Status readLocomotionModelFromString(std::string_view text, ProcLocomotionModel& out, std::string_view label) {
  Status status;
  json document;
  try {
    document = json::parse(text.begin(), text.end());
  } catch (const std::exception& error) {
    status.error(std::string(label) + ": " + error.what());
    return status;
  }
  if (!document.is_object() || document.value("format", "") != "PROC_LOCOMOTION_MODEL" || document.value("version", 0) != 1) {
    status.error(std::string(label) + ": expected PROC_LOCOMOTION_MODEL version 1");
    return status;
  }
  if (document.value("phase_samples", 0) != kLocoPhaseSamples) {
    status.error(std::string(label) + ": phase_samples must be 100");
    return status;
  }
  if (document.value("speed_domain", "") != "dimensionless_speed") {
    status.error(std::string(label) + ": speed_domain must be dimensionless_speed");
    return status;
  }
  ProcLocomotionModel model;
  model.skeleton = document.value("skeleton", kSkeletonId);
  model.phase_samples = kLocoPhaseSamples;
  model.speed_domain = "dimensionless_speed";
  model.gravity_mps2 = document.value("gravity_mps2", kLocomotionGravity);
  model.bin_width = document.value("bin_width", 0.20);
  model.max_seam_error = document.value("max_seam_error", 0.0);
  model.max_seam_channel = document.value("max_seam_channel", "");
  const json regimes = document.value("regimes", json::object());
  std::string error;
  if (!readRegime(regimes.value("walk", json::object()), model.walk, error) ||
      !readRegime(regimes.value("run", json::object()), model.run, error) ||
      !readRegime(regimes.value("sprint", json::object()), model.sprint, error)) {
    status.error(std::string(label) + ": " + error);
    return status;
  }
  out = std::move(model);
  return status;
}

Status readLocomotionModelFromFile(const std::filesystem::path& path, ProcLocomotionModel& out) {
  std::ifstream stream(path, std::ios::binary);
  if (!stream) {
    Status status;
    status.error("could not open " + path.string());
    return status;
  }
  std::ostringstream buffer;
  buffer << stream.rdbuf();
  return readLocomotionModelFromString(buffer.str(), out, path.filename().string());
}

}  // namespace proc
