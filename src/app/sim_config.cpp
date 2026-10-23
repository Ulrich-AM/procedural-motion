#include "app/sim_config.h"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <fstream>
#include <iostream>

namespace pm {

bool loadSimConfig(const std::filesystem::path& path, SimConfig& config, std::string& error) {
  error.clear();
  std::ifstream input(path);
  if (!input) {
    std::cout << "Config not found, using defaults: " << path.string() << "\n";
    return true;
  }

  try {
    nlohmann::json json;
    input >> json;
    if (json.contains("fixed_timestep_seconds")) {
      config.timestep = json.at("fixed_timestep_seconds").get<double>();
    }
    if (json.contains("gravity_z")) {
      config.gravityZ = json.at("gravity_z").get<double>();
    }
    if (json.contains("spawn_clearance_meters")) {
      config.spawnClearance = json.at("spawn_clearance_meters").get<double>();
    }
    if (json.contains("solver_iterations")) {
      config.solverIterations = json.at("solver_iterations").get<int>();
    }
    if (json.contains("max_substeps_per_frame")) {
      config.maxSubsteps = json.at("max_substeps_per_frame").get<int>();
    }
  } catch (const std::exception& exception) {
    error = std::string("Failed to parse lab.json: ") + exception.what();
    return false;
  }

  if (config.timestep < 1.0 / 1000.0 || config.timestep > 1.0 / 30.0) {
    std::cout << "Clamping timestep " << config.timestep << " into [1/1000, 1/30]\n";
    config.timestep = std::clamp(config.timestep, 1.0 / 1000.0, 1.0 / 30.0);
  }
  config.spawnClearance = std::clamp(config.spawnClearance, 0.0, 1.0);
  config.solverIterations = std::clamp(config.solverIterations, 1, 200);
  config.maxSubsteps = std::clamp(config.maxSubsteps, 1, 30);
  return true;
}

}  // namespace pm
