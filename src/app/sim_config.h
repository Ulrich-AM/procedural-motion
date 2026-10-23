#pragma once

#include <filesystem>
#include <string>

namespace pm {

struct SimConfig {
  double timestep = 1.0 / 120.0;
  double gravityZ = -9.81;
  double spawnClearance = 0.08;
  int solverIterations = 20;
  int maxSubsteps = 8;
};

// Missing file keeps the defaults. A present file overrides individual keys.
bool loadSimConfig(const std::filesystem::path& path, SimConfig& config, std::string& error);

}  // namespace pm
