#pragma once

#include "app/sim_config.h"
#include "physics/body_state.h"
#include "skeleton/skeleton_definition.h"

#include <mujoco/mujoco.h>

#include <filesystem>
#include <string>

namespace pm {

// Passive articulated body. The pelvis is a free joint. There are no actuators
// and nothing in this type holds a pose.
class PhysicalSkeleton {
 public:
  PhysicalSkeleton() = default;
  ~PhysicalSkeleton();

  PhysicalSkeleton(const PhysicalSkeleton&) = delete;
  PhysicalSkeleton& operator=(const PhysicalSkeleton&) = delete;

  bool load(const SkeletonDefinition& skeleton, const SimConfig& config, const std::filesystem::path& xmlPath);
  void reset();
  void step();
  void setGravityEnabled(bool enabled);
  bool gravityEnabled() const { return gravityEnabled_; }
  BodyState sample();

  mjModel* model() { return model_; }
  const mjModel* model() const { return model_; }
  mjData* data() { return data_; }
  const std::string& error() const { return error_; }
  double timestep() const;

 private:
  void destroy();

  mjModel* model_ = nullptr;
  mjData* data_ = nullptr;
  double gravityZ_ = -9.81;
  bool gravityEnabled_ = true;
  std::string error_;
};

}  // namespace pm
