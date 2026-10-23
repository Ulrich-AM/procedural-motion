#pragma once

#include "app/sim_config.h"
#include "physics/body_state.h"
#include "skeleton/skeleton_definition.h"

#include <mujoco/mujoco.h>

#include <filesystem>
#include <string>
#include <unordered_map>

namespace pm {

// Articulated body with a free pelvis. There are no MuJoCo actuators (nu stays 0).
// A controller may write limited hinge torques. Cartesian pushes are separate
// and never go through those hinge torques. Nothing here sets the root pose.
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

  // Hinge torques for the next step. Cleared when the step runs, so a value
  // must be written again every tick. Root and unknown names are rejected.
  void clearJointTorques();
  bool setJointTorque(const std::string& name, double torque, double absoluteLimit);
  double appliedJointTorque(const std::string& name) const;
  double lastPeakJointTorque() const { return lastPeakTorque_; }
  double lastRootAppliedForce() const { return lastRootApplied_; }

  // World-frame force at a body center of mass, held for a short time.
  // This writes xfrc_applied, not qfrc_applied.
  bool startBodyForcePulse(const std::string& bodyName, Vec3 force, double durationSeconds);

  mjModel* model() { return model_; }
  const mjModel* model() const { return model_; }
  mjData* data() { return data_; }
  const std::string& error() const { return error_; }
  double timestep() const;

 private:
  struct HingeSlot {
    int dof = -1;
    double command = 0;
    double applied = 0;
  };

  void destroy();
  void indexHinges();

  mjModel* model_ = nullptr;
  mjData* data_ = nullptr;
  double gravityZ_ = -9.81;
  bool gravityEnabled_ = true;
  std::string error_;
  std::unordered_map<std::string, HingeSlot> hinges_;
  int rootDof_ = 0;
  double lastPeakTorque_ = 0;
  double lastRootApplied_ = 0;
  int pulseBody_ = -1;
  Vec3 pulseForce_{};
  double pulseRemaining_ = 0;
  bool pulseActive_ = false;
};

}  // namespace pm
