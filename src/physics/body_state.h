#pragma once

#include "math_types.h"

#include <string>
#include <vector>

namespace pm {

// One rigid body after the latest forward kinematics.
struct BodyKinematics {
  std::string name;
  Vec3 position;
  Vec3 com;
  double rotation[9]{};
  double quat[4]{1, 0, 0, 0};
  Vec3 linearVelocity;
  Vec3 angularVelocity;
};

// One MuJoCo joint. Hinge angle is in radians. Free joints leave angle at 0;
// the root pose lives on the pelvis body instead.
struct JointState {
  std::string name;
  std::string type;
  double angle = 0;
  double rangeMin = 0;
  double rangeMax = 0;
  Vec3 anchor;
  Vec3 axis;
};

struct ContactState {
  Vec3 position;
  Vec3 normal;
  double normalForce = 0;
  std::string geom1;
  std::string geom2;
};

// Read-only snapshot for later balance, contact, and gait work.
// Phase 0 only displays it. Nothing corrects the pose from this data.
struct BodyState {
  double time = 0;
  Vec3 centerOfMass;
  Vec3 centerOfMassVelocity;
  double mass = 0;
  std::vector<BodyKinematics> bodies;
  std::vector<JointState> joints;
  std::vector<ContactState> contacts;
};

}  // namespace pm
