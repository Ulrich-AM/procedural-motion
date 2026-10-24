#include "control/foot_contact.h"

#include <algorithm>
#include <cmath>

namespace pm {
namespace {

double rawLoad(const BodyState& state, const char* side) {
  const std::string token = std::string(side) + "_foot";
  double load = 0;
  for (const ContactState& contact : state.contacts) {
    const bool foot = contact.geom1.find(token) != std::string::npos || contact.geom2.find(token) != std::string::npos;
    const bool floor = contact.geom1 == "floor" || contact.geom2 == "floor";
    if (foot && floor && contact.position.z < 0.12) load += std::max(0.0, contact.normalForce);
  }
  return load;
}

void enter(FootContact& foot, FootContactMode mode) {
  foot.mode = mode;
  foot.modeTime = 0;
}

}  // namespace

const char* footContactLabel(FootContactMode mode) {
  switch (mode) {
    case FootContactMode::Air:
      return "Air";
    case FootContactMode::Touching:
      return "Touching";
    case FootContactMode::Planted:
      return "Planted";
    case FootContactMode::Releasing:
      return "Releasing";
  }
  return "Air";
}

void FootContactTracker::reset() {
  left_ = {};
  right_ = {};
}

void FootContactTracker::update(const BodyState& state, double dt) {
  dt = std::clamp(dt, 1e-4, 0.05);
  updateSide(left_, "left", state, dt);
  updateSide(right_, "right", state, dt);
}

void FootContactTracker::updateSide(FootContact& foot, const char* side, const BodyState& state, double dt) {
  foot.load = rawLoad(state, side);
  const double follow = 1.0 - std::exp(-dt / 0.02);
  foot.filtered += (foot.load - foot.filtered) * follow;
  foot.modeTime += dt;
  const double force = foot.filtered;

  // Separate enter and exit levels. A hard landing still becomes Planted in a
  // few frames. A one-frame scuff does not.
  switch (foot.mode) {
    case FootContactMode::Air:
      if (force > 18.0 && foot.modeTime > 0.008) enter(foot, FootContactMode::Touching);
      else if (force <= 18.0) foot.modeTime = 0;
      break;
    case FootContactMode::Touching:
      if (force > 35.0 && foot.modeTime > 0.02) enter(foot, FootContactMode::Planted);
      else if (force < 8.0 && foot.modeTime > 0.015) enter(foot, FootContactMode::Air);
      else if (force <= 35.0 && force >= 8.0) foot.modeTime = 0;
      break;
    case FootContactMode::Planted:
      if (force < 15.0 && foot.modeTime > 0.02) enter(foot, FootContactMode::Releasing);
      else if (force >= 15.0) foot.modeTime = 0;
      break;
    case FootContactMode::Releasing:
      if (force > 35.0 && foot.modeTime > 0.012) enter(foot, FootContactMode::Planted);
      else if (force < 8.0 && foot.modeTime > 0.02) enter(foot, FootContactMode::Air);
      else if (force <= 35.0 && force >= 8.0) foot.modeTime = 0;
      break;
  }
}

}  // namespace pm
