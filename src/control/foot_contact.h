#pragma once

#include "physics/body_state.h"

#include <string>

namespace pm {

enum class FootContactMode { Air, Touching, Planted, Releasing };

struct FootContact {
  FootContactMode mode = FootContactMode::Air;
  double load = 0;
  double filtered = 0;
  double modeTime = 0;
};

const char* footContactLabel(FootContactMode mode);

// Hysteresis around foot load so one noisy frame cannot start or end a step.
class FootContactTracker {
 public:
  void reset();
  void update(const BodyState& state, double dt);
  const FootContact& left() const { return left_; }
  const FootContact& right() const { return right_; }
  const FootContact& of(const std::string& side) const { return side == "left" ? left_ : right_; }

 private:
  void updateSide(FootContact& foot, const char* side, const BodyState& state, double dt);

  FootContact left_;
  FootContact right_;
};

}  // namespace pm
