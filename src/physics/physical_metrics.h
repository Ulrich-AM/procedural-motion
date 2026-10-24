#pragma once

namespace pm {

// Sole and stance numbers shared by the body builder and the step planners.
// Leg lengths are filled from the generated skeleton at capture time.
struct PhysicalMetrics {
  double toe = 0.09;
  double heel = 0.09;
  double halfWidth = 0.04;
  double soleHeight = 0.022;
  double ankleHeight = 0.074;
  double stanceWidth = 0.11;
  double thigh = 0.40;
  double shin = 0.42;
  double maxReach = 0.78;
};

inline PhysicalMetrics defaultPhysicalMetrics() { return {}; }

}  // namespace pm
