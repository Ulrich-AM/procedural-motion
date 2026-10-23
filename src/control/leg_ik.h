#pragma once

#include "math_types.h"

namespace pm {

struct LegLengths {
  double thigh = 0.4;
  double shin = 0.4;
  double kneeSign = -1;
  // Joint zero is not the geometric thigh-vertical angle once the leg is crouched.
  // Measured at the start of a step so a "hold the foot" solution does not yank the hip.
  double hipFlexBias = 0;
  double hipAbdBias = 0;
};

struct LegPose {
  double hipFlex = 0;
  double hipAbd = 0;
  double knee = 0;
  double anklePitch = 0;
  double ankleRoll = 0;
  bool reachable = false;
};

// Geometric two-segment IK. Returns desired joint angles only. It does not move a body.
LegPose solveLeg(Vec3 hip, Vec3 ankleTarget, Vec3 forward, Vec3 right, Vec3 up, const LegLengths& lengths);

}  // namespace pm
