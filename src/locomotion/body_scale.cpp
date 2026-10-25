#include "locomotion/body_scale.h"

#include <cmath>

namespace proc {

double targetLegLength(const ProcBody& body) {
  auto lengthOf = [&](const char* name) {
    for (const BoneGeom& bone : body.bones) {
      if (bone.name == name) return bone.length_m;
    }
    return 0.0;
  };
  const double left = lengthOf("lleguppb") + lengthOf("lleglowb");
  const double right = lengthOf("rleguppb") + lengthOf("rleglowb");
  if (left > 0.0 && right > 0.0) return 0.5 * (left + right);
  return std::max(left, right);
}

double dimensionlessSpeed(double speed_mps, double leg_length_m, double gravity_mps2) {
  if (!(leg_length_m > 0.0) || !(gravity_mps2 > 0.0)) return 0.0;
  return speed_mps / std::sqrt(gravity_mps2 * leg_length_m);
}

}  // namespace proc
