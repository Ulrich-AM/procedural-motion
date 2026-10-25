#pragma once

#include "locomotion/locomotion_generator.h"

#include <cmath>

namespace proc {

// Display-only copy. Horizontal travel is removed for the viewport.
// The source pose and LocomotionState are not modified.
inline Pose poseForInPlacePreview(const Pose& raw) {
  Pose display = raw;
  display.root_translation.x = 0.0;
  display.root_translation.y = 0.0;
  return display;
}

// A render sample at another phase. Does not write the live state.
inline LocomotionState ghostSampleState(const LocomotionState& state, double phase) {
  LocomotionState copy = state;
  copy.phase = wrapPhase(phase);
  return copy;
}

}  // namespace proc
