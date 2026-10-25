#include "body/proc_body.h"
#include "locomotion/body_scale.h"
#include "locomotion/locomotion_generator.h"

// Links the runtime only. Import, Assimp, analysis, and ImGui are not available here.
int main() {
  proc::ProcBody body;
  proc::ProcLocomotionModel model;
  proc::LocomotionState state;
  const proc::LocomotionSample sample = proc::generateLocomotionPose(body, model, state);
  const double leg = proc::targetLegLength(body);
  return sample.pose.root_rotation.w == 0.0 && leg == 0.0 ? 0 : 0;
}
