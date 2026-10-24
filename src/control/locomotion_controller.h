#pragma once

#include "control/control_frame.h"
#include "control/foot_contact.h"
#include "control/leg_ik.h"
#include "control/standing_controller.h"
#include "physics/physical_metrics.h"
#include "skeleton/skeleton_definition.h"

#include <string>

namespace pm {

class PhysicalSkeleton;

enum class LocoState { Standing, Starting, LeftSwing, RightSwing, Landing, Stopping, Fallen };

struct LocomotionReport {
  bool enabled = false;
  LocoState state = LocoState::Standing;
  std::string swingFoot;
  Vec3 capture;
  Vec3 support;
  bool supportValid = false;
  Vec3 landing;
  bool landingValid = false;
  Vec3 desired;
  double captureError = 0;
  double stepDistance = 0;
  double phase = 0;
  double duration = 0;
  int completedSteps = 0;
  int sameSideSteps = 0;
  bool usedLeft = false;
  bool usedRight = false;
  Vec3 path[8]{};
  int pathCount = 0;
};

const char* locoStateLabel(LocoState state);

// Repeated physical steps from a desired horizontal velocity.
// Joint objectives go into the control frame. The root is never written.
class LocomotionController {
 public:
  void reset();
  void capture(const SkeletonDefinition& skeleton, const BodyState& state);
  LocomotionReport update(const BodyState& state, StandingController& standing, ControlFrame& frame,
                          const FootContactTracker& feet, const PhysicalMetrics& metrics, Vec3 desiredWorld,
                          double dt, bool enabled);

 private:
  struct Side {
    std::string name;
    LegLengths lengths;
    double sign = 1;
    bool calibrated = false;
  };

  Vec3 axisOf(const BodyKinematics& body, Vec3 local) const;
  void holdContactAngles(const BodyState& state);
  void commandHold(const BodyState& state, ControlFrame& frame, double weight) const;
  void lowerArms(const BodyState& state, ControlFrame& frame, double phase, bool swinging) const;

  Vec3 forward_{0, -1, 0};
  Vec3 right_{-1, 0, 0};
  Vec3 up_{0, 0, 1};
  Side leftLeg_;
  Side rightLeg_;
  bool captured_ = false;

  LocoState state_ = LocoState::Standing;
  std::string swing_;
  std::string lastSwing_;
  double stateTime_ = 0;
  double duration_ = 0.6;
  double swingWeight_ = 0;
  Vec3 swingStart_{};
  Vec3 ankleTarget_{};
  Vec3 landing_{};
  double stepDistance_ = 0;
  double hold_[5]{};
  bool haveHold_ = false;
  int completedSteps_ = 0;
  int sameSideSteps_ = 0;
  bool usedLeft_ = false;
  bool usedRight_ = false;
  bool braking_ = false;
  double armBlend_ = 0;
  double readyTime_ = 0;
  Vec3 capture_{};
  double captureError_ = 0;
};

}  // namespace pm
