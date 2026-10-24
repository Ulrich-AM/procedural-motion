#pragma once

#include "control/control_frame.h"
#include "control/foot_contact.h"
#include "control/leg_ik.h"
#include "control/standing_controller.h"
#include "physics/physical_metrics.h"
#include "skeleton/skeleton_definition.h"

#include <string>

namespace pm {

enum class StepState { Idle, Prepare, Swing, Plant, Settle };

struct RecoveryReport {
  bool enabled = false;
  StepState state = StepState::Idle;
  std::string swingFoot;
  Vec3 capture;
  Vec3 support;
  bool supportValid = false;
  Vec3 landing;
  bool landingValid = false;
  double captureError = 0;
  double stepDistance = 0;
  double phase = 0;
  double duration = 0;
  Vec3 path[8]{};
  int pathCount = 0;
};

const char* stepStateLabel(StepState state);

// One recovery step. Standing remains the first response. Swing objectives are
// written into the control frame and faded out once the foot is planted.
class RecoveryStep {
 public:
  void reset();
  void capture(const SkeletonDefinition& skeleton, const BodyState& state);
  RecoveryReport update(const BodyState& state, StandingController& standing, ControlFrame& frame,
                        const FootContactTracker& feet, double dt, bool enabled);
  const PhysicalMetrics& metrics() const { return metrics_; }

 private:
  struct Side {
    std::string name;
    LegLengths lengths;
    double sign = 1;
    bool calibrated = false;
  };

  Vec3 axisOf(const BodyKinematics& body, Vec3 local) const;
  double footLoad(const BodyState& state, const std::string& side) const;
  void supportOf(const BodyState& state, Vec3 forward, Vec3 right, Vec3& center, double& sagMin, double& sagMax,
                 double& latMin, double& latMax, bool& valid) const;

  Vec3 forward_{0, -1, 0};
  Vec3 right_{-1, 0, 0};
  Vec3 up_{0, 0, 1};
  Side leftLeg_;
  Side rightLeg_;
  PhysicalMetrics metrics_{};
  bool captured_ = false;

  StepState state_ = StepState::Idle;
  std::string swing_;
  bool armed_ = true;
  double stateTime_ = 0;
  double duration_ = 0.36;
  double swingWeight_ = 0;
  Vec3 swingStart_{};
  Vec3 landing_{};
  Vec3 ankleTarget_{};
  double stepDistance_ = 0;
  double outsideTime_ = 0;
  double insideTime_ = 0;
  int stepsTaken_ = 0;
  double hold_[5]{};
  bool haveHold_ = false;
};

}  // namespace pm
