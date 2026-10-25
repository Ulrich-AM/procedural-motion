#pragma once

#include "body/skeleton.h"
#include "math/quat.h"

#include <array>
#include <string>
#include <vector>

namespace proc {

inline constexpr int kLocoPhaseSamples = 100;
inline constexpr int kLocoChannels = 13;
inline constexpr double kLocomotionGravity = 9.80665;

// Same channel order as the analysis rotational signals.
enum class LocoChannel : int {
  Pelvis = 0,
  Spine1,
  Spine2,
  LeftHip,
  RightHip,
  LeftKnee,
  RightKnee,
  LeftAnkle,
  RightAnkle,
  LeftShoulder,
  RightShoulder,
  LeftElbow,
  RightElbow
};

enum class LocomotionRegime { Idle, Walk, Run, Sprint };
enum class RegimeForce { Auto, Walk, Run, Sprint };

struct SpeedKnot {
  double dimensionless_speed = 0.0;
  double median_speed_mps = 0.0;
  int cycle_count = 0;
  double quality_median = 0.0;
  double cadence_spm = 0.0;
  double normalized_stride_length = 0.0;
  double stance_fraction = 0.0;
  double flight_fraction = 0.0;
  double spread = 0.0;
  bool low_confidence = false;
  std::array<std::array<glm::dvec3, kLocoPhaseSamples>, kLocoChannels> rotation{};
  std::array<double, kLocoPhaseSamples> pelvis_vertical{};
  std::array<double, kLocoPhaseSamples> contact_left_heel{};
  std::array<double, kLocoPhaseSamples> contact_left_toe{};
  std::array<double, kLocoPhaseSamples> contact_right_heel{};
  std::array<double, kLocoPhaseSamples> contact_right_toe{};
};

struct RegimeModel {
  bool present = false;
  double support_min = 0.0;
  double support_max = 0.0;
  bool transition_inferred = false;
  double seam_error = 0.0;
  std::string seam_channel;
  std::vector<SpeedKnot> knots;
};

struct ProcLocomotionModel {
  std::string format = "PROC_LOCOMOTION_MODEL";
  int version = 1;
  std::string skeleton = kSkeletonId;
  int phase_samples = kLocoPhaseSamples;
  std::string speed_domain = "dimensionless_speed";
  double gravity_mps2 = kLocomotionGravity;
  double bin_width = 0.20;
  RegimeModel walk;
  RegimeModel run;
  RegimeModel sprint;
  double max_seam_error = 0.0;
  std::string max_seam_channel;
};

struct ModelBuildSettings {
  double bin_width = 0.20;
  int min_cycles_per_knot = 2;
  double min_quality = 0.0;
  std::string regime;
  std::string subject;
  double gravity_mps2 = kLocomotionGravity;
  double seam_warn = 0.35;
};

const char* locoChannelName(int channel);
const char* regimeName(LocomotionRegime regime);
JointId locoJoint(int channel);

// Reflection across the sagittal plane (flip X), then the caller swaps left and right.
glm::dquat mirrorSagittal(glm::dquat rotation);
void mirrorLogChannels(std::array<glm::dvec3, kLocoChannels>& logs);

double wrapPhase(double phase);
glm::dvec3 sampleLog(const std::array<glm::dvec3, kLocoPhaseSamples>& curve, double phase);
double samplePeriodic(const std::array<double, kLocoPhaseSamples>& curve, double phase);

}  // namespace proc
