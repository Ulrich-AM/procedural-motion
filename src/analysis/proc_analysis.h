#pragma once

#include "analysis/signal_processing.h"
#include "import/source_motion.h"
#include "motion/proc_motion.h"

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace proc {

inline constexpr int kRotChannelCount = 13;
inline constexpr int kPhaseSamples = 101;

// Rotational channels measured from the retargeted PROC pose, not from the source skeleton.
enum class RotChannel {
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

struct AnalysisSettings {
  int algorithm_version = 1;
  double position_smooth_s = 0.050;
  double velocity_smooth_s = 0.030;
  double ground_low_percentile = 0.08;
  double ground_speed_mps = 0.30;
  double contact_height_m = 0.045;
  double contact_speed_mps = 0.55;
  double contact_vertical_speed_mps = 0.50;
  double contact_enter = 0.58;
  double contact_exit = 0.38;
  double stability_window_s = 0.040;
  double event_refine_s = 0.050;
  double min_cycle_s = 0.28;
  double max_cycle_s = 2.50;
  double min_travel_m = 0.20;
  double standing_travel_m = 0.05;
  double max_heading_deg = 35.0;
  double min_contact_confidence = 0.40;
  int phase_samples = kPhaseSamples;
  // Heel sits this fraction of the source foot length behind the ankle, opposite the toe.
  double heel_fraction = 0.22;
  // froude = speed_mps / sqrt(gravity_mps2 * source_leg_length_m)
  double gravity_mps2 = 9.80665;
};

struct SignalFrame {
  double time = 0.0;
  double root_x = 0.0;
  double root_y = 0.0;
  double root_z = 0.0;
  double root_horizontal_speed = 0.0;
  double root_speed = 0.0;
  double left_foot_x = 0.0;
  double left_foot_y = 0.0;
  double left_foot_z = 0.0;
  double right_foot_x = 0.0;
  double right_foot_y = 0.0;
  double right_foot_z = 0.0;
  double left_toe_x = 0.0;
  double left_toe_y = 0.0;
  double left_toe_z = 0.0;
  double right_toe_x = 0.0;
  double right_toe_y = 0.0;
  double right_toe_z = 0.0;
  double left_heel_x = 0.0;
  double left_heel_y = 0.0;
  double left_heel_z = 0.0;
  double right_heel_x = 0.0;
  double right_heel_y = 0.0;
  double right_heel_z = 0.0;
  double left_foot_horizontal_speed = 0.0;
  double left_foot_vertical_speed = 0.0;
  double left_foot_speed = 0.0;
  double right_foot_horizontal_speed = 0.0;
  double right_foot_vertical_speed = 0.0;
  double right_foot_speed = 0.0;
  double left_toe_horizontal_speed = 0.0;
  double left_toe_vertical_speed = 0.0;
  double right_toe_horizontal_speed = 0.0;
  double right_toe_vertical_speed = 0.0;
  double left_heel_horizontal_speed = 0.0;
  double left_heel_vertical_speed = 0.0;
  double right_heel_horizontal_speed = 0.0;
  double right_heel_vertical_speed = 0.0;
  double left_foot_height = 0.0;
  double right_foot_height = 0.0;
  double left_toe_height = 0.0;
  double right_toe_height = 0.0;
  double left_heel_height = 0.0;
  double right_heel_height = 0.0;
  std::array<glm::dquat, kRotChannelCount> rotation{};
  std::array<glm::dvec3, kRotChannelCount> log_vector{};
  std::array<double, kRotChannelCount> sagittal_deg{};
  double pelvis_pitch_deg = 0.0;
  double pelvis_roll_deg = 0.0;
};

struct GroundEstimate {
  double height_m = 0.0;
  double confidence = 0.0;
  std::string method;
};

struct ContactTrack {
  std::string marker;
  std::string provenance;
  std::vector<double> confidence;
  std::vector<std::uint8_t> contact;
  std::vector<std::uint8_t> automatic_contact;
  bool manual_override = false;
};

struct ContactOverride {
  std::string marker;
  int frame0 = 0;
  int frame1 = 0;
  bool contact = false;
};

struct GaitEvent {
  double time = 0.0;
  double automatic_time = 0.0;
  int frame = 0;
  int automatic_frame = 0;
  std::string side;
  std::string type;
  std::string automatic_type;
  double confidence = 0.0;
  bool manual_override = false;
};

struct CycleQuality {
  double contact = 0.0;
  double timing = 0.0;
  double heading = 0.0;
  double continuity = 0.0;
  double overall = 0.0;
};

struct CycleMeasurements {
  double stride_time = 0.0;
  double step_time = 0.0;
  double stance_time = 0.0;
  double swing_time = 0.0;
  double cadence_spm = 0.0;
  double speed_mps = 0.0;
  double stride_length_m = 0.0;
  double step_length_m = 0.0;
  double stance_fraction = 0.0;
  double swing_fraction = 0.0;
  double double_support_fraction = 0.0;
  double flight_fraction = 0.0;
  double foot_clearance_m = 0.0;
  double toe_clearance_m = 0.0;
  double contact_velocity_mps = 0.0;
  double pelvis_height_m = 0.0;
  double pelvis_vertical_excursion_m = 0.0;
  double pelvis_pitch_excursion_deg = 0.0;
  double pelvis_roll_excursion_deg = 0.0;
  double hip_flex_min_deg = 0.0;
  double hip_flex_max_deg = 0.0;
  double hip_flex_min_phase = 0.0;
  double hip_flex_max_phase = 0.0;
  double knee_flex_min_deg = 0.0;
  double knee_flex_max_deg = 0.0;
  double knee_flex_min_phase = 0.0;
  double knee_flex_max_phase = 0.0;
  double ankle_flex_min_deg = 0.0;
  double ankle_flex_max_deg = 0.0;
  double ankle_flex_min_phase = 0.0;
  double ankle_flex_max_phase = 0.0;
  double shoulder_swing_deg = 0.0;
  double arm_leg_phase = 0.0;
  double travel_direction_deg = 0.0;
  double heading_variation_deg = 0.0;
  double normalized_stride_length = 0.0;
  double normalized_step_length = 0.0;
  double normalized_foot_clearance = 0.0;
  double normalized_pelvis_height = 0.0;
  double froude_number = 0.0;
};

struct PhaseCurves {
  std::vector<double> hip_flex_deg;
  std::vector<double> knee_flex_deg;
  std::vector<double> ankle_flex_deg;
  std::vector<double> opposite_hip_flex_deg;
  std::vector<double> pelvis_height_m;
  std::vector<double> pelvis_pitch_deg;
  std::vector<double> pelvis_roll_deg;
  std::vector<double> foot_height_m;
  std::vector<double> toe_height_m;
  std::vector<double> contact_confidence;
  std::vector<double> shoulder_flex_deg;
};

struct GaitCycle {
  int index = 0;
  std::string side;
  std::string canonical_side;
  bool mirrored = false;
  double start_time = 0.0;
  double end_time = 0.0;
  int start_frame = 0;
  int end_frame = 0;
  bool accepted = false;
  bool automatic_accepted = false;
  std::string rejection_reason;
  std::string automatic_rejection_reason;
  std::string classification;
  std::string observed_regime;
  bool manual_override = false;
  CycleQuality quality;
  CycleMeasurements measurements;
  PhaseCurves curves;
};

struct SpeedBin {
  double speed_center = 0.0;
  double median = 0.0;
  double p25 = 0.0;
  double p75 = 0.0;
  int count = 0;
};

struct RelationshipFit {
  std::string name;
  std::string regime;
  int sample_count = 0;
  double speed_min = 0.0;
  double speed_max = 0.0;
  double median = 0.0;
  double p25 = 0.0;
  double p75 = 0.0;
  double intercept = 0.0;
  double slope = 0.0;
  double r2 = 0.0;
  bool has_linear_fit = false;
  std::vector<SpeedBin> bins;
};

struct AnalysisSourceInfo {
  std::string dataset;
  std::string clip;
  std::string motion_path;
  std::string theme;
  std::string subject;
  std::string take;
  double fps = 0.0;
  int frame_count = 0;
  double duration_s = 0.0;
};

struct ProcAnalysis {
  std::string format = "PROC_ANALYSIS";
  int version = 1;
  AnalysisSourceInfo source;
  AnalysisSettings settings;
  GroundEstimate ground;
  double source_leg_length_m = 0.0;
  std::string leg_length_definition;
  std::string heel_provenance = "inferred_virtual";
  std::vector<SignalFrame> signals;
  std::vector<ContactTrack> contacts;
  std::vector<ContactOverride> contact_overrides;
  std::vector<GaitEvent> events;
  std::vector<GaitCycle> cycles;
  std::vector<RelationshipFit> relationships;
  std::vector<std::string> warnings;
};

struct AnalysisInput {
  const SourceClip* source = nullptr;
  const ProcMotion* motion = nullptr;
  std::string dataset;
  std::string theme;
  std::string subject;
  std::string take;
  std::string motion_path;
};

ProcAnalysis analyzeClip(const AnalysisInput& input, const AnalysisSettings& settings = {});

void moveEvent(ProcAnalysis& analysis, int event_index, double time_s);
void setEventType(ProcAnalysis& analysis, int event_index, const std::string& type);
void setCycleAccepted(ProcAnalysis& analysis, int cycle_index, bool accepted, const std::string& reason);
void overrideContactRange(ProcAnalysis& analysis, const std::string& marker, int frame0, int frame1, bool contact);

const char* rotChannelName(RotChannel channel);
int nearestFrame(const ProcAnalysis& analysis, double time_s);

}  // namespace proc
