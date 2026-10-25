#include "analysis/measurements.h"

#include "analysis/contact_detector.h"

#include <algorithm>
#include <cmath>

namespace proc {
namespace {

constexpr double kPi = 3.14159265358979323846;

const ContactTrack* findTrack(const ProcAnalysis& analysis, const std::string& name) {
  for (const ContactTrack& track : analysis.contacts) {
    if (track.marker == name) return &track;
  }
  return nullptr;
}

bool sideSupport(const ProcAnalysis& analysis, const std::string& side, int frame) {
  const ContactTrack* heel = findTrack(analysis, side + "_heel");
  const ContactTrack* toe = findTrack(analysis, side + "_toe");
  const bool heel_on = heel != nullptr && frame >= 0 && frame < static_cast<int>(heel->contact.size()) && heel->contact[static_cast<std::size_t>(frame)] != 0;
  const bool toe_on = toe != nullptr && frame >= 0 && frame < static_cast<int>(toe->contact.size()) && toe->contact[static_cast<std::size_t>(frame)] != 0;
  return heel_on || toe_on;
}

double meanConfidence(const ProcAnalysis& analysis, const std::string& side, int begin, int end, bool stance_only) {
  const ContactTrack* heel = findTrack(analysis, side + "_heel");
  const ContactTrack* toe = findTrack(analysis, side + "_toe");
  double sum = 0.0;
  int count = 0;
  for (int frame = begin; frame < end; ++frame) {
    if (stance_only && !sideSupport(analysis, side, frame)) continue;
    double value = 0.0;
    int parts = 0;
    if (heel != nullptr && frame < static_cast<int>(heel->confidence.size())) {
      value += heel->confidence[static_cast<std::size_t>(frame)];
      ++parts;
    }
    if (toe != nullptr && frame < static_cast<int>(toe->confidence.size())) {
      value += toe->confidence[static_cast<std::size_t>(frame)];
      ++parts;
    }
    if (parts == 0) continue;
    sum += value / static_cast<double>(parts);
    ++count;
  }
  return count == 0 ? 0.0 : sum / static_cast<double>(count);
}

RotChannel hipOf(const std::string& side) { return side == "left" ? RotChannel::LeftHip : RotChannel::RightHip; }
RotChannel kneeOf(const std::string& side) { return side == "left" ? RotChannel::LeftKnee : RotChannel::RightKnee; }
RotChannel ankleOf(const std::string& side) { return side == "left" ? RotChannel::LeftAnkle : RotChannel::RightAnkle; }
RotChannel shoulderOf(const std::string& side) { return side == "left" ? RotChannel::LeftShoulder : RotChannel::RightShoulder; }

double footHeight(const SignalFrame& frame, const std::string& side) { return side == "left" ? frame.left_foot_height : frame.right_foot_height; }
double toeHeight(const SignalFrame& frame, const std::string& side) { return side == "left" ? frame.left_toe_height : frame.right_toe_height; }
double footHorizontalSpeed(const SignalFrame& frame, const std::string& side) {
  return side == "left" ? frame.left_foot_horizontal_speed : frame.right_foot_horizontal_speed;
}
double footY(const SignalFrame& frame, const std::string& side) { return side == "left" ? frame.left_foot_y : frame.right_foot_y; }
double footX(const SignalFrame& frame, const std::string& side) { return side == "left" ? frame.left_foot_x : frame.right_foot_x; }

void extrema(const ProcAnalysis& analysis, int begin, int end, double start, double stride, RotChannel channel, double& min_value,
             double& max_value, double& min_phase, double& max_phase) {
  min_value = 0.0;
  max_value = 0.0;
  min_phase = 0.0;
  max_phase = 0.0;
  if (begin >= end) return;
  bool first = true;
  for (int frame = begin; frame < end; ++frame) {
    const SignalFrame& sample = analysis.signals[static_cast<std::size_t>(frame)];
    const double value = sample.sagittal_deg[static_cast<int>(channel)];
    const double phase = stride > 1e-9 ? (sample.time - start) / stride : 0.0;
    if (first || value < min_value) {
      min_value = value;
      min_phase = phase;
    }
    if (first || value > max_value) {
      max_value = value;
      max_phase = phase;
    }
    first = false;
  }
}

}  // namespace

void measureCycle(ProcAnalysis& analysis, GaitCycle& cycle) {
  CycleMeasurements& measured = cycle.measurements;
  measured = {};
  if (analysis.signals.empty()) return;
  const int count = static_cast<int>(analysis.signals.size());
  const int begin = std::clamp(cycle.start_frame, 0, count - 1);
  const int end = std::clamp(cycle.end_frame, begin, count - 1);
  const SignalFrame& first = analysis.signals[static_cast<std::size_t>(begin)];
  const SignalFrame& last = analysis.signals[static_cast<std::size_t>(end)];
  measured.stride_time = std::max(0.0, last.time - first.time);
  const double dx = last.root_x - first.root_x;
  const double dy = last.root_y - first.root_y;
  measured.stride_length_m = std::sqrt(dx * dx + dy * dy);
  measured.speed_mps = measured.stride_time > 1e-9 ? measured.stride_length_m / measured.stride_time : 0.0;
  measured.travel_direction_deg = std::atan2(dx, dy) * (180.0 / kPi);

  const int mid = std::clamp((begin + end) / 2, begin, end);
  const SignalFrame& middle = analysis.signals[static_cast<std::size_t>(mid)];
  const double ax = middle.root_x - first.root_x;
  const double ay = middle.root_y - first.root_y;
  const double bx = last.root_x - middle.root_x;
  const double by = last.root_y - middle.root_y;
  const double an = std::sqrt(ax * ax + ay * ay);
  const double bn = std::sqrt(bx * bx + by * by);
  if (an > 1e-4 && bn > 1e-4) {
    const double cosine = std::clamp((ax * bx + ay * by) / (an * bn), -1.0, 1.0);
    measured.heading_variation_deg = std::acos(cosine) * (180.0 / kPi);
  }

  const std::string other = cycle.side == "left" ? "right" : "left";
  double opposite_time = -1.0;
  int opposite_frame = -1;
  for (const GaitEvent& event : analysis.events) {
    if (event.manual_override) {
      // Manual edits are honored. The automatic copy stays on automatic_time.
    }
    if (event.side == other && event.type == "initial_contact" && event.time > first.time + 1e-6 && event.time < last.time - 1e-6) {
      opposite_time = event.time;
      opposite_frame = event.frame;
      break;
    }
  }
  if (opposite_time >= 0.0) {
    measured.step_time = opposite_time - first.time;
    const int foot_frame = std::clamp(opposite_frame, 0, count - 1);
    const SignalFrame& other_frame = analysis.signals[static_cast<std::size_t>(foot_frame)];
    const double step_x = footX(other_frame, other) - footX(first, cycle.side);
    const double step_y = footY(other_frame, other) - footY(first, cycle.side);
    const double travel = measured.stride_length_m;
    if (travel > 1e-6) measured.step_length_m = (step_x * dx + step_y * dy) / travel;
    else measured.step_length_m = std::sqrt(step_x * step_x + step_y * step_y);
  }
  measured.cadence_spm = measured.step_time > 1e-6 ? 60.0 / measured.step_time : (measured.stride_time > 1e-6 ? 120.0 / measured.stride_time : 0.0);

  double stance = 0.0;
  double swing = 0.0;
  double doubles = 0.0;
  double flight = 0.0;
  double span = 0.0;
  double pelvis_min = first.root_z;
  double pelvis_max = first.root_z;
  double pelvis_sum = 0.0;
  int pelvis_count = 0;
  double pitch_min = first.pelvis_pitch_deg;
  double pitch_max = first.pelvis_pitch_deg;
  double roll_min = first.pelvis_roll_deg;
  double roll_max = first.pelvis_roll_deg;
  double foot_clear = 0.0;
  double toe_clear = 0.0;
  int transitions = 0;
  bool previous = sideSupport(analysis, cycle.side, begin);
  for (int frame = begin; frame < end; ++frame) {
    const double dt = analysis.signals[static_cast<std::size_t>(frame + 1)].time - analysis.signals[static_cast<std::size_t>(frame)].time;
    const bool own = sideSupport(analysis, cycle.side, frame);
    const bool opposite = sideSupport(analysis, other, frame);
    if (own) stance += dt;
    else swing += dt;
    if (own && opposite) doubles += dt;
    if (!own && !opposite) flight += dt;
    span += dt;
    const SignalFrame& sample = analysis.signals[static_cast<std::size_t>(frame)];
    pelvis_min = std::min(pelvis_min, sample.root_z);
    pelvis_max = std::max(pelvis_max, sample.root_z);
    pelvis_sum += sample.root_z;
    ++pelvis_count;
    pitch_min = std::min(pitch_min, sample.pelvis_pitch_deg);
    pitch_max = std::max(pitch_max, sample.pelvis_pitch_deg);
    roll_min = std::min(roll_min, sample.pelvis_roll_deg);
    roll_max = std::max(roll_max, sample.pelvis_roll_deg);
    if (!own) {
      foot_clear = std::max(foot_clear, footHeight(sample, cycle.side));
      toe_clear = std::max(toe_clear, toeHeight(sample, cycle.side));
    }
    if (frame > begin && own != previous) ++transitions;
    previous = own;
  }
  const double denom = span > 1e-9 ? span : 1.0;
  measured.stance_time = stance;
  measured.swing_time = swing;
  measured.stance_fraction = stance / denom;
  measured.swing_fraction = swing / denom;
  measured.double_support_fraction = doubles / denom;
  measured.flight_fraction = flight / denom;
  measured.foot_clearance_m = foot_clear;
  measured.toe_clearance_m = toe_clear;
  measured.contact_velocity_mps = footHorizontalSpeed(first, cycle.side);
  measured.pelvis_height_m = pelvis_count > 0 ? pelvis_sum / static_cast<double>(pelvis_count) : first.root_z;
  measured.pelvis_vertical_excursion_m = pelvis_max - pelvis_min;
  measured.pelvis_pitch_excursion_deg = pitch_max - pitch_min;
  measured.pelvis_roll_excursion_deg = roll_max - roll_min;

  extrema(analysis, begin, end, first.time, measured.stride_time, hipOf(cycle.side), measured.hip_flex_min_deg, measured.hip_flex_max_deg,
          measured.hip_flex_min_phase, measured.hip_flex_max_phase);
  extrema(analysis, begin, end, first.time, measured.stride_time, kneeOf(cycle.side), measured.knee_flex_min_deg, measured.knee_flex_max_deg,
          measured.knee_flex_min_phase, measured.knee_flex_max_phase);
  extrema(analysis, begin, end, first.time, measured.stride_time, ankleOf(cycle.side), measured.ankle_flex_min_deg,
          measured.ankle_flex_max_deg, measured.ankle_flex_min_phase, measured.ankle_flex_max_phase);

  double shoulder_min = 0.0;
  double shoulder_max = 0.0;
  double shoulder_min_phase = 0.0;
  double shoulder_max_phase = 0.0;
  extrema(analysis, begin, end, first.time, measured.stride_time, shoulderOf(cycle.side), shoulder_min, shoulder_max, shoulder_min_phase,
          shoulder_max_phase);
  measured.shoulder_swing_deg = shoulder_max - shoulder_min;
  double phase_delta = shoulder_max_phase - measured.hip_flex_max_phase;
  while (phase_delta > 0.5) phase_delta -= 1.0;
  while (phase_delta < -0.5) phase_delta += 1.0;
  measured.arm_leg_phase = phase_delta;

  const double leg = analysis.source_leg_length_m;
  if (leg > 1e-6) {
    measured.normalized_stride_length = measured.stride_length_m / leg;
    measured.normalized_step_length = measured.step_length_m / leg;
    measured.normalized_foot_clearance = measured.foot_clearance_m / leg;
    measured.normalized_pelvis_height = measured.pelvis_height_m / leg;
    measured.froude_number = measured.speed_mps / std::sqrt(analysis.settings.gravity_mps2 * leg);
  }

  const bool duration_ok = measured.stride_time >= 0.35 && measured.stride_time <= 2.0;
  int toe_offs = 0;
  for (const GaitEvent& event : analysis.events) {
    if (event.side == cycle.side && event.type == "toe_off" && event.time > first.time + 1e-6 && event.time < last.time - 1e-6) ++toe_offs;
  }
  cycle.quality.contact = meanConfidence(analysis, cycle.side, begin, end, true);
  cycle.quality.timing = (duration_ok ? 1.0 : 0.35) * (toe_offs == 1 ? 1.0 : 0.45);
  cycle.quality.heading = std::clamp(1.0 - measured.heading_variation_deg / std::max(analysis.settings.max_heading_deg, 1.0), 0.0, 1.0);
  cycle.quality.continuity = std::clamp(1.0 - std::max(0, transitions - 1) / 4.0, 0.0, 1.0);
  cycle.quality.overall = 0.40 * cycle.quality.contact + 0.25 * cycle.quality.timing + 0.20 * cycle.quality.heading + 0.15 * cycle.quality.continuity;

  if (measured.flight_fraction > 0.08) cycle.observed_regime = measured.speed_mps >= 5.0 ? "sprint" : "run";
  else if (measured.speed_mps >= 5.0 && measured.stance_fraction < 0.35) cycle.observed_regime = "sprint";
  else if (measured.speed_mps < 0.25 && measured.stride_length_m < analysis.settings.min_travel_m) cycle.observed_regime = "unknown";
  else cycle.observed_regime = "walk";
}

}  // namespace proc
