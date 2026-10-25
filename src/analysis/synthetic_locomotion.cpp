#include "analysis/synthetic_locomotion.h"

#include "analysis/locomotion_model_builder.h"
#include "body/joint_limits.h"
#include "locomotion/body_scale.h"
#include "rig/forward_kinematics.h"

#include <algorithm>
#include <cmath>

namespace proc {
namespace {

const glm::dvec3 kX(1.0, 0.0, 0.0);
constexpr double kPi = 3.14159265358979323846;

struct GaitStyle {
  double knee_flex = 0.90;
  double ankle_dorsi = 0.26;
  double ankle_plantar = 0.38;
  double arm_swing = 0.28;
  double elbow_flex = 0.42;
  double elbow_mod = 0.10;
  double pelvis_pitch = 0.05;
  double pelvis_roll = 0.025;
  double spine_yaw = 0.07;
};

GaitStyle styleFor(const std::string& regime) {
  GaitStyle style;
  if (regime == "run") {
    style.knee_flex = 1.20;
    style.ankle_dorsi = 0.32;
    style.ankle_plantar = 0.48;
    style.arm_swing = 0.46;
    style.elbow_flex = 0.72;
    style.elbow_mod = 0.16;
    style.pelvis_pitch = 0.07;
    style.pelvis_roll = 0.035;
    style.spine_yaw = 0.09;
  } else if (regime == "sprint") {
    style.knee_flex = 1.50;
    style.ankle_dorsi = 0.36;
    style.ankle_plantar = 0.55;
    style.arm_swing = 0.58;
    style.elbow_flex = 0.95;
    style.elbow_mod = 0.18;
    style.pelvis_pitch = 0.08;
    style.pelvis_roll = 0.04;
    style.spine_yaw = 0.11;
  }
  return style;
}

glm::dquat pelvisRotation(double phase, const GaitStyle& style) {
  const double pitch = style.pelvis_pitch * std::sin(4.0 * kPi * phase);
  const double roll = style.pelvis_roll * std::sin(2.0 * kPi * phase);
  return quatAxisAngle(glm::dvec3(0.0, 1.0, 0.0), roll) * quatAxisAngle(kX, pitch);
}

double footY(const ProcBody& body, JointId hip, JointId knee, JointId foot, double root_y, double hip_angle, double knee_angle,
             const glm::dquat& pelvis) {
  Pose pose;
  pose.root_translation = glm::dvec3(0.0, root_y, 0.0);
  pose.root_rotation = pelvis;
  pose.local_rotation[jointIndex(hip)] = quatAxisAngle(kX, hip_angle);
  pose.local_rotation[jointIndex(knee)] = quatAxisAngle(body.constraints[jointIndex(knee)].axis, knee_angle);
  return forwardKinematics(body, pose).joints[jointIndex(foot)].position.y;
}

double solveHip(const ProcBody& body, JointId hip, JointId knee, JointId foot, double root_y, double target_y, const glm::dquat& pelvis) {
  const double at_low = footY(body, hip, knee, foot, root_y, -1.05, 0.0, pelvis);
  const double at_high = footY(body, hip, knee, foot, root_y, 1.05, 0.0, pelvis);
  const bool rising = at_high > at_low;
  double low = -1.05;
  double high = 1.05;
  for (int step = 0; step < 28; ++step) {
    const double mid = 0.5 * (low + high);
    const double value = footY(body, hip, knee, foot, root_y, mid, 0.0, pelvis);
    if ((value < target_y) == rising) low = mid;
    else high = mid;
  }
  return std::clamp(0.5 * (low + high), -1.05, 1.05);
}

struct LegCurve {
  std::array<double, kLocoPhaseSamples> hip{};
  std::array<double, kLocoPhaseSamples> knee{};
  std::array<double, kLocoPhaseSamples> ankle{};
};

double ankleAngle(double phase, double stance, const GaitStyle& style) {
  if (phase < stance) {
    const double u = phase / std::max(stance, 1e-4);
    if (u < 0.55) return style.ankle_dorsi * std::sin(u / 0.55 * kPi * 0.5);
    const double blend = (u - 0.55) / 0.45;
    return style.ankle_dorsi * (1.0 - blend) - style.ankle_plantar * blend;
  }
  const double swing = (phase - stance) / std::max(1e-4, 1.0 - stance);
  if (swing < 0.35) {
    const double blend = swing / 0.35;
    return -style.ankle_plantar * (1.0 - blend) + style.ankle_dorsi * 0.55 * blend;
  }
  const double blend = (swing - 0.35) / 0.65;
  return style.ankle_dorsi * 0.55 * (1.0 - blend);
}

LegCurve makeLegCurve(const ProcBody& body, JointId hip, JointId knee, JointId foot, double speed, double period, double stance,
                      const GaitStyle& style) {
  LegCurve curve;
  const double plant = footY(body, hip, knee, foot, 0.0, 0.35, 0.0, quatIdentity());
  const double stance_end = solveHip(body, hip, knee, foot, speed * stance * period, plant, pelvisRotation(stance, style));
  for (int sample = 0; sample < kLocoPhaseSamples; ++sample) {
    const double phase = static_cast<double>(sample) / static_cast<double>(kLocoPhaseSamples);
    if (phase < stance) {
      curve.hip[static_cast<std::size_t>(sample)] =
          solveHip(body, hip, knee, foot, speed * phase * period, plant, pelvisRotation(phase, style));
      curve.knee[static_cast<std::size_t>(sample)] = 0.0;
    } else {
      const double swing = (phase - stance) / std::max(1e-6, 1.0 - stance);
      const double shaped = swing * swing * (3.0 - 2.0 * swing);
      curve.hip[static_cast<std::size_t>(sample)] = stance_end * (1.0 - shaped) + 0.35 * shaped;
      curve.knee[static_cast<std::size_t>(sample)] = -std::sin(kPi * swing) * style.knee_flex;
    }
    curve.ankle[static_cast<std::size_t>(sample)] = ankleAngle(phase, stance, style);
  }
  curve.hip[0] = 0.35;
  curve.knee[0] = 0.0;
  curve.ankle[0] = 0.0;
  return curve;
}

double sampleCurve(const std::array<double, kLocoPhaseSamples>& curve, double phase) {
  phase -= std::floor(phase);
  const double position = phase * static_cast<double>(kLocoPhaseSamples);
  const int lower = static_cast<int>(std::floor(position)) % kLocoPhaseSamples;
  const int upper = (lower + 1) % kLocoPhaseSamples;
  const double weight = position - std::floor(position);
  return curve[static_cast<std::size_t>(lower)] * (1.0 - weight) + curve[static_cast<std::size_t>(upper)] * weight;
}

glm::dvec3 boneDirection(const ProcBody& body, const char* name) {
  for (const BoneGeom& bone : body.bones) {
    if (bone.name == name) return bone.rest_direction;
  }
  return glm::dvec3(1.0, 0.0, 0.0);
}

glm::dvec3 logOf(const glm::dquat& rotation) { return quatLog(rotation); }

}  // namespace

ProcAnalysis makeSyntheticLocomotionAnalysis(const ProcBody& body, const SyntheticClipSpec& spec) {
  const GaitStyle style = styleFor(spec.regime);
  const double leg = targetLegLength(body);
  const LegCurve left = makeLegCurve(body, JointId::LeftHip, JointId::LeftKnee, JointId::LeftFoot, spec.speed_mps, spec.period_s,
                                     spec.stance_fraction, style);
  const LegCurve right = makeLegCurve(body, JointId::RightHip, JointId::RightKnee, JointId::RightFoot, spec.speed_mps, spec.period_s,
                                      spec.stance_fraction, style);
  const glm::dvec3 left_arm_rest = boneDirection(body, "larmuppb");
  const glm::dvec3 right_arm_rest = boneDirection(body, "rarmuppb");
  const glm::dvec3 left_elbow_axis = body.constraints[jointIndex(JointId::LeftElbow)].axis;
  const glm::dvec3 right_elbow_axis = body.constraints[jointIndex(JointId::RightElbow)].axis;
  const glm::dvec3 left_knee_axis = body.constraints[jointIndex(JointId::LeftKnee)].axis;
  const glm::dvec3 right_knee_axis = body.constraints[jointIndex(JointId::RightKnee)].axis;
  const glm::dvec3 left_ankle_axis = body.constraints[jointIndex(JointId::LeftFoot)].axis;
  const glm::dvec3 right_ankle_axis = body.constraints[jointIndex(JointId::RightFoot)].axis;

  ProcAnalysis analysis;
  analysis.source.clip = spec.regime + "_" + std::to_string(spec.speed_mps);
  analysis.source.subject = spec.subject;
  analysis.source_leg_length_m = leg;
  const double fps = 50.0;
  const double duration = 4.0;
  const int frames = static_cast<int>(std::round(duration * fps)) + 1;
  const glm::dvec3 rest = body.joints[jointIndex(JointId::LegRoot)].rest_position;
  for (int index = 0; index < frames; ++index) {
    SignalFrame frame;
    frame.time = static_cast<double>(index) / fps;
    const double phase = std::fmod(frame.time, spec.period_s) / spec.period_s;
    const double right_phase = phase >= 0.5 ? phase - 0.5 : phase + 0.5;
    frame.root_y = spec.speed_mps * frame.time;
    frame.root_z = rest.z + 0.02 * std::sin(2.0 * kPi * phase);
    const double left_hip = sampleCurve(left.hip, phase);
    const double left_knee = sampleCurve(left.knee, phase);
    const double left_ankle = sampleCurve(left.ankle, phase);
    const double right_hip = sampleCurve(right.hip, right_phase);
    const double right_knee = sampleCurve(right.knee, right_phase);
    const double right_ankle = sampleCurve(right.ankle, right_phase);
    frame.log_vector[static_cast<int>(RotChannel::LeftHip)] = logOf(quatAxisAngle(kX, left_hip));
    frame.log_vector[static_cast<int>(RotChannel::RightHip)] = logOf(quatAxisAngle(kX, right_hip));
    frame.log_vector[static_cast<int>(RotChannel::LeftKnee)] = logOf(quatAxisAngle(left_knee_axis, left_knee));
    frame.log_vector[static_cast<int>(RotChannel::RightKnee)] = logOf(quatAxisAngle(right_knee_axis, right_knee));
    frame.log_vector[static_cast<int>(RotChannel::LeftAnkle)] = logOf(quatAxisAngle(left_ankle_axis, left_ankle));
    frame.log_vector[static_cast<int>(RotChannel::RightAnkle)] = logOf(quatAxisAngle(right_ankle_axis, right_ankle));

    const double arm = std::cos(2.0 * kPi * phase);
    const glm::dvec3 left_down = glm::normalize(glm::dvec3(-0.10, -style.arm_swing * arm, -1.0));
    const glm::dvec3 right_down = glm::normalize(glm::dvec3(0.10, style.arm_swing * arm, -1.0));
    frame.log_vector[static_cast<int>(RotChannel::LeftShoulder)] = logOf(quatRotateFromTo(left_arm_rest, left_down));
    frame.log_vector[static_cast<int>(RotChannel::RightShoulder)] = logOf(quatRotateFromTo(right_arm_rest, right_down));
    const double left_elbow = style.elbow_flex + style.elbow_mod * arm;
    const double right_elbow = style.elbow_flex - style.elbow_mod * arm;
    frame.log_vector[static_cast<int>(RotChannel::LeftElbow)] = logOf(quatAxisAngle(left_elbow_axis, left_elbow));
    frame.log_vector[static_cast<int>(RotChannel::RightElbow)] = logOf(quatAxisAngle(right_elbow_axis, right_elbow));

    frame.log_vector[static_cast<int>(RotChannel::Pelvis)] = logOf(pelvisRotation(phase, style));
    const double yaw = -style.spine_yaw * std::sin(2.0 * kPi * phase);
    frame.log_vector[static_cast<int>(RotChannel::Spine1)] = logOf(quatAxisAngle(glm::dvec3(0.0, 0.0, 1.0), yaw));
    frame.log_vector[static_cast<int>(RotChannel::Spine2)] = logOf(quatAxisAngle(glm::dvec3(0.0, 0.0, 1.0), yaw * 0.45));
    analysis.signals.push_back(frame);
  }

  auto track = [&](const std::string& name, bool left_side) {
    ContactTrack marker;
    marker.marker = name;
    marker.contact.resize(analysis.signals.size(), 0);
    for (std::size_t index = 0; index < analysis.signals.size(); ++index) {
      double phase = std::fmod(analysis.signals[index].time, spec.period_s) / spec.period_s;
      if (!left_side) phase = phase >= 0.5 ? phase - 0.5 : phase + 0.5;
      marker.contact[index] = phase < spec.stance_fraction ? 1 : 0;
    }
    analysis.contacts.push_back(std::move(marker));
  };
  track("left_heel", true);
  track("left_toe", true);
  track("right_heel", false);
  track("right_toe", false);

  GaitCycle cycle;
  cycle.side = "left";
  cycle.accepted = true;
  cycle.classification = "straight";
  cycle.observed_regime = spec.regime;
  cycle.start_time = spec.period_s;
  cycle.end_time = spec.period_s * 2.0;
  cycle.quality.overall = 0.92;
  cycle.measurements.speed_mps = spec.speed_mps;
  cycle.measurements.cadence_spm = 120.0 / spec.period_s;
  cycle.measurements.stride_length_m = spec.speed_mps * spec.period_s;
  cycle.measurements.normalized_stride_length = leg > 0.0 ? spec.speed_mps * spec.period_s / leg : 0.0;
  cycle.measurements.stance_fraction = spec.stance_fraction;
  cycle.measurements.flight_fraction = std::max(0.0, 1.0 - 2.0 * spec.stance_fraction);
  analysis.cycles.push_back(cycle);

  GaitCycle rejected = cycle;
  rejected.index = 1;
  rejected.accepted = false;
  rejected.classification = "rejected";
  rejected.quality.overall = 0.1;
  analysis.cycles.push_back(rejected);
  GaitCycle mirrored = cycle;
  mirrored.index = 2;
  mirrored.mirrored = true;
  analysis.cycles.push_back(mirrored);
  return analysis;
}

std::vector<ProcAnalysis> makeSyntheticLocomotionSet(const ProcBody& body) {
  const SyntheticClipSpec specs[] = {
      {"walk", 0.90, 1.10, 0.62, "synthetic"}, {"walk", 1.30, 1.00, 0.50, "synthetic"}, {"walk", 1.70, 0.92, 0.58, "synthetic"},
      {"run", 3.00, 0.72, 0.34, "synthetic"},  {"run", 3.80, 0.64, 0.30, "synthetic"},  {"sprint", 5.80, 0.52, 0.24, "synthetic"},
      {"sprint", 6.60, 0.46, 0.22, "synthetic"}};
  std::vector<ProcAnalysis> analyses;
  for (const SyntheticClipSpec& spec : specs) analyses.push_back(makeSyntheticLocomotionAnalysis(body, spec));
  return analyses;
}

ProcLocomotionModel buildSyntheticLocomotionModel(const ProcBody& body, std::vector<std::string>& warnings) {
  ModelBuildSettings settings;
  settings.bin_width = 0.12;
  settings.min_cycles_per_knot = 1;
  settings.subject = "synthetic";
  return buildLocomotionModel(makeSyntheticLocomotionSet(body), settings, warnings);
}

}  // namespace proc
