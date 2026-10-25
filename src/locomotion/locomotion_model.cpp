#include "locomotion/locomotion_model.h"

#include <algorithm>
#include <cmath>

namespace proc {
namespace {

int channelPair(int channel) {
  switch (static_cast<LocoChannel>(channel)) {
    case LocoChannel::LeftHip: return static_cast<int>(LocoChannel::RightHip);
    case LocoChannel::RightHip: return static_cast<int>(LocoChannel::LeftHip);
    case LocoChannel::LeftKnee: return static_cast<int>(LocoChannel::RightKnee);
    case LocoChannel::RightKnee: return static_cast<int>(LocoChannel::LeftKnee);
    case LocoChannel::LeftAnkle: return static_cast<int>(LocoChannel::RightAnkle);
    case LocoChannel::RightAnkle: return static_cast<int>(LocoChannel::LeftAnkle);
    case LocoChannel::LeftShoulder: return static_cast<int>(LocoChannel::RightShoulder);
    case LocoChannel::RightShoulder: return static_cast<int>(LocoChannel::LeftShoulder);
    case LocoChannel::LeftElbow: return static_cast<int>(LocoChannel::RightElbow);
    case LocoChannel::RightElbow: return static_cast<int>(LocoChannel::LeftElbow);
    default: return -1;
  }
}

}  // namespace

const char* locoChannelName(int channel) {
  switch (static_cast<LocoChannel>(channel)) {
    case LocoChannel::Pelvis: return "pelvis";
    case LocoChannel::Spine1: return "spine1";
    case LocoChannel::Spine2: return "spine2";
    case LocoChannel::LeftHip: return "left_hip";
    case LocoChannel::RightHip: return "right_hip";
    case LocoChannel::LeftKnee: return "left_knee";
    case LocoChannel::RightKnee: return "right_knee";
    case LocoChannel::LeftAnkle: return "left_ankle";
    case LocoChannel::RightAnkle: return "right_ankle";
    case LocoChannel::LeftShoulder: return "left_shoulder";
    case LocoChannel::RightShoulder: return "right_shoulder";
    case LocoChannel::LeftElbow: return "left_elbow";
    case LocoChannel::RightElbow: return "right_elbow";
  }
  return "unknown";
}

const char* regimeName(LocomotionRegime regime) {
  switch (regime) {
    case LocomotionRegime::Idle: return "idle";
    case LocomotionRegime::Walk: return "walk";
    case LocomotionRegime::Run: return "run";
    case LocomotionRegime::Sprint: return "sprint";
  }
  return "idle";
}

JointId locoJoint(int channel) {
  switch (static_cast<LocoChannel>(channel)) {
    case LocoChannel::Pelvis: return JointId::LegRoot;
    case LocoChannel::Spine1: return JointId::Spine1;
    case LocoChannel::Spine2: return JointId::Spine2;
    case LocoChannel::LeftHip: return JointId::LeftHip;
    case LocoChannel::RightHip: return JointId::RightHip;
    case LocoChannel::LeftKnee: return JointId::LeftKnee;
    case LocoChannel::RightKnee: return JointId::RightKnee;
    case LocoChannel::LeftAnkle: return JointId::LeftFoot;
    case LocoChannel::RightAnkle: return JointId::RightFoot;
    case LocoChannel::LeftShoulder: return JointId::LeftShoulder;
    case LocoChannel::RightShoulder: return JointId::RightShoulder;
    case LocoChannel::LeftElbow: return JointId::LeftElbow;
    case LocoChannel::RightElbow: return JointId::RightElbow;
  }
  return JointId::LegRoot;
}

glm::dquat mirrorSagittal(glm::dquat rotation) {
  rotation = quatUnit(rotation);
  return glm::dquat(rotation.w, rotation.x, -rotation.y, -rotation.z);
}

void mirrorLogChannels(std::array<glm::dvec3, kLocoChannels>& logs) {
  for (glm::dvec3& log_vector : logs) log_vector = quatLog(mirrorSagittal(quatExp(log_vector)));
  std::array<bool, kLocoChannels> swapped{};
  for (int channel = 0; channel < kLocoChannels; ++channel) {
    const int other = channelPair(channel);
    if (other < 0 || swapped[static_cast<std::size_t>(channel)]) continue;
    std::swap(logs[static_cast<std::size_t>(channel)], logs[static_cast<std::size_t>(other)]);
    swapped[static_cast<std::size_t>(channel)] = true;
    swapped[static_cast<std::size_t>(other)] = true;
  }
}

double wrapPhase(double phase) {
  if (!std::isfinite(phase)) return 0.0;
  phase = std::fmod(phase, 1.0);
  if (phase < 0.0) phase += 1.0;
  if (phase >= 1.0) phase = 0.0;
  return phase;
}

glm::dvec3 sampleLog(const std::array<glm::dvec3, kLocoPhaseSamples>& curve, double phase) {
  phase = wrapPhase(phase);
  const double position = phase * static_cast<double>(kLocoPhaseSamples);
  const int lower = static_cast<int>(std::floor(position)) % kLocoPhaseSamples;
  const int upper = (lower + 1) % kLocoPhaseSamples;
  const double weight = position - std::floor(position);
  return curve[static_cast<std::size_t>(lower)] * (1.0 - weight) + curve[static_cast<std::size_t>(upper)] * weight;
}

double samplePeriodic(const std::array<double, kLocoPhaseSamples>& curve, double phase) {
  phase = wrapPhase(phase);
  const double position = phase * static_cast<double>(kLocoPhaseSamples);
  const int lower = static_cast<int>(std::floor(position)) % kLocoPhaseSamples;
  const int upper = (lower + 1) % kLocoPhaseSamples;
  const double weight = position - std::floor(position);
  return curve[static_cast<std::size_t>(lower)] * (1.0 - weight) + curve[static_cast<std::size_t>(upper)] * weight;
}

}  // namespace proc
