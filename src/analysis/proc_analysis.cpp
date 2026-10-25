#include "analysis/proc_analysis.h"

#include "analysis/contact_detector.h"
#include "analysis/gait_cycles.h"
#include "analysis/gait_events.h"
#include "analysis/ground_estimator.h"
#include "analysis/relationship_model.h"
#include "motion/interpolation.h"

#include <algorithm>
#include <cmath>
#include <cctype>

namespace proc {
namespace {

constexpr const char* kRotNames[kRotChannelCount] = {
    "legrootj", "spine1j", "spine2j", "lleguppj", "rleguppj", "lleglowj", "rleglowj",
    "lfootj",   "rfootj",  "larmuppj", "rarmuppj", "larmlowj", "rarmlowj"};

std::string foldName(std::string name) {
  const std::size_t colon = name.rfind(':');
  if (colon != std::string::npos) name = name.substr(colon + 1);
  if (name.rfind("mixamorig", 0) == 0) name = name.substr(9);
  for (char& character : name) {
    character = static_cast<char>(std::tolower(static_cast<unsigned char>(character)));
  }
  return name;
}

int findSource(const SourceClip& clip, std::initializer_list<const char*> names) {
  for (int index = 0; index < static_cast<int>(clip.joints.size()); ++index) {
    const std::string folded = foldName(clip.joints[static_cast<std::size_t>(index)].name);
    for (const char* name : names) {
      if (folded == name) return index;
    }
  }
  return -1;
}

struct MarkerIds {
  int root = -1;
  int left_hip = -1;
  int left_knee = -1;
  int left_foot = -1;
  int left_toe = -1;
  int right_hip = -1;
  int right_knee = -1;
  int right_foot = -1;
  int right_toe = -1;
};

MarkerIds resolveIds(const SourceClip& clip, std::vector<std::string>& warnings) {
  MarkerIds ids;
  ids.root = findSource(clip, {"hips", "hip", "pelvis", "root", "legrootj"});
  ids.left_hip = findSource(clip, {"leftupleg", "leftupperleg", "lleguppj"});
  ids.left_knee = findSource(clip, {"leftleg", "leftlowerleg", "lleglowj"});
  ids.left_foot = findSource(clip, {"leftfoot", "lfootj"});
  ids.left_toe = findSource(clip, {"lefttoe", "lefttoebase", "lefttoeend", "ltoesj"});
  ids.right_hip = findSource(clip, {"rightupleg", "rightupperleg", "rleguppj"});
  ids.right_knee = findSource(clip, {"rightleg", "rightlowerleg", "rleglowj"});
  ids.right_foot = findSource(clip, {"rightfoot", "rfootj"});
  ids.right_toe = findSource(clip, {"righttoe", "righttoebase", "righttoeend", "rtoesj"});
  if (ids.root < 0) warnings.push_back("source root was not found; root trajectory stays at the origin");
  if (ids.left_foot < 0) warnings.push_back("left foot was not found; left contact is unavailable");
  if (ids.right_foot < 0) warnings.push_back("right foot was not found; right contact is unavailable");
  if (ids.left_toe < 0) warnings.push_back("left toe was not found; the left toe copies the ankle and the heel offset uses a default");
  if (ids.right_toe < 0) warnings.push_back("right toe was not found; the right toe copies the ankle and the heel offset uses a default");
  return ids;
}

glm::dvec3 positionAt(const SourceFrame& frame, int index) {
  if (index < 0 || index >= static_cast<int>(frame.global_position.size())) return glm::dvec3(0.0);
  return frame.global_position[static_cast<std::size_t>(index)];
}

glm::dvec3 restAt(const SourceClip& clip, int index) {
  if (index < 0 || index >= static_cast<int>(clip.joints.size())) return glm::dvec3(0.0);
  return clip.joints[static_cast<std::size_t>(index)].rest_global_position;
}

double limbLength(const SourceClip& clip, int hip, int knee, int ankle) {
  if (hip < 0 || knee < 0 || ankle < 0) return 0.0;
  return glm::length(restAt(clip, knee) - restAt(clip, hip)) + glm::length(restAt(clip, ankle) - restAt(clip, knee));
}

void writeSpeed(std::vector<SignalFrame>& frames, const std::vector<glm::dvec3>& velocity, double SignalFrame::*horizontal,
                double SignalFrame::*vertical, double SignalFrame::*total) {
  for (std::size_t index = 0; index < frames.size() && index < velocity.size(); ++index) {
    const glm::dvec3& value = velocity[index];
    frames[index].*horizontal = std::sqrt(value.x * value.x + value.y * value.y);
    frames[index].*vertical = value.z;
    if (total != nullptr) frames[index].*total = glm::length(value);
  }
}

glm::dvec3 heelPoint(const glm::dvec3& ankle, const glm::dvec3& toe, bool toe_missing, double offset) {
  glm::dvec3 direction = ankle - toe;
  direction.z = 0.0;
  if (toe_missing || glm::length(direction) < 1e-5) direction = glm::dvec3(0.0, -1.0, 0.0);
  else direction = glm::normalize(direction);
  return ankle + direction * offset;
}

double footLength(const SourceClip& clip, int foot, int toe) {
  if (foot < 0 || toe < 0) return 0.0;
  return glm::length(restAt(clip, toe) - restAt(clip, foot));
}

void fillRotations(ProcAnalysis& analysis, const ProcMotion* motion) {
  if (motion == nullptr || motion->frames.empty()) {
    analysis.warnings.push_back("no PROC motion; rotational channels stay at identity");
    return;
  }
  std::array<glm::dquat, kRotChannelCount> previous;
  previous.fill(quatIdentity());
  const int joints[kRotChannelCount] = {0, 1, 2, 6, 10, 7, 11, 8, 12, 14, 18, 15, 19};
  for (SignalFrame& frame : analysis.signals) {
    const Pose pose = sampleMotion(*motion, frame.time, false);
    for (int channel = 0; channel < kRotChannelCount; ++channel) {
      glm::dquat rotation = channel == static_cast<int>(RotChannel::Pelvis) ? pose.root_rotation : pose.local_rotation[joints[channel]];
      rotation = hemisphereContinuous(previous[static_cast<std::size_t>(channel)], rotation);
      previous[static_cast<std::size_t>(channel)] = rotation;
      frame.rotation[static_cast<std::size_t>(channel)] = rotation;
      frame.log_vector[static_cast<std::size_t>(channel)] = quaternionLog(rotation);
      frame.sagittal_deg[static_cast<std::size_t>(channel)] = sagittalDegrees(frame.log_vector[static_cast<std::size_t>(channel)]);
    }
    frame.pelvis_pitch_deg = frame.sagittal_deg[static_cast<int>(RotChannel::Pelvis)];
    frame.pelvis_roll_deg = rollDegrees(frame.log_vector[static_cast<int>(RotChannel::Pelvis)]);
  }
}

ContactTrack makeTrack(const std::string& name, const std::string& provenance, const ContactObservation& observation) {
  ContactTrack track;
  track.marker = name;
  track.provenance = provenance;
  track.confidence = observation.confidence;
  track.contact = observation.contact;
  track.automatic_contact = observation.contact;
  return track;
}

std::vector<double> timesOf(const ProcAnalysis& analysis) {
  std::vector<double> time;
  time.reserve(analysis.signals.size());
  for (const SignalFrame& frame : analysis.signals) time.push_back(frame.time);
  return time;
}

}  // namespace

const char* rotChannelName(RotChannel channel) {
  const int index = static_cast<int>(channel);
  if (index < 0 || index >= kRotChannelCount) return "unknown";
  return kRotNames[index];
}

int nearestFrame(const ProcAnalysis& analysis, double time_s) {
  if (analysis.signals.empty()) return 0;
  int best = 0;
  double best_distance = std::abs(analysis.signals.front().time - time_s);
  for (int index = 1; index < static_cast<int>(analysis.signals.size()); ++index) {
    const double distance = std::abs(analysis.signals[static_cast<std::size_t>(index)].time - time_s);
    if (distance < best_distance) {
      best = index;
      best_distance = distance;
    }
  }
  return best;
}

ProcAnalysis analyzeClip(const AnalysisInput& input, const AnalysisSettings& settings) {
  ProcAnalysis analysis;
  analysis.settings = settings;
  if (input.source == nullptr || input.source->frames.empty()) {
    analysis.warnings.push_back("no source frames");
    return analysis;
  }
  const SourceClip& clip = *input.source;
  analysis.source.dataset = input.dataset;
  analysis.source.clip = input.source->name.empty() ? input.theme : input.source->name;
  if (analysis.source.clip.empty()) analysis.source.clip = "clip";
  analysis.source.motion_path = input.motion_path.empty() ? clip.path : input.motion_path;
  analysis.source.theme = input.theme;
  analysis.source.subject = input.subject;
  analysis.source.take = input.take;
  analysis.source.fps = clip.nominal_fps;
  analysis.source.frame_count = static_cast<int>(clip.frames.size());
  analysis.source.duration_s = clip.duration > 0.0 ? clip.duration : clip.frames.back().time;

  const MarkerIds ids = resolveIds(clip, analysis.warnings);
  const double left_leg = limbLength(clip, ids.left_hip, ids.left_knee, ids.left_foot);
  const double right_leg = limbLength(clip, ids.right_hip, ids.right_knee, ids.right_foot);
  if (left_leg > 0.0 && right_leg > 0.0) analysis.source_leg_length_m = 0.5 * (left_leg + right_leg);
  else analysis.source_leg_length_m = std::max(left_leg, right_leg);
  analysis.leg_length_definition = "mean of available source rest lengths (hip to knee) + (knee to ankle); not the loaded PROC body";

  const int frame_count = static_cast<int>(clip.frames.size());
  std::vector<double> time(static_cast<std::size_t>(frame_count));
  std::vector<glm::dvec3> root(static_cast<std::size_t>(frame_count));
  std::vector<glm::dvec3> left_foot(static_cast<std::size_t>(frame_count));
  std::vector<glm::dvec3> right_foot(static_cast<std::size_t>(frame_count));
  std::vector<glm::dvec3> left_toe(static_cast<std::size_t>(frame_count));
  std::vector<glm::dvec3> right_toe(static_cast<std::size_t>(frame_count));
  for (int index = 0; index < frame_count; ++index) {
    const SourceFrame& frame = clip.frames[static_cast<std::size_t>(index)];
    time[static_cast<std::size_t>(index)] = frame.time;
    root[static_cast<std::size_t>(index)] = positionAt(frame, ids.root);
    left_foot[static_cast<std::size_t>(index)] = positionAt(frame, ids.left_foot);
    right_foot[static_cast<std::size_t>(index)] = positionAt(frame, ids.right_foot);
    left_toe[static_cast<std::size_t>(index)] = ids.left_toe >= 0 ? positionAt(frame, ids.left_toe) : left_foot[static_cast<std::size_t>(index)];
    right_toe[static_cast<std::size_t>(index)] = ids.right_toe >= 0 ? positionAt(frame, ids.right_toe) : right_foot[static_cast<std::size_t>(index)];
  }
  root = smoothPositions(time, root, settings.position_smooth_s);
  left_foot = smoothPositions(time, left_foot, settings.position_smooth_s);
  right_foot = smoothPositions(time, right_foot, settings.position_smooth_s);
  left_toe = smoothPositions(time, left_toe, settings.position_smooth_s);
  right_toe = smoothPositions(time, right_toe, settings.position_smooth_s);

  const double left_offset = std::max(0.04, settings.heel_fraction * std::max(0.05, footLength(clip, ids.left_foot, ids.left_toe)));
  const double right_offset = std::max(0.04, settings.heel_fraction * std::max(0.05, footLength(clip, ids.right_foot, ids.right_toe)));
  std::vector<glm::dvec3> left_heel(static_cast<std::size_t>(frame_count));
  std::vector<glm::dvec3> right_heel(static_cast<std::size_t>(frame_count));
  for (int index = 0; index < frame_count; ++index) {
    left_heel[static_cast<std::size_t>(index)] = heelPoint(left_foot[static_cast<std::size_t>(index)], left_toe[static_cast<std::size_t>(index)], ids.left_toe < 0, left_offset);
    right_heel[static_cast<std::size_t>(index)] = heelPoint(right_foot[static_cast<std::size_t>(index)], right_toe[static_cast<std::size_t>(index)], ids.right_toe < 0, right_offset);
  }

  const std::vector<glm::dvec3> root_velocity = smoothPositions(time, centralDifference(time, root), settings.velocity_smooth_s);
  const std::vector<glm::dvec3> left_foot_velocity = smoothPositions(time, centralDifference(time, left_foot), settings.velocity_smooth_s);
  const std::vector<glm::dvec3> right_foot_velocity = smoothPositions(time, centralDifference(time, right_foot), settings.velocity_smooth_s);
  const std::vector<glm::dvec3> left_toe_velocity = smoothPositions(time, centralDifference(time, left_toe), settings.velocity_smooth_s);
  const std::vector<glm::dvec3> right_toe_velocity = smoothPositions(time, centralDifference(time, right_toe), settings.velocity_smooth_s);
  const std::vector<glm::dvec3> left_heel_velocity = smoothPositions(time, centralDifference(time, left_heel), settings.velocity_smooth_s);
  const std::vector<glm::dvec3> right_heel_velocity = smoothPositions(time, centralDifference(time, right_heel), settings.velocity_smooth_s);

  analysis.signals.resize(static_cast<std::size_t>(frame_count));
  for (int index = 0; index < frame_count; ++index) {
    SignalFrame& frame = analysis.signals[static_cast<std::size_t>(index)];
    frame.time = time[static_cast<std::size_t>(index)];
    frame.rotation.fill(quatIdentity());
    frame.root_x = root[static_cast<std::size_t>(index)].x;
    frame.root_y = root[static_cast<std::size_t>(index)].y;
    frame.root_z = root[static_cast<std::size_t>(index)].z;
    frame.left_foot_x = left_foot[static_cast<std::size_t>(index)].x;
    frame.left_foot_y = left_foot[static_cast<std::size_t>(index)].y;
    frame.left_foot_z = left_foot[static_cast<std::size_t>(index)].z;
    frame.right_foot_x = right_foot[static_cast<std::size_t>(index)].x;
    frame.right_foot_y = right_foot[static_cast<std::size_t>(index)].y;
    frame.right_foot_z = right_foot[static_cast<std::size_t>(index)].z;
    frame.left_toe_x = left_toe[static_cast<std::size_t>(index)].x;
    frame.left_toe_y = left_toe[static_cast<std::size_t>(index)].y;
    frame.left_toe_z = left_toe[static_cast<std::size_t>(index)].z;
    frame.right_toe_x = right_toe[static_cast<std::size_t>(index)].x;
    frame.right_toe_y = right_toe[static_cast<std::size_t>(index)].y;
    frame.right_toe_z = right_toe[static_cast<std::size_t>(index)].z;
    frame.left_heel_x = left_heel[static_cast<std::size_t>(index)].x;
    frame.left_heel_y = left_heel[static_cast<std::size_t>(index)].y;
    frame.left_heel_z = left_heel[static_cast<std::size_t>(index)].z;
    frame.right_heel_x = right_heel[static_cast<std::size_t>(index)].x;
    frame.right_heel_y = right_heel[static_cast<std::size_t>(index)].y;
    frame.right_heel_z = right_heel[static_cast<std::size_t>(index)].z;
  }
  writeSpeed(analysis.signals, root_velocity, &SignalFrame::root_horizontal_speed, &SignalFrame::root_z, &SignalFrame::root_speed);
  // root_z was overwritten by the vertical component. Restore height, keep vertical speed in a dedicated path.
  for (int index = 0; index < frame_count; ++index) {
    analysis.signals[static_cast<std::size_t>(index)].root_z = root[static_cast<std::size_t>(index)].z;
  }
  // The call above stored vertical root speed in root_z temporarily. Recompute root speeds explicitly.
  for (int index = 0; index < frame_count; ++index) {
    const glm::dvec3& value = root_velocity[static_cast<std::size_t>(index)];
    SignalFrame& frame = analysis.signals[static_cast<std::size_t>(index)];
    frame.root_horizontal_speed = std::sqrt(value.x * value.x + value.y * value.y);
    frame.root_speed = glm::length(value);
    frame.root_z = root[static_cast<std::size_t>(index)].z;
  }
  writeSpeed(analysis.signals, left_foot_velocity, &SignalFrame::left_foot_horizontal_speed, &SignalFrame::left_foot_vertical_speed, &SignalFrame::left_foot_speed);
  writeSpeed(analysis.signals, right_foot_velocity, &SignalFrame::right_foot_horizontal_speed, &SignalFrame::right_foot_vertical_speed, &SignalFrame::right_foot_speed);
  writeSpeed(analysis.signals, left_toe_velocity, &SignalFrame::left_toe_horizontal_speed, &SignalFrame::left_toe_vertical_speed, nullptr);
  writeSpeed(analysis.signals, right_toe_velocity, &SignalFrame::right_toe_horizontal_speed, &SignalFrame::right_toe_vertical_speed, nullptr);
  writeSpeed(analysis.signals, left_heel_velocity, &SignalFrame::left_heel_horizontal_speed, &SignalFrame::left_heel_vertical_speed, nullptr);
  writeSpeed(analysis.signals, right_heel_velocity, &SignalFrame::right_heel_horizontal_speed, &SignalFrame::right_heel_vertical_speed, nullptr);

  std::vector<double> heights;
  std::vector<double> speeds;
  heights.reserve(static_cast<std::size_t>(frame_count) * 6);
  speeds.reserve(static_cast<std::size_t>(frame_count) * 6);
  auto pushMarker = [&](const std::vector<glm::dvec3>& points, const std::vector<glm::dvec3>& velocity) {
    for (std::size_t index = 0; index < points.size(); ++index) {
      heights.push_back(points[index].z);
      speeds.push_back(std::sqrt(velocity[index].x * velocity[index].x + velocity[index].y * velocity[index].y));
    }
  };
  pushMarker(left_foot, left_foot_velocity);
  pushMarker(right_foot, right_foot_velocity);
  pushMarker(left_toe, left_toe_velocity);
  pushMarker(right_toe, right_toe_velocity);
  pushMarker(left_heel, left_heel_velocity);
  pushMarker(right_heel, right_heel_velocity);
  analysis.ground = estimateGround(heights, speeds, settings);

  for (SignalFrame& frame : analysis.signals) {
    frame.left_foot_height = frame.left_foot_z - analysis.ground.height_m;
    frame.right_foot_height = frame.right_foot_z - analysis.ground.height_m;
    frame.left_toe_height = frame.left_toe_z - analysis.ground.height_m;
    frame.right_toe_height = frame.right_toe_z - analysis.ground.height_m;
    frame.left_heel_height = frame.left_heel_z - analysis.ground.height_m;
    frame.right_heel_height = frame.right_heel_z - analysis.ground.height_m;
  }
  fillRotations(analysis, input.motion);

  const std::vector<double> sample_time = timesOf(analysis);
  auto heightsOf = [&](double SignalFrame::*member) {
    std::vector<double> values;
    values.reserve(analysis.signals.size());
    for (const SignalFrame& frame : analysis.signals) values.push_back(frame.*member);
    return values;
  };
  auto speedsOf = heightsOf;
  analysis.contacts.push_back(makeTrack("left_heel", "inferred_virtual", detectMarkerContact(sample_time, heightsOf(&SignalFrame::left_heel_height), speedsOf(&SignalFrame::left_heel_horizontal_speed), speedsOf(&SignalFrame::left_heel_vertical_speed), settings)));
  analysis.contacts.push_back(makeTrack("left_toe", ids.left_toe >= 0 ? "tracked" : "copied_from_ankle", detectMarkerContact(sample_time, heightsOf(&SignalFrame::left_toe_height), speedsOf(&SignalFrame::left_toe_horizontal_speed), speedsOf(&SignalFrame::left_toe_vertical_speed), settings)));
  analysis.contacts.push_back(makeTrack("right_heel", "inferred_virtual", detectMarkerContact(sample_time, heightsOf(&SignalFrame::right_heel_height), speedsOf(&SignalFrame::right_heel_horizontal_speed), speedsOf(&SignalFrame::right_heel_vertical_speed), settings)));
  analysis.contacts.push_back(makeTrack("right_toe", ids.right_toe >= 0 ? "tracked" : "copied_from_ankle", detectMarkerContact(sample_time, heightsOf(&SignalFrame::right_toe_height), speedsOf(&SignalFrame::right_toe_horizontal_speed), speedsOf(&SignalFrame::right_toe_vertical_speed), settings)));

  extractGaitEvents(analysis);
  extractGaitCycles(analysis);
  mirrorCycles(analysis);
  fitRelationships(analysis);
  return analysis;
}

void moveEvent(ProcAnalysis& analysis, int event_index, double time_s) {
  if (event_index < 0 || event_index >= static_cast<int>(analysis.events.size()) || analysis.signals.empty()) return;
  GaitEvent& event = analysis.events[static_cast<std::size_t>(event_index)];
  const double start = analysis.signals.front().time;
  const double end = analysis.signals.back().time;
  event.time = std::clamp(time_s, start, end);
  event.frame = nearestFrame(analysis, event.time);
  event.time = analysis.signals[static_cast<std::size_t>(event.frame)].time;
  event.manual_override = true;
}

void setEventType(ProcAnalysis& analysis, int event_index, const std::string& type) {
  if (event_index < 0 || event_index >= static_cast<int>(analysis.events.size())) return;
  if (type != "initial_contact" && type != "toe_off") return;
  GaitEvent& event = analysis.events[static_cast<std::size_t>(event_index)];
  event.type = type;
  event.manual_override = true;
}

void setCycleAccepted(ProcAnalysis& analysis, int cycle_index, bool accepted, const std::string& reason) {
  if (cycle_index < 0 || cycle_index >= static_cast<int>(analysis.cycles.size())) return;
  GaitCycle& cycle = analysis.cycles[static_cast<std::size_t>(cycle_index)];
  cycle.accepted = accepted;
  cycle.rejection_reason = accepted ? "" : (reason.empty() ? "manual" : reason);
  cycle.manual_override = true;
  if (accepted && cycle.curves.hip_flex_deg.empty()) {
    // Curves are filled by the caller through a fresh analyze. Keep the decision either way.
  }
  fitRelationships(analysis);
}

void overrideContactRange(ProcAnalysis& analysis, const std::string& marker, int frame0, int frame1, bool contact) {
  if (frame1 < frame0) std::swap(frame0, frame1);
  for (ContactTrack& track : analysis.contacts) {
    if (track.marker != marker) continue;
    frame0 = std::clamp(frame0, 0, static_cast<int>(track.contact.size()) - 1);
    frame1 = std::clamp(frame1, 0, static_cast<int>(track.contact.size()) - 1);
    for (int frame = frame0; frame <= frame1; ++frame) track.contact[static_cast<std::size_t>(frame)] = contact ? 1 : 0;
    track.manual_override = true;
    ContactOverride override_range;
    override_range.marker = marker;
    override_range.frame0 = frame0;
    override_range.frame1 = frame1;
    override_range.contact = contact;
    analysis.contact_overrides.push_back(override_range);
    return;
  }
}

}  // namespace proc
