#include "analysis/phase_normalizer.h"

#include <algorithm>
#include <cmath>

namespace proc {
namespace {

template <typename Pick>
double interp(const ProcAnalysis& analysis, double time_s, Pick pick) {
  const std::vector<SignalFrame>& frames = analysis.signals;
  if (frames.empty()) return 0.0;
  if (time_s <= frames.front().time) return pick(frames.front());
  if (time_s >= frames.back().time) return pick(frames.back());
  std::size_t upper = 1;
  while (upper + 1 < frames.size() && frames[upper].time < time_s) ++upper;
  const double span = frames[upper].time - frames[upper - 1].time;
  const double weight = span > 1e-12 ? (time_s - frames[upper - 1].time) / span : 0.0;
  return pick(frames[upper - 1]) * (1.0 - weight) + pick(frames[upper]) * weight;
}

double confidenceAt(const ProcAnalysis& analysis, const std::string& marker, double time_s) {
  const ContactTrack* track = nullptr;
  for (const ContactTrack& candidate : analysis.contacts) {
    if (candidate.marker == marker) track = &candidate;
  }
  if (track == nullptr || track->confidence.empty() || analysis.signals.empty()) return 0.0;
  std::vector<double> time;
  time.reserve(analysis.signals.size());
  for (const SignalFrame& frame : analysis.signals) time.push_back(frame.time);
  const std::size_t count = std::min(time.size(), track->confidence.size());
  time.resize(count);
  std::vector<double> values(track->confidence.begin(), track->confidence.begin() + static_cast<std::ptrdiff_t>(count));
  return sampleSeries(time, values, time_s);
}

}  // namespace

void normalizeCycle(ProcAnalysis& analysis, GaitCycle& cycle) {
  const int samples = std::max(2, analysis.settings.phase_samples);
  PhaseCurves curves;
  curves.hip_flex_deg.resize(static_cast<std::size_t>(samples));
  curves.knee_flex_deg.resize(static_cast<std::size_t>(samples));
  curves.ankle_flex_deg.resize(static_cast<std::size_t>(samples));
  curves.opposite_hip_flex_deg.resize(static_cast<std::size_t>(samples));
  curves.pelvis_height_m.resize(static_cast<std::size_t>(samples));
  curves.pelvis_pitch_deg.resize(static_cast<std::size_t>(samples));
  curves.pelvis_roll_deg.resize(static_cast<std::size_t>(samples));
  curves.foot_height_m.resize(static_cast<std::size_t>(samples));
  curves.toe_height_m.resize(static_cast<std::size_t>(samples));
  curves.contact_confidence.resize(static_cast<std::size_t>(samples));
  curves.shoulder_flex_deg.resize(static_cast<std::size_t>(samples));
  const bool left = cycle.side == "left";
  const RotChannel hip = left ? RotChannel::LeftHip : RotChannel::RightHip;
  const RotChannel other_hip = left ? RotChannel::RightHip : RotChannel::LeftHip;
  const RotChannel knee = left ? RotChannel::LeftKnee : RotChannel::RightKnee;
  const RotChannel ankle = left ? RotChannel::LeftAnkle : RotChannel::RightAnkle;
  const RotChannel shoulder = left ? RotChannel::LeftShoulder : RotChannel::RightShoulder;
  const double start = cycle.start_time;
  const double span = cycle.end_time - cycle.start_time;
  for (int index = 0; index < samples; ++index) {
    const double phase = static_cast<double>(index) / static_cast<double>(samples - 1);
    const double time_s = start + span * phase;
    auto at = [&](auto pick) { return interp(analysis, time_s, pick); };
    curves.hip_flex_deg[static_cast<std::size_t>(index)] = at([&](const SignalFrame& frame) { return frame.sagittal_deg[static_cast<int>(hip)]; });
    curves.opposite_hip_flex_deg[static_cast<std::size_t>(index)] =
        at([&](const SignalFrame& frame) { return frame.sagittal_deg[static_cast<int>(other_hip)]; });
    curves.knee_flex_deg[static_cast<std::size_t>(index)] = at([&](const SignalFrame& frame) { return frame.sagittal_deg[static_cast<int>(knee)]; });
    curves.ankle_flex_deg[static_cast<std::size_t>(index)] =
        at([&](const SignalFrame& frame) { return frame.sagittal_deg[static_cast<int>(ankle)]; });
    curves.pelvis_height_m[static_cast<std::size_t>(index)] = at([](const SignalFrame& frame) { return frame.root_z; });
    curves.pelvis_pitch_deg[static_cast<std::size_t>(index)] = at([](const SignalFrame& frame) { return frame.pelvis_pitch_deg; });
    curves.pelvis_roll_deg[static_cast<std::size_t>(index)] = at([](const SignalFrame& frame) { return frame.pelvis_roll_deg; });
    curves.foot_height_m[static_cast<std::size_t>(index)] =
        at([&](const SignalFrame& frame) { return left ? frame.left_foot_height : frame.right_foot_height; });
    curves.toe_height_m[static_cast<std::size_t>(index)] =
        at([&](const SignalFrame& frame) { return left ? frame.left_toe_height : frame.right_toe_height; });
    curves.shoulder_flex_deg[static_cast<std::size_t>(index)] =
        at([&](const SignalFrame& frame) { return frame.sagittal_deg[static_cast<int>(shoulder)]; });
    const double heel = confidenceAt(analysis, cycle.side + "_heel", time_s);
    const double toe = confidenceAt(analysis, cycle.side + "_toe", time_s);
    curves.contact_confidence[static_cast<std::size_t>(index)] = 0.5 * (heel + toe);
  }
  cycle.curves = std::move(curves);
}

}  // namespace proc
