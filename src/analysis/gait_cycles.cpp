#include "analysis/gait_cycles.h"

#include "analysis/measurements.h"
#include "analysis/phase_normalizer.h"

#include <algorithm>
#include <cmath>

namespace proc {
namespace {

int supportTransitions(const ProcAnalysis& analysis, const std::string& side, int begin, int end) {
  if (end <= begin) return 0;
  auto on = [&](int frame) {
    for (const ContactTrack& track : analysis.contacts) {
      if (track.marker != side + "_heel" && track.marker != side + "_toe") continue;
      if (frame >= 0 && frame < static_cast<int>(track.contact.size()) && track.contact[static_cast<std::size_t>(frame)] != 0) return true;
    }
    return false;
  };
  int transitions = 0;
  bool previous = on(begin);
  for (int frame = begin + 1; frame < end; ++frame) {
    const bool current = on(frame);
    if (current != previous) ++transitions;
    previous = current;
  }
  return transitions;
}

bool hasToeOff(const ProcAnalysis& analysis, const GaitCycle& cycle) {
  for (const GaitEvent& event : analysis.events) {
    if (event.side == cycle.side && event.type == "toe_off" && event.time > cycle.start_time + 1e-6 && event.time < cycle.end_time - 1e-6) return true;
  }
  return false;
}

void classify(ProcAnalysis& analysis, GaitCycle& cycle) {
  const CycleMeasurements& measured = cycle.measurements;
  const AnalysisSettings& settings = analysis.settings;
  std::string reason;
  if (measured.stride_time < settings.min_cycle_s) reason = "duration too short";
  else if (measured.stride_time > settings.max_cycle_s) reason = "duration too long";
  else if (!hasToeOff(analysis, cycle)) reason = "missing event";
  else if (measured.stride_length_m < settings.standing_travel_m) reason = "standing";
  else if (measured.stride_length_m < settings.min_travel_m) reason = "insufficient travel";
  else if (supportTransitions(analysis, cycle.side, cycle.start_frame, cycle.end_frame) > 3) reason = "ambiguous contacts";
  else if (cycle.quality.contact < settings.min_contact_confidence) reason = "low confidence";
  else if (measured.flight_fraction > 0.45 && measured.pelvis_vertical_excursion_m > 0.20) reason = "jump interruption";
  else if (measured.heading_variation_deg > settings.max_heading_deg) reason = "turning";

  cycle.rejection_reason = reason;
  cycle.automatic_rejection_reason = reason;
  cycle.accepted = reason.empty();
  cycle.automatic_accepted = cycle.accepted;
  cycle.classification = reason == "turning" ? "turning" : (reason == "standing" ? "standing" : (cycle.accepted ? "straight" : "rejected"));
}

}  // namespace

void extractGaitCycles(ProcAnalysis& analysis) {
  analysis.cycles.clear();
  if (analysis.signals.empty()) return;
  const int frame_count = static_cast<int>(analysis.signals.size());
  int index = 0;
  for (const std::string side : {"left", "right"}) {
    std::vector<const GaitEvent*> contacts;
    for (const GaitEvent& event : analysis.events) {
      if (event.side == side && event.type == "initial_contact") contacts.push_back(&event);
    }
    for (std::size_t pair = 0; pair + 1 < contacts.size(); ++pair) {
      GaitCycle cycle;
      cycle.index = index++;
      cycle.side = side;
      cycle.canonical_side = side;
      cycle.start_time = contacts[pair]->time;
      cycle.end_time = contacts[pair + 1]->time;
      cycle.start_frame = std::clamp(contacts[pair]->frame, 0, frame_count - 1);
      cycle.end_frame = std::clamp(contacts[pair + 1]->frame, 0, frame_count - 1);
      if (cycle.end_frame <= cycle.start_frame || cycle.end_time <= cycle.start_time + 1e-6) {
        cycle.accepted = false;
        cycle.automatic_accepted = false;
        cycle.rejection_reason = "duration too short";
        cycle.automatic_rejection_reason = cycle.rejection_reason;
        cycle.classification = "rejected";
        analysis.cycles.push_back(cycle);
        continue;
      }
      measureCycle(analysis, cycle);
      classify(analysis, cycle);
      if (cycle.accepted) normalizeCycle(analysis, cycle);
      analysis.cycles.push_back(std::move(cycle));
    }
  }
}

void mirrorCycles(ProcAnalysis& analysis) {
  const std::size_t raw = analysis.cycles.size();
  for (std::size_t index = 0; index < raw; ++index) {
    const GaitCycle& source = analysis.cycles[index];
    if (source.mirrored || source.side != "right" || !source.accepted) continue;
    GaitCycle copy = source;
    copy.index = static_cast<int>(analysis.cycles.size());
    copy.mirrored = true;
    copy.canonical_side = "left";
    std::swap(copy.curves.hip_flex_deg, copy.curves.opposite_hip_flex_deg);
    for (double& roll : copy.curves.pelvis_roll_deg) roll = -roll;
    copy.measurements.travel_direction_deg = -copy.measurements.travel_direction_deg;
    copy.measurements.pelvis_roll_excursion_deg = source.measurements.pelvis_roll_excursion_deg;
    analysis.cycles.push_back(std::move(copy));
  }
}

}  // namespace proc
