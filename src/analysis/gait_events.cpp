#include "analysis/gait_events.h"

#include <algorithm>
#include <cmath>

namespace proc {
namespace {

const ContactTrack* track(const ProcAnalysis& analysis, const std::string& name) {
  for (const ContactTrack& candidate : analysis.contacts) {
    if (candidate.marker == name) return &candidate;
  }
  return nullptr;
}

std::vector<std::uint8_t> supportOf(const ProcAnalysis& analysis, const std::string& side) {
  const ContactTrack* heel = track(analysis, side + "_heel");
  const ContactTrack* toe = track(analysis, side + "_toe");
  const std::size_t count = analysis.signals.size();
  std::vector<std::uint8_t> support(count, 0);
  for (std::size_t index = 0; index < count; ++index) {
    const bool heel_on = heel != nullptr && index < heel->contact.size() && heel->contact[index] != 0;
    const bool toe_on = toe != nullptr && index < toe->contact.size() && toe->contact[index] != 0;
    support[index] = (heel_on || toe_on) ? 1 : 0;
  }
  return support;
}

int refine(const ProcAnalysis& analysis, int rough, bool initial_contact, const std::string& side) {
  if (analysis.signals.empty()) return 0;
  const int count = static_cast<int>(analysis.signals.size());
  rough = std::clamp(rough, 0, count - 1);
  const double half = std::max(analysis.settings.event_refine_s, 0.0);
  const double center = analysis.signals[static_cast<std::size_t>(rough)].time;
  int best = rough;
  double best_score = initial_contact ? 1e9 : -1e9;
  for (int index = 0; index < count; ++index) {
    const SignalFrame& frame = analysis.signals[static_cast<std::size_t>(index)];
    if (std::abs(frame.time - center) > half + 1e-9) continue;
    const double height = side == "left" ? frame.left_foot_height : frame.right_foot_height;
    const double vertical = side == "left" ? frame.left_foot_vertical_speed : frame.right_foot_vertical_speed;
    const double score = initial_contact ? height : vertical;
    const bool better = initial_contact ? score < best_score - 1e-9 : score > best_score + 1e-9;
    const bool tie = std::abs(score - best_score) <= 1e-9 && std::abs(index - rough) < std::abs(best - rough);
    if (better || tie) {
      best = index;
      best_score = score;
    }
  }
  return best;
}

double eventConfidence(const ProcAnalysis& analysis, const std::string& side, int frame) {
  const ContactTrack* heel = track(analysis, side + "_heel");
  const ContactTrack* toe = track(analysis, side + "_toe");
  double sum = 0.0;
  int count = 0;
  if (heel != nullptr && frame >= 0 && frame < static_cast<int>(heel->confidence.size())) {
    sum += heel->confidence[static_cast<std::size_t>(frame)];
    ++count;
  }
  if (toe != nullptr && frame >= 0 && frame < static_cast<int>(toe->confidence.size())) {
    sum += toe->confidence[static_cast<std::size_t>(frame)];
    ++count;
  }
  return count == 0 ? 0.0 : sum / static_cast<double>(count);
}

void addTransitions(ProcAnalysis& analysis, const std::string& side, const std::vector<std::uint8_t>& support) {
  if (support.empty()) return;
  auto push = [&](int frame, const char* type) {
    const int rough = frame;
    frame = refine(analysis, frame, std::string(type) == "initial_contact", side);
    double confidence = eventConfidence(analysis, side, frame);
    confidence = std::max(confidence, eventConfidence(analysis, side, rough));
    if (rough > 0) confidence = std::max(confidence, eventConfidence(analysis, side, rough - 1));
    GaitEvent event;
    event.frame = frame;
    event.automatic_frame = frame;
    event.time = analysis.signals[static_cast<std::size_t>(frame)].time;
    event.automatic_time = event.time;
    event.side = side;
    event.type = type;
    event.automatic_type = type;
    event.confidence = confidence;
    analysis.events.push_back(event);
  };
  if (support.front() != 0) push(0, "initial_contact");
  for (int index = 1; index < static_cast<int>(support.size()); ++index) {
    if (support[static_cast<std::size_t>(index - 1)] == 0 && support[static_cast<std::size_t>(index)] != 0) push(index, "initial_contact");
    if (support[static_cast<std::size_t>(index - 1)] != 0 && support[static_cast<std::size_t>(index)] == 0) push(index, "toe_off");
  }
}

}  // namespace

void extractGaitEvents(ProcAnalysis& analysis) {
  analysis.events.clear();
  addTransitions(analysis, "left", supportOf(analysis, "left"));
  addTransitions(analysis, "right", supportOf(analysis, "right"));
  std::sort(analysis.events.begin(), analysis.events.end(), [](const GaitEvent& a, const GaitEvent& b) {
    if (a.time != b.time) return a.time < b.time;
    if (a.side != b.side) return a.side < b.side;
    return a.type < b.type;
  });
}

}  // namespace proc
