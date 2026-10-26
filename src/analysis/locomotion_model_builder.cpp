#include "analysis/locomotion_model_builder.h"

#include "analysis/signal_processing.h"

#include <glm/geometric.hpp>
#include <glm/trigonometric.hpp>

#include <algorithm>
#include <cassert>
#include <cmath>
#include <map>
#include <optional>

namespace proc {
namespace {

static_assert(kRotChannelCount == kLocoChannels, "locomotion channels must match the analysis rotation channels");

struct BuiltCycle {
  double dimensionless = 0.0;
  double speed = 0.0;
  double quality = 0.0;
  double cadence = 0.0;
  double normalized_stride = 0.0;
  double stance = 0.0;
  double flight = 0.0;
  double seam = 0.0;
  std::string seam_channel;
  std::string regime;
  std::array<std::array<glm::dvec3, kLocoPhaseSamples>, kLocoChannels> rotation{};
  std::array<double, kLocoPhaseSamples> pelvis{};
  std::array<double, kLocoPhaseSamples> left_heel{};
  std::array<double, kLocoPhaseSamples> left_toe{};
  std::array<double, kLocoPhaseSamples> right_heel{};
  std::array<double, kLocoPhaseSamples> right_toe{};
};

int frameIndex(const ProcAnalysis& analysis, double time_s) {
  int best = 0;
  double best_distance = 1e9;
  for (int index = 0; index < static_cast<int>(analysis.signals.size()); ++index) {
    const double distance = std::abs(analysis.signals[static_cast<std::size_t>(index)].time - time_s);
    if (distance < best_distance) {
      best = index;
      best_distance = distance;
    }
  }
  return best;
}

glm::dvec3 logAt(const ProcAnalysis& analysis, double time_s, int channel) {
  if (analysis.signals.empty()) return {};
  if (analysis.signals.size() == 1) return analysis.signals.front().log_vector[static_cast<std::size_t>(channel)];
  int upper = 1;
  while (upper + 1 < static_cast<int>(analysis.signals.size()) && analysis.signals[static_cast<std::size_t>(upper)].time < time_s) ++upper;
  const SignalFrame& a = analysis.signals[static_cast<std::size_t>(upper - 1)];
  const SignalFrame& b = analysis.signals[static_cast<std::size_t>(upper)];
  const double span = b.time - a.time;
  const double weight = span > 1e-9 ? std::clamp((time_s - a.time) / span, 0.0, 1.0) : 0.0;
  glm::dquat rotation = quatExp(a.log_vector[static_cast<std::size_t>(channel)]);
  rotation = quatSlerp(rotation, quatExp(b.log_vector[static_cast<std::size_t>(channel)]), weight);
  return quatLog(rotation);
}

double scalarAt(const ProcAnalysis& analysis, double time_s, double SignalFrame::*member) {
  if (analysis.signals.empty()) return 0.0;
  int upper = 1;
  if (analysis.signals.size() == 1) return analysis.signals.front().*member;
  while (upper + 1 < static_cast<int>(analysis.signals.size()) && analysis.signals[static_cast<std::size_t>(upper)].time < time_s) ++upper;
  const SignalFrame& a = analysis.signals[static_cast<std::size_t>(upper - 1)];
  const SignalFrame& b = analysis.signals[static_cast<std::size_t>(upper)];
  const double span = b.time - a.time;
  const double weight = span > 1e-9 ? std::clamp((time_s - a.time) / span, 0.0, 1.0) : 0.0;
  return a.*member * (1.0 - weight) + b.*member * weight;
}

double contactAt(const ProcAnalysis& analysis, const std::string& marker, double time_s) {
  const ContactTrack* track = nullptr;
  for (const ContactTrack& candidate : analysis.contacts) {
    if (candidate.marker == marker) track = &candidate;
  }
  if (track == nullptr || track->contact.empty()) return 0.0;
  const int frame = std::clamp(frameIndex(analysis, time_s), 0, static_cast<int>(track->contact.size()) - 1);
  return track->contact[static_cast<std::size_t>(frame)] != 0 ? 1.0 : 0.0;
}

void removeHeading(BuiltCycle& cycle, double travel_degrees) {
  const glm::dquat heading = quatAxisAngle(glm::dvec3(0.0, 0.0, 1.0), -glm::radians(travel_degrees));
  const glm::dquat inverse = quatInverse(heading);
  for (glm::dvec3& log_vector : cycle.rotation[static_cast<int>(LocoChannel::Pelvis)]) {
    log_vector = quatLog(quatUnit(inverse * quatExp(log_vector)));
  }
}

double medianOf(std::vector<double> values) {
  if (values.empty()) return 0.0;
  std::sort(values.begin(), values.end());
  const std::size_t mid = values.size() / 2;
  if (values.size() % 2 == 1) return values[mid];
  return 0.5 * (values[mid - 1] + values[mid]);
}

RegimeModel* regimeSlot(ProcLocomotionModel& model, const std::string& name) {
  if (name == "walk") return &model.walk;
  if (name == "run") return &model.run;
  if (name == "sprint") return &model.sprint;
  return nullptr;
}

double edgePhase(const std::array<double, kLocoPhaseSamples>& curve, bool falling) {
  for (int index = 1; index < kLocoPhaseSamples; ++index) {
    const double previous = curve[static_cast<std::size_t>(index - 1)];
    const double value = curve[static_cast<std::size_t>(index)];
    if (falling && previous >= 0.5 && value < 0.5) return static_cast<double>(index) / static_cast<double>(kLocoPhaseSamples);
    if (!falling && previous < 0.5 && value >= 0.5) return static_cast<double>(index) / static_cast<double>(kLocoPhaseSamples);
  }
  return -1.0;
}

double channelRms(const BuiltCycle& cycle, const SpeedKnot& knot, int channel) {
  double sum = 0.0;
  for (int sample = 0; sample < kLocoPhaseSamples; ++sample) {
    const glm::dvec3 delta = cycle.rotation[static_cast<std::size_t>(channel)][static_cast<std::size_t>(sample)] -
                             knot.rotation[static_cast<std::size_t>(channel)][static_cast<std::size_t>(sample)];
    sum += glm::dot(delta, delta);
  }
  return std::sqrt(sum / static_cast<double>(kLocoPhaseSamples));
}

std::optional<BuiltCycle> canonicalizeCycle(const ReferenceCycleInput& input, const ModelBuildSettings& settings,
                                            std::vector<std::string>& warnings) {
  if (input.analysis == nullptr) return std::nullopt;
  const ProcAnalysis& analysis = *input.analysis;
  if (input.cycle_index < 0 || input.cycle_index >= static_cast<int>(analysis.cycles.size())) return std::nullopt;
  const GaitCycle& cycle = analysis.cycles[static_cast<std::size_t>(input.cycle_index)];
  if (cycle.mirrored || !cycle.accepted || cycle.classification != "straight") return std::nullopt;
  const std::string regime = input.regime.empty() ? cycle.observed_regime : input.regime;
  if (!settings.regime.empty() && regime != settings.regime) return std::nullopt;
  if (regime != "walk" && regime != "run" && regime != "sprint") return std::nullopt;
  if (cycle.quality.overall < settings.min_quality) return std::nullopt;
  if (!(analysis.source_leg_length_m > 0.0) || analysis.signals.empty() || cycle.end_time <= cycle.start_time) return std::nullopt;
  BuiltCycle built;
  built.regime = regime;
  built.speed = input.speed_override_mps ? *input.speed_override_mps : cycle.measurements.speed_mps;
  built.dimensionless = built.speed / std::sqrt(settings.gravity_mps2 * analysis.source_leg_length_m);
  built.quality = cycle.quality.overall;
  built.cadence = cycle.measurements.cadence_spm;
  built.normalized_stride = cycle.measurements.normalized_stride_length;
  built.stance = cycle.measurements.stance_fraction;
  built.flight = cycle.measurements.flight_fraction;
  const double duration = cycle.end_time - cycle.start_time;
  std::array<double, kLocoPhaseSamples> height{};
  for (int sample = 0; sample < kLocoPhaseSamples; ++sample) {
    const double phase = static_cast<double>(sample) / static_cast<double>(kLocoPhaseSamples);
    const double time_s = cycle.start_time + phase * duration;
    for (int channel = 0; channel < kLocoChannels; ++channel) {
      built.rotation[static_cast<std::size_t>(channel)][static_cast<std::size_t>(sample)] = logAt(analysis, time_s, channel);
    }
    height[static_cast<std::size_t>(sample)] = scalarAt(analysis, time_s, &SignalFrame::root_z);
    built.left_heel[static_cast<std::size_t>(sample)] = contactAt(analysis, "left_heel", time_s);
    built.left_toe[static_cast<std::size_t>(sample)] = contactAt(analysis, "left_toe", time_s);
    built.right_heel[static_cast<std::size_t>(sample)] = contactAt(analysis, "right_heel", time_s);
    built.right_toe[static_cast<std::size_t>(sample)] = contactAt(analysis, "right_toe", time_s);
  }
  double mean_height = 0.0;
  for (double value : height) mean_height += value;
  mean_height /= static_cast<double>(kLocoPhaseSamples);
  for (int sample = 0; sample < kLocoPhaseSamples; ++sample) {
    built.pelvis[static_cast<std::size_t>(sample)] = (height[static_cast<std::size_t>(sample)] - mean_height) / analysis.source_leg_length_m;
  }
  std::array<glm::dvec3, kLocoChannels> endpoint{};
  for (int channel = 0; channel < kLocoChannels; ++channel) endpoint[static_cast<std::size_t>(channel)] = logAt(analysis, cycle.end_time, channel);
  removeHeading(built, cycle.measurements.travel_direction_deg);
  {
    const glm::dquat heading = quatAxisAngle(glm::dvec3(0.0, 0.0, 1.0), -glm::radians(cycle.measurements.travel_direction_deg));
    endpoint[static_cast<int>(LocoChannel::Pelvis)] = quatLog(quatUnit(quatInverse(heading) * quatExp(endpoint[static_cast<int>(LocoChannel::Pelvis)])));
  }
  if (cycle.side == "right") mirrorLogChannels(endpoint);
  if (cycle.side == "right") {
    for (int sample = 0; sample < kLocoPhaseSamples; ++sample) {
      std::array<glm::dvec3, kLocoChannels> logs;
      for (int channel = 0; channel < kLocoChannels; ++channel) logs[static_cast<std::size_t>(channel)] = built.rotation[static_cast<std::size_t>(channel)][static_cast<std::size_t>(sample)];
      mirrorLogChannels(logs);
      for (int channel = 0; channel < kLocoChannels; ++channel) built.rotation[static_cast<std::size_t>(channel)][static_cast<std::size_t>(sample)] = logs[static_cast<std::size_t>(channel)];
    }
    std::swap(built.left_heel, built.right_heel);
    std::swap(built.left_toe, built.right_toe);
  }
  for (int channel = 0; channel < kLocoChannels; ++channel) {
    const double error = glm::length(built.rotation[static_cast<std::size_t>(channel)][0] - endpoint[static_cast<std::size_t>(channel)]);
    if (error > built.seam) {
      built.seam = error;
      built.seam_channel = locoChannelName(channel);
    }
  }
  if (built.seam > settings.seam_warn) {
    warnings.push_back(analysis.source.clip + " cycle " + std::to_string(cycle.index) + " seam " + built.seam_channel);
  }
  return built;
}

SpeedKnot knotFrom(const std::vector<BuiltCycle>& cycles, int min_count) {
  SpeedKnot knot;
  knot.cycle_count = static_cast<int>(cycles.size());
  knot.low_confidence = knot.cycle_count < min_count;
  std::vector<double> dims, speeds, qualities, cadences, strides, stances, flights;
  for (const BuiltCycle& cycle : cycles) {
    dims.push_back(cycle.dimensionless);
    speeds.push_back(cycle.speed);
    qualities.push_back(cycle.quality);
    cadences.push_back(cycle.cadence);
    strides.push_back(cycle.normalized_stride);
    stances.push_back(cycle.stance);
    flights.push_back(cycle.flight);
    if (cycle.seam >= knot.spread) {
      // seam is tracked separately below
    }
  }
  knot.dimensionless_speed = medianOf(dims);
  knot.median_speed_mps = medianOf(speeds);
  knot.quality_median = medianOf(qualities);
  knot.cadence_spm = medianOf(cadences);
  knot.normalized_stride_length = medianOf(strides);
  knot.stance_fraction = medianOf(stances);
  knot.flight_fraction = medianOf(flights);
  double spread_sum = 0.0;
  int spread_count = 0;
  for (int channel = 0; channel < kLocoChannels; ++channel) {
    for (int sample = 0; sample < kLocoPhaseSamples; ++sample) {
      const glm::dquat reference = quatExp(cycles.front().rotation[static_cast<std::size_t>(channel)][static_cast<std::size_t>(sample)]);
      std::vector<double> x, y, z;
      for (const BuiltCycle& cycle : cycles) {
        glm::dquat rotation = quatExp(cycle.rotation[static_cast<std::size_t>(channel)][static_cast<std::size_t>(sample)]);
        rotation = hemisphereContinuous(reference, rotation);
        const glm::dvec3 log_vector = quatLog(rotation);
        x.push_back(log_vector.x);
        y.push_back(log_vector.y);
        z.push_back(log_vector.z);
      }
      auto iqr = [](std::vector<double> values) {
        if (values.size() < 2) return 0.0;
        std::sort(values.begin(), values.end());
        const double p25 = values[(values.size() - 1) / 4];
        const double p75 = values[(values.size() * 3) / 4];
        return p75 - p25;
      };
      spread_sum += iqr(x) + iqr(y) + iqr(z);
      spread_count += 3;
      knot.rotation[static_cast<std::size_t>(channel)][static_cast<std::size_t>(sample)] =
          glm::dvec3(medianOf(x), medianOf(y), medianOf(z));
    }
  }
  knot.spread = spread_count > 0 ? spread_sum / static_cast<double>(spread_count) : 0.0;
  auto medianCurve = [&](const std::array<double, kLocoPhaseSamples> BuiltCycle::*member) {
    std::array<double, kLocoPhaseSamples> curve{};
    for (int sample = 0; sample < kLocoPhaseSamples; ++sample) {
      std::vector<double> values;
      for (const BuiltCycle& cycle : cycles) values.push_back((cycle.*member)[static_cast<std::size_t>(sample)]);
      curve[static_cast<std::size_t>(sample)] = medianOf(values);
    }
    return curve;
  };
  knot.pelvis_vertical = medianCurve(&BuiltCycle::pelvis);
  knot.contact_left_heel = medianCurve(&BuiltCycle::left_heel);
  knot.contact_left_toe = medianCurve(&BuiltCycle::left_toe);
  knot.contact_right_heel = medianCurve(&BuiltCycle::right_heel);
  knot.contact_right_toe = medianCurve(&BuiltCycle::right_toe);
  return knot;
}

}  // namespace

const SpeedKnot* nearestKnot(const ProcLocomotionModel& model, const std::string& regime, double speed) {
  const RegimeModel* slot = regime == "walk" ? &model.walk : regime == "run" ? &model.run : regime == "sprint" ? &model.sprint : nullptr;
  if (slot == nullptr || slot->knots.empty()) return nullptr;
  const SpeedKnot* best = &slot->knots.front();
  for (const SpeedKnot& knot : slot->knots) {
    if (std::abs(knot.dimensionless_speed - speed) < std::abs(best->dimensionless_speed - speed)) best = &knot;
  }
  return best;
}

ProcLocomotionModel buildLocomotionModel(const std::vector<ReferenceCycleInput>& inputs, const ModelBuildSettings& settings,
                                         std::vector<std::string>& warnings) {
  ProcLocomotionModel model;
  model.gravity_mps2 = settings.gravity_mps2;
  model.bin_width = settings.bin_width > 1e-4 ? settings.bin_width : 0.20;
  const int min_count = std::max(1, settings.min_cycles_per_knot);
  std::map<std::string, std::map<int, std::vector<BuiltCycle>>> grouped;
  for (const ReferenceCycleInput& input : inputs) {
    if (input.analysis == nullptr) continue;
    if (!settings.subject.empty() && input.analysis->source.subject != settings.subject) continue;
    if (!(input.analysis->source_leg_length_m > 0.0)) {
      warnings.push_back(input.analysis->source.clip + ": skipped, source leg length is missing");
      continue;
    }
    std::optional<BuiltCycle> built = canonicalizeCycle(input, settings, warnings);
    if (!built) continue;
    const int bin = static_cast<int>(std::floor(built->dimensionless / model.bin_width));
    grouped[built->regime][bin].push_back(std::move(*built));
  }

  for (auto& regime_entry : grouped) {
    std::vector<std::pair<int, std::vector<BuiltCycle>>> bins(regime_entry.second.begin(), regime_entry.second.end());
    bool merged = true;
    while (merged) {
      merged = false;
      for (std::size_t index = 0; index < bins.size(); ++index) {
        if (static_cast<int>(bins[index].second.size()) >= min_count || bins.size() < 2) continue;
        std::size_t neighbor = index == 0 ? 1 : index - 1;
        int best = std::abs(bins[neighbor].first - bins[index].first);
        for (std::size_t other = 0; other < bins.size(); ++other) {
          if (other == index) continue;
          const int distance = std::abs(bins[other].first - bins[index].first);
          if (distance < best) {
            best = distance;
            neighbor = other;
          }
        }
        bins[neighbor].second.insert(bins[neighbor].second.end(), bins[index].second.begin(), bins[index].second.end());
        bins.erase(bins.begin() + static_cast<std::ptrdiff_t>(index));
        merged = true;
        break;
      }
    }
    RegimeModel* slot = regimeSlot(model, regime_entry.first);
    if (slot == nullptr) continue;
    for (const auto& bin : bins) {
      slot->knots.push_back(knotFrom(bin.second, min_count));
      if (bin.second.size() < static_cast<std::size_t>(min_count)) {
        warnings.push_back(regime_entry.first + " knot kept with low confidence");
      }
    }
    std::sort(slot->knots.begin(), slot->knots.end(), [](const SpeedKnot& a, const SpeedKnot& b) {
      return a.dimensionless_speed < b.dimensionless_speed;
    });
    slot->present = !slot->knots.empty();
    if (!slot->present) continue;
    slot->support_min = slot->knots.front().dimensionless_speed - 0.5 * model.bin_width;
    slot->support_max = slot->knots.back().dimensionless_speed + 0.5 * model.bin_width;
    for (const auto& bin : bins) {
      for (const BuiltCycle& cycle : bin.second) {
        if (cycle.seam > slot->seam_error) {
          slot->seam_error = cycle.seam;
          slot->seam_channel = cycle.seam_channel;
        }
      }
    }
    if (slot->seam_error > model.max_seam_error) {
      model.max_seam_error = slot->seam_error;
      model.max_seam_channel = slot->seam_channel;
    }
  }
  auto markGap = [](RegimeModel& lower, RegimeModel& upper) {
    if (!lower.present || !upper.present) return;
    if (lower.support_max < upper.support_min) {
      lower.transition_inferred = true;
      upper.transition_inferred = true;
    }
  };
  markGap(model.walk, model.run);
  markGap(model.run, model.sprint);
  return model;
}

ProcLocomotionModel buildLocomotionModel(const std::vector<ProcAnalysis>& analyses, const ModelBuildSettings& settings,
                                         std::vector<std::string>& warnings) {
  std::vector<ReferenceCycleInput> inputs;
  for (const ProcAnalysis& analysis : analyses) {
    for (int index = 0; index < static_cast<int>(analysis.cycles.size()); ++index) {
      ReferenceCycleInput input;
      input.analysis = &analysis;
      input.cycle_index = index;
      inputs.push_back(input);
    }
  }
  return buildLocomotionModel(inputs, settings, warnings);
}

ReferenceFidelity measureReferenceFidelity(const ProcLocomotionModel& model, const std::vector<ReferenceCycleInput>& inputs,
                                           const ModelBuildSettings& settings) {
  ReferenceFidelity fidelity;
  double hip = 0.0;
  double knee = 0.0;
  double ankle = 0.0;
  double pelvis = 0.0;
  double shoulder = 0.0;
  double elbow = 0.0;
  double stance = 0.0;
  double flight = 0.0;
  double toe = 0.0;
  double contact = 0.0;
  int contact_count = 0;
  std::vector<std::string> ignored;
  for (const ReferenceCycleInput& input : inputs) {
    const std::optional<BuiltCycle> built = canonicalizeCycle(input, settings, ignored);
    if (!built) continue;
    const SpeedKnot* knot = nearestKnot(model, built->regime, built->dimensionless);
    if (knot == nullptr) continue;
    hip += channelRms(*built, *knot, static_cast<int>(LocoChannel::LeftHip));
    knee += channelRms(*built, *knot, static_cast<int>(LocoChannel::LeftKnee));
    ankle += channelRms(*built, *knot, static_cast<int>(LocoChannel::LeftAnkle));
    pelvis += channelRms(*built, *knot, static_cast<int>(LocoChannel::Pelvis));
    shoulder += channelRms(*built, *knot, static_cast<int>(LocoChannel::LeftShoulder));
    elbow += channelRms(*built, *knot, static_cast<int>(LocoChannel::LeftElbow));
    stance += knot->stance_fraction - built->stance;
    flight += knot->flight_fraction - built->flight;
    const double source_toe = edgePhase(built->left_heel, true);
    const double model_toe = edgePhase(knot->contact_left_heel, true);
    const double source_contact = edgePhase(built->left_heel, false);
    const double model_contact = edgePhase(knot->contact_left_heel, false);
    if (source_toe >= 0.0 && model_toe >= 0.0) {
      toe += model_toe - source_toe;
      ++contact_count;
    }
    if (source_contact >= 0.0 && model_contact >= 0.0) contact += model_contact - source_contact;
    ++fidelity.cycles;
  }
  if (fidelity.cycles == 0) return fidelity;
  const double count = static_cast<double>(fidelity.cycles);
  fidelity.available = true;
  fidelity.hip_rms = hip / count;
  fidelity.knee_rms = knee / count;
  fidelity.ankle_rms = ankle / count;
  fidelity.pelvis_rms = pelvis / count;
  fidelity.shoulder_rms = shoulder / count;
  fidelity.elbow_rms = elbow / count;
  fidelity.stance_difference = stance / count;
  fidelity.flight_difference = flight / count;
  if (contact_count > 0) {
    fidelity.toe_off_phase_difference = toe / static_cast<double>(contact_count);
    fidelity.initial_contact_phase_difference = contact / static_cast<double>(contact_count);
  }
  return fidelity;
}

}  // namespace proc
