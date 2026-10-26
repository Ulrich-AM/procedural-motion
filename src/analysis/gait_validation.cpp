#include "analysis/gait_validation.h"

#include "analysis/analysis_io.h"
#include "analysis/locomotion_model_builder.h"
#include "body/proc_body.h"
#include "dataset/batch_analyze.h"
#include "locomotion/locomotion_audit.h"
#include "locomotion/locomotion_diagnostics.h"
#include "locomotion/locomotion_generator.h"
#include "locomotion/locomotion_model_io.h"
#include "motion/proc_motion.h"
#include "retarget/source_semantics.h"
#include "rig/forward_kinematics.h"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <map>
#include <set>
#include <sstream>

namespace proc {
namespace {

std::string lowerCopy(std::string text) {
  for (char& character : text) {
    if (character >= 'A' && character <= 'Z') character = static_cast<char>(character - 'A' + 'a');
  }
  return text;
}

std::string trim(std::string text) {
  while (!text.empty() && (text.front() == ' ' || text.front() == '\t')) text.erase(text.begin());
  while (!text.empty() && (text.back() == ' ' || text.back() == '\t')) text.pop_back();
  return text;
}

const RegimeModel* regimeOf(const ProcLocomotionModel& model, const std::string& name) {
  if (name == "walk") return &model.walk;
  if (name == "run") return &model.run;
  if (name == "sprint") return &model.sprint;
  return nullptr;
}

struct KnotSample {
  const SpeedKnot* a = nullptr;
  const SpeedKnot* b = nullptr;
  double weight = 0.0;
};

KnotSample knotsAt(const RegimeModel& regime, double speed) {
  KnotSample sample;
  if (regime.knots.empty()) return sample;
  if (speed <= regime.knots.front().dimensionless_speed || regime.knots.size() == 1) {
    sample.a = sample.b = &regime.knots.front();
    return sample;
  }
  if (speed >= regime.knots.back().dimensionless_speed) {
    sample.a = sample.b = &regime.knots.back();
    return sample;
  }
  for (std::size_t index = 0; index + 1 < regime.knots.size(); ++index) {
    const SpeedKnot& left = regime.knots[index];
    const SpeedKnot& right = regime.knots[index + 1];
    if (speed >= left.dimensionless_speed && speed <= right.dimensionless_speed) {
      const double span = right.dimensionless_speed - left.dimensionless_speed;
      sample.a = &left;
      sample.b = &right;
      sample.weight = span > 1e-9 ? (speed - left.dimensionless_speed) / span : 0.0;
      return sample;
    }
  }
  sample.a = sample.b = &regime.knots.back();
  return sample;
}

double blendScalar(double a, double b, double weight) { return a * (1.0 - weight) + b * weight; }

std::vector<double> channelCurve(const RegimeModel& regime, double speed, int channel) {
  std::vector<double> curve(kLocoPhaseSamples, 0.0);
  const KnotSample knots = knotsAt(regime, speed);
  if (knots.a == nullptr) return curve;
  for (int sample = 0; sample < kLocoPhaseSamples; ++sample) {
    const double phase = static_cast<double>(sample) / static_cast<double>(kLocoPhaseSamples);
    const double left = sampleLog(knots.a->rotation[static_cast<std::size_t>(channel)], phase).x;
    const double right = sampleLog(knots.b->rotation[static_cast<std::size_t>(channel)], phase).x;
    curve[static_cast<std::size_t>(sample)] = blendScalar(left, right, knots.weight);
  }
  return curve;
}

std::vector<double> pelvisCurve(const RegimeModel& regime, double speed) {
  std::vector<double> curve(kLocoPhaseSamples, 0.0);
  const KnotSample knots = knotsAt(regime, speed);
  if (knots.a == nullptr) return curve;
  for (int sample = 0; sample < kLocoPhaseSamples; ++sample) {
    const double phase = static_cast<double>(sample) / static_cast<double>(kLocoPhaseSamples);
    curve[static_cast<std::size_t>(sample)] =
        blendScalar(samplePeriodic(knots.a->pelvis_vertical, phase), samplePeriodic(knots.b->pelvis_vertical, phase), knots.weight);
  }
  return curve;
}

double rmsOf(const std::vector<double>& a, const std::vector<double>& b) {
  if (a.empty() || a.size() != b.size()) return 0.0;
  double sum = 0.0;
  for (std::size_t index = 0; index < a.size(); ++index) {
    const double delta = a[index] - b[index];
    sum += delta * delta;
  }
  return std::sqrt(sum / static_cast<double>(a.size()));
}

double rangeOf(const std::vector<double>& values) {
  if (values.empty()) return 0.0;
  const auto [low, high] = std::minmax_element(values.begin(), values.end());
  return *high - *low;
}

double peakPhase(const std::vector<double>& values) {
  if (values.empty()) return 0.0;
  const auto low = std::min_element(values.begin(), values.end());
  return static_cast<double>(std::distance(values.begin(), low)) / static_cast<double>(values.size());
}

double phaseDelta(double a, double b) {
  double delta = std::abs(a - b);
  if (delta > 0.5) delta = 1.0 - delta;
  return delta;
}

double scalarAtSpeed(const RegimeModel& regime, double speed, double SpeedKnot::*member) {
  const KnotSample knots = knotsAt(regime, speed);
  if (knots.a == nullptr) return 0.0;
  return blendScalar(knots.a->*member, knots.b->*member, knots.weight);
}

bool eligible(const ManifestCycle& cycle) {
  return cycle.accepted && cycle.classification == "straight" &&
         (cycle.observed_regime == "walk" || cycle.observed_regime == "run" || cycle.observed_regime == "sprint");
}

int binOf(double dimensionless, double bin_width) {
  const double width = bin_width > 1e-4 ? bin_width : 0.20;
  return static_cast<int>(std::floor(dimensionless / width));
}

std::vector<double> resample(const std::vector<double>& source, std::size_t count) {
  std::vector<double> values(count, 0.0);
  if (source.empty() || count == 0) return values;
  if (source.size() == 1) {
    std::fill(values.begin(), values.end(), source.front());
    return values;
  }
  for (std::size_t index = 0; index < count; ++index) {
    const double position = static_cast<double>(index) / static_cast<double>(count) * static_cast<double>(source.size());
    const std::size_t lower = std::min(source.size() - 1, static_cast<std::size_t>(position));
    const std::size_t upper = std::min(source.size() - 1, lower + 1);
    const double weight = position - std::floor(position);
    values[index] = source[lower] * (1.0 - weight) + source[upper] * weight;
  }
  return values;
}

nlohmann::json cycleJson(const ManifestCycle& cycle) {
  return {{"source_clip", cycle.source_clip},
          {"dataset", cycle.dataset},
          {"subject", cycle.subject},
          {"theme", cycle.theme},
          {"cycle_index", cycle.cycle_index},
          {"side", cycle.side},
          {"start_time", cycle.start_time},
          {"end_time", cycle.end_time},
          {"speed_mps", cycle.speed_mps},
          {"dimensionless_speed", cycle.dimensionless_speed},
          {"cadence_spm", cycle.cadence_spm},
          {"stride_length_m", cycle.stride_length_m},
          {"stance_fraction", cycle.stance_fraction},
          {"flight_fraction", cycle.flight_fraction},
          {"quality",
           {{"contact", cycle.quality_contact},
            {"timing", cycle.quality_timing},
            {"heading", cycle.quality_heading},
            {"overall", cycle.quality_overall}}},
          {"accepted", cycle.accepted},
          {"rejection_reason", cycle.rejection_reason},
          {"classification", cycle.classification},
          {"observed_regime", cycle.observed_regime},
          {"manual_override", cycle.manual_override},
          {"review", cycle.review},
          {"reviewed", cycle.review == "manually reviewed"},
          {"holdout", cycle.holdout},
          {"used_in_model", cycle.used_in_model}};
}

void countRegime(const ExperimentManifest& manifest, const std::string& regime, int& candidates, int& accepted, int& rejected) {
  candidates = accepted = rejected = 0;
  for (const ManifestCycle& cycle : manifest.cycles) {
    if (cycle.observed_regime != regime && lowerCopy(cycle.theme) != regime) continue;
    if (cycle.observed_regime != regime && cycle.observed_regime != "walk" && cycle.observed_regime != "run" && cycle.observed_regime != "sprint") {
      if (lowerCopy(cycle.theme).find(regime) == std::string::npos) continue;
    }
    if (cycle.observed_regime != regime) continue;
    ++candidates;
    if (cycle.accepted) ++accepted;
    else ++rejected;
  }
}

std::string tierOf(const GaitExperimentResult& result) {
  if (!result.dataset_found) return "0";
  bool walk = false;
  bool run = false;
  bool sprint = false;
  for (const KnotReport& knot : result.knots) {
    if (knot.regime == "walk" && knot.cycle_count > 0) walk = true;
    if (knot.regime == "run" && knot.cycle_count > 0) run = true;
    if (knot.regime == "sprint" && knot.cycle_count > 0) sprint = true;
  }
  if (result.holdout.cycles > 0 && (walk || run || sprint)) return "4";
  if (walk && run && sprint) return "3";
  if (walk && run) return "2";
  if (walk) return "1";
  return "0";
}

}  // namespace

GaitArgumentParse parseGaitExperimentArgs(int argc, char** argv) {
  GaitArgumentParse parsed;
  parsed.ok = true;
  for (int index = 1; index < argc; ++index) {
    const std::string argument = argv[index];
    auto need = [&](std::string& slot) {
      if (index + 1 >= argc) {
        parsed.ok = false;
        parsed.error = "missing value for " + argument;
        return false;
      }
      slot = argv[++index];
      return true;
    };
    if (argument == "--help" || argument == "-h") {
      parsed.help = true;
      return parsed;
    }
    std::string value;
    if (argument == "--dataset") {
      if (!need(value)) return parsed;
      parsed.config.dataset = value;
    } else if (argument == "--output") {
      if (!need(value)) return parsed;
      parsed.config.output = value;
    } else if (argument == "--body") {
      if (!need(value)) return parsed;
      parsed.config.body = value;
    } else if (argument == "--profile") {
      if (!need(value)) return parsed;
      parsed.config.profile = value;
    } else if (argument == "--profiles") {
      if (!need(value)) return parsed;
      parsed.config.profile_directory = value;
    } else if (argument == "--themes") {
      if (!need(value)) return parsed;
      parsed.config.themes = value;
    } else if (argument == "--synthetic") {
      if (!need(value)) return parsed;
      parsed.config.synthetic_model = value;
    } else if (argument == "--min-quality") {
      if (!need(value)) return parsed;
      parsed.config.min_quality = std::stod(value);
    } else if (argument == "--min-cycles-per-knot") {
      if (!need(value)) return parsed;
      parsed.config.min_cycles_per_knot = std::stoi(value);
    } else if (argument == "--max-cycles-per-subject") {
      if (!need(value)) return parsed;
      parsed.config.max_cycles_per_subject = std::stoi(value);
    } else if (argument == "--bin-width") {
      if (!need(value)) return parsed;
      parsed.config.bin_width = std::stod(value);
    } else {
      parsed.ok = false;
      parsed.error = "unknown argument " + argument;
      return parsed;
    }
  }
  if (parsed.config.dataset.empty()) {
    parsed.ok = false;
    parsed.error = "missing --dataset";
  }
  return parsed;
}

bool themeAllowed(const std::string& theme, const std::string& filename, const std::string& filter) {
  if (filter.empty()) return true;
  const std::string hay_theme = lowerCopy(theme);
  const std::string hay_name = lowerCopy(filename);
  std::string rest = filter;
  while (!rest.empty()) {
    const std::size_t comma = rest.find(',');
    const std::string part = trim(rest.substr(0, comma));
    const std::string needle = lowerCopy(part);
    if (!needle.empty() && (hay_theme.find(needle) != std::string::npos || hay_name.find(needle) != std::string::npos)) return true;
    if (comma == std::string::npos) break;
    rest = rest.substr(comma + 1);
  }
  return false;
}

bool outputIsPublicAsset(const std::filesystem::path& output) {
  const std::filesystem::path normal = output.lexically_normal();
  auto next = normal.begin();
  if (next == normal.end()) return false;
  for (auto it = normal.begin(); it != normal.end(); ++it) {
    next = it;
    ++next;
    if (next == normal.end()) break;
    if (it->string() == "assets" && next->string() == "models") return true;
  }
  return false;
}

ExperimentManifest manifestFromAnalyses(const std::vector<ProcAnalysis>& analyses, double gravity_mps2) {
  ExperimentManifest manifest;
  for (const ProcAnalysis& analysis : analyses) {
    const double scale = analysis.source_leg_length_m > 0.0 ? std::sqrt(gravity_mps2 * analysis.source_leg_length_m) : 0.0;
    for (const GaitCycle& cycle : analysis.cycles) {
      if (cycle.mirrored) continue;
      ManifestCycle row;
      row.source_clip = analysis.source.clip.empty() ? analysis.source.motion_path : analysis.source.clip;
      row.dataset = analysis.source.dataset;
      row.subject = analysis.source.subject;
      row.theme = analysis.source.theme;
      row.cycle_index = cycle.index;
      row.side = cycle.side;
      row.start_time = cycle.start_time;
      row.end_time = cycle.end_time;
      row.speed_mps = cycle.measurements.speed_mps;
      row.dimensionless_speed = scale > 0.0 ? cycle.measurements.speed_mps / scale : cycle.measurements.froude_number;
      row.cadence_spm = cycle.measurements.cadence_spm;
      row.stride_length_m = cycle.measurements.stride_length_m;
      row.stance_fraction = cycle.measurements.stance_fraction;
      row.flight_fraction = cycle.measurements.flight_fraction;
      row.quality_contact = cycle.quality.contact;
      row.quality_timing = cycle.quality.timing;
      row.quality_heading = cycle.quality.heading;
      row.quality_overall = cycle.quality.overall;
      row.accepted = cycle.accepted;
      row.rejection_reason = cycle.rejection_reason;
      row.classification = cycle.classification;
      row.observed_regime = cycle.observed_regime;
      row.manual_override = cycle.manual_override;
      row.review = cycle.manual_override ? "manually reviewed" : "automatic";
      row.hip_deg = cycle.curves.hip_flex_deg;
      row.knee_deg = cycle.curves.knee_flex_deg;
      row.ankle_deg = cycle.curves.ankle_flex_deg;
      manifest.cycles.push_back(std::move(row));
    }
  }
  return manifest;
}

void noteFailedClip(ExperimentManifest& manifest, const std::string& filename, const std::string& reason) {
  manifest.failed_clips.push_back(FailedClip{filename, reason});
}

void capCyclesPerSubject(std::vector<ProcAnalysis>& analyses, ExperimentManifest& manifest, int cap, double bin_width, double gravity_mps2,
                         const std::vector<int>& build_rows) {
  if (cap < 1) return;
  const std::set<int> build_set(build_rows.begin(), build_rows.end());
  struct Pick {
    int analysis = 0;
    int cycle = 0;
    int manifest = -1;
    double quality = 0.0;
    std::string key;
  };
  std::vector<Pick> picks;
  int manifest_index = 0;
  for (int analysis_index = 0; analysis_index < static_cast<int>(analyses.size()); ++analysis_index) {
    ProcAnalysis& analysis = analyses[static_cast<std::size_t>(analysis_index)];
    const double scale = analysis.source_leg_length_m > 0.0 ? std::sqrt(gravity_mps2 * analysis.source_leg_length_m) : 0.0;
    for (int cycle_index = 0; cycle_index < static_cast<int>(analysis.cycles.size()); ++cycle_index) {
      GaitCycle& cycle = analysis.cycles[static_cast<std::size_t>(cycle_index)];
      int row = -1;
      if (!cycle.mirrored && manifest_index < static_cast<int>(manifest.cycles.size())) row = manifest_index++;
      if (cycle.mirrored || !cycle.accepted || cycle.classification != "straight") continue;
      if (!build_set.empty() && build_set.count(row) == 0) continue;
      if (cycle.observed_regime != "walk" && cycle.observed_regime != "run" && cycle.observed_regime != "sprint") continue;
      const double dimensionless = scale > 0.0 ? cycle.measurements.speed_mps / scale : 0.0;
      Pick pick;
      pick.analysis = analysis_index;
      pick.cycle = cycle_index;
      pick.manifest = row;
      pick.quality = cycle.quality.overall;
      pick.key = cycle.observed_regime + "|" + analysis.source.subject + "|" + std::to_string(binOf(dimensionless, bin_width));
      picks.push_back(pick);
    }
  }
  std::stable_sort(picks.begin(), picks.end(), [](const Pick& a, const Pick& b) {
    if (a.key != b.key) return a.key < b.key;
    if (a.quality != b.quality) return a.quality > b.quality;
    return a.manifest < b.manifest;
  });
  std::map<std::string, int> kept;
  for (const Pick& pick : picks) {
    int& count = kept[pick.key];
    if (count < cap) {
      ++count;
      continue;
    }
    GaitCycle& cycle = analyses[static_cast<std::size_t>(pick.analysis)].cycles[static_cast<std::size_t>(pick.cycle)];
    cycle.accepted = false;
    cycle.rejection_reason = "subject cap";
    if (pick.manifest >= 0 && pick.manifest < static_cast<int>(manifest.cycles.size())) {
      manifest.cycles[static_cast<std::size_t>(pick.manifest)].accepted = false;
      manifest.cycles[static_cast<std::size_t>(pick.manifest)].rejection_reason = "subject cap";
    }
  }
}

ValidationSplit splitManifest(const ExperimentManifest& manifest) {
  ValidationSplit split;
  std::vector<int> eligible_rows;
  for (int index = 0; index < static_cast<int>(manifest.cycles.size()); ++index) {
    if (eligible(manifest.cycles[static_cast<std::size_t>(index)])) eligible_rows.push_back(index);
  }
  std::set<std::string> subjects;
  std::set<std::string> clips;
  for (int index : eligible_rows) {
    subjects.insert(manifest.cycles[static_cast<std::size_t>(index)].subject);
    clips.insert(manifest.cycles[static_cast<std::size_t>(index)].source_clip);
  }
  if (subjects.size() >= 2) {
    split.method = "subject";
    const std::string held = *subjects.rbegin();
    for (int index : eligible_rows) {
      if (manifest.cycles[static_cast<std::size_t>(index)].subject == held) split.holdout.push_back(index);
      else split.build.push_back(index);
    }
    return split;
  }
  if (clips.size() >= 2) {
    split.method = "clip";
    const std::string held = *clips.rbegin();
    for (int index : eligible_rows) {
      if (manifest.cycles[static_cast<std::size_t>(index)].source_clip == held) split.holdout.push_back(index);
      else split.build.push_back(index);
    }
    return split;
  }
  split.method = eligible_rows.empty() ? "none" : "no holdout";
  split.build = eligible_rows;
  return split;
}

std::vector<KnotReport> classifyKnots(const ProcLocomotionModel& model, const ExperimentManifest& manifest, const ValidationSplit& split,
                                      int min_cycles) {
  std::vector<KnotReport> reports;
  for (const char* regime_name : {"walk", "run", "sprint"}) {
    const RegimeModel* regime = regimeOf(model, regime_name);
    if (regime == nullptr) continue;
    std::vector<const ManifestCycle*> members;
    for (int index : split.build) {
      const ManifestCycle& cycle = manifest.cycles[static_cast<std::size_t>(index)];
      if (cycle.observed_regime == regime_name && cycle.used_in_model) members.push_back(&cycle);
    }
    for (const SpeedKnot& knot : regime->knots) {
      KnotReport report;
      report.regime = regime_name;
      report.dimensionless_speed = knot.dimensionless_speed;
      report.speed_mps = knot.median_speed_mps;
      report.cycle_count = knot.cycle_count;
      report.quality_median = knot.quality_median;
      report.spread = knot.spread;
      report.cadence_spm = knot.cadence_spm;
      report.stance_fraction = knot.stance_fraction;
      report.flight_fraction = knot.flight_fraction;
      report.low_confidence = knot.low_confidence || knot.cycle_count < min_cycles;
      report.trusted = knot.cycle_count >= min_cycles && !knot.low_confidence;
      std::set<std::string> clips;
      std::set<std::string> subjects;
      for (const ManifestCycle* cycle : members) {
        const SpeedKnot* best = &regime->knots.front();
        for (const SpeedKnot& candidate : regime->knots) {
          if (std::abs(candidate.dimensionless_speed - cycle->dimensionless_speed) < std::abs(best->dimensionless_speed - cycle->dimensionless_speed)) {
            best = &candidate;
          }
        }
        if (best != &knot) continue;
        clips.insert(cycle->source_clip);
        subjects.insert(cycle->subject);
      }
      report.clip_count = static_cast<int>(clips.size());
      report.subject_count = static_cast<int>(subjects.size());
      reports.push_back(report);
    }
  }
  return reports;
}

HoldoutMetrics measureHoldout(const ProcLocomotionModel& model, const ExperimentManifest& manifest, const ValidationSplit& split) {
  HoldoutMetrics metrics;
  double cadence = 0.0;
  double stride = 0.0;
  double stance = 0.0;
  double flight = 0.0;
  double hip = 0.0;
  double knee = 0.0;
  double ankle = 0.0;
  int curves = 0;
  for (int index : split.holdout) {
    const ManifestCycle& cycle = manifest.cycles[static_cast<std::size_t>(index)];
    const RegimeModel* regime = regimeOf(model, cycle.observed_regime);
    if (regime == nullptr || regime->knots.empty()) continue;
    ++metrics.cycles;
    const double model_cadence = scalarAtSpeed(*regime, cycle.dimensionless_speed, &SpeedKnot::cadence_spm);
    const double model_stance = scalarAtSpeed(*regime, cycle.dimensionless_speed, &SpeedKnot::stance_fraction);
    const double model_flight = scalarAtSpeed(*regime, cycle.dimensionless_speed, &SpeedKnot::flight_fraction);
    const double model_stride = model_cadence > 1e-6 ? cycle.speed_mps * 120.0 / model_cadence : 0.0;
    cadence += model_cadence - cycle.cadence_spm;
    stride += model_stride - cycle.stride_length_m;
    stance += model_stance - cycle.stance_fraction;
    flight += model_flight - cycle.flight_fraction;
    if (!cycle.hip_deg.empty() && !cycle.knee_deg.empty() && !cycle.ankle_deg.empty()) {
      const std::vector<double> hip_model = channelCurve(*regime, cycle.dimensionless_speed, static_cast<int>(LocoChannel::LeftHip));
      const std::vector<double> knee_model = channelCurve(*regime, cycle.dimensionless_speed, static_cast<int>(LocoChannel::LeftKnee));
      const std::vector<double> ankle_model = channelCurve(*regime, cycle.dimensionless_speed, static_cast<int>(LocoChannel::LeftAnkle));
      auto degrees = [](const std::vector<double>& logs, std::size_t count) {
        std::vector<double> values;
        values.reserve(logs.size());
        for (double value : logs) values.push_back(value * 2.0 * 180.0 / 3.14159265358979323846);
        return resample(values, count);
      };
      hip += rmsOf(degrees(hip_model, cycle.hip_deg.size()), cycle.hip_deg);
      knee += rmsOf(degrees(knee_model, cycle.knee_deg.size()), cycle.knee_deg);
      ankle += rmsOf(degrees(ankle_model, cycle.ankle_deg.size()), cycle.ankle_deg);
      ++curves;
    }
  }
  if (metrics.cycles > 0) {
    const double count = static_cast<double>(metrics.cycles);
    metrics.cadence_difference = cadence / count;
    metrics.stride_difference = stride / count;
    metrics.stance_difference = stance / count;
    metrics.flight_difference = flight / count;
  }
  if (curves > 0) {
    metrics.curves_available = true;
    metrics.hip_rms = hip / static_cast<double>(curves);
    metrics.knee_rms = knee / static_cast<double>(curves);
    metrics.ankle_rms = ankle / static_cast<double>(curves);
  }
  return metrics;
}

CurveDifference compareModelsAtSpeed(const ProcLocomotionModel& primary, const ProcLocomotionModel& other, const std::string& regime,
                                      double dimensionless_speed) {
  CurveDifference difference;
  difference.regime = regime;
  difference.dimensionless_speed = dimensionless_speed;
  const RegimeModel* left = regimeOf(primary, regime);
  const RegimeModel* right = regimeOf(other, regime);
  if (left == nullptr || right == nullptr || left->knots.empty() || right->knots.empty()) return difference;
  difference.available = true;
  const std::vector<double> hip_a = channelCurve(*left, dimensionless_speed, static_cast<int>(LocoChannel::LeftHip));
  const std::vector<double> hip_b = channelCurve(*right, dimensionless_speed, static_cast<int>(LocoChannel::LeftHip));
  const std::vector<double> knee_a = channelCurve(*left, dimensionless_speed, static_cast<int>(LocoChannel::LeftKnee));
  const std::vector<double> knee_b = channelCurve(*right, dimensionless_speed, static_cast<int>(LocoChannel::LeftKnee));
  const std::vector<double> ankle_a = channelCurve(*left, dimensionless_speed, static_cast<int>(LocoChannel::LeftAnkle));
  const std::vector<double> ankle_b = channelCurve(*right, dimensionless_speed, static_cast<int>(LocoChannel::LeftAnkle));
  const std::vector<double> pelvis_a = pelvisCurve(*left, dimensionless_speed);
  const std::vector<double> pelvis_b = pelvisCurve(*right, dimensionless_speed);
  difference.hip_rms = rmsOf(hip_a, hip_b);
  difference.knee_rms = rmsOf(knee_a, knee_b);
  difference.ankle_rms = rmsOf(ankle_a, ankle_b);
  difference.pelvis_rms = rmsOf(pelvis_a, pelvis_b);
  difference.hip_range_difference = rangeOf(hip_a) - rangeOf(hip_b);
  difference.knee_range_difference = rangeOf(knee_a) - rangeOf(knee_b);
  difference.ankle_range_difference = rangeOf(ankle_a) - rangeOf(ankle_b);
  difference.hip_peak_phase_difference = phaseDelta(peakPhase(hip_a), peakPhase(hip_b));
  difference.knee_peak_phase_difference = phaseDelta(peakPhase(knee_a), peakPhase(knee_b));
  difference.ankle_peak_phase_difference = phaseDelta(peakPhase(ankle_a), peakPhase(ankle_b));
  difference.cadence_difference = scalarAtSpeed(*left, dimensionless_speed, &SpeedKnot::cadence_spm) -
                                   scalarAtSpeed(*right, dimensionless_speed, &SpeedKnot::cadence_spm);
  difference.stance_difference = scalarAtSpeed(*left, dimensionless_speed, &SpeedKnot::stance_fraction) -
                                  scalarAtSpeed(*right, dimensionless_speed, &SpeedKnot::stance_fraction);
  difference.flight_difference = scalarAtSpeed(*left, dimensionless_speed, &SpeedKnot::flight_fraction) -
                                  scalarAtSpeed(*right, dimensionless_speed, &SpeedKnot::flight_fraction);
  return difference;
}

Status writeCycleManifest(const ExperimentManifest& manifest, const std::filesystem::path& json_path, const std::filesystem::path& csv_path) {
  if (json_path.has_parent_path()) std::filesystem::create_directories(json_path.parent_path());
  nlohmann::json cycles = nlohmann::json::array();
  for (const ManifestCycle& cycle : manifest.cycles) cycles.push_back(cycleJson(cycle));
  nlohmann::json failed = nlohmann::json::array();
  for (const FailedClip& clip : manifest.failed_clips) failed.push_back({{"filename", clip.filename}, {"reason", clip.reason}});
  const nlohmann::json document = {{"format", "PROC_GAIT_MANIFEST"}, {"version", 1}, {"cycles", cycles}, {"failed_clips", failed}};
  std::ofstream json_file(json_path);
  if (!json_file) {
    Status status;
    status.error("could not write " + json_path.string());
    return status;
  }
  json_file << document.dump(2) << '\n';
  std::ofstream csv_file(csv_path);
  if (!csv_file) {
    Status status;
    status.error("could not write " + csv_path.string());
    return status;
  }
  csv_file << "source_clip,dataset,subject,theme,cycle_index,side,speed_mps,dimensionless_speed,cadence_spm,stride_length_m,"
              "stance_fraction,flight_fraction,accepted,rejection_reason,review,holdout,used_in_model\n";
  for (const ManifestCycle& cycle : manifest.cycles) {
    csv_file << cycle.source_clip << ',' << cycle.dataset << ',' << cycle.subject << ',' << cycle.theme << ',' << cycle.cycle_index << ','
             << cycle.side << ',' << cycle.speed_mps << ',' << cycle.dimensionless_speed << ',' << cycle.cadence_spm << ','
             << cycle.stride_length_m << ',' << cycle.stance_fraction << ',' << cycle.flight_fraction << ','
             << (cycle.accepted ? "yes" : "no") << ',' << cycle.rejection_reason << ',' << cycle.review << ','
             << (cycle.holdout ? "yes" : "no") << ',' << (cycle.used_in_model ? "yes" : "no") << '\n';
  }
  return {};
}

Status writeValidationReport(const GaitExperimentResult& result, const GaitExperimentConfig& config, const std::filesystem::path& directory) {
  std::filesystem::create_directories(directory);
  std::ofstream markdown(directory / "VALIDATION.md");
  if (!markdown) {
    Status status;
    status.error("could not write VALIDATION.md");
    return status;
  }
  markdown << "# Real gait validation\n\n";
  markdown << "This file is local experiment output. It is not a public claim that real gait has been validated.\n\n";
  if (!result.dataset_found) {
    markdown << "REAL DATA VALIDATION: NOT RUN\n\nreason: " << result.reason << "\n\n";
    markdown << "dataset path: " << result.dataset_path << "\n\n";
    markdown << "Phase 4.3 tier: 0\n";
  } else {
    markdown << "dataset: " << result.dataset_path << "\n\n";
    markdown << "clips indexed: " << result.clips_indexed << "\n";
    markdown << "clips processed: " << result.clips_processed << "\n";
    markdown << "clips failed: " << result.clips_failed << "\n";
    markdown << "subjects: " << result.subjects << "\n\n";
    markdown << "split: " << result.split.method << "\n";
    markdown << "build cycles: " << result.split.build.size() << "\n";
    markdown << "holdout cycles: " << result.split.holdout.size() << "\n\n";
    markdown << "Subject balancing caps accepted cycles at " << config.max_cycles_per_subject
             << " per subject in each regime and dimensionless-speed bin. Surplus cycles are rejected as `subject cap` "
                "before the median. This is a count cap, not a learned weight.\n\n";
    markdown << "A knot is TRUSTED when it has at least " << config.min_cycles_per_knot
             << " accepted real cycles and the builder did not mark it low-confidence. Fewer cycles stay in the model for inspection "
                "and are labeled LOW CONFIDENCE.\n\n";
    for (const KnotReport& knot : result.knots) {
      markdown << knot.regime << " knot speed " << knot.speed_mps << " m/s dim " << knot.dimensionless_speed << " cycles " << knot.cycle_count
               << " clips " << knot.clip_count << " subjects " << knot.subject_count << " cadence " << knot.cadence_spm << " stance "
               << knot.stance_fraction << " flight " << knot.flight_fraction << (knot.trusted ? " TRUSTED" : " LOW CONFIDENCE") << "\n";
    }
    markdown << "\nconstraint violations: " << result.audit_violations << "\nseam: " << result.seam_error << "\n";
    markdown << "\nPhase 4.3 tier: " << result.tier << "\n";
  }
  markdown << "\nFoot IK, foot locking, and root correction are not part of this experiment.\n";
  for (const std::string& warning : result.warnings) markdown << "\nwarning: " << warning << "\n";

  nlohmann::json document = {{"format", "PROC_GAIT_VALIDATION"},
                             {"version", 1},
                             {"dataset_found", result.dataset_found},
                             {"dataset_path", result.dataset_path},
                             {"reason", result.reason},
                             {"tier", result.tier},
                             {"clips_indexed", result.clips_indexed},
                             {"clips_processed", result.clips_processed},
                             {"clips_failed", result.clips_failed},
                             {"subjects", result.subjects},
                             {"split", result.split.method},
                             {"build_cycles", result.split.build.size()},
                             {"holdout_cycles", result.split.holdout.size()},
                             {"audit_violations", result.audit_violations},
                             {"seam_error", result.seam_error},
                             {"min_cycles_per_knot", config.min_cycles_per_knot},
                             {"max_cycles_per_subject", config.max_cycles_per_subject}};
  std::ofstream json_file(directory / "validation.json");
  if (!json_file) {
    Status status;
    status.error("could not write validation.json");
    return status;
  }
  json_file << document.dump(2) << '\n';
  return {};
}

GaitExperimentResult runGaitExperiment(const GaitExperimentConfig& config) {
  GaitExperimentResult result;
  result.dataset_path = config.dataset.string();
  if (outputIsPublicAsset(config.output)) {
    result.reason = "refusing to write a real-derived model into public assets";
    result.warnings.push_back(result.reason);
    result.tier = "0";
    return result;
  }
  if (config.dataset.empty() || !std::filesystem::exists(config.dataset)) {
    result.dataset_found = false;
    result.reason = "dataset unavailable";
    result.tier = "0";
    if (!config.output.empty()) writeValidationReport(result, config, config.output);
    return result;
  }
  result.dataset_found = true;
  BatchRequest request;
  request.input = config.dataset;
  request.output = config.output;
  request.profile_directory = config.profile_directory;
  request.body = config.body;
  request.profile = config.profile;
  request.theme = config.themes;
  request.dataset = config.dataset_name;
  if (std::filesystem::exists(config.output / "clips")) {
    for (const std::filesystem::directory_entry& entry : std::filesystem::directory_iterator(config.output / "clips")) {
      ProcAnalysis prior;
      if (!readAnalysisFromFile(entry.path(), prior).ok()) continue;
      const bool reviewed = std::any_of(prior.cycles.begin(), prior.cycles.end(), [](const GaitCycle& cycle) { return cycle.manual_override; });
      if (reviewed) request.preserve_stems.push_back(entry.path().stem().string());
    }
  }
  const BatchReport batch = analyzeBatch(request);
  result.clips_processed = batch.processed;
  result.clips_failed = batch.failed;
  result.warnings = batch.warnings;

  std::vector<ProcAnalysis> analyses;
  if (std::filesystem::exists(config.output / "clips")) {
    std::vector<std::filesystem::path> files;
    for (const std::filesystem::directory_entry& entry : std::filesystem::directory_iterator(config.output / "clips")) {
      if (entry.path().filename().string().find("procanalysis") != std::string::npos) files.push_back(entry.path());
    }
    std::sort(files.begin(), files.end());
    for (const std::filesystem::path& file : files) {
      ProcAnalysis analysis;
      if (readAnalysisFromFile(file, analysis).ok()) analyses.push_back(std::move(analysis));
    }
  }
  result.manifest = manifestFromAnalyses(analyses, kLocomotionGravity);
  for (const std::string& rejected : batch.rejected_clips) noteFailedClip(result.manifest, rejected, rejected);
  result.split = splitManifest(result.manifest);
  for (int index : result.split.holdout) result.manifest.cycles[static_cast<std::size_t>(index)].holdout = true;
  capCyclesPerSubject(analyses, result.manifest, config.max_cycles_per_subject, config.bin_width, kLocomotionGravity, result.split.build);

  std::vector<ProcAnalysis> build_analyses = analyses;
  if (result.split.method == "subject" || result.split.method == "clip") {
    std::set<std::string> held;
    for (int index : result.split.holdout) {
      const ManifestCycle& cycle = result.manifest.cycles[static_cast<std::size_t>(index)];
      held.insert(result.split.method == "subject" ? cycle.subject : cycle.source_clip);
    }
    build_analyses.clear();
    for (const ProcAnalysis& analysis : analyses) {
      const std::string key = result.split.method == "subject" ? analysis.source.subject : analysis.source.clip;
      if (held.count(key) == 0) build_analyses.push_back(analysis);
    }
  }
  ModelBuildSettings settings;
  settings.bin_width = config.bin_width;
  settings.min_cycles_per_knot = std::max(1, config.min_cycles_per_knot);
  settings.min_quality = config.min_quality;
  std::vector<std::string> build_warnings;
  ProcLocomotionModel model = buildLocomotionModel(build_analyses, settings, build_warnings);
  result.warnings.insert(result.warnings.end(), build_warnings.begin(), build_warnings.end());
  result.seam_error = model.max_seam_error;
  for (ManifestCycle& cycle : result.manifest.cycles) {
    if (!cycle.holdout && eligible(cycle)) cycle.used_in_model = true;
  }
  std::set<std::string> subjects;
  for (const ManifestCycle& cycle : result.manifest.cycles) {
    if (!cycle.subject.empty()) subjects.insert(cycle.subject);
  }
  result.subjects = static_cast<int>(subjects.size());
  result.clips_indexed = batch.indexed;

  const std::filesystem::path model_path = config.output / "models" / "real_locomotion.procloco.json";
  if (model_path.has_parent_path()) std::filesystem::create_directories(model_path.parent_path());
  const Status written = writeLocomotionModelToFile(model, model_path);
  if (!written.ok()) result.warnings.push_back(written.text());

  ProcBody body;
  if (!config.body.empty() && loadProcBodyFromFile(config.body, body).ok()) {
    const ConstraintAudit audit = auditLocomotionModel(body, model);
    result.audit_violations = audit.violations;
  }
  result.knots = classifyKnots(model, result.manifest, result.split, config.min_cycles_per_knot);
  result.holdout = measureHoldout(model, result.manifest, result.split);

  if (!config.synthetic_model.empty()) {
    ProcLocomotionModel synthetic;
    if (readLocomotionModelFromFile(config.synthetic_model, synthetic).ok()) {
      for (const char* regime_name : {"walk", "run", "sprint"}) {
        const RegimeModel* regime = regimeOf(model, regime_name);
        if (regime == nullptr || regime->knots.empty()) continue;
        for (const SpeedKnot& knot : regime->knots) {
          result.synthetic_comparison.push_back(compareModelsAtSpeed(model, synthetic, regime_name, knot.dimensionless_speed));
        }
      }
    }
  }

  if (body.joints.size() == static_cast<std::size_t>(kJointCount)) {
    for (const char* regime_name : {"walk", "run", "sprint"}) {
      const RegimeModel* regime = regimeOf(model, regime_name);
      ClosedLoopReport loop;
      loop.regime = regime_name;
      if (regime == nullptr || regime->knots.empty()) {
        loop.cycle_note = std::string(regime_name) + ": no real knots";
        result.closed_loop.push_back(loop);
        continue;
      }
      const SpeedKnot& knot = regime->knots[regime->knots.size() / 2];
      loop.attempted = true;
      loop.commanded_speed = knot.median_speed_mps > 0.0 ? knot.median_speed_mps : 1.0;
      loop.model_cadence = knot.cadence_spm;
      loop.model_stride = knot.cadence_spm > 1e-6 ? loop.commanded_speed * 120.0 / knot.cadence_spm : 0.0;
      loop.model_stance = knot.stance_fraction;
      loop.model_flight = knot.flight_fraction;
      LocomotionCommand command;
      command.desired_speed_mps = loop.commanded_speed;
      command.max_accel_mps2 = 8.0;
      command.force = regime_name == std::string("walk") ? RegimeForce::Walk : regime_name == std::string("run") ? RegimeForce::Run : RegimeForce::Sprint;
      const ProcMotion motion = bakeLocomotion(body, model, command, 6.0, 50.0);
      const std::filesystem::path motion_path = config.output / "generated" / (std::string(regime_name) + ".procmo");
      if (motion_path.has_parent_path()) std::filesystem::create_directories(motion_path.parent_path());
      writeProcMotionToFile(motion, motion_path);
      SourceClip clip;
      clip.name = "generated";
      clip.format = "procedural";
      clip.nominal_fps = motion.fps;
      clip.duration = motion.duration_s;
      clip.root = jointIndex(JointId::LegRoot);
      for (int index = 0; index < kJointCount; ++index) {
        SourceJoint joint;
        joint.name = kJointNames[index];
        joint.parent = body.joints[static_cast<std::size_t>(index)].parent;
        joint.rest_global_position = body.joints[static_cast<std::size_t>(index)].rest_position;
        clip.joints.push_back(joint);
      }
      for (const Pose& pose : motion.frames) {
        const WorldPose world = forwardKinematics(body, pose);
        SourceFrame frame;
        frame.time = pose.time;
        frame.global_position.resize(kJointCount);
        frame.global_rotation.resize(kJointCount, quatIdentity());
        for (int index = 0; index < kJointCount; ++index) frame.global_position[static_cast<std::size_t>(index)] = world.joints[index].position;
        clip.frames.push_back(frame);
      }
      AnalysisInput input;
      input.source = &clip;
      input.motion = &motion;
      std::array<int, kJointCount> identity{};
      for (int index = 0; index < kJointCount; ++index) identity[static_cast<std::size_t>(index)] = index;
      input.semantics = semanticsFromMapping(identity);
      const ProcAnalysis closed = analyzeClip(input);
      double speed_sum = 0.0;
      int speed_count = 0;
      for (std::size_t index = closed.signals.size() / 5; index + closed.signals.size() / 5 < closed.signals.size(); ++index) {
        speed_sum += closed.signals[index].root_horizontal_speed;
        ++speed_count;
      }
      loop.measured_speed = speed_count > 0 ? speed_sum / static_cast<double>(speed_count) : 0.0;
      for (const GaitCycle& cycle : closed.cycles) {
        if (cycle.mirrored) continue;
        ++loop.detected_cycles;
        if (!cycle.accepted) continue;
        ++loop.accepted_cycles;
        loop.measured_cadence += cycle.measurements.cadence_spm;
        loop.measured_stride += cycle.measurements.stride_length_m;
        loop.detected_stance += cycle.measurements.stance_fraction;
        loop.detected_flight += cycle.measurements.flight_fraction;
      }
      if (loop.accepted_cycles > 0) {
        const double count = static_cast<double>(loop.accepted_cycles);
        loop.measured_cadence /= count;
        loop.measured_stride /= count;
        loop.detected_stance /= count;
        loop.detected_flight /= count;
        loop.cycle_note = "cycles recovered";
      } else if (loop.detected_cycles == 0) {
        loop.cycle_note = "no initial contact events";
      } else {
        loop.cycle_note = "cycles detected but none accepted";
      }
      const FootSlipReport slip = measureContactSlip(body, motion);
      loop.slip_left_mean = slip.left_mean_mps;
      loop.slip_left_peak = slip.left_peak_mps;
      loop.slip_right_mean = slip.right_mean_mps;
      loop.slip_right_peak = slip.right_peak_mps;
      result.closed_loop.push_back(loop);
    }
  }

  result.tier = tierOf(result);
  writeCycleManifest(result.manifest, config.output / "cycle_manifest.json", config.output / "cycle_manifest.csv");
  writeValidationReport(result, config, config.output);
  return result;
}

}  // namespace proc
