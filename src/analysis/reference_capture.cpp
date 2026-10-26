#include "analysis/reference_capture.h"

#include "analysis/gait_validation.h"
#include "import/bvh_importer.h"
#include "import/coordinate_normalizer.h"
#include "import/fbx_importer.h"
#include "locomotion/locomotion_diagnostics.h"
#include "locomotion/locomotion_generator.h"
#include "locomotion/locomotion_model_io.h"
#include "retarget/retarget_profile.h"
#include "retarget/retargeter.h"
#include "retarget/source_semantics.h"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <set>

namespace proc {
namespace {

std::string lowerCopy(std::string text) {
  for (char& character : text) {
    if (character >= 'A' && character <= 'Z') character = static_cast<char>(character - 'A' + 'a');
  }
  return text;
}

bool regimeNameOk(const std::string& regime) { return regime == "walk" || regime == "run" || regime == "sprint"; }

const RegimeModel* regimeSlot(const ProcLocomotionModel& model, const std::string& regime) {
  if (regime == "walk") return &model.walk;
  if (regime == "run") return &model.run;
  if (regime == "sprint") return &model.sprint;
  return nullptr;
}

std::vector<int> parseCycleList(const std::string& text) {
  std::vector<int> cycles;
  std::string token;
  for (char character : text + ",") {
    if (character == ',' || character == ' ') {
      if (!token.empty()) {
        cycles.push_back(std::stoi(token));
        token.clear();
      }
    } else {
      token.push_back(character);
    }
  }
  return cycles;
}

ReferenceClipRecord recordFromAnalysis(ProcAnalysis analysis, const std::string& path, const std::string& format, const std::string& profile,
                                       const std::string& regime, const std::optional<double>& speed, const std::vector<int>& cycles) {
  ReferenceClipRecord record;
  record.source_path = path;
  record.format = format;
  record.profile = profile;
  record.suggested_regime = suggestRegime(path, analysis.source.theme);
  record.regime = regime.empty() ? (record.suggested_regime.empty() ? "walk" : record.suggested_regime) : regime;
  record.measured_speed_mps = measuredRootSpeed(analysis);
  record.in_place = sourceAppearsInPlace(analysis);
  record.speed_override_mps = speed;
  record.source_leg_length_m = analysis.source_leg_length_m;
  record.review = std::any_of(analysis.cycles.begin(), analysis.cycles.end(), [](const GaitCycle& cycle) { return cycle.manual_override; })
                      ? "manually reviewed"
                      : "automatic";
  record.analysis = std::move(analysis);
  if (cycles.empty()) selectAcceptedCycles(record);
  else {
    for (int wanted : cycles) {
      for (int slot = 0; slot < static_cast<int>(record.analysis.cycles.size()); ++slot) {
        const GaitCycle& cycle = record.analysis.cycles[static_cast<std::size_t>(slot)];
        if (!cycle.mirrored && cycle.index == wanted) record.included_cycles.push_back(slot);
      }
    }
  }
  double quality = 0.0;
  int count = 0;
  for (int slot : record.included_cycles) {
    if (slot < 0 || slot >= static_cast<int>(record.analysis.cycles.size())) continue;
    quality += record.analysis.cycles[static_cast<std::size_t>(slot)].quality.overall;
    ++count;
  }
  record.quality = count > 0 ? quality / static_cast<double>(count) : 0.0;
  return record;
}

}  // namespace

std::string suggestRegime(const std::string& filename, const std::string& theme) {
  const std::string text = lowerCopy(filename + " " + theme);
  if (text.find("sprint") != std::string::npos) return "sprint";
  if (text.find("run") != std::string::npos) return "run";
  if (text.find("walk") != std::string::npos) return "walk";
  return {};
}

double measuredRootSpeed(const ProcAnalysis& analysis) {
  if (!analysis.signals.empty()) {
    double sum = 0.0;
    for (const SignalFrame& frame : analysis.signals) sum += std::abs(frame.root_horizontal_speed);
    return sum / static_cast<double>(analysis.signals.size());
  }
  double sum = 0.0;
  int count = 0;
  for (const GaitCycle& cycle : analysis.cycles) {
    if (cycle.mirrored) continue;
    sum += std::abs(cycle.measurements.speed_mps);
    ++count;
  }
  return count > 0 ? sum / static_cast<double>(count) : 0.0;
}

bool sourceAppearsInPlace(const ProcAnalysis& analysis) { return measuredRootSpeed(analysis) < kInPlaceSpeedMps; }

SpeedProvenance speedProvenance(const ReferenceClipRecord& clip) {
  return clip.speed_override_mps ? SpeedProvenance::Manual : SpeedProvenance::MeasuredRoot;
}

const char* speedProvenanceText(SpeedProvenance provenance) {
  return provenance == SpeedProvenance::Manual ? "manual override for in-place source" : "measured root";
}

double referencePhaseTime(const GaitCycle& cycle, double phase) {
  const double span = std::max(0.0, cycle.end_time - cycle.start_time);
  double wrapped = std::fmod(phase, 1.0);
  if (wrapped < 0.0) wrapped += 1.0;
  return cycle.start_time + wrapped * span;
}

bool poseCurveClamped(const ProcLocomotionModel& model, const std::string& regime, double dimensionless_speed) {
  const RegimeModel* slot = regimeSlot(model, regime);
  if (slot == nullptr || slot->knots.empty()) return false;
  if (slot->knots.size() == 1) return std::abs(dimensionless_speed - slot->knots.front().dimensionless_speed) > 1e-6;
  return dimensionless_speed < slot->knots.front().dimensionless_speed - 1e-9 ||
         dimensionless_speed > slot->knots.back().dimensionless_speed + 1e-9;
}

void selectAcceptedCycles(ReferenceClipRecord& clip) {
  clip.included_cycles.clear();
  for (int index = 0; index < static_cast<int>(clip.analysis.cycles.size()); ++index) {
    const GaitCycle& cycle = clip.analysis.cycles[static_cast<std::size_t>(index)];
    if (cycle.mirrored || !cycle.accepted || cycle.classification != "straight") continue;
    clip.included_cycles.push_back(index);
  }
}

bool addReferenceClip(ReferenceSet& set, ReferenceClipRecord clip) {
  if (!regimeNameOk(clip.regime)) return false;
  for (ReferenceClipRecord& existing : set.clips) {
    if (existing.source_path == clip.source_path && !clip.source_path.empty()) {
      existing = std::move(clip);
      return true;
    }
  }
  set.clips.push_back(std::move(clip));
  return true;
}

bool removeReferenceClip(ReferenceSet& set, int index) {
  if (index < 0 || index >= static_cast<int>(set.clips.size())) return false;
  set.clips.erase(set.clips.begin() + index);
  return true;
}

std::vector<ReferenceCycleInput> inputsFromReferenceSet(const ReferenceSet& set) {
  std::vector<ReferenceCycleInput> inputs;
  for (const ReferenceClipRecord& clip : set.clips) {
    for (int slot : clip.included_cycles) {
      if (slot < 0 || slot >= static_cast<int>(clip.analysis.cycles.size())) continue;
      const GaitCycle& cycle = clip.analysis.cycles[static_cast<std::size_t>(slot)];
      if (cycle.mirrored || !cycle.accepted) continue;
      ReferenceCycleInput input;
      input.analysis = &clip.analysis;
      input.cycle_index = slot;
      input.regime = clip.regime;
      input.speed_override_mps = clip.speed_override_mps;
      inputs.push_back(input);
    }
  }
  return inputs;
}

ModelBuildSettings referenceBuildSettings() {
  ModelBuildSettings settings;
  settings.min_cycles_per_knot = 1;
  settings.bin_width = 0.20;
  settings.min_quality = 0.0;
  return settings;
}

ReferenceBuildResult buildReferenceModel(const ReferenceSet& set, const ModelBuildSettings& settings) {
  ReferenceBuildResult result;
  for (const ReferenceClipRecord& clip : set.clips) {
    if (!clip.in_place || clip.speed_override_mps) continue;
    if (clip.included_cycles.empty()) continue;
    result.error = "source appears in-place; provide --speed <m/s>";
    result.warnings.push_back(clip.source_path + ": " + result.error);
    return result;
  }
  const std::vector<ReferenceCycleInput> inputs = inputsFromReferenceSet(set);
  if (inputs.empty()) {
    result.error = "no selected cycles";
    return result;
  }
  result.model = buildLocomotionModel(inputs, settings, result.warnings);
  const bool any = result.model.walk.present || result.model.run.present || result.model.sprint.present;
  if (!any) {
    result.error = "the selected cycles did not produce a locomotion knot";
    return result;
  }
  result.coverage = referenceCoverage(set, result.model);
  result.single_speed = std::all_of(result.coverage.begin(), result.coverage.end(), [](const RegimeCoverage& coverage) {
    return coverage.knot_count <= 1;
  });
  result.fidelity = measureReferenceFidelity(result.model, inputs, settings);
  result.ok = true;
  return result;
}

std::vector<RegimeCoverage> referenceCoverage(const ReferenceSet& set, const ProcLocomotionModel& model) {
  std::vector<RegimeCoverage> coverage;
  for (const char* name : {"walk", "run", "sprint"}) {
    const RegimeModel* slot = regimeSlot(model, name);
    if (slot == nullptr || !slot->present) continue;
    RegimeCoverage report;
    report.regime = name;
    report.knot_count = static_cast<int>(slot->knots.size());
    report.single_speed = report.knot_count == 1;
    if (!slot->knots.empty()) {
      report.dimensionless_min = slot->knots.front().dimensionless_speed;
      report.dimensionless_max = slot->knots.back().dimensionless_speed;
      report.speed_min_mps = slot->knots.front().median_speed_mps;
      report.speed_max_mps = slot->knots.back().median_speed_mps;
    }
    std::set<std::string> clips;
    for (const ReferenceClipRecord& clip : set.clips) {
      if (clip.regime != name) continue;
      int used = 0;
      for (int slot_index : clip.included_cycles) {
        if (slot_index < 0 || slot_index >= static_cast<int>(clip.analysis.cycles.size())) continue;
        if (!clip.analysis.cycles[static_cast<std::size_t>(slot_index)].accepted) continue;
        ++used;
      }
      if (used == 0) continue;
      clips.insert(clip.source_path);
      report.cycle_count += used;
    }
    report.clip_count = static_cast<int>(clips.size());
    coverage.push_back(report);
  }
  return coverage;
}

Status writeReferenceSet(const ReferenceSet& set, const std::filesystem::path& path) {
  nlohmann::json clips = nlohmann::json::array();
  for (const ReferenceClipRecord& clip : set.clips) {
    nlohmann::json item = {{"source_path", clip.source_path},
                           {"format", clip.format},
                           {"profile", clip.profile},
                           {"regime", clip.regime},
                           {"suggested_regime", clip.suggested_regime},
                           {"review", clip.review},
                           {"measured_speed_mps", clip.measured_speed_mps},
                           {"in_place", clip.in_place},
                           {"speed_provenance", speedProvenance(clip) == SpeedProvenance::Manual ? "manual" : "measured"},
                           {"included_cycles", clip.included_cycles},
                           {"source_leg_length_m", clip.source_leg_length_m}};
    if (clip.speed_override_mps) item["speed_override_mps"] = *clip.speed_override_mps;
    clips.push_back(std::move(item));
  }
  const nlohmann::json document = {{"format", "PROC_REFERENCE_SET"}, {"version", 1}, {"clips", clips}};
  if (path.has_parent_path()) std::filesystem::create_directories(path.parent_path());
  std::ofstream file(path);
  if (!file) {
    Status status;
    status.error("could not write " + path.string());
    return status;
  }
  file << document.dump(2) << '\n';
  return {};
}

Status readReferenceSet(const std::filesystem::path& path, ReferenceSet& out) {
  std::ifstream file(path);
  if (!file) {
    Status status;
    status.error("could not read " + path.string());
    return status;
  }
  nlohmann::json document;
  file >> document;
  if (document.value("format", "") != "PROC_REFERENCE_SET") {
    Status status;
    status.error("not a PROC_REFERENCE_SET");
    return status;
  }
  ReferenceSet set;
  for (const nlohmann::json& item : document.value("clips", nlohmann::json::array())) {
    ReferenceClipRecord clip;
    clip.source_path = item.value("source_path", "");
    clip.format = item.value("format", "");
    clip.profile = item.value("profile", "");
    clip.regime = item.value("regime", "walk");
    clip.suggested_regime = item.value("suggested_regime", "");
    clip.review = item.value("review", "automatic");
    clip.measured_speed_mps = item.value("measured_speed_mps", 0.0);
    clip.in_place = item.value("in_place", false);
    clip.source_leg_length_m = item.value("source_leg_length_m", 0.0);
    if (item.contains("speed_override_mps") && !item["speed_override_mps"].is_null()) {
      clip.speed_override_mps = item["speed_override_mps"].get<double>();
    }
    for (const nlohmann::json& cycle : item.value("included_cycles", nlohmann::json::array())) clip.included_cycles.push_back(cycle.get<int>());
    set.clips.push_back(std::move(clip));
  }
  out = std::move(set);
  return {};
}

Status exportReferenceModel(const ReferenceSet& set, const ReferenceBuildResult& build, int audit_violations,
                            const std::filesystem::path& model_path) {
  if (outputIsPublicAsset(model_path)) {
    Status status;
    status.error("refusing to write a reference model into public assets");
    return status;
  }
  if (model_path.has_parent_path()) std::filesystem::create_directories(model_path.parent_path());
  const Status written = writeLocomotionModelToFile(build.model, model_path);
  if (!written.ok()) return written;
  nlohmann::json clips = nlohmann::json::array();
  for (const ReferenceClipRecord& clip : set.clips) {
    nlohmann::json item = {{"source", clip.source_path},
                           {"format", clip.format},
                           {"profile", clip.profile},
                           {"regime", clip.regime},
                           {"included_cycles", clip.included_cycles},
                           {"speed_provenance", speedProvenance(clip) == SpeedProvenance::Manual ? "manual" : "measured"},
                           {"measured_speed_mps", clip.measured_speed_mps},
                           {"in_place", clip.in_place}};
    if (clip.speed_override_mps) item["speed_override_mps"] = *clip.speed_override_mps;
    clips.push_back(std::move(item));
  }
  std::filesystem::path sidecar = model_path;
  std::string filename = sidecar.filename().string();
  const std::string suffix = ".procloco.json";
  if (filename.size() > suffix.size() && filename.substr(filename.size() - suffix.size()) == suffix) {
    filename.resize(filename.size() - suffix.size());
  } else {
    filename = sidecar.stem().string();
  }
  sidecar.replace_filename(filename + ".reference.json");
  const nlohmann::json document = {{"format", "PROC_REFERENCE_SIDECAR"},
                                   {"version", 1},
                                   {"note", "Authoring provenance. This is not a Phase 4.3 validation result."},
                                   {"clips", clips},
                                   {"single_speed", build.single_speed},
                                   {"seam_error", build.model.max_seam_error},
                                   {"audit_violations", audit_violations},
                                   {"min_cycles_per_knot", 1}};
  std::ofstream file(sidecar);
  if (!file) {
    Status status;
    status.error("could not write " + sidecar.string());
    return status;
  }
  file << document.dump(2) << '\n';
  return {};
}

ReferenceArgumentParse parseReferenceArgs(int argc, char** argv) {
  ReferenceArgumentParse parsed;
  parsed.ok = true;
  for (int index = 1; index < argc; ++index) {
    const std::string arg = argv[index];
    auto need = [&](const std::string& name) -> std::string {
      if (index + 1 >= argc) {
        parsed.ok = false;
        parsed.error = "missing value for " + name;
        return {};
      }
      return argv[++index];
    };
    if (arg == "--help" || arg == "-h") {
      parsed.help = true;
      return parsed;
    }
    if (arg == "--input") {
      ReferenceCaptureJob job;
      job.input = need(arg);
      parsed.request.jobs.push_back(std::move(job));
    } else if (arg == "--profile") {
      parsed.request.profile = need(arg);
    } else if (arg == "--body") {
      parsed.request.body = need(arg);
    } else if (arg == "--profiles") {
      parsed.request.profile_directory = need(arg);
    } else if (arg == "--output") {
      parsed.request.output = need(arg);
    } else if (arg == "--regime") {
      if (parsed.request.jobs.empty()) {
        parsed.ok = false;
        parsed.error = "--regime follows --input";
      } else {
        parsed.request.jobs.back().regime = lowerCopy(need(arg));
      }
    } else if (arg == "--speed") {
      if (parsed.request.jobs.empty()) {
        parsed.ok = false;
        parsed.error = "--speed follows --input";
      } else {
        parsed.request.jobs.back().speed_override_mps = std::stod(need(arg));
      }
    } else if (arg == "--cycles") {
      if (parsed.request.jobs.empty()) {
        parsed.ok = false;
        parsed.error = "--cycles follows --input";
      } else {
        parsed.request.jobs.back().cycles = parseCycleList(need(arg));
      }
    } else {
      parsed.ok = false;
      parsed.error = "unknown argument " + arg;
    }
    if (!parsed.ok) return parsed;
  }
  if (!parsed.help && parsed.request.jobs.empty()) {
    parsed.ok = false;
    parsed.error = "missing --input";
  }
  for (ReferenceCaptureJob& job : parsed.request.jobs) {
    if (job.profile.empty()) job.profile = parsed.request.profile;
  }
  return parsed;
}

ReferenceCaptureReport runReferenceCapture(const ReferenceCaptureRequest& request) {
  ReferenceCaptureReport report;
  if (request.output.empty()) {
    report.error = "missing --output";
    return report;
  }
  if (outputIsPublicAsset(request.output)) {
    report.error = "refusing to write a reference model into public assets";
    return report;
  }
  ProcBody body;
  Status status = loadProcBodyFromFile(request.body, body);
  if (!status.ok()) {
    report.error = status.text();
    return report;
  }
  std::vector<RetargetProfile> profiles;
  if (!request.profile_directory.empty()) loadRetargetProfiles(request.profile_directory, profiles);
  for (const ReferenceCaptureJob& job : request.jobs) {
    const std::string extension = lowerCopy(job.input.extension().string());
    SourceClip clip;
    if (extension == ".bvh") status = loadBvhFromFile(job.input, identityConvention(), clip);
    else if (extension == ".fbx") status = loadFbxFromFile(job.input, identityConvention(), clip);
    else status.error("source files must be .bvh or .fbx");
    if (!status.ok()) {
      report.error = job.input.filename().string() + ": " + status.text();
      return report;
    }
    std::vector<std::string> names;
    for (const SourceJoint& joint : clip.joints) names.push_back(joint.name);
    const RetargetProfile* profile = nullptr;
    std::array<int, kJointCount> mapping{};
    mapping.fill(-1);
    const std::string wanted = job.profile.empty() ? request.profile : job.profile;
    for (const RetargetProfile& candidate : profiles) {
      if (lowerCopy(candidate.name) == lowerCopy(wanted)) profile = &candidate;
    }
    if (profile == nullptr) {
      const ProfileMatch match = matchProfile(profiles, names);
      if (match.profile_index >= 0) profile = &profiles[static_cast<std::size_t>(match.profile_index)];
    }
    if (profile != nullptr && profile->has_coordinates) applyAxisConvention(clip, profile->coordinates);
    else applyAxisConvention(clip, detectHumanConvention(clip));
    if (profile != nullptr) mapping = mappingFromProfile(*profile, names);
    std::array<bool, kJointCount> required{};
    required.fill(false);
    if (profile != nullptr) required = profile->required;
    ProcMotion motion;
    RetargetReport retarget_report;
    status = retargetClip(body, clip, mapping, required, {}, motion, retarget_report);
    if (!status.ok()) {
      report.error = job.input.filename().string() + ": " + status.text();
      return report;
    }
    AnalysisInput input;
    input.source = &clip;
    input.motion = &motion;
    input.semantics = semanticsFromMapping(mapping);
    input.motion_path = job.input.string();
    ProcAnalysis analysis = analyzeClip(input);
    ReferenceClipRecord record = recordFromAnalysis(std::move(analysis), job.input.string(), extension == ".fbx" ? "fbx" : "bvh",
                                                    profile != nullptr ? profile->name : "", job.regime, job.speed_override_mps, job.cycles);
    if (!addReferenceClip(report.set, std::move(record))) {
      report.error = "regime must be walk, run, or sprint";
      return report;
    }
  }
  report.build = buildReferenceModel(report.set);
  if (!report.build.ok) {
    report.error = report.build.error;
    return report;
  }
  const ConstraintAudit audit = auditLocomotionModel(body, report.build.model);
  report.audit_violations = audit.violations;
  report.seam_error = report.build.model.max_seam_error;
  status = exportReferenceModel(report.set, report.build, report.audit_violations, request.output);
  if (!status.ok()) {
    report.error = status.text();
    return report;
  }
  const RegimeModel* slot = report.build.model.walk.present ? &report.build.model.walk
                           : report.build.model.run.present ? &report.build.model.run
                                                            : &report.build.model.sprint;
  if (slot != nullptr && !slot->knots.empty() && body.joints.size() == static_cast<std::size_t>(kJointCount)) {
    LocomotionCommand command;
    command.desired_speed_mps = slot->knots.front().median_speed_mps;
    command.max_accel_mps2 = 8.0;
    command.force = report.build.model.walk.present ? RegimeForce::Walk : report.build.model.run.present ? RegimeForce::Run : RegimeForce::Sprint;
    const ProcMotion generated = bakeLocomotion(body, report.build.model, command, 2.0, 50.0);
    const FootSlipReport slip = measureContactSlip(body, generated);
    report.slip_mean_mps = 0.5 * (slip.left_mean_mps + slip.right_mean_mps);
    report.slip_peak_mps = std::max(slip.left_peak_mps, slip.right_peak_mps);
  }
  report.ok = true;
  return report;
}

}  // namespace proc
