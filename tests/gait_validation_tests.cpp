#include "test_support.h"

#include "analysis/gait_validation.h"

#include <fstream>
#include <sstream>

namespace {

proc::ProcAnalysis analysisWith(const std::string& subject, const std::string& clip, const std::string& regime, double speed, bool accepted) {
  proc::ProcAnalysis analysis;
  analysis.source.subject = subject;
  analysis.source.clip = clip;
  analysis.source.theme = regime;
  analysis.source.dataset = "fixture";
  analysis.source_leg_length_m = 0.8;
  proc::GaitCycle cycle;
  cycle.index = 0;
  cycle.side = "left";
  cycle.accepted = accepted;
  cycle.classification = "straight";
  cycle.observed_regime = regime;
  cycle.start_time = 0.0;
  cycle.end_time = 1.0;
  cycle.measurements.speed_mps = speed;
  cycle.measurements.cadence_spm = 100.0;
  cycle.measurements.stride_length_m = 1.2;
  cycle.measurements.stance_fraction = 0.6;
  cycle.measurements.flight_fraction = 0.0;
  cycle.quality.overall = 0.8;
  cycle.curves.hip_flex_deg = {0.0, 1.0, 0.0, -1.0};
  cycle.curves.knee_flex_deg = {0.0, -2.0, 0.0, -1.0};
  cycle.curves.ankle_flex_deg = {0.0, 1.0, 0.0, 0.0};
  analysis.cycles.push_back(cycle);
  proc::GaitCycle mirrored = cycle;
  mirrored.mirrored = true;
  analysis.cycles.push_back(mirrored);
  return analysis;
}

proc::ProcLocomotionModel modelWithKnee(double speed, int sample, double value, int cycles) {
  proc::ProcLocomotionModel model;
  proc::SpeedKnot knot;
  knot.dimensionless_speed = speed;
  knot.median_speed_mps = speed;
  knot.cycle_count = cycles;
  knot.low_confidence = cycles < 3;
  knot.cadence_spm = 120.0;
  knot.stance_fraction = 0.5;
  knot.flight_fraction = 0.1;
  knot.rotation[static_cast<int>(proc::LocoChannel::LeftKnee)][static_cast<std::size_t>(sample)] = glm::dvec3(value, 0.0, 0.0);
  model.walk.present = true;
  model.walk.knots.push_back(knot);
  model.walk.support_min = speed - 0.1;
  model.walk.support_max = speed + 0.1;
  return model;
}

std::string readFile(const std::filesystem::path& path) {
  std::ifstream stream(path);
  std::stringstream buffer;
  buffer << stream.rdbuf();
  return buffer.str();
}

}  // namespace

int runGaitValidationTests() {
  TestStats stats;
  const std::filesystem::path root = std::filesystem::path(PROCMOTION_SOURCE_DIR);
  char program[] = "proc_motion_validate_gait";
  char dataset_arg[] = "--dataset";
  char dataset_value[] = "datasets/lafan1";
  char themes_arg[] = "--themes";
  char themes_value[] = "walk,run,sprint";
  char min_arg[] = "--min-cycles-per-knot";
  char min_value[] = "3";
  char* argv[] = {program, dataset_arg, dataset_value, themes_arg, themes_value, min_arg, min_value};
  const proc::GaitArgumentParse parsed = proc::parseGaitExperimentArgs(7, argv);
  stats.expect(parsed.ok && parsed.config.dataset == "datasets/lafan1" && parsed.config.themes == "walk,run,sprint" &&
                   parsed.config.min_cycles_per_knot == 3,
               "real-gait experiment configuration parsing");
  stats.expect(proc::themeAllowed("walk", "walk1_subject2.bvh", "walk,run,sprint"), "candidate theme filtering keeps walk");
  stats.expect(!proc::themeAllowed("dance", "dance1_subject2.bvh", "walk,run,sprint"), "candidate theme filtering drops dance");

  std::vector<proc::ProcAnalysis> analyses = {analysisWith("a", "walk1_subject1", "walk", 1.2, true),
                                              analysisWith("b", "walk1_subject2", "walk", 1.3, true)};
  proc::ExperimentManifest manifest = proc::manifestFromAnalyses(analyses, proc::kLocomotionGravity);
  proc::noteFailedClip(manifest, "broken.bvh", "import failed");
  stats.expect(manifest.cycles.size() == 2 && manifest.failed_clips.size() == 1, "a failed clip does not drop the other cycles");

  const std::filesystem::path temp = std::filesystem::temp_directory_path() / "proc_gait_validation";
  std::filesystem::create_directories(temp);
  stats.expect(proc::writeCycleManifest(manifest, temp / "cycle_manifest.json", temp / "cycle_manifest.csv").ok(), "manifest generation");
  const std::string manifest_text = readFile(temp / "cycle_manifest.json");
  stats.expect(manifest_text.find("walk1_subject1") != std::string::npos && manifest_text.find("broken.bvh") != std::string::npos,
               "manifest records cycles and the failed clip");

  proc::ProcLocomotionModel trusted = modelWithKnee(0.4, 10, -0.2, 4);
  proc::ProcLocomotionModel weak = modelWithKnee(0.4, 10, -0.2, 1);
  proc::ValidationSplit split = proc::splitManifest(manifest);
  const proc::ValidationSplit again = proc::splitManifest(manifest);
  stats.expect(split.method == "subject" && split.holdout.size() == 1 && split.build.size() == 1 && split.holdout == again.holdout &&
                   split.build == again.build,
               "build/holdout split determinism");
  stats.expect(manifest.cycles[static_cast<std::size_t>(split.holdout.front())].subject == "b", "the last subject is held out");
  for (int index : split.build) manifest.cycles[static_cast<std::size_t>(index)].used_in_model = true;
  const std::vector<proc::KnotReport> trusted_knots = proc::classifyKnots(trusted, manifest, split, 3);
  const std::vector<proc::KnotReport> weak_knots = proc::classifyKnots(weak, manifest, split, 3);
  stats.expect(!trusted_knots.empty() && trusted_knots.front().trusted, "trusted knot classification");
  stats.expect(!weak_knots.empty() && weak_knots.front().low_confidence && !weak_knots.front().trusted, "low-confidence knot classification");
  stats.expect(trusted_knots.front().clip_count == 1 && trusted_knots.front().subject_count == 1, "source/clip/subject cycle counts");

  const proc::HoldoutMetrics holdout = proc::measureHoldout(trusted, manifest, split);
  stats.expect(holdout.cycles == 1 && std::abs(holdout.cadence_difference - 20.0) < 1e-6, "held-out cadence difference");
  stats.expect(holdout.curves_available && holdout.knee_rms > 0.0, "held-out curve RMS");

  proc::ProcLocomotionModel other = modelWithKnee(0.9, 40, -0.5, 4);
  other.walk.knots.front().dimensionless_speed = 0.4;
  const proc::CurveDifference diff = proc::compareModelsAtSpeed(trusted, other, "walk", 0.4);
  stats.expect(diff.available && diff.knee_rms > 0.0, "curve RMS calculation");
  stats.expect(std::abs(diff.knee_peak_phase_difference - 0.30) < 1e-6, "peak-phase comparison");
  stats.expect(std::abs(diff.dimensionless_speed - 0.4) < 1e-9, "model comparison at matched dimensionless speed");

  proc::GaitExperimentConfig missing;
  missing.dataset = temp / "missing_lafan1";
  missing.output = temp / "not_run";
  const proc::GaitExperimentResult not_run = proc::runGaitExperiment(missing);
  stats.expect(!not_run.dataset_found && not_run.tier == "0", "missing dataset does not invent a real model");
  const std::string report = readFile(temp / "not_run" / "VALIDATION.md");
  stats.expect(report.find("REAL DATA VALIDATION: NOT RUN") != std::string::npos && report.find("dataset unavailable") != std::string::npos,
               "validation report generation");

  const std::string ignore = readFile(root / ".gitignore");
  stats.expect(ignore.find("analysis_output/") != std::string::npos && ignore.find("local_models/") != std::string::npos &&
                   ignore.find("datasets/") != std::string::npos,
               "local output path remains ignored");
  stats.expect(proc::outputIsPublicAsset(root / "assets" / "models" / "real_locomotion.procloco.json"), "public asset paths are recognized");
  const std::filesystem::path synthetic = root / "assets" / "models" / "synthetic_locomotion.procloco.json";
  const std::string before = readFile(synthetic);
  proc::GaitExperimentConfig blocked;
  blocked.dataset = temp / "missing_lafan1";
  blocked.output = root / "assets" / "models";
  const proc::GaitExperimentResult refused = proc::runGaitExperiment(blocked);
  stats.expect(!refused.dataset_found && refused.reason.find("public assets") != std::string::npos, "real model is not written into public assets");
  stats.expect(readFile(synthetic) == before, "existing synthetic model remains unchanged");

  const std::filesystem::path generator = root / "src" / "locomotion" / "locomotion_generator.cpp";
  const std::string generator_text = readFile(generator);
  stats.expect(generator_text.find("lafan") == std::string::npos && generator_text.find("SourceClip") == std::string::npos &&
                   generator_text.find("gait_validation") == std::string::npos,
               "proc_runtime generator stays free of the real-gait experiment");
  std::cout << "gait validation tests ok\n";
  return stats.failed;
}
