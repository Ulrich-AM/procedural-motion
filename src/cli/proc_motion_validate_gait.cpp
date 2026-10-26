#include "analysis/gait_validation.h"

#include <filesystem>
#include <iostream>

namespace {

std::filesystem::path findRoot() {
  std::filesystem::path seed = std::filesystem::current_path();
  for (int step = 0; step < 6 && !seed.empty(); ++step) {
    if (std::filesystem::exists(seed / "assets" / "bodies" / "neutral.procbody.json")) return seed;
    seed = seed.parent_path();
  }
  return std::filesystem::current_path();
}

void usage() {
  std::cout << "proc_motion_validate_gait --dataset <dir> --profile lafan1 --body <procbody.json> --output <dir>\n"
               "  [--themes walk,run,sprint] [--min-quality 0] [--min-cycles-per-knot 3]\n"
               "  [--max-cycles-per-subject 8] [--bin-width 0.2] [--synthetic <model.procloco.json>]\n";
}

}  // namespace

int main(int argc, char** argv) {
  proc::GaitArgumentParse parsed = proc::parseGaitExperimentArgs(argc, argv);
  if (parsed.help) {
    usage();
    return 0;
  }
  if (!parsed.ok) {
    std::cerr << parsed.error << '\n';
    usage();
    return 1;
  }
  const std::filesystem::path root = findRoot();
  if (parsed.config.body.empty()) parsed.config.body = root / "assets" / "bodies" / "neutral.procbody.json";
  if (parsed.config.profile_directory.empty()) parsed.config.profile_directory = root / "config" / "retarget";
  if (parsed.config.synthetic_model.empty()) {
    const std::filesystem::path synthetic = root / "assets" / "models" / "synthetic_locomotion.procloco.json";
    if (std::filesystem::exists(synthetic)) parsed.config.synthetic_model = synthetic;
  }
  const proc::GaitExperimentResult result = proc::runGaitExperiment(parsed.config);
  if (!result.dataset_found) {
    std::cout << "REAL DATA VALIDATION: NOT RUN\nreason: " << result.reason << '\n';
    if (!result.dataset_path.empty()) std::cout << "dataset: " << result.dataset_path << '\n';
    return result.reason.find("public assets") != std::string::npos ? 1 : 2;
  }
  std::cout << "clips processed " << result.clips_processed << "\nclips failed " << result.clips_failed << "\nsubjects " << result.subjects
            << "\ntier " << result.tier << "\n";
  for (const proc::KnotReport& knot : result.knots) {
    std::cout << knot.regime << " cycles " << knot.cycle_count << " dim " << knot.dimensionless_speed
              << (knot.trusted ? " TRUSTED" : " LOW CONFIDENCE") << '\n';
  }
  for (const std::string& warning : result.warnings) std::cout << "warning: " << warning << '\n';
  return 0;
}
