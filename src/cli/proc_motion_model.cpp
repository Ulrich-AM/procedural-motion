#include "analysis/analysis_io.h"
#include "analysis/locomotion_model_builder.h"
#include "body/proc_body.h"
#include "locomotion/locomotion_audit.h"
#include "locomotion/locomotion_model_io.h"

#include <algorithm>
#include <filesystem>
#include <iostream>
#include <string>

namespace {

void printHelp() {
  std::cout << "proc_motion_model --input <analysis dir or file> --output <model.procloco.json> "
               "[--min-quality 0] [--regime walk] [--subject name] [--bin-width 0.2] [--body neutral.procbody.json]\n";
}

std::string argument(int argc, char** argv, const std::string& name) {
  for (int index = 1; index + 1 < argc; ++index) {
    if (argv[index] == name) return argv[index + 1];
  }
  return {};
}

bool flag(int argc, char** argv, const std::string& name) {
  for (int index = 1; index < argc; ++index) {
    if (argv[index] == name) return true;
  }
  return false;
}

}  // namespace

int main(int argc, char** argv) {
  if (flag(argc, argv, "--help") || argc < 2) {
    printHelp();
    return argc < 2 ? 1 : 0;
  }
  const std::filesystem::path input = argument(argc, argv, "--input");
  const std::filesystem::path output = argument(argc, argv, "--output");
  if (input.empty() || output.empty()) {
    printHelp();
    return 1;
  }
  proc::ModelBuildSettings settings;
  if (!argument(argc, argv, "--min-quality").empty()) settings.min_quality = std::stod(argument(argc, argv, "--min-quality"));
  if (!argument(argc, argv, "--bin-width").empty()) settings.bin_width = std::stod(argument(argc, argv, "--bin-width"));
  settings.regime = argument(argc, argv, "--regime");
  settings.subject = argument(argc, argv, "--subject");
  const std::filesystem::path body_path = argument(argc, argv, "--body");

  std::vector<std::filesystem::path> files;
  if (!std::filesystem::exists(input)) {
    std::cerr << "input not found: " << input.string() << '\n';
    return 1;
  }
  if (std::filesystem::is_directory(input)) {
    for (const std::filesystem::directory_entry& entry : std::filesystem::recursive_directory_iterator(input)) {
      if (entry.path().extension() == ".json" && entry.path().filename().string().find("procanalysis") != std::string::npos) {
        files.push_back(entry.path());
      }
    }
  } else {
    files.push_back(input);
  }
  std::sort(files.begin(), files.end());
  std::vector<proc::ProcAnalysis> analyses;
  int failed = 0;
  for (const std::filesystem::path& file : files) {
    proc::ProcAnalysis analysis;
    const proc::Status status = proc::readAnalysisFromFile(file, analysis);
    if (!status.ok()) {
      ++failed;
      std::cerr << file.filename().string() << ": " << status.text() << '\n';
      continue;
    }
    analyses.push_back(std::move(analysis));
  }
  std::vector<std::string> warnings;
  const proc::ProcLocomotionModel model = proc::buildLocomotionModel(analyses, settings, warnings);
  if (output.has_parent_path()) std::filesystem::create_directories(output.parent_path());
  const proc::Status written = proc::writeLocomotionModelToFile(model, output);
  if (!written.ok()) {
    std::cerr << written.text() << '\n';
    return 1;
  }
  auto count = [](const proc::RegimeModel& regime) {
    int cycles = 0;
    for (const proc::SpeedKnot& knot : regime.knots) cycles += knot.cycle_count;
    return cycles;
  };
  std::cout << "files " << files.size() << "\nfailed " << failed << "\nwalk knots " << model.walk.knots.size() << " cycles "
            << count(model.walk) << "\nrun knots " << model.run.knots.size() << " cycles " << count(model.run) << "\nsprint knots "
            << model.sprint.knots.size() << " cycles " << count(model.sprint) << "\nseam " << model.max_seam_error << " "
            << model.max_seam_channel << "\n";
  for (const std::string& warning : warnings) std::cout << "warning: " << warning << '\n';
  if (!body_path.empty()) {
    proc::ProcBody body;
    const proc::Status body_status = proc::loadProcBodyFromFile(body_path, body);
    if (!body_status.ok()) {
      std::cerr << body_status.text() << '\n';
      return 1;
    }
    const proc::ConstraintAudit audit = proc::auditLocomotionModel(body, model);
    std::cout << proc::formatConstraintAudit(audit, body.name.empty() ? body_path.filename().string() : body.name);
    std::cout << "audit does not modify the locomotion model\n";
  }
  return 0;
}
