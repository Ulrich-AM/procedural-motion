#include "dataset/batch_analyze.h"

#include <filesystem>
#include <iostream>
#include <string>

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
  std::cout << "proc_motion_analyze --input <dataset dir> --output <analysis dir> [--profile lafan1] [--theme Walk] [--body <procbody.json>] [--profiles <dir>]\n";
}

}  // namespace

int main(int argc, char** argv) {
  proc::BatchRequest request;
  const std::filesystem::path root = findRoot();
  request.profile_directory = root / "config" / "retarget";
  request.body = root / "assets" / "bodies" / "neutral.procbody.json";
  request.profile = "lafan1";
  for (int index = 1; index < argc; ++index) {
    const std::string argument = argv[index];
    auto need = [&](std::filesystem::path& slot) {
      if (index + 1 >= argc) return false;
      slot = argv[++index];
      return true;
    };
    auto needText = [&](std::string& slot) {
      if (index + 1 >= argc) return false;
      slot = argv[++index];
      return true;
    };
    if (argument == "--input") {
      if (!need(request.input)) return 1;
    } else if (argument == "--output") {
      if (!need(request.output)) return 1;
    } else if (argument == "--profile") {
      if (!needText(request.profile)) return 1;
    } else if (argument == "--theme") {
      if (!needText(request.theme)) return 1;
    } else if (argument == "--body") {
      if (!need(request.body)) return 1;
    } else if (argument == "--profiles") {
      if (!need(request.profile_directory)) return 1;
    } else if (argument == "--dataset") {
      if (!needText(request.dataset)) return 1;
    } else if (argument == "--help" || argument == "-h") {
      usage();
      return 0;
    } else {
      std::cerr << "unknown argument " << argument << '\n';
      usage();
      return 1;
    }
  }
  if (request.input.empty() || request.output.empty()) {
    usage();
    return 1;
  }
  if (request.dataset.empty()) request.dataset = request.profile == "lafan1" ? "LAFAN1" : request.input.filename().string();
  const proc::BatchReport report = proc::analyzeBatch(request);
  return report.failed > 0 && report.analyzed == 0 ? 1 : 0;
}
