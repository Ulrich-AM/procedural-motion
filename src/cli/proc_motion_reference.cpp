#include "analysis/reference_capture.h"

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
  std::cout << "proc_motion_reference --input Walk.fbx --profile mixamo --body neutral.procbody.json --regime walk --output local_models\\walk.procloco.json\n"
               "  [--speed 1.25] [--cycles 1,2,3] [--profiles config\\retarget]\n"
               "  Repeat --input for another clip. --regime, --speed, and --cycles apply to the preceding input.\n";
}

}  // namespace

int main(int argc, char** argv) {
  proc::ReferenceArgumentParse parsed = proc::parseReferenceArgs(argc, argv);
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
  if (parsed.request.body.empty()) parsed.request.body = root / "assets" / "bodies" / "neutral.procbody.json";
  if (parsed.request.profile_directory.empty()) parsed.request.profile_directory = root / "config" / "retarget";
  const proc::ReferenceCaptureReport report = proc::runReferenceCapture(parsed.request);
  if (!report.ok) {
    std::cerr << report.error << '\n';
    return report.error.find("in-place") != std::string::npos ? 2 : 1;
  }
  std::cout << "REFERENCE MODEL\n";
  for (const proc::RegimeCoverage& coverage : report.build.coverage) {
    std::cout << coverage.regime << " clips " << coverage.clip_count << " cycles " << coverage.cycle_count << " knots " << coverage.knot_count
              << "\n";
    std::cout << "speed " << coverage.speed_min_mps << " - " << coverage.speed_max_mps << " m/s\n";
    if (coverage.single_speed) std::cout << "SINGLE-SPEED REFERENCE\n";
  }
  if (report.build.fidelity.available) {
    std::cout << "hip rms " << report.build.fidelity.hip_rms << " knee rms " << report.build.fidelity.knee_rms << " ankle rms "
              << report.build.fidelity.ankle_rms << "\n";
  }
  std::cout << "seam " << report.seam_error << " audit violations " << report.audit_violations << "\n";
  std::cout << "slip mean " << report.slip_mean_mps << " peak " << report.slip_peak_mps << "\n";
  std::cout << "wrote " << parsed.request.output.string() << '\n';
  return 0;
}
