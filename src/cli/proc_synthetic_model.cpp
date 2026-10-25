#include "analysis/synthetic_locomotion.h"
#include "body/proc_body.h"
#include "locomotion/locomotion_audit.h"
#include "locomotion/locomotion_model_io.h"

#include <filesystem>
#include <iostream>
#include <string>

namespace {

std::string argument(int argc, char** argv, const std::string& name) {
  for (int index = 1; index + 1 < argc; ++index) {
    if (argv[index] == name) return argv[index + 1];
  }
  return {};
}

}  // namespace

int main(int argc, char** argv) {
  const std::filesystem::path body_path = argument(argc, argv, "--body");
  const std::filesystem::path output = argument(argc, argv, "--output");
  if (body_path.empty() || output.empty()) {
    std::cout << "proc_synthetic_model --body assets\\bodies\\neutral.procbody.json "
                 "--output assets\\models\\synthetic_locomotion.procloco.json\n";
    return argc < 2 ? 1 : 0;
  }
  proc::ProcBody body;
  const proc::Status loaded = proc::loadProcBodyFromFile(body_path, body);
  if (!loaded.ok()) {
    std::cerr << loaded.text() << '\n';
    return 1;
  }
  std::vector<std::string> warnings;
  const proc::ProcLocomotionModel model = proc::buildSyntheticLocomotionModel(body, warnings);
  if (output.has_parent_path()) std::filesystem::create_directories(output.parent_path());
  const proc::Status written = proc::writeLocomotionModelToFile(model, output);
  if (!written.ok()) {
    std::cerr << written.text() << '\n';
    return 1;
  }
  const proc::ConstraintAudit audit = proc::auditLocomotionModel(body, model);
  std::cout << proc::formatConstraintAudit(audit, body.name.empty() ? body_path.filename().string() : body.name);
  std::cout << "seam " << model.max_seam_error << " " << model.max_seam_channel << "\n";
  std::cout << "wrote " << output.string() << "\n";
  for (const std::string& warning : warnings) std::cout << "warning: " << warning << '\n';
  return audit.ok ? 0 : 1;
}
