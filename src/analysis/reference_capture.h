#pragma once

#include "analysis/locomotion_model_builder.h"
#include "body/proc_body.h"
#include "locomotion/locomotion_audit.h"

#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace proc {

inline constexpr double kInPlaceSpeedMps = 0.08;

enum class SpeedProvenance { MeasuredRoot, Manual };

struct ReferenceClipRecord {
  std::string source_path;
  std::string format;
  std::string profile;
  std::string regime = "walk";
  std::string suggested_regime;
  std::string review = "automatic";
  double measured_speed_mps = 0.0;
  bool in_place = false;
  std::optional<double> speed_override_mps;
  double source_leg_length_m = 0.0;
  double quality = 0.0;
  ProcAnalysis analysis;
  std::vector<int> included_cycles;
};

struct ReferenceSet {
  std::vector<ReferenceClipRecord> clips;
};

struct RegimeCoverage {
  std::string regime;
  int clip_count = 0;
  int cycle_count = 0;
  int knot_count = 0;
  double speed_min_mps = 0.0;
  double speed_max_mps = 0.0;
  double dimensionless_min = 0.0;
  double dimensionless_max = 0.0;
  bool single_speed = false;
};

struct ReferenceBuildResult {
  bool ok = false;
  std::string error;
  ProcLocomotionModel model;
  std::vector<std::string> warnings;
  std::vector<RegimeCoverage> coverage;
  ReferenceFidelity fidelity;
  bool single_speed = false;
};

struct ReferenceCaptureJob {
  std::filesystem::path input;
  std::string profile;
  std::string regime;
  std::optional<double> speed_override_mps;
  std::vector<int> cycles;
};

struct ReferenceCaptureRequest {
  std::filesystem::path body;
  std::filesystem::path profile_directory;
  std::filesystem::path output;
  std::string profile = "mixamo";
  std::vector<ReferenceCaptureJob> jobs;
};

struct ReferenceArgumentParse {
  ReferenceCaptureRequest request;
  bool help = false;
  bool ok = false;
  std::string error;
};

struct ReferenceCaptureReport {
  bool ok = false;
  std::string error;
  ReferenceSet set;
  ReferenceBuildResult build;
  int audit_violations = -1;
  double seam_error = 0.0;
  double slip_mean_mps = 0.0;
  double slip_peak_mps = 0.0;
};

std::string suggestRegime(const std::string& filename, const std::string& theme);
bool sourceAppearsInPlace(const ProcAnalysis& analysis);
double measuredRootSpeed(const ProcAnalysis& analysis);
SpeedProvenance speedProvenance(const ReferenceClipRecord& clip);
const char* speedProvenanceText(SpeedProvenance provenance);
double referencePhaseTime(const GaitCycle& cycle, double phase);
bool poseCurveClamped(const ProcLocomotionModel& model, const std::string& regime, double dimensionless_speed);

void selectAcceptedCycles(ReferenceClipRecord& clip);
bool addReferenceClip(ReferenceSet& set, ReferenceClipRecord clip);
bool removeReferenceClip(ReferenceSet& set, int index);
std::vector<ReferenceCycleInput> inputsFromReferenceSet(const ReferenceSet& set);
ModelBuildSettings referenceBuildSettings();
ReferenceBuildResult buildReferenceModel(const ReferenceSet& set, const ModelBuildSettings& settings = referenceBuildSettings());
std::vector<RegimeCoverage> referenceCoverage(const ReferenceSet& set, const ProcLocomotionModel& model);

Status writeReferenceSet(const ReferenceSet& set, const std::filesystem::path& path);
Status readReferenceSet(const std::filesystem::path& path, ReferenceSet& out);
Status exportReferenceModel(const ReferenceSet& set, const ReferenceBuildResult& build, int audit_violations,
                            const std::filesystem::path& model_path);

ReferenceArgumentParse parseReferenceArgs(int argc, char** argv);
ReferenceCaptureReport runReferenceCapture(const ReferenceCaptureRequest& request);

}  // namespace proc
