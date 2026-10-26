#pragma once

#include "analysis/proc_analysis.h"
#include "dataset/dataset_index.h"
#include "locomotion/locomotion_model.h"

#include <filesystem>
#include <string>
#include <vector>

namespace proc {

// Offline real-gait experiment. This does not live in proc_runtime.
// A real-derived model is local experiment output, never a public asset.

struct GaitExperimentConfig {
  std::filesystem::path dataset;
  std::filesystem::path output = "analysis_output/real_gait";
  std::filesystem::path body;
  std::filesystem::path profile_directory;
  std::filesystem::path synthetic_model;
  std::string profile = "lafan1";
  std::string themes = "walk,run,sprint";
  std::string dataset_name = "LAFAN1";
  double min_quality = 0.0;
  int min_cycles_per_knot = 3;
  int max_cycles_per_subject = 8;
  double bin_width = 0.20;
};

struct GaitArgumentParse {
  GaitExperimentConfig config;
  bool help = false;
  bool ok = false;
  std::string error;
};

struct ManifestCycle {
  std::string source_clip;
  std::string dataset;
  std::string subject;
  std::string theme;
  int cycle_index = 0;
  std::string side;
  double start_time = 0.0;
  double end_time = 0.0;
  double speed_mps = 0.0;
  double dimensionless_speed = 0.0;
  double cadence_spm = 0.0;
  double stride_length_m = 0.0;
  double stance_fraction = 0.0;
  double flight_fraction = 0.0;
  double quality_contact = 0.0;
  double quality_timing = 0.0;
  double quality_heading = 0.0;
  double quality_overall = 0.0;
  bool accepted = false;
  std::string rejection_reason;
  std::string classification;
  std::string observed_regime;
  bool manual_override = false;
  std::string review = "automatic";
  bool holdout = false;
  bool used_in_model = false;
  std::vector<double> hip_deg;
  std::vector<double> knee_deg;
  std::vector<double> ankle_deg;
};

struct FailedClip {
  std::string filename;
  std::string reason;
};

struct ExperimentManifest {
  std::vector<ManifestCycle> cycles;
  std::vector<FailedClip> failed_clips;
};

struct KnotReport {
  std::string regime;
  double dimensionless_speed = 0.0;
  double speed_mps = 0.0;
  int cycle_count = 0;
  int clip_count = 0;
  int subject_count = 0;
  double quality_median = 0.0;
  double spread = 0.0;
  double cadence_spm = 0.0;
  double stance_fraction = 0.0;
  double flight_fraction = 0.0;
  bool low_confidence = false;
  bool trusted = false;
};

struct ValidationSplit {
  std::string method = "none";
  std::vector<int> build;
  std::vector<int> holdout;
};

struct HoldoutMetrics {
  int cycles = 0;
  double cadence_difference = 0.0;
  double stride_difference = 0.0;
  double stance_difference = 0.0;
  double flight_difference = 0.0;
  double hip_rms = 0.0;
  double knee_rms = 0.0;
  double ankle_rms = 0.0;
  bool curves_available = false;
};

struct CurveDifference {
  std::string regime;
  double dimensionless_speed = 0.0;
  double hip_rms = 0.0;
  double knee_rms = 0.0;
  double ankle_rms = 0.0;
  double pelvis_rms = 0.0;
  double hip_range_difference = 0.0;
  double knee_range_difference = 0.0;
  double ankle_range_difference = 0.0;
  double hip_peak_phase_difference = 0.0;
  double knee_peak_phase_difference = 0.0;
  double ankle_peak_phase_difference = 0.0;
  double cadence_difference = 0.0;
  double stance_difference = 0.0;
  double flight_difference = 0.0;
  bool available = false;
};

struct ClosedLoopReport {
  std::string regime;
  bool attempted = false;
  double commanded_speed = 0.0;
  double measured_speed = 0.0;
  double model_cadence = 0.0;
  double measured_cadence = 0.0;
  double model_stride = 0.0;
  double measured_stride = 0.0;
  double model_stance = 0.0;
  double detected_stance = 0.0;
  double model_flight = 0.0;
  double detected_flight = 0.0;
  int accepted_cycles = 0;
  int detected_cycles = 0;
  std::string cycle_note;
  double slip_left_mean = 0.0;
  double slip_left_peak = 0.0;
  double slip_right_mean = 0.0;
  double slip_right_peak = 0.0;
};

struct GaitExperimentResult {
  bool dataset_found = false;
  std::string dataset_path;
  std::string reason;
  std::string tier = "0";
  int clips_indexed = 0;
  int clips_processed = 0;
  int clips_failed = 0;
  int subjects = 0;
  ExperimentManifest manifest;
  ValidationSplit split;
  std::vector<KnotReport> knots;
  HoldoutMetrics holdout;
  std::vector<CurveDifference> synthetic_comparison;
  std::vector<ClosedLoopReport> closed_loop;
  int audit_violations = -1;
  double seam_error = 0.0;
  std::vector<std::string> warnings;
};

GaitArgumentParse parseGaitExperimentArgs(int argc, char** argv);

// Comma-separated themes. An empty filter keeps every clip.
bool themeAllowed(const std::string& theme, const std::string& filename, const std::string& filter);

bool outputIsPublicAsset(const std::filesystem::path& output);

ExperimentManifest manifestFromAnalyses(const std::vector<ProcAnalysis>& analyses, double gravity_mps2);
void noteFailedClip(ExperimentManifest& manifest, const std::string& filename, const std::string& reason);

// Caps accepted cycles per subject inside each regime and speed bin.
// Surplus cycles are rejected with reason "subject cap".
void capCyclesPerSubject(std::vector<ProcAnalysis>& analyses, ExperimentManifest& manifest, int cap, double bin_width,
                         double gravity_mps2, const std::vector<int>& build_rows = {});

ValidationSplit splitManifest(const ExperimentManifest& manifest);

std::vector<KnotReport> classifyKnots(const ProcLocomotionModel& model, const ExperimentManifest& manifest, const ValidationSplit& split,
                                      int min_cycles);

HoldoutMetrics measureHoldout(const ProcLocomotionModel& model, const ExperimentManifest& manifest, const ValidationSplit& split);

CurveDifference compareModelsAtSpeed(const ProcLocomotionModel& primary, const ProcLocomotionModel& other, const std::string& regime,
                                      double dimensionless_speed);

Status writeCycleManifest(const ExperimentManifest& manifest, const std::filesystem::path& json_path, const std::filesystem::path& csv_path);
Status writeValidationReport(const GaitExperimentResult& result, const GaitExperimentConfig& config, const std::filesystem::path& directory);

GaitExperimentResult runGaitExperiment(const GaitExperimentConfig& config);

}  // namespace proc
