#include "dataset/batch_analyze.h"

#include "analysis/analysis_io.h"
#include "analysis/proc_analysis.h"
#include "analysis/relationship_model.h"
#include "body/proc_body.h"
#include "import/bvh_importer.h"
#include "import/fbx_importer.h"
#include "retarget/retarget_profile.h"
#include "retarget/retargeter.h"
#include "retarget/source_semantics.h"

#include <algorithm>
#include <fstream>
#include <iostream>

namespace proc {
namespace {

std::string lowerCopy(std::string text) {
  for (char& character : text) {
    if (character >= 'A' && character <= 'Z') character = static_cast<char>(character - 'A' + 'a');
  }
  return text;
}

bool themeMatches(const IndexedClip& clip, const std::string& theme) {
  if (theme.empty()) return true;
  const std::string needle = lowerCopy(theme);
  return lowerCopy(clip.theme).find(needle) != std::string::npos || lowerCopy(clip.filename).find(needle) != std::string::npos;
}

}  // namespace

BatchReport analyzeBatch(const BatchRequest& request) {
  BatchReport report;
  DatasetIndex index = scanDataset(request.input, request.dataset, request.profile_directory, request.profile);
  std::vector<RetargetProfile> profiles;
  loadRetargetProfiles(request.profile_directory, profiles);
  const RetargetProfile* selected = nullptr;
  for (const RetargetProfile& profile : profiles) {
    if (profile.name == request.profile) selected = &profile;
  }
  if (selected == nullptr) report.warnings.push_back("profile '" + request.profile + "' was not found; source analysis still runs");

  ProcBody body;
  bool has_body = false;
  if (!request.body.empty()) {
    Status status = loadProcBodyFromFile(request.body, body);
    if (!status.ok()) report.warnings.push_back(status.text());
    else has_body = true;
  }

  std::vector<ProcAnalysis> analyses;
  int ordinal = 0;
  const int total = static_cast<int>(std::count_if(index.clips.begin(), index.clips.end(), [&](const IndexedClip& clip) {
    return themeMatches(clip, request.theme) && clip.format != "procmo";
  }));
  for (IndexedClip& clip : index.clips) {
    if (!themeMatches(clip, request.theme) || clip.format == "procmo") continue;
    ++ordinal;
    ++report.processed;
    std::cout << "[" << ordinal << "/" << total << "] " << clip.filename << '\n';
    if (clip.import_status == "failed") {
      ++report.failed;
      clip.analysis_status = "failed";
      report.rejected_clips.push_back(clip.filename + ": " + clip.error);
      report.warnings.push_back(clip.filename + ": " + clip.error);
      continue;
    }
    try {
      SourceClip source;
      const AxisConvention convention = selected != nullptr && selected->has_coordinates ? selected->coordinates : identityConvention();
      Status status = clip.format == "bvh" ? loadBvhFromFile(clip.path, convention, source) : loadFbxFromFile(clip.path, convention, source);
      if (!status.ok()) {
        ++report.failed;
        clip.import_status = "failed";
        clip.analysis_status = "failed";
        clip.error = status.text();
        report.rejected_clips.push_back(clip.filename + ": " + status.text());
        report.warnings.push_back(clip.filename + ": " + status.text());
        continue;
      }
      ProcMotion motion;
      std::array<int, kJointCount> mapping{};
      mapping.fill(-1);
      if (selected != nullptr) {
        std::vector<std::string> names;
        for (const SourceJoint& joint : source.joints) names.push_back(joint.name);
        mapping = mappingFromProfile(*selected, names);
      }
      if (has_body && selected != nullptr) {
        RetargetReport retarget_report;
        status = retargetClip(body, source, mapping, selected->required, {}, motion, retarget_report);
        if (!status.ok()) {
          report.warnings.push_back(clip.filename + ": retarget failed (" + status.text() + "); rotational channels were left at identity");
          motion = {};
        }
      } else if (!has_body) {
        report.warnings.push_back(clip.filename + ": no PROC body; rotational channels were left at identity");
      }
      AnalysisInput input;
      input.source = &source;
      input.motion = motion.frames.empty() ? nullptr : &motion;
      input.semantics = semanticsFromMapping(mapping);
      input.dataset = clip.dataset;
      input.theme = clip.theme;
      input.subject = clip.subject;
      input.take = clip.take;
      input.motion_path = clip.path;
      ProcAnalysis analysis = analyzeClip(input);
      const std::filesystem::path clip_path = request.output / "clips" / (std::filesystem::path(clip.filename).stem().string() + ".procanalysis.json");
      status = writeAnalysisToFile(analysis, clip_path);
      if (!status.ok()) {
        ++report.failed;
        clip.analysis_status = "failed";
        report.warnings.push_back(status.text());
        continue;
      }
      int accepted = 0;
      int rejected = 0;
      for (const GaitCycle& cycle : analysis.cycles) {
        if (cycle.mirrored) continue;
        ++report.cycles_detected;
        if (cycle.accepted) ++accepted;
        else ++rejected;
      }
      report.cycles_accepted += accepted;
      report.cycles_rejected += rejected;
      ++report.analyzed;
      clip.analysis_status = "ok";
      clip.import_status = "ok";
      analyses.push_back(std::move(analysis));
    } catch (const std::exception& exception) {
      ++report.failed;
      clip.analysis_status = "failed";
      clip.error = exception.what();
      report.rejected_clips.push_back(clip.filename + ": " + exception.what());
      report.warnings.push_back(clip.filename + ": " + exception.what());
    }
  }

  ProcAnalysis pooled;
  for (const ProcAnalysis& analysis : analyses) {
    for (const GaitCycle& cycle : analysis.cycles) {
      if (!cycle.mirrored) pooled.cycles.push_back(cycle);
    }
  }
  fitRelationships(pooled);
  writeRelationshipFile(pooled.relationships, "walk", request.output / "models" / "walk_reference.json");
  writeRelationshipFile(pooled.relationships, "run", request.output / "models" / "run_reference.json");
  writeRelationshipFile(pooled.relationships, "sprint", request.output / "models" / "sprint_reference.json");
  {
    std::error_code error;
    std::filesystem::create_directories(request.output / "tables", error);
    std::ofstream csv(request.output / "tables" / "gait_cycles.csv");
    if (csv) csv << cyclesToCsv(analyses);
  }
  writeDatasetIndex(index, request.output / "index.json");
  std::ofstream summary(request.output / "summary.txt");
  if (summary) {
    summary << "processed: " << report.processed << "\nanalyzed: " << report.analyzed << "\nfailed: " << report.failed
            << "\n\ncycles detected: " << report.cycles_detected << "\ncycles accepted: " << report.cycles_accepted
            << "\ncycles rejected: " << report.cycles_rejected << "\n";
    for (const std::string& warning : report.warnings) summary << "warning: " << warning << '\n';
    for (const std::string& rejected : report.rejected_clips) summary << "rejected clip: " << rejected << '\n';
  }
  std::cout << "processed: " << report.processed << "\nanalyzed: " << report.analyzed << "\nfailed: " << report.failed
            << "\n\ncycles detected: " << report.cycles_detected << "\ncycles accepted: " << report.cycles_accepted
            << "\ncycles rejected: " << report.cycles_rejected << '\n';
  return report;
}

}  // namespace proc
