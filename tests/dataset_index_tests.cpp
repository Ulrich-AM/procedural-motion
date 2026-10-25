#include "test_support.h"

#include "dataset/batch_analyze.h"
#include "dataset/dataset_index.h"
#include "dataset/lafan1_index.h"

#include <filesystem>
#include <fstream>

namespace {

void writeFile(const std::filesystem::path& path, const std::string& text) {
  std::ofstream stream(path);
  stream << text;
}

}  // namespace

int runDatasetIndexTests() {
  TestStats stats;
  const proc::LafanName walk = proc::parseLafanFilename("walk1_subject2");
  stats.expect(walk.matched && walk.theme == "walk" && walk.take == "1" && walk.subject == "2", "LAFAN walk filename");
  const proc::LafanName fall = proc::parseLafanFilename("fallAndGetUp1_subject3");
  stats.expect(fall.theme == "fallAndGetUp" && fall.take == "1" && fall.subject == "3", "LAFAN camel-case theme");
  const proc::LafanName dance = proc::parseLafanFilename("dance2_subject1");
  stats.expect(dance.theme == "dance", "dance theme stays a label, not a gait class");

  const std::filesystem::path directory = std::filesystem::temp_directory_path() / "proc_phase3_index";
  std::error_code error;
  std::filesystem::remove_all(directory, error);
  std::filesystem::create_directories(directory);
  writeFile(directory / "walk1_subject2.bvh",
            "HIERARCHY\nROOT Hips\n{\n  OFFSET 0 0 0\n  CHANNELS 6 Xposition Yposition Zposition Zrotation Xrotation Yrotation\n"
            "  JOINT LeftUpLeg\n  {\n    OFFSET -0.1 0 -0.4\n    CHANNELS 3 Xrotation Yrotation Zrotation\n    End Site\n    {\n"
            "      OFFSET 0 0 -0.2\n    }\n  }\n}\nMOTION\nFrames: 2\nFrame Time: 0.5\n"
            "0 0 0 0 0 0 0 0 0\n0 0.1 0 0 0 0 0 0 0\n");
  writeFile(directory / "broken.bvh", "this is not a hierarchy\n");
  const std::filesystem::path root = std::filesystem::path(PROCMOTION_SOURCE_DIR);
  const proc::DatasetIndex index = proc::scanDataset(directory, "LAFAN1", root / "config" / "retarget", "lafan1");
  stats.expect(index.clips.size() == 2, "malformed file does not stop the scan");
  int failed = 0;
  int walks = 0;
  for (const proc::IndexedClip& clip : index.clips) {
    if (clip.import_status == "failed") ++failed;
    if (clip.theme == "walk" && clip.subject == "2" && clip.import_status == "ok") ++walks;
  }
  stats.expect(failed == 1 && walks == 1, "index records the failure and the walk clip");

  proc::BatchRequest request;
  request.input = directory;
  request.output = directory / "out";
  request.profile_directory = root / "config" / "retarget";
  request.body = root / "assets" / "bodies" / "neutral.procbody.json";
  request.profile = "lafan1";
  request.dataset = "LAFAN1";
  request.theme = "Walk";
  const proc::BatchReport report = proc::analyzeBatch(request);
  stats.expect(report.processed == 1 && report.failed == 0 && report.analyzed == 1, "theme filter analyzes the walk clip and skips the broken file");
  stats.expect(std::filesystem::exists(request.output / "summary.txt"), "batch writes a summary");
  stats.expect(std::filesystem::exists(request.output / "tables" / "gait_cycles.csv"), "batch writes the cycle table");
  std::filesystem::remove_all(directory, error);
  if (stats.failed == 0) std::cout << "dataset index tests ok\n";
  return stats.failed;
}
