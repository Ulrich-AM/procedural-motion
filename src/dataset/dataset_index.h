#pragma once

#include "dataset/lafan1_index.h"
#include "status.h"

#include <filesystem>
#include <string>
#include <vector>

namespace proc {

struct IndexedClip {
  std::string path;
  std::string filename;
  std::string dataset;
  std::string theme;
  std::string subject;
  std::string take;
  std::string format;
  double duration_s = 0.0;
  double fps = 0.0;
  int frame_count = 0;
  std::string profile;
  std::string profile_confidence;
  std::string import_status;
  std::string analysis_status;
  std::string error;
};

struct DatasetIndex {
  std::vector<IndexedClip> clips;
};

DatasetIndex scanDataset(const std::filesystem::path& directory, const std::string& dataset_name, const std::filesystem::path& profile_directory,
                         const std::string& profile_name);
Status writeDatasetIndex(const DatasetIndex& index, const std::filesystem::path& path);

}  // namespace proc
