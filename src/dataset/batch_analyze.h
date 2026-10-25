#pragma once

#include "dataset/dataset_index.h"

#include <filesystem>
#include <string>
#include <vector>

namespace proc {

struct BatchRequest {
  std::filesystem::path input;
  std::filesystem::path output;
  std::filesystem::path profile_directory;
  std::filesystem::path body;
  std::string profile = "lafan1";
  std::string theme;
  std::string dataset;
};

struct BatchReport {
  int processed = 0;
  int analyzed = 0;
  int failed = 0;
  int cycles_detected = 0;
  int cycles_accepted = 0;
  int cycles_rejected = 0;
  std::vector<std::string> warnings;
  std::vector<std::string> rejected_clips;
};

BatchReport analyzeBatch(const BatchRequest& request);

}  // namespace proc
