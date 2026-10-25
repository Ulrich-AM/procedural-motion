#include "dataset/dataset_index.h"

#include "import/bvh_importer.h"
#include "import/fbx_importer.h"
#include "retarget/retarget_profile.h"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cctype>
#include <fstream>

namespace proc {
namespace {

std::string lowerExtension(const std::filesystem::path& path) {
  std::string extension = path.extension().string();
  for (char& character : extension) {
    if (character >= 'A' && character <= 'Z') character = static_cast<char>(character - 'A' + 'a');
  }
  return extension;
}

bool containsLafan(const std::filesystem::path& path) {
  std::string text = path.generic_string();
  for (char& character : text) character = static_cast<char>(std::tolower(static_cast<unsigned char>(character)));
  return text.find("lafan") != std::string::npos;
}

}  // namespace

DatasetIndex scanDataset(const std::filesystem::path& directory, const std::string& dataset_name, const std::filesystem::path& profile_directory,
                         const std::string& profile_name) {
  DatasetIndex index;
  if (!std::filesystem::exists(directory)) return index;
  std::vector<RetargetProfile> profiles;
  loadRetargetProfiles(profile_directory, profiles);
  const RetargetProfile* selected = nullptr;
  for (const RetargetProfile& profile : profiles) {
    if (profile.name == profile_name) selected = &profile;
  }
  std::vector<std::filesystem::path> files;
  std::error_code error;
  for (std::filesystem::recursive_directory_iterator cursor(directory, error), end; cursor != end; cursor.increment(error)) {
    if (error) {
      error.clear();
      continue;
    }
    if (!cursor->is_regular_file()) continue;
    const std::string extension = lowerExtension(cursor->path());
    if (extension == ".bvh" || extension == ".fbx" || extension == ".procmo") files.push_back(cursor->path());
  }
  std::sort(files.begin(), files.end());
  for (const std::filesystem::path& path : files) {
    IndexedClip record;
    record.path = path.string();
    record.filename = path.filename().string();
    record.format = lowerExtension(path);
    if (!record.format.empty() && record.format.front() == '.') record.format.erase(record.format.begin());
    const LafanName lafan = parseLafanFilename(path.stem().string());
    record.theme = lafan.theme;
    record.subject = lafan.subject;
    record.take = lafan.take;
    const bool lafan_dataset = dataset_name == "LAFAN1" || dataset_name == "lafan1" || containsLafan(path) || profile_name == "lafan1";
    record.dataset = lafan_dataset ? "LAFAN1" : (dataset_name.empty() ? directory.filename().string() : dataset_name);
    record.profile = profile_name;
    record.analysis_status = "pending";
    record.import_status = "pending";
    try {
      if (record.format == "procmo") {
        record.import_status = "skipped";
        record.analysis_status = "skipped_no_source";
        record.error = "PROC motion has no source skeleton; gait contact is measured from SourceClip";
      } else {
        SourceClip clip;
        const AxisConvention convention = selected != nullptr && selected->has_coordinates ? selected->coordinates : identityConvention();
        Status status = record.format == "bvh" ? loadBvhFromFile(path, convention, clip) : loadFbxFromFile(path, convention, clip);
        if (!status.ok()) {
          record.import_status = "failed";
          record.analysis_status = "not_run";
          record.error = status.text();
        } else {
          record.import_status = "ok";
          record.duration_s = clip.duration;
          record.fps = clip.nominal_fps;
          record.frame_count = static_cast<int>(clip.frames.size());
          std::vector<std::string> names;
          for (const SourceJoint& joint : clip.joints) names.push_back(joint.name);
          if (selected != nullptr) {
            const std::array<int, kJointCount> mapping = mappingFromProfile(*selected, names);
            int matched = 0;
            int required = 0;
            for (int joint = 0; joint < kJointCount; ++joint) {
              if (!selected->required[joint]) continue;
              ++required;
              if (mapping[joint] >= 0) ++matched;
            }
            record.profile_confidence = required > 0 && matched == required ? "HIGH" : "INCOMPLETE";
          } else if (!profile_name.empty()) {
            record.profile_confidence = "missing_profile";
          }
        }
      }
    } catch (const std::exception& exception) {
      record.import_status = "failed";
      record.analysis_status = "not_run";
      record.error = exception.what();
    }
    index.clips.push_back(std::move(record));
  }
  return index;
}

Status writeDatasetIndex(const DatasetIndex& index, const std::filesystem::path& path) {
  nlohmann::json clips = nlohmann::json::array();
  for (const IndexedClip& clip : index.clips) {
    clips.push_back({{"path", clip.path},
                     {"filename", clip.filename},
                     {"dataset", clip.dataset},
                     {"theme", clip.theme},
                     {"subject", clip.subject},
                     {"take", clip.take},
                     {"format", clip.format},
                     {"duration_s", clip.duration_s},
                     {"fps", clip.fps},
                     {"frame_count", clip.frame_count},
                     {"profile", clip.profile},
                     {"profile_confidence", clip.profile_confidence},
                     {"import_status", clip.import_status},
                     {"analysis_status", clip.analysis_status},
                     {"error", clip.error}});
  }
  nlohmann::json document = {{"format", "PROC_DATASET_INDEX"}, {"version", 1}, {"clips", clips}};
  Status status;
  std::error_code error;
  if (path.has_parent_path()) std::filesystem::create_directories(path.parent_path(), error);
  std::ofstream stream(path);
  if (!stream) {
    status.error("could not write " + path.string());
    return status;
  }
  stream << document.dump(2);
  return status;
}

}  // namespace proc
