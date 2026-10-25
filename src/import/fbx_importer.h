#pragma once

#include "import/coordinate_normalizer.h"
#include "status.h"

#include <filesystem>

namespace proc {

Status loadFbxFromFile(const std::filesystem::path& path, const AxisConvention& convention, SourceClip& out);

}  // namespace proc
