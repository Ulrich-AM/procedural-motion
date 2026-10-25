#pragma once

#include "import/coordinate_normalizer.h"
#include "status.h"

#include <filesystem>
#include <string>
#include <string_view>

namespace proc {

Status loadBvhFromString(std::string_view text, const AxisConvention& convention, SourceClip& out, std::string_view label);
Status loadBvhFromFile(const std::filesystem::path& path, const AxisConvention& convention, SourceClip& out);

}  // namespace proc
