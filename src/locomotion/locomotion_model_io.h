#pragma once

#include "locomotion/locomotion_model.h"
#include "status.h"

#include <filesystem>
#include <string>
#include <string_view>

namespace proc {

std::string writeLocomotionModelToString(const ProcLocomotionModel& model);
Status writeLocomotionModelToFile(const ProcLocomotionModel& model, const std::filesystem::path& path);
Status readLocomotionModelFromString(std::string_view text, ProcLocomotionModel& out, std::string_view label = "locomotion model");
Status readLocomotionModelFromFile(const std::filesystem::path& path, ProcLocomotionModel& out);

}  // namespace proc
