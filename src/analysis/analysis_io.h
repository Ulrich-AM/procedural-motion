#pragma once

#include "analysis/proc_analysis.h"
#include "status.h"

#include <filesystem>
#include <string>
#include <string_view>

namespace proc {

std::string writeAnalysisToString(const ProcAnalysis& analysis);
Status readAnalysisFromString(std::string_view text, ProcAnalysis& out, std::string_view label = "analysis");
Status writeAnalysisToFile(const ProcAnalysis& analysis, const std::filesystem::path& path);
Status readAnalysisFromFile(const std::filesystem::path& path, ProcAnalysis& out);

std::string cyclesToCsv(const std::vector<ProcAnalysis>& analyses);
Status writeRelationshipFile(const std::vector<RelationshipFit>& fits, const std::string& regime, const std::filesystem::path& path);

}  // namespace proc
