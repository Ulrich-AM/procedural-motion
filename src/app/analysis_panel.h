#pragma once

#include "analysis/proc_analysis.h"
#include "motion/motion_player.h"

#include <array>
#include <filesystem>
#include <string>

namespace proc {

struct AnalysisUi {
  ProcAnalysis analysis;
  bool valid = false;
  std::string message;
  int event_index = -1;
  int cycle_index = -1;
  int plot = 0;
  int marker = 0;
};

void drawAnalysisSection(AnalysisUi& ui, const SourceClip* source, const ProcMotion* motion,
                         const std::array<int, kJointCount>& mapping, MotionPlayer& player,
                         const std::filesystem::path& asset_root);
void drawAnalysisOverlay(const AnalysisUi& ui, MotionPlayer& player, float x0, float y0, float x1, float y1);

}  // namespace proc
