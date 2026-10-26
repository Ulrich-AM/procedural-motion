#pragma once

#include "analysis/reference_capture.h"
#include "app/analysis_panel.h"
#include "app/import_panel.h"

struct GLFWwindow;

namespace proc {

enum class ReferenceView { Source, Procedural, Overlay };
enum class ReferenceCurves { SourceCycle, ModelResult, Overlay };

struct ReferenceUi {
  std::string regime = "walk";
  bool manual_speed = false;
  float speed_override = 1.25f;
  std::vector<char> include;
  int review_cycle = 0;
  ReferenceSet set;
  bool has_model = false;
  ProcLocomotionModel model;
  ReferenceBuildResult build;
  int audit_violations = -1;
  std::string status;
  ReferenceView view = ReferenceView::Procedural;
  ReferenceCurves curves = ReferenceCurves::Overlay;
  float phase = 0.0f;
  bool playing = false;
  bool in_place = true;
  bool show_ghosts = false;
  float preview_speed = 1.2f;
  int analysis_cycle_count = -1;
};

void drawReferencePanel(ReferenceUi& ui, AnalysisUi& analysis, const ImportSession& import, const ProcMotion* motion, bool has_motion,
                        GLFWwindow* window, const ProcBody* body, bool has_body, const std::filesystem::path& asset_root);

}  // namespace proc
