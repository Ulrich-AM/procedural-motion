#include "retarget/retarget_diagnostics.h"

#include <sstream>

namespace proc {

std::string formatRetargetReport(const RetargetReport& report) {
  std::ostringstream out;
  out << "mapped " << report.mapped << " / " << report.required << "\n";
  out << "rest retarget max error " << report.rest_max_error_deg << " deg\n";
  out << "segment direction max error " << report.direction_max_error_deg << " deg";
  if (!report.direction_worst_bone.empty()) out << " (" << report.direction_worst_bone << ")";
  out << "\nPROC non-root translations exported: " << report.non_root_translations_exported << "\n";
  out << "source non-root translation channels: " << report.translation_notes.size();
  return out.str();
}

}  // namespace proc
