#pragma once

#include "import/source_motion.h"

#include <glm/mat3x3.hpp>

#include <string>

namespace proc {

struct AxisConvention {
  // proc = basis * file_vector, then positions are multiplied by unit_to_meters.
  glm::dmat3 basis{1.0};
  double unit_to_meters = 1.0;
  std::string description;
};

AxisConvention identityConvention();
AxisConvention conventionFromAxes(const std::string& up, const std::string& forward, double unit_to_meters, std::string& error);
AxisConvention detectHumanConvention(const SourceClip& file_space_clip);

void applyAxisConvention(SourceClip& clip, const AxisConvention& convention);

}  // namespace proc
