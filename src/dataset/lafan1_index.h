#pragma once

#include <string>

namespace proc {

struct LafanName {
  bool matched = false;
  std::string theme;
  std::string subject;
  std::string take;
};

// walk1_subject2 -> theme walk, take 1, subject 2.
// fallAndGetUp1_subject3 keeps the camel-case theme and splits the trailing digits into the take.
LafanName parseLafanFilename(const std::string& stem);

}  // namespace proc
