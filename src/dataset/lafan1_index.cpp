#include "dataset/lafan1_index.h"

#include <cctype>

namespace proc {

LafanName parseLafanFilename(const std::string& stem) {
  LafanName parsed;
  std::string lower = stem;
  for (char& character : lower) character = static_cast<char>(std::tolower(static_cast<unsigned char>(character)));
  const std::size_t subject_at = lower.find("_subject");
  std::string prefix = subject_at == std::string::npos ? stem : stem.substr(0, subject_at);
  if (subject_at != std::string::npos) {
    parsed.subject = stem.substr(subject_at + std::string("_subject").size());
    parsed.matched = !parsed.subject.empty();
  }
  std::size_t digit = prefix.size();
  while (digit > 0 && std::isdigit(static_cast<unsigned char>(prefix[digit - 1]))) --digit;
  if (digit < prefix.size()) {
    parsed.take = prefix.substr(digit);
    prefix = prefix.substr(0, digit);
    parsed.matched = true;
  }
  parsed.theme = prefix;
  if (!parsed.theme.empty() && !parsed.take.empty()) parsed.matched = true;
  return parsed;
}

}  // namespace proc
