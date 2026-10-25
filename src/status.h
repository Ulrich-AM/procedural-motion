#pragma once

#include <string>
#include <utility>
#include <vector>

namespace proc {

struct Status {
  std::vector<std::string> errors;
  std::vector<std::string> warnings;

  bool ok() const { return errors.empty(); }

  void error(std::string message) { errors.push_back(std::move(message)); }
  void warn(std::string message) { warnings.push_back(std::move(message)); }

  std::string text() const {
    std::string out;
    for (const std::string& message : errors) {
      if (!out.empty()) out += '\n';
      out += message;
    }
    for (const std::string& message : warnings) {
      if (!out.empty()) out += '\n';
      out += "warning: " + message;
    }
    return out;
  }
};

}  // namespace proc
