#pragma once

#include <algorithm>
#include <string>
#include <unordered_map>

namespace pm {

// One hinge objective. The servo turns this into a limited torque.
// Feedforward is extra torque (capture balance). It is not a root force.
struct JointCommand {
  double target = 0;
  double kp = 0;
  double kd = 0;
  double limit = 0;
  double feedforward = 0;
  bool used = false;
};

// Stance commands and swing commands stay separate so a landing can fade
// swing authority without depending on which controller ran last.
struct ControlFrame {
  void clear() {
    stance.clear();
    swing.clear();
    swingWeight.clear();
  }

  void setStance(const std::string& name, JointCommand command) {
    command.used = true;
    stance[name] = command;
  }

  void setSwing(const std::string& name, JointCommand command, double weight) {
    command.used = true;
    swing[name] = command;
    swingWeight[name] = std::clamp(weight, 0.0, 1.0);
  }

  JointCommand resolve(const std::string& name) const {
    const auto stanceIt = stance.find(name);
    const auto swingIt = swing.find(name);
    const bool hasStance = stanceIt != stance.end() && stanceIt->second.used;
    const bool hasSwing = swingIt != swing.end() && swingIt->second.used;
    double weight = 0;
    if (hasSwing) {
      const auto found = swingWeight.find(name);
      if (found != swingWeight.end()) weight = found->second;
    }
    if (!hasSwing || weight <= 1e-4) return hasStance ? stanceIt->second : JointCommand{};
    if (!hasStance || weight >= 1.0 - 1e-4) return swingIt->second;
    const JointCommand& base = stanceIt->second;
    const JointCommand& over = swingIt->second;
    JointCommand mixed;
    mixed.used = true;
    mixed.target = base.target * (1.0 - weight) + over.target * weight;
    mixed.kp = base.kp * (1.0 - weight) + over.kp * weight;
    mixed.kd = base.kd * (1.0 - weight) + over.kd * weight;
    mixed.feedforward = base.feedforward * (1.0 - weight) + over.feedforward * weight;
    mixed.limit = std::max(base.limit, over.limit);
    return mixed;
  }

  std::unordered_map<std::string, JointCommand> stance;
  std::unordered_map<std::string, JointCommand> swing;
  std::unordered_map<std::string, double> swingWeight;
};

}  // namespace pm
