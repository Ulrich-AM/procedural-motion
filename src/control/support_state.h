#pragma once

#include "control/foot_contact.h"
#include "physics/body_state.h"
#include "physics/physical_metrics.h"

#include <algorithm>
#include <cmath>
#include <string>

namespace pm {

enum class SupportKind { None, Left, Right, Double };

// Support from the feet that are actually carrying load. The pelvis is not a support point.
struct SupportState {
  SupportKind kind = SupportKind::None;
  Vec3 center{};
  bool valid = false;
  double leftLoad = 0;
  double rightLoad = 0;
  double totalLoad = 0;
  double leftFraction = 0;
  double rightFraction = 0;
  bool leftPlanted = false;
  bool rightPlanted = false;
  Vec3 leftFoot{};
  Vec3 rightFoot{};
  bool leftFound = false;
  bool rightFound = false;
  double sagMin = 0;
  double sagMax = 0;
  double latMin = 0;
  double latMax = 0;
  double captureOutside = 0;
};

inline const char* supportKindLabel(SupportKind kind) {
  switch (kind) {
    case SupportKind::Left:
      return "Left";
    case SupportKind::Right:
      return "Right";
    case SupportKind::Double:
      return "Double";
    case SupportKind::None:
      return "None";
  }
  return "None";
}

inline SupportState measureSupport(const BodyState& state, const FootContactTracker& feet, const PhysicalMetrics& metrics,
                                   Vec3 forward, Vec3 right, Vec3 capture, bool plantedOnly = true) {
  SupportState support;
  forward.z = 0;
  right.z = 0;
  if (length(forward) < 0.5) forward = {0, -1, 0};
  if (length(right) < 0.5) right = {-1, 0, 0};
  forward = normalize(forward);
  right = normalize(right);

  support.leftLoad = std::max(0.0, feet.left().load);
  support.rightLoad = std::max(0.0, feet.right().load);
  support.totalLoad = support.leftLoad + support.rightLoad;
  if (support.totalLoad > 40.0) {
    support.leftFraction = support.leftLoad / support.totalLoad;
    support.rightFraction = support.rightLoad / support.totalLoad;
  }
  auto planted = [&](const FootContact& foot) {
    if (!plantedOnly) return foot.load > 12.0 || foot.mode != FootContactMode::Air;
    return foot.mode == FootContactMode::Planted || (foot.mode == FootContactMode::Releasing && foot.load > 25.0) ||
           foot.load > 45.0;
  };
  support.leftPlanted = planted(feet.left());
  support.rightPlanted = planted(feet.right());

  for (const BodyKinematics& body : state.bodies) {
    if (body.name == "left_foot") {
      support.leftFoot = body.position;
      support.leftFoot.z = 0;
      support.leftFound = true;
    } else if (body.name == "right_foot") {
      support.rightFoot = body.position;
      support.rightFoot.z = 0;
      support.rightFound = true;
    }
  }

  int count = 0;
  Vec3 sum{};
  bool bounds = false;
  auto add = [&](bool use, Vec3 foot) {
    if (!use) return;
    sum = sum + foot;
    ++count;
    const double sag = dot(foot, forward);
    const double lat = dot(foot, right);
    const double sag0 = sag - metrics.heel;
    const double sag1 = sag + metrics.toe;
    const double lat0 = lat - metrics.halfWidth;
    const double lat1 = lat + metrics.halfWidth;
    if (!bounds) {
      support.sagMin = sag0;
      support.sagMax = sag1;
      support.latMin = lat0;
      support.latMax = lat1;
      bounds = true;
    } else {
      support.sagMin = std::min(support.sagMin, sag0);
      support.sagMax = std::max(support.sagMax, sag1);
      support.latMin = std::min(support.latMin, lat0);
      support.latMax = std::max(support.latMax, lat1);
    }
  };
  add(support.leftPlanted && support.leftFound, support.leftFoot);
  add(support.rightPlanted && support.rightFound, support.rightFoot);
  if (count > 0) {
    support.valid = true;
    support.center = sum / static_cast<double>(count);
    support.center.z = 0;
    if (count == 2) support.kind = SupportKind::Double;
    else support.kind = support.leftPlanted ? SupportKind::Left : SupportKind::Right;
    Vec3 point = capture;
    point.z = 0;
    const double sag = dot(point, forward);
    const double lat = dot(point, right);
    const double sagOver = std::max(support.sagMin - sag, std::max(0.0, sag - support.sagMax));
    const double latOver = std::max(support.latMin - lat, std::max(0.0, lat - support.latMax));
    support.captureOutside = std::hypot(sagOver, latOver);
  }
  return support;
}

}  // namespace pm
