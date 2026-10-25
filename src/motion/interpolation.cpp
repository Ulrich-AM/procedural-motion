#include "motion/interpolation.h"

#include <algorithm>
#include <cmath>

namespace proc {

Pose interpolatePoses(const Pose& a, const Pose& b, double u) {
  const double weight = std::clamp(u, 0.0, 1.0);
  Pose pose;
  pose.root_translation = a.root_translation + (b.root_translation - a.root_translation) * weight;
  pose.root_rotation = quatSlerp(a.root_rotation, b.root_rotation, weight);
  for (int index = 0; index < kJointCount; ++index) {
    pose.local_rotation[index] = quatSlerp(a.local_rotation[index], b.local_rotation[index], weight);
  }
  pose.contacts = a.contacts;
  pose.time = a.time + (b.time - a.time) * weight;
  return pose;
}

Pose sampleMotion(const ProcMotion& motion, double time_s, bool loop) {
  if (motion.frames.empty()) return Pose{};
  if (motion.frames.size() == 1) {
    Pose pose = motion.frames.front();
    pose.time = time_s;
    return pose;
  }

  const double start = motion.frames.front().time;
  const double end = motion.frames.back().time;
  double t = time_s;
  if (loop && motion.duration_s > end + 1e-9) {
    double wrapped = std::fmod(t - start, motion.duration_s);
    if (wrapped < 0.0) wrapped += motion.duration_s;
    t = start + wrapped;
    if (t > end + 1e-9) {
      const double span = motion.duration_s - end;
      const double u = span > 1e-12 ? std::clamp((t - end) / span, 0.0, 1.0) : 0.0;
      Pose pose = interpolatePoses(motion.frames.back(), motion.frames.front(), u);
      pose.time = t;
      pose.contacts = motion.frames.back().contacts;
      return pose;
    }
  } else {
    t = std::clamp(t, start, end);
  }

  int index = 0;
  for (int frame = 0; frame < static_cast<int>(motion.frames.size()); ++frame) {
    if (motion.frames[frame].time <= t + 1e-12) index = frame;
    else break;
  }
  if (index + 1 >= static_cast<int>(motion.frames.size()) || std::abs(motion.frames[index].time - t) <= 1e-8) {
    Pose pose = motion.frames[index];
    pose.time = t;
    return pose;
  }
  const Pose& left = motion.frames[index];
  const Pose& right = motion.frames[index + 1];
  const double span = right.time - left.time;
  const double u = span > 1e-12 ? (t - left.time) / span : 0.0;
  Pose pose = interpolatePoses(left, right, u);
  pose.time = t;
  pose.contacts = left.contacts;
  return pose;
}

}  // namespace proc
