#include "locomotion/locomotion_diagnostics.h"

#include "rig/forward_kinematics.h"

#include <algorithm>
#include <cmath>

namespace proc {

FootSlipReport measureContactSlip(const ProcBody& body, const ProcMotion& motion) {
  FootSlipReport report;
  if (motion.frames.size() < 2) return report;
  const int left = jointIndex(JointId::LeftFoot);
  const int right = jointIndex(JointId::RightFoot);
  double left_sum = 0.0;
  double right_sum = 0.0;
  glm::dvec3 previous_left = forwardKinematics(body, motion.frames.front()).joints[left].position;
  glm::dvec3 previous_right = forwardKinematics(body, motion.frames.front()).joints[right].position;
  for (std::size_t index = 1; index < motion.frames.size(); ++index) {
    const Pose& pose = motion.frames[index];
    const double dt = pose.time - motion.frames[index - 1].time;
    if (dt <= 1e-8) continue;
    const WorldPose world = forwardKinematics(body, pose);
    auto slip = [&](bool down, const glm::dvec3& current, glm::dvec3& previous, double& sum, double& peak, int& count) {
      if (down) {
        const glm::dvec3 delta = current - previous;
        const double speed = std::sqrt(delta.x * delta.x + delta.y * delta.y) / dt;
        sum += speed;
        peak = std::max(peak, speed);
        ++count;
      }
      previous = current;
    };
    const bool left_down = (pose.contacts.left_heel && *pose.contacts.left_heel) || (pose.contacts.left_toe && *pose.contacts.left_toe);
    const bool right_down = (pose.contacts.right_heel && *pose.contacts.right_heel) || (pose.contacts.right_toe && *pose.contacts.right_toe);
    slip(left_down, world.joints[left].position, previous_left, left_sum, report.left_peak_mps, report.left_samples);
    slip(right_down, world.joints[right].position, previous_right, right_sum, report.right_peak_mps, report.right_samples);
  }
  if (report.left_samples > 0) report.left_mean_mps = left_sum / static_cast<double>(report.left_samples);
  if (report.right_samples > 0) report.right_mean_mps = right_sum / static_cast<double>(report.right_samples);
  return report;
}

}  // namespace proc
