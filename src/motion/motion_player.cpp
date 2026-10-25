#include "motion/motion_player.h"

#include <algorithm>
#include <cmath>

namespace proc {

void MotionPlayer::setMotion(const ProcMotion* motion) {
  motion_ = motion;
  playing_ = false;
  time_ = (motion_ != nullptr && !motion_->frames.empty()) ? motion_->frames.front().time : 0.0;
}

void MotionPlayer::setSpeed(double speed) {
  if (!std::isfinite(speed) || speed <= 0.0) speed = 1.0;
  speed_ = speed;
}

void MotionPlayer::setLoop(bool loop) { loop_ = loop; }
void MotionPlayer::play() { playing_ = motion_ != nullptr && !motion_->frames.empty(); }
void MotionPlayer::pause() { playing_ = false; }
void MotionPlayer::togglePlay() {
  if (playing_) pause();
  else play();
}

void MotionPlayer::seekSeconds(double time_s) {
  if (motion_ == nullptr || motion_->frames.empty()) {
    time_ = 0.0;
    return;
  }
  const double start = motion_->frames.front().time;
  const double end = motion_->frames.back().time;
  if (loop_ && motion_->duration_s > 1e-8) {
    double wrapped = std::fmod(time_s - start, motion_->duration_s);
    if (wrapped < 0.0) wrapped += motion_->duration_s;
    time_ = start + wrapped;
  } else {
    time_ = std::clamp(time_s, start, end);
  }
}

int MotionPlayer::frameAtOrBefore() const {
  if (motion_ == nullptr || motion_->frames.empty()) return 0;
  int index = 0;
  for (int frame = 0; frame < static_cast<int>(motion_->frames.size()); ++frame) {
    if (motion_->frames[frame].time <= time_ + 1e-9) index = frame;
    else break;
  }
  return index;
}

int MotionPlayer::frameIndex() const { return frameAtOrBefore(); }

void MotionPlayer::nextFrame() {
  if (motion_ == nullptr || motion_->frames.empty()) return;
  const int index = frameAtOrBefore();
  int next = index + 1;
  if (next >= static_cast<int>(motion_->frames.size())) next = loop_ ? 0 : index;
  time_ = motion_->frames[next].time;
}

void MotionPlayer::previousFrame() {
  if (motion_ == nullptr || motion_->frames.empty()) return;
  const int index = frameAtOrBefore();
  const bool on_frame = std::abs(motion_->frames[index].time - time_) <= 1e-6;
  int previous = on_frame ? index - 1 : index;
  if (previous < 0) previous = loop_ ? static_cast<int>(motion_->frames.size()) - 1 : 0;
  time_ = motion_->frames[previous].time;
}

void MotionPlayer::jumpStart() {
  if (motion_ == nullptr || motion_->frames.empty()) return;
  time_ = motion_->frames.front().time;
}

void MotionPlayer::jumpEnd() {
  if (motion_ == nullptr || motion_->frames.empty()) return;
  time_ = motion_->frames.back().time;
}

void MotionPlayer::update(double dt_s) {
  if (!playing_ || motion_ == nullptr || motion_->frames.empty()) return;
  if (!std::isfinite(dt_s) || dt_s < 0.0) return;
  time_ += dt_s * speed_;
  const double start = motion_->frames.front().time;
  const double end = motion_->frames.back().time;
  if (loop_ && motion_->duration_s > 1e-8) {
    double wrapped = std::fmod(time_ - start, motion_->duration_s);
    if (wrapped < 0.0) wrapped += motion_->duration_s;
    time_ = start + wrapped;
  } else if (time_ >= end) {
    time_ = end;
    playing_ = false;
  } else if (time_ < start) {
    time_ = start;
  }
}

Pose MotionPlayer::sample() const {
  if (motion_ == nullptr) return Pose{};
  return sampleMotion(*motion_, time_, loop_);
}

}  // namespace proc
