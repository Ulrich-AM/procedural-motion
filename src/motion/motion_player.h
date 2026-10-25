#pragma once

#include "motion/interpolation.h"

namespace proc {

class MotionPlayer {
 public:
  void setMotion(const ProcMotion* motion);
  void setSpeed(double speed);
  void setLoop(bool loop);
  void play();
  void pause();
  void togglePlay();
  void seekSeconds(double time_s);
  void nextFrame();
  void previousFrame();
  void jumpStart();
  void jumpEnd();
  void update(double dt_s);

  const ProcMotion* motion() const { return motion_; }
  double time() const { return time_; }
  double speed() const { return speed_; }
  bool loop() const { return loop_; }
  bool playing() const { return playing_; }
  int frameIndex() const;
  Pose sample() const;

 private:
  const ProcMotion* motion_ = nullptr;
  double time_ = 0.0;
  double speed_ = 1.0;
  bool loop_ = true;
  bool playing_ = false;

  int frameAtOrBefore() const;
};

}  // namespace proc
