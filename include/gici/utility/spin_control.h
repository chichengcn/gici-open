/**
* @Function: Tick control for spinning
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
*
* Copyright (C) 2023 by Cheng Chi, All rights reserved.
**/
#pragma once

#include <iostream>
#include <vikit/timer.h>
#include <thread>

namespace gici {

// Spin control
class SpinControl {
public:
  SpinControl(double duration);
  ~SpinControl() { }

  // Sleep to ensure spinning rate and restart timer
  void sleep();

  // (Re)set loop duration
  void setDuration(double duration) { duration_ = duration; }

  // Check global status
  static bool ok() { 
    while (wait_) std::this_thread::sleep_for(std::chrono::nanoseconds(int(1e5)));
    return ok_; 
  }

  // Shutdown all spin controllers
  static void kill() { ok_ = false; }

  // All spin should wait
  static void wait() { wait_ = true; }

  // All spin end waiting
  static void run() { wait_ = false; }

private:
  // Tick controllers
  vk::Timer timer_;
  double duration_;

  // Whether enable spin in all objects
  static bool ok_;

  // If all spin should wait
  static bool wait_;
};

}
