/**
* @Function: Tick control for spinning
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
*
* Copyright (C) 2023 by Cheng Chi, All rights reserved.
**/
#include "gici/utility/spin_control.h"

namespace gici {

// Static variable
bool SpinControl::ok_ = true;
bool SpinControl::wait_ = true;

SpinControl::SpinControl(double duration) : duration_(duration)
{
  timer_ = vk::Timer();
}

// Sleep to ensure spinning rate and restart timer
void SpinControl::sleep()
{
  double duration = timer_.stop();
  if (duration < duration_) {
    int nano = static_cast<int>((duration_ - duration) * 1e9);
    std::this_thread::sleep_for(std::chrono::nanoseconds(nano));
  }
  timer_.resume();
}

}