/**
* @Function: Ceres iteration callback for debuging
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
*
* Copyright (C) 2023 by Cheng Chi, All rights reserved.
**/
#pragma once

#include <ceres/iteration_callback.h>

namespace gici {

class CeresDebugCallback : public ceres::IterationCallback {
public:
  explicit CeresDebugCallback() {}

  ~CeresDebugCallback() {}

  ceres::CallbackReturnType operator()(const ceres::IterationSummary& summary) {
    return handle(summary);
  }
 
private: 
  ceres::CallbackReturnType handle(const ceres::IterationSummary& summary);
};

}  // namespace gici
