/**
* @Function: Phase center handler
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
*
* Copyright (C) 2023 by Cheng Chi, All rights reserved.
**/
#pragma once

#include <memory>
#include <map>
#include <unordered_map>

#include "gici/utility/rtklib_safe.h"

namespace gici {

// Phase center handle
class PhaseCenter {
public:
  PhaseCenter(pcv_t *pcvs) : pcvs_(pcvs) {}
  ~PhaseCenter() {}

private:
  pcv_t *pcvs_;
};

using PhaseCenterPtr = std::shared_ptr<PhaseCenter>;

} // namespace gici