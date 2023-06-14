/**
* @Function: Phase bias handler
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
#include <mutex>

#include "gici/utility/rtklib_safe.h"

namespace gici {

// Phase bias handle
class PhaseBias {
public:
  // Map from PRN and code type to bias
  using BiasMap = std::unordered_map<std::string, std::unordered_map<int, double>>;

  PhaseBias() {}
  ~PhaseBias() { }

  // Set zero-difference phase bias
  void setPhaseBias(const std::string prn, 
    const int phase_id, const double value);

  // Get phase bias correction
  // Unlike code bias, we do not need to arrange to base frequencies because we will 
  // apply Between-Satellite Difference (BSD) before we solve integer ambiguity.
  double getPhaseBias(const std::string prn, const int phase_id);

  // If we have phase biases
  const bool valid() const { return biases_.size() > 0; }

private:
  BiasMap biases_;

  // Locker
  std::mutex mutex_;
};

using PhaseBiasPtr = std::shared_ptr<PhaseBias>;

} // namespace gici