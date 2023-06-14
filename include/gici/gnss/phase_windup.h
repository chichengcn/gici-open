/**
* @Function: Phase wind-up handler
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
#include <Eigen/Core>

namespace gici {

// Phase wind-up handle
class PhaseWindup {
public:
  PhaseWindup() {}
  ~PhaseWindup() { }

  // Get phase-windup
  double get(const double time, const std::string& prn, 
    const Eigen::Vector3d& satellite_position,
    const Eigen::Vector3d& receiver_position);

protected:
  // Phase windup model
  void windupcorr(const double time, 
    const double* rs, const double* rr, double* phw);

private:
  // Phase wind-ups
  std::unordered_map<std::string, double> windups_;

  // Locker
  std::mutex mutex_;
};

using PhaseWindupPtr = std::shared_ptr<PhaseWindup>;

} // namespace gici