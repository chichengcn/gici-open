/**
* @Function: Phase bias handler
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
*
* Copyright (C) 2023 by Cheng Chi, All rights reserved.
**/
#include "gici/gnss/phase_bias.h"

#include <vector>
#include <algorithm>
#include <Eigen/Core>

#include "gici/gnss/gnss_common.h"

namespace gici {

// Set zero-difference phase bias
void PhaseBias::setPhaseBias(const std::string prn, 
  const int phase_id, const double value)
{
  CHECK(prn.size() == 3);

  mutex_.lock();
  if (biases_.find(prn) == biases_.end()) {
    biases_.insert(std::make_pair(prn, std::unordered_map<int, double>()));
  }
  std::unordered_map<int, double>& phase_id_to_bias = biases_.at(prn);
  if (phase_id_to_bias.find(phase_id) == phase_id_to_bias.end()) {
    phase_id_to_bias.insert(std::make_pair(phase_id, value));
  }
  else {
    // check if consecutive
    double wavelength = CLIGHT / gnss_common::phaseToFrequency(prn[0], phase_id);
    double cycle = value / wavelength;
    double last_cycle = phase_id_to_bias.at(phase_id) / wavelength;
    while (last_cycle - cycle > 0.5) cycle += 1.0;
    while (cycle - last_cycle > 0.5) cycle -= 1.0;
    phase_id_to_bias.at(phase_id) = cycle * wavelength;
  }
  mutex_.unlock();
}

// Get phase bias correction
double PhaseBias::getPhaseBias(const std::string prn, const int phase_id)
{
  mutex_.lock();
  if (biases_.find(prn) != biases_.end()) {
    std::unordered_map<int, double> biases_sat = biases_.at(prn);
    if (biases_sat.find(phase_id) != biases_sat.end()) {
      double bias = biases_sat.at(phase_id);
      // we need check if it is zero (invalid) outside
      if (bias == 0.0) bias = 1e-4;
      mutex_.unlock();
      return bias;
    }
  }
  mutex_.unlock();

  return 0.0;
}

}