/**
* @Function: Real-Time Kinematic implementation
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
*
* Copyright (C) 2023 by Cheng Chi, All rights reserved.
**/
#pragma once

#include "gici/gnss/gnss_estimator_base.h"
#include "gici/gnss/spp_estimator.h"
#include "gici/gnss/differential_measurement_align.h"

namespace gici {

// RTK options
struct RtkEstimatorOptions {
  // Max window length
  size_t max_window_length = 3;

  // Use ambiguity resolution
  bool use_ambiguity_resolution = true;

  // Estimate velocity or not
  bool estimate_velocity = true;

  // Maximum age to apply difference
  double max_age = 20.0;
};

// Estimator
class RtkEstimator : public GnssEstimatorBase {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  RtkEstimator(const RtkEstimatorOptions& options, 
               const GnssEstimatorBaseOptions& gnss_base_options, 
               const EstimatorBaseOptions& base_options,
               const AmbiguityResolutionOptions& ambiguity_options);
  ~RtkEstimator();

  // Add measurement
  bool addMeasurement(const EstimatorDataCluster& measurement) override;

  // Estimate current graph
  bool estimate() override;

  // Get state que
  const State getState() { return getLast(states_); }

protected:
  // Add GNSS measurements and state
  bool addGnssMeasurementAndState(
    const GnssMeasurement& measurement_rov, 
    const GnssMeasurement& measurement_ref);

  // Marginalization
  bool marginalization();

  // Shift memory for states and measurements
  inline void shiftMemory() {
    states_.push_back(State());
    ambiguity_states_.push_back(AmbiguityState());
    gnss_measurement_pairs_.push_back(
      std::make_pair(GnssMeasurement(), GnssMeasurement()));
    while (states_.size() > rtk_options_.max_window_length) {
      states_.pop_front();
      ambiguity_states_.pop_front();
      gnss_measurement_pairs_.pop_front();
    }
  } 

protected:
  // Options
  RtkEstimatorOptions rtk_options_;
  AmbiguityResolutionOptions ambiguity_options_;

  // SPP estimator to get initial states
  std::unique_ptr<SppEstimator> spp_estimator_;

  // Measurement alignment handle
  DifferentialMeasurementsAlign meausrement_align_;

  // Status control
  int num_continuous_unfix_ = 0;
  int num_cotinuous_reject_gnss_ = 0;
};

}