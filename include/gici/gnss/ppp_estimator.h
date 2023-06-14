/**
* @Function: Precise Point positioning implementation
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
*
* Copyright (C) 2023 by Cheng Chi, All rights reserved.
**/
#pragma once

#include "gici/gnss/gnss_estimator_base.h"

#include "gici/gnss/spp_estimator.h"

namespace gici {

// PPP options
struct PppEstimatorOptions {
  // Max window length
  size_t max_window_length = 10;

  // Use ambiguity resolution
  bool use_ambiguity_resolution = false;

  // Estimate velocity or not
  bool estimate_velocity = true;
};

// Estimator
class PppEstimator : public GnssEstimatorBase {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  PppEstimator(const PppEstimatorOptions& options, 
               const GnssEstimatorBaseOptions& gnss_base_options, 
               const EstimatorBaseOptions& base_options,
               const AmbiguityResolutionOptions& ambiguity_options);
  ~PppEstimator();

  // Add measurement
  bool addMeasurement(const EstimatorDataCluster& measurement) override;

  // Estimate current graph
  bool estimate() override;

  // Log intermediate data
  void logIntermediateData() override;

protected:
  // Add GNSS measurements and state
  bool addGnssMeasurementAndState(const GnssMeasurement& measurement);

  // Marginalization
  bool marginalization();

  // Shift memory for states and measurements
  inline void shiftMemory() {
    states_.push_back(State());
    ionosphere_states_.push_back(IonosphereState());
    ambiguity_states_.push_back(AmbiguityState());
    gnss_measurements_.push_back(GnssMeasurement());
    while (states_.size() > ppp_options_.max_window_length) {
      states_.pop_front();
      ionosphere_states_.pop_front();
      ambiguity_states_.pop_front();
      gnss_measurements_.pop_front();
    }
  } 

protected:
  // Options
  PppEstimatorOptions ppp_options_;

  // SPP estimator to get initial states
  std::unique_ptr<SppEstimator> spp_estimator_;

  // Phase wind-up handle
  PhaseWindupPtr phase_windup_;

  // Status control
  int num_cotinuous_reject_gnss_ = 0;
};

}