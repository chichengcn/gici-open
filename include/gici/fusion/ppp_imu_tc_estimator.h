/**
* @Function: PPP/IMU tightly couple estimator
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
*
* Copyright (C) 2023 by Cheng Chi, All rights reserved.
**/
#pragma once

#include "gici/gnss/gnss_estimator_base.h"
#include "gici/imu/imu_estimator_base.h"
#include "gici/gnss/ppp_estimator.h"
#include "gici/fusion/gnss_imu_initializer.h"

namespace gici {

// PPP/IMU tightly couple options
struct PppImuTcEstimatorOptions {
  // Max window length
  size_t max_window_length = 10;
};

// Estimator
class PppImuTcEstimator : 
  public GnssEstimatorBase, 
  public ImuEstimatorBase {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  PppImuTcEstimator(const PppImuTcEstimatorOptions& options, 
               const GnssImuInitializerOptions& init_options, 
               const PppEstimatorOptions ppp_options, 
               const GnssEstimatorBaseOptions& gnss_base_options, 
               const GnssLooseEstimatorBaseOptions& gnss_loose_base_options, 
               const ImuEstimatorBaseOptions& imu_base_options,
               const EstimatorBaseOptions& base_options,
               const AmbiguityResolutionOptions& ambiguity_options);
  ~PppImuTcEstimator();

  // Add measurement
  bool addMeasurement(const EstimatorDataCluster& measurement) override;

  // Estimate current graph
  bool estimate() override;

  // Set initializatin result
  void setInitializationResult(
    const std::shared_ptr<MultisensorInitializerBase>& initializer) override;

protected:
  // Add GNSS measurements and state
  bool addGnssMeasurementAndState(
    const GnssMeasurement& measurement);

  // Marginalization
  bool marginalization();

  // Shift memory for states and measurements
  inline void shiftMemory() {
    states_.push_back(State());
    ambiguity_states_.push_back(AmbiguityState());
    ionosphere_states_.push_back(IonosphereState());
    gnss_measurements_.push_back(GnssMeasurement());
    while (states_.size() > tc_options_.max_window_length) {
      states_.pop_front();
      ambiguity_states_.pop_front();
      ionosphere_states_.pop_front();
      gnss_measurements_.pop_front();
    }
  } 

protected:
  // Options
  PppImuTcEstimatorOptions tc_options_;
  PppEstimatorOptions ppp_options_;

  // SPP estimator to get initial states
  std::unique_ptr<SppEstimator> spp_estimator_;

  // Phase wind-up handle
  PhaseWindupPtr phase_windup_;

  // Initialization control
  std::shared_ptr<GnssImuInitializer> initializer_;
  std::shared_ptr<PppEstimator> initializer_sub_estimator_;

  // Status control
  int num_continuous_unfix_ = 0;
  int num_cotinuous_reject_gnss_ = 0;
};

}