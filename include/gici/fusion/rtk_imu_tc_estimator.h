/**
* @Function: RTK/IMU tightly couple estimator
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
*
* Copyright (C) 2023 by Cheng Chi, All rights reserved.
**/
#pragma once

#include "gici/gnss/gnss_estimator_base.h"
#include "gici/imu/imu_estimator_base.h"
#include "gici/gnss/rtk_estimator.h"
#include "gici/fusion/gnss_imu_initializer.h"

namespace gici {

// RTK/IMU tightly couple options
struct RtkImuTcEstimatorOptions {
  // Max window length
  size_t max_window_length = 10;
};

// Estimator
class RtkImuTcEstimator : 
  public GnssEstimatorBase, 
  public ImuEstimatorBase {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  RtkImuTcEstimator(const RtkImuTcEstimatorOptions& options, 
               const GnssImuInitializerOptions& init_options, 
               const RtkEstimatorOptions rtk_options, 
               const GnssEstimatorBaseOptions& gnss_base_options, 
               const GnssLooseEstimatorBaseOptions& gnss_loose_base_options, 
               const ImuEstimatorBaseOptions& imu_base_options,
               const EstimatorBaseOptions& base_options,
               const AmbiguityResolutionOptions& ambiguity_options);
  ~RtkImuTcEstimator();

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
    while (states_.size() > tc_options_.max_window_length) {
      states_.pop_front();
      ambiguity_states_.pop_front();
      gnss_measurement_pairs_.pop_front();
    }
  } 

protected:
  // Options
  RtkImuTcEstimatorOptions tc_options_;
  RtkEstimatorOptions rtk_options_;

  // Measurement alignment handle
  DifferentialMeasurementsAlign meausrement_align_;

  // Initialization control
  std::shared_ptr<GnssImuInitializer> initializer_;
  std::shared_ptr<RtkEstimator> initializer_sub_estimator_;

  // Status control
  int num_continuous_unfix_ = 0;
  int num_cotinuous_reject_gnss_ = 0;
};

}