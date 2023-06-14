/**
* @Function: SPP/IMU tightly couple estimator
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
*
* Copyright (C) 2023 by Cheng Chi, All rights reserved.
**/
#pragma once

#include "gici/gnss/gnss_estimator_base.h"
#include "gici/imu/imu_estimator_base.h"
#include "gici/gnss/spp_estimator.h"
#include "gici/fusion/gnss_imu_initializer.h"

namespace gici {

// SPP/IMU tightly couple options
struct SppImuTcEstimatorOptions {
  // Max window length
  size_t max_window_length = 10;
};

// Estimator
class SppImuTcEstimator : 
  public GnssEstimatorBase, 
  public ImuEstimatorBase {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  SppImuTcEstimator(const SppImuTcEstimatorOptions& options, 
               const GnssImuInitializerOptions& init_options, 
               const SppEstimatorOptions spp_options, 
               const GnssEstimatorBaseOptions& gnss_base_options, 
               const GnssLooseEstimatorBaseOptions& gnss_loose_base_options, 
               const ImuEstimatorBaseOptions& imu_base_options,
               const EstimatorBaseOptions& base_options);
  ~SppImuTcEstimator();

  // Add measurement
  bool addMeasurement(const EstimatorDataCluster& measurement) override;

  // Estimate current graph
  bool estimate() override;

  // Set initializatin result
  void setInitializationResult(
    const std::shared_ptr<MultisensorInitializerBase>& initializer) override;

protected:
  // Add GNSS measurements and state
  bool addGnssMeasurementAndState(const GnssMeasurement& measurement);

  // Marginalization
  bool marginalization();

  // Shift memory for states and measurements
  inline void shiftMemory() {
    states_.push_back(State());
    gnss_measurements_.push_back(GnssMeasurement());
    while (states_.size() > tc_options_.max_window_length) {
      states_.pop_front();
      gnss_measurements_.pop_front();
    }
  } 

protected:
  // Options
  SppImuTcEstimatorOptions tc_options_;
  SppEstimatorOptions spp_options_;

  // Initialization control
  std::shared_ptr<GnssImuInitializer> initializer_;
  std::shared_ptr<SppEstimator> initializer_sub_estimator_;

  // Status control
  int num_cotinuous_reject_gnss_ = 0;
};

}