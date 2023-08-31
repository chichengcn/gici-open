/**
* @Function: GNSS/IMU loosely couple estimator
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
*
* Copyright (C) 2023 by Cheng Chi, All rights reserved.
**/
#pragma once

#include "gici/gnss/gnss_loose_estimator_base.h"
#include "gici/imu/imu_estimator_base.h"
#include "gici/fusion/gnss_imu_initializer.h"

namespace gici {

// GNSS/IMU loosely couple options
struct GnssImuLcEstimatorOptions {
  // Max window length
  size_t max_window_length = 10;
};

// Estimator
class GnssImuLcEstimator : 
  public GnssLooseEstimatorBase, 
  public ImuEstimatorBase {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  GnssImuLcEstimator(const GnssImuLcEstimatorOptions& options, 
               const GnssImuInitializerOptions& init_options, 
               const GnssLooseEstimatorBaseOptions& gnss_loose_base_options, 
               const ImuEstimatorBaseOptions& imu_base_options,
               const EstimatorBaseOptions& base_options);
  ~GnssImuLcEstimator();

  // Add measurement
  bool addMeasurement(const EstimatorDataCluster& measurement) override;

  // Estimate current graph
  bool estimate() override;

  // Set initializatin result
  void setInitializationResult(
    const std::shared_ptr<MultisensorInitializerBase>& initializer) override;

protected:
  // Add GNSS measurements and state
  bool addGnssSolutionMeasurementAndState(const GnssSolution& measurement);

  // Marginalization
  bool marginalization();

  // Shift memory for states and measurements
  inline void shiftMemory() {
    states_.push_back(State());
    gnss_solution_measurements_.push_back(GnssSolution());
    while (states_.size() > lc_options_.max_window_length) {
      states_.pop_front();
      gnss_solution_measurements_.pop_front();
    }
  } 

protected:
  // Options
  GnssImuLcEstimatorOptions lc_options_;

  // Initialization control
  std::shared_ptr<GnssImuInitializer> initializer_;

  // Status control
  int num_cotinuous_reject_gnss_ = 0;
};

}