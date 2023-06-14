/**
* @Function: Single-differenced pseudorange positioning implementation
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
*
* Copyright (C) 2023 by Cheng Chi, All rights reserved.
**/
#pragma once

#include "gici/gnss/spp_estimator.h"
#include "gici/gnss/differential_measurement_align.h"

namespace gici {

// SDGNSS options
struct SdgnssEstimatorOptions {
  // Estimate velocity or not
  bool estimate_velocity = true;

  // Maximum age to apply difference
  double max_age = 20.0;
};

// Estimator
class SdgnssEstimator : public GnssEstimatorBase {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // The default constructor
  SdgnssEstimator(const SdgnssEstimatorOptions& options, 
               const GnssEstimatorBaseOptions& gnss_base_options, 
               const EstimatorBaseOptions& base_options);
  ~SdgnssEstimator();

  // Add measurement
  bool addMeasurement(const EstimatorDataCluster& measurement) override;

  // Add GNSS measurements and state
  bool addGnssMeasurementAndState(
    const GnssMeasurement& measurement_rov, 
    const GnssMeasurement& measurement_ref); 

  // Estimate current graph
  bool estimate() override;

protected:
  // Options
  SdgnssEstimatorOptions sdgnss_options_;
  double min_elevation_;
  bool estimate_velocity_;

  // SPP estimator to get initial states
  std::unique_ptr<SppEstimator> spp_estimator_;

  // Measurement alignment handle
  DifferentialMeasurementsAlign meausrement_align_;
};

}