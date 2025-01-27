/**
* @Function: Single Point positioning implementation
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
*
* Copyright (C) 2023 by Cheng Chi, All rights reserved.
**/
#pragma once

#include "gici/gnss/gnss_estimator_base.h"

namespace gici {

// SPP options
struct SppEstimatorOptions {
  // Estimate velocity or not
  bool estimate_velocity = true;

  // Whether to use dual-frequency
  bool use_dual_frequency = false;
};

// Estimator
class SppEstimator : public GnssEstimatorBase {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // Contructor with full options
  SppEstimator(const SppEstimatorOptions& options, 
               const GnssEstimatorBaseOptions& gnss_base_options, 
               const EstimatorBaseOptions& base_options);

  // Contructor with GNSS estimator and SPP options. Other options are setted as defualt
  SppEstimator(const SppEstimatorOptions& options, 
               const GnssEstimatorBaseOptions& gnss_base_options);

  // Contructor with GNSS estimator options. Other options are setted as defualt
  SppEstimator(const GnssEstimatorBaseOptions& gnss_base_options);

  ~SppEstimator();

  // Set as PPP mode
  // to enable the estimated parameters are self consistent with PPP
  void setPPPMode() { 
    is_ppp_ = true; 
    spp_options_.use_dual_frequency = true;
  }

  // Add measurement
  bool addMeasurement(const EstimatorDataCluster& measurement) override;

  // Add GNSS measurements and state
  bool addGnssMeasurementAndState(const GnssMeasurement& measurement); 

  // Estimate current graph
  bool estimate() override;

  // Get position estimate in ECEF
  inline Eigen::Vector3d getPositionEstimate() {
    return getPositionEstimate(lastState());
  }

  // Get clock estimate
  inline std::map<char, double> getClockEstimate() {
    return getClockEstimate(lastState());
  }

  // Get velocity estimate in ECEF
  inline Eigen::Vector3d getVelocityEstimate() {
    return getVelocityEstimate(lastState());
  }

  // Get frequency estimate
  inline std::map<char, double> getFrequencyEstimate() {
    return getFrequencyEstimate(lastState());
  }

  // Get velocity covariance in ECEF
  inline Eigen::Matrix3d getVelocityCovariance() {
    return getVelocityCovariance(lastState());
  }

protected:
  // Erase non-base-frequency measurements and non-dual-frequency satellites
  void arrangeDualFrequency(GnssMeasurement& measurement);

  // Get position estimate in ECEF
  Eigen::Vector3d getPositionEstimate(const State& state);

  // Get clock estimate
  std::map<char, double> getClockEstimate(const State& state);

  // Get velocity estimate in ECEF
  Eigen::Vector3d getVelocityEstimate(const State& state);

  // Get frequency estimate
  std::map<char, double> getFrequencyEstimate(const State& state);

  // Get velocity covariance in ECEF
  Eigen::Matrix3d getVelocityCovariance(const State& state);

protected:
  // Options
  SppEstimatorOptions spp_options_;
};

}