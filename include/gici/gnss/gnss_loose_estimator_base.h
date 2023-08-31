/**
* @Function: Base class for GNSS loose loosely coupling with other sensors
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
*
* Copyright (C) 2023 by Cheng Chi, All rights reserved.
**/
#pragma once

#include "gici/estimate/estimator_base.h"
#include "gici/gnss/gnss_types.h"
#include "gici/gnss/gnss_common.h"
#include "gici/utility/common.h"

namespace gici {

// GNSS loose estimator common options
struct GnssLooseEstimatorBaseOptions {
  // Use Fault Detection and Exclusion (FDE)
  bool use_outlier_rejection = true;

  // Maximum position error to exclude (m)
  double max_position_error = 100.0;

  // Maximum velocity error to exclude (m/s)
  double max_velocity_error = 10.0;

  // Minimum number of continuous large amount rejection to be considered as divergence
  size_t diverge_min_num_continuous_reject = 10;
};

// Estimator
class GnssLooseEstimatorBase : public virtual EstimatorBase {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  GnssLooseEstimatorBase(const GnssLooseEstimatorBaseOptions& options,
                    const EstimatorBaseOptions& base_options);
  ~GnssLooseEstimatorBase();

  // Add GNSS solution measurement and state
  virtual bool addGnssSolutionMeasurementAndState(
    const GnssSolution& measurement)
  { return false; }

  // Convert solution to GNSS solution
  GnssSolution convertSolutionToGnssSolution(
    const Solution& solution, const SolutionRole& role);

  // Compute coarse velocity from relative position
  bool computeCoarseVelocityFromPosition(
    const GnssSolution& last_gnss_solution,
    const GnssSolution& cur_gnss_solution, 
    Eigen::Vector3d& velocity);

  // Get solution status
  inline GnssSolutionStatus getSolutionStatus() { return lastState().status; }

  // Get number of satellite
  inline int getNumberSatellite() { return lastGnssSolution().num_satellites; }

  // Get differential age
  inline int getDifferentialAge() { return lastGnssSolution().differential_age; } 

protected:
  // Add GNSS extrinsics block to graph
  BackendId addGnssExtrinsicsParameterBlock(
    const int32_t id, 
    const Eigen::Vector3d& t_SR_S_prior);

  // Add position block to graph
  void addGnssPositionResidualBlock(
    const GnssSolution& measurement,
    const State& state);

  // Add velocity block to graph
  void addGnssVelocityResidualBlock(
    const GnssSolution& measurement,
    const State& state, 
    const Eigen::Vector3d& angular_velocity);

  // Add initial GNSS extrinsics error
  void addGnssExtrinsicsResidualBlock(
    const BackendId& gnss_extrinsics_id, 
    const Eigen::Vector3d& t_SR_S_prior, 
    const Eigen::Vector3d& std);

  // Reject position and velocity outliers
  bool rejectGnssPositionAndVelocityOutliers(const State& state);

  // Add GNSS position residual block to marginalizer
  void addGnssPositionResidualMarginBlock(const State& state);

  // Add GNSS velocity residual block to marginalizer
  void addGnssVelocityResidualMarginBlock(const State& state);

  // Add all GNSS loosely coupled residual blocks to marginalier
  void addGnssLooseResidualMarginBlocks(const State& state);

  // Erase GNSS extrinsics
  void eraseGnssExtrinsicsParameterBlock(BackendId& extrinsics_id);

  // Erase GNSS position and velocity residual block
  void eraseGnssLooseResidualBlocks(const State& state);

  // Get extrinsics estimate
  Eigen::Vector3d getGnssExtrinsicsEstimate();

  // Get current GNSS solution measurement
  inline GnssSolution& curGnssSolution() { 
    return getCurrent(gnss_solution_measurements_); 
  }

  // Get last GNSS solution measurement
  inline GnssSolution& lastGnssSolution() { 
    return getLast(gnss_solution_measurements_); 
  }

  // Get oldest GNSS solution measurement
  inline GnssSolution& oldestGnssSolution() { 
    return getOldest(gnss_solution_measurements_); 
  }

  // Get latest GNSS state
  inline State& latestGnssState() {
    for (auto it = states_.rbegin(); it != states_.rend(); it++) {
      State& state = *it;
      if (!state.valid()) continue;
      if (state.id.type() == IdType::gPose || 
          state.id.type() == IdType::gPosition) return state;
    }
    return null_state_;
  }

  // Count size of the GNSS states
  inline int sizeOfGnssStates() {
    return std::count_if(states_.begin(), states_.end(), 
      [](State& state) { return state.valid() && state.id.type() == IdType::gPose; });
  }

protected:
  // Options
  GnssLooseEstimatorBaseOptions gnss_loose_base_options_;

  // Measurements
  std::deque<GnssSolution> gnss_solution_measurements_;

  // States
  BackendId gnss_extrinsics_id_;
};

}