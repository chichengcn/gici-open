/**
* @Function: Base class of estimator
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
*
* Copyright (C) 2023 by Cheng Chi, All rights reserved.
**/
#pragma once

#include <memory>

#include "gici/estimate/graph.h"
#include "gici/estimate/estimator_types.h"
#include "gici/estimate/ceres_iteration_callback.h"
#include "gici/estimate/marginalization_error.h"
#include "gici/utility/common.h"

namespace gici {

// Declares
class MultisensorInitializerBase;

// Options
struct EstimatorBaseOptions {
  // Max iteration number for ceres optimization
  int max_iteration = 10;

  // Number of threads used for ceres optimization
  int num_threads = 2;

  // Maximum time in second for which the optimizer should run for
  double max_solver_time = 0.05;

  // Ceres solver type
  ceres::LinearSolverType solver_type = ceres::DENSE_SCHUR;

  // Ceres trust region strategy type
  ceres::TrustRegionStrategyType trust_region_strategy_type = ceres::DOGLEG;

  // Verbose optimization output
  bool verbose_output = false;

  // Forcely set initial position in global frame
  bool force_initial_global_position = false;

  // If force_initial_global_position = ture, this parameter sets the initial
  // position in LLA coordinate (in deg).
  Eigen::Vector3d initial_global_position = Eigen::Vector3d::Zero();

  // If we should compute covariance
  bool compute_covariance = true;

  // Log estimator intermediate information
  bool log_intermediate_data = false;

  // Log estiamtor intermediate information to this directory if enabled
  std::string log_intermediate_data_directory = "";
};

// Estimator
class EstimatorBase {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EstimatorBase(const EstimatorBaseOptions& options);
  ~EstimatorBase();

  // Add measurement
  // The return value tells outter function if it can call estimate().
  virtual bool addMeasurement(const EstimatorDataCluster& measurement) = 0;

  // Estimate current graph
  virtual bool estimate() = 0;

  // Set coordinate. If GNSS is invalid, just set an arbitrary one.
  void setCoordinate(const GeoCoordinatePtr coordinate) { 
    coordinate_ = coordinate;
  }

  // Set initializatin result
  virtual void setInitializationResult(
    const std::shared_ptr<MultisensorInitializerBase>& initializer)
  {}

  // Get graph pointer
  const std::shared_ptr<Graph>& getGraph() const { return graph_; }

  // Get timestamp
  double getTimestamp() { return lastState().timestamp; }

  // Get oldest timestamp
  double getOldestTimestamp() { return oldestState().timestamp; }

  // Get coordinate
  GeoCoordinatePtr getCoordinate() { return coordinate_; }

  // Get latest pose
  Transformation getPoseEstimate();

  // Get pose estimate at given timestmap
  virtual bool getPoseEstimateAt(
    const double timestamp, Transformation& T_WS);

  // Get latest speed and bias (for estimator types that do not contain IMU, the biases
  // will be setted as zeros).
  SpeedAndBias getSpeedAndBiasEstimate();

  // Get speed and bias estimate at given timestmap
  virtual bool getSpeedAndBiasEstimateAt(
    const double timestamp, SpeedAndBias& speed_and_bias);

  // Get latest minimal covariance 
  // (in position, attitude, speed, bias of gyro, bias of acc order)
  Eigen::Matrix<double, 15, 15> getCovariance();

  // Get covariance at given timestamp
  virtual bool getCovarianceAt(
    const double timestamp, Eigen::Matrix<double, 15, 15>& covariance);

  // Get estimator status
  EstimatorStatus getStatus() { return status_; }

  // Log intermediate data
  virtual void logIntermediateData() {}

  // Check if it is the first epoch
  bool isFirstEpoch() { return states_.size() < 2; }

protected:
  // Apply ceres optimization
  virtual void optimize();

  // Erase old marginalization item
  bool eraseOldMarginalization();

  // Apply new marginalization
  // To apply marginalization, call eraseOldMarginalization first. Then add parameter 
  // blocks to marginalizer using add...MarginBlock(s) functions defined by sensor base 
  // classes. And finally call this function, which computes linearized marginaliztion 
  // items and add them into graph.
  bool applyMarginalization();

  // Get pose estimate at a given state
  Transformation getPoseEstimate(const State& state);

  // Get latest speed and bias at a given state
  SpeedAndBias getSpeedAndBiasEstimate(const State& state);

  // Get minimal covariance at a given state. 
  Eigen::Matrix<double, 15, 15> getCovariance(const State& state);

  // Compute and get minimal covariance at a given state
  Eigen::Matrix<double, 15, 15> computeAndGetCovariance(const State& state);

  // Update covariance storage
  void updateCovariance(const State& state);

  // Convert from estimated states (in ENU) to body states
  virtual void convertStateAndCovarianceToBody(
    Transformation* T_WS, SpeedAndBias* speed_and_bias, 
    Eigen::Matrix<double, 15, 15>* covariance) {
    // In default, no convertion.
    return;
  }

  // Get current state
  inline State& curState() { return getCurrent(states_); }

  // Get last state
  inline State& lastState() { return getLast(states_); }

  // Get oldest state
  inline State& oldestState() { return getOldest(states_); }

  // Get latest state. Latest states are not always pushed to back.
  inline virtual State& latestState() { return curState(); }

protected:
  // Graph that handles residuals and states
  std::shared_ptr<Graph> graph_;

  // Options
  EstimatorBaseOptions base_options_;
  EstimatorType type_;

  // loss function 
  std::shared_ptr<ceres::LossFunction> cauchy_loss_function_; 
  std::shared_ptr<ceres::LossFunction> huber_loss_function_; 

  // States
  std::deque<State> states_; // the state memory should be shifted every time
                             // a estimate step has been processed!
  State null_state_;  // just for check
  size_t latest_state_index_;
  // we compute covariance every time an optimization has finished, this is to
  // avoid thread conflict when we call compteCovariance while the other threads
  // are operating the graph.
  std::map<double, Eigen::Matrix<double, 15, 15>> covariances_;  
  bool can_compute_covariance_ = false;

  // Coordinate handle
  GeoCoordinatePtr coordinate_;

  // Status
  EstimatorStatus status_;

  // the marginalized error term
  std::shared_ptr<MarginalizationError> marginalization_error_;
  ceres::ResidualBlockId marginalization_residual_id_;
  std::vector<BackendId> marginalization_parameter_ids_;
  std::vector<bool> marginalization_keep_parameter_blocks_;

  // For Debug
  std::unique_ptr<CeresDebugCallback> debug_callback_;

};

}