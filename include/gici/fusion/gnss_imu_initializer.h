/**
* @Function: GNSS/IMU initialization
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
*
* Copyright (C) 2023 by Cheng Chi, All rights reserved.
**/
#pragma once

#include "gici/fusion/multisensor_initializer_base.h"
#include "gici/gnss/gnss_loose_estimator_base.h"
#include "gici/imu/imu_estimator_base.h"

namespace gici {

// IMU initialization options
struct GnssImuInitializerOptions {
  // Max iteration number for ceres optimization
  int max_iteration = 30;

  // Number of threads used for ceres optimization
  int num_threads = 4;

  // Maximum time in second for which the optimizer should run for
  double max_solver_time = 0.5;

  // We should keep stady during this period to initialize roll, pitch and angular rate bias.
  double time_window_length_slow_motion = 0.1;

  // We should do a dynamic motion after the slow motion initilization has finished, the time 
  // window will be used for bundle adjustment.
  double time_window_length_dynamic_motion = 0.5;

  // At least one state should have horizontal acceleration larger than this in the dynamic motion 
  // window, we need a relatively large acceleration to ensure the observability of yaw attitude.
  double min_acceleration = 0.5;

  // Relative position from IMU to GNSS in IMU frame
  Eigen::Vector3d gnss_extrinsics = Eigen::Vector3d::Zero();

  // GNSS extrinsics initial variance
  Eigen::Vector3d gnss_extrinsics_initial_std = Eigen::Vector3d::Zero();
};

// Estimator
class GnssImuInitializer : 
  public MultisensorInitializerBase, 
  public GnssLooseEstimatorBase,
  public ImuEstimatorBase {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  GnssImuInitializer(const GnssImuInitializerOptions& options, 
               const GnssLooseEstimatorBaseOptions& gnss_loose_base_options,
               const ImuEstimatorBaseOptions& imu_base_options,
               const EstimatorBaseOptions& base_options,
               const std::shared_ptr<Graph>& graph, 
               const std::shared_ptr<EstimatorBase>& sub_gnss_estimator = nullptr);
  ~GnssImuInitializer() {}

  // Add measurement
  bool addMeasurement(const EstimatorDataCluster& measurement) override;

  // Apply initialization
  bool estimate() override;

  // Arrange the initialization results to estimator
  bool arrangeToEstimator(const int window_length,
            const std::shared_ptr<MarginalizationError>& marginalization_error,
            std::deque<State>& states, 
            ceres::ResidualBlockId& marginalization_residual_id,
            BackendId& gnss_extrinsics_id,
            std::deque<GnssSolution>& gnss_solution_measurements,
            ImuMeasurements& imu_measurements);

protected:
  // Add GNSS solution measurements
  bool addGnssSolutionMeasurement(const GnssSolution& measurement);

  // Get initial pitch, roll, and anguler rate bias under slow motion
  void slowMotionInitialization();

  // Put state and measurements to graph with given initial values
  void putMeasurementAndStateToGraph(
    const Eigen::Quaterniond& q_WS_0, const SpeedAndBias& speed_and_bias_0);

protected:
  // Options
  GnssImuInitializerOptions options_;

  // flag
  bool zero_motion_finished_ = false;
  bool dynamic_window_full_ = false;
  bool has_any_velocity_measurement_ = false;

  // initialized by zero motion
  Transformation T_WS_0_;
  SpeedAndBias speed_and_bias_0_;
};

}