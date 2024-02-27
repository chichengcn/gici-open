/**
* @Function: GNSS/IMU initialization
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
*
* Copyright (C) 2023 by Cheng Chi, All rights reserved.
**/
#include "gici/fusion/gnss_imu_initializer.h"
#include "gici/gnss/gnss_estimator_base.h"
#include "gici/utility/transform.h"
#include "gici/utility/common.h"

namespace gici {


// The default constructor
GnssImuInitializer::GnssImuInitializer(
              const GnssImuInitializerOptions& options, 
              const GnssLooseEstimatorBaseOptions& gnss_loose_base_options,
              const ImuEstimatorBaseOptions& imu_base_options,
              const EstimatorBaseOptions& base_options,
              const std::shared_ptr<Graph>& graph,
              const std::shared_ptr<EstimatorBase>& sub_gnss_estimator) :
  options_(options), 
  MultisensorInitializerBase(base_options), 
  GnssLooseEstimatorBase(gnss_loose_base_options, base_options),
  ImuEstimatorBase(imu_base_options, base_options),
  EstimatorBase(base_options)
{
  setGraph(graph);
  sub_gnss_estimator_ = sub_gnss_estimator;

  // Set optimization options
  base_options_ = base_options;
  base_options_.solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  base_options_.trust_region_strategy_type = ceres::DOGLEG;
  base_options_.num_threads = options.num_threads;
  base_options_.max_iteration = options.max_iteration;
  base_options_.max_solver_time = options.max_solver_time;
}

// Add measurement
bool GnssImuInitializer::addMeasurement(const EstimatorDataCluster& measurement)
{
  if (finished_) return false;

  // Add IMU
  if (measurement.imu && measurement.imu_role == ImuRole::Major) {
    addImuMeasurement(*measurement.imu);
  }

  // Add GNSS by solution measurement
  if (measurement.solution && 
      (measurement.solution_role == SolutionRole::Position || 
       measurement.solution_role == SolutionRole::Velocity ||
       measurement.solution_role == SolutionRole::PositionAndVelocity)) {
    GnssSolution gnss_solution = convertSolutionToGnssSolution(
      *measurement.solution, measurement.solution_role);
    return addGnssSolutionMeasurement(gnss_solution);
  }
  // Add GNSS by raw measurement
  if (sub_gnss_estimator_ && measurement.gnss) {
    if (sub_gnss_estimator_->addMeasurement(*measurement.gnss)) {
      if (sub_gnss_estimator_->estimate()) {
        Solution solution;
        solution.coordinate = coordinate_;
        solution.timestamp = sub_gnss_estimator_->getTimestamp();
        solution.pose = sub_gnss_estimator_->getPoseEstimate();
        solution.speed_and_bias = sub_gnss_estimator_->getSpeedAndBiasEstimate();
        solution.covariance = sub_gnss_estimator_->getCovariance();
        std::shared_ptr<GnssEstimatorBase> gnss_estimator = 
          std::dynamic_pointer_cast<GnssEstimatorBase>(sub_gnss_estimator_);
        solution.status = gnss_estimator->getSolutionStatus();
        solution.num_satellites = gnss_estimator->getNumberSatellite();
        solution.differential_age = gnss_estimator->getDifferentialAge();
        SolutionRole role;
        if (gnss_estimator->hasVelocityEstimate()) {
          role = SolutionRole::PositionAndVelocity;
        }
        else {
          role = SolutionRole::Position;
        }
        
        GnssSolution gnss_solution = convertSolutionToGnssSolution(solution, role);
        return addGnssSolutionMeasurement(gnss_solution);
      }
    }
  }

  return false;
}


// Add GNSS solution measurements
bool GnssImuInitializer::addGnssSolutionMeasurement(
  const GnssSolution& measurement)
{
  // Check if we have already finished initialization
  if (finished_) return false;

  // Get initial pitch, roll, and anguler rate bias under slow motion
  slowMotionInitialization();
  if (!zero_motion_finished_) {
    if (imu_measurements_.size() > 0)
    LOG(INFO) << "Waiting for zero motion initialization!";
    return false;
  }

  // Store measurements
  gnss_solution_measurements_.push_back(measurement);

  // Ensure that no duration overlaped with slow motion initialization
  while (gnss_solution_measurements_.size() > 0 &&
    gnss_solution_measurements_.front().timestamp < imu_measurements_.front().timestamp) {
    gnss_solution_measurements_.pop_front();
  }

  // Check if we have velocity measurement
  if (measurement.has_velocity) has_any_velocity_measurement_ = true;

  // Check if we have enough dynamic data
  const double oldest_timestamp = gnss_solution_measurements_.front().timestamp;
  const double cur_timestamp = gnss_solution_measurements_.back().timestamp;
  if (cur_timestamp - oldest_timestamp > 
      options_.time_window_length_dynamic_motion) {
    gnss_solution_measurements_.pop_front();
    imu_mutex_.lock();
    while (imu_measurements_.front().timestamp < 
           gnss_solution_measurements_.front().timestamp - 1.0) {
      imu_measurements_.pop_front();
    }
    imu_mutex_.unlock();
    dynamic_window_full_ = true;
  }

  // Check dynamic window 
  if (!dynamic_window_full_) {
    LOG(INFO) << "Full filling dynamic window!";
    return false;
  }

  // Check coordinate
  if (coordinate_ == nullptr) {
    LOG(ERROR) << "Coordinate not setted!";
    return false;
  }

  // Check acceleration or velocity
  Eigen::Vector3d initial_velocity;
  bool acc_ensured = false;
  double max_acc = 0.0;
  for (size_t i = 1; i < gnss_solution_measurements_.size(); i++) {
    if (has_any_velocity_measurement_) {
      Eigen::Vector3d velocity = coordinate_->rotate(
        gnss_solution_measurements_[i].velocity, GeoType::ECEF, GeoType::ENU);
      Eigen::Vector3d last_velocity = coordinate_->rotate(
        gnss_solution_measurements_[i - 1].velocity, GeoType::ECEF, GeoType::ENU);
      if (imu_base_options_.car_motion) {
        const double angular_velocity_norm = getImuMeasurementNear(
          gnss_solution_measurements_[i - 1].timestamp).angular_velocity.norm();
        if (last_velocity.norm() < 
            imu_base_options_.car_motion_min_velocity || 
            angular_velocity_norm > 
            imu_base_options_.car_motion_max_anguler_velocity) {
          gnss_solution_measurements_.clear(); 
          dynamic_window_full_ = false; break;
        }
      }
      initial_velocity = last_velocity;
      double dt = gnss_solution_measurements_[i].timestamp - 
                  gnss_solution_measurements_[i - 1].timestamp;
      double horizontal_acc = (velocity.head<2>().norm() - 
                              last_velocity.head<2>().norm()) / dt;
      if (horizontal_acc > max_acc) max_acc = horizontal_acc;
      if (horizontal_acc > options_.min_acceleration) acc_ensured = true;
    }
    else if (i > 1) {
      GnssSolution& last_last = gnss_solution_measurements_[i - 2];
      GnssSolution& last = gnss_solution_measurements_[i - 1];
      GnssSolution& cur = gnss_solution_measurements_[i];
      Eigen::Vector3d velocity, last_velocity;
      if (!computeCoarseVelocityFromPosition(last, cur, velocity) || 
          !computeCoarseVelocityFromPosition(last_last, last, last_velocity)) {
        continue;
      }
      velocity = coordinate_->rotate(velocity, GeoType::ECEF, GeoType::ENU);
      last_velocity = coordinate_->rotate(last_velocity, GeoType::ECEF, GeoType::ENU);
      if (imu_base_options_.car_motion) {
        const double angular_velocity_norm = getImuMeasurementNear(
          gnss_solution_measurements_[i - 1].timestamp).angular_velocity.norm();
        if (last_velocity.norm() < 
            imu_base_options_.car_motion_min_velocity || 
            angular_velocity_norm > 
            imu_base_options_.car_motion_max_anguler_velocity) {
          gnss_solution_measurements_.clear(); 
          dynamic_window_full_ = false; break;
        }
      }
      initial_velocity = last_velocity;
      double dt = (cur.timestamp - last_last.timestamp) / 2.0;
      double horizontal_acc = (velocity.head<2>().norm() - 
                              last_velocity.head<2>().norm()) / dt;
      if (horizontal_acc > max_acc) max_acc = horizontal_acc;
      if (horizontal_acc > options_.min_acceleration) acc_ensured = true;
    }
  }
  if (imu_base_options_.car_motion && !dynamic_window_full_) {
    LOG(INFO) << "Waiting for sufficient velocity: " << initial_velocity.transpose();
    return false;
  }
  if (!imu_base_options_.car_motion && !acc_ensured) {
    LOG(INFO) << "Waiting for sufficient acceleration!";
    return false;
  }

  // Add dynamic initialization items to graphs
  double initial_yaw = 0.0;
  speed_and_bias_0_.head<3>() = initial_velocity;
  if (imu_base_options_.car_motion) {
    // compute initial yaw from GNSS velocity
    initial_yaw = -atan2(initial_velocity(0), initial_velocity(1)) * R2D;
  }
  // set initial yaw and put items to graph
  Eigen::Vector3d rpy = quaternionToEulerAngle(T_WS_0_.getEigenQuaternion());
  rpy.z() = initial_yaw * D2R;
  Eigen::Quaterniond q_WS = eulerAngleToQuaternion(rpy);
  putMeasurementAndStateToGraph(q_WS, speed_and_bias_0_);

  return true;
}

// Apply initialization
bool GnssImuInitializer::estimate()
{
  optimize();

  // Log information
  if (base_options_.verbose_output) {
    LOG(INFO) << "GNSS/IMU initialization: " 
      << "Iterations: " << graph_->summary.iterations.size() << ", "
      << std::scientific << std::setprecision(3) 
      << "Initial cost: " << graph_->summary.initial_cost << ", "
      << "Final cost: " << graph_->summary.final_cost;
  }

  finished_ = true;
  return true;
}

// Arrange the initialization results to estimator
bool GnssImuInitializer::arrangeToEstimator(const int window_length,
            const std::shared_ptr<MarginalizationError>& marginalization_error,
            std::deque<State>& states, 
            ceres::ResidualBlockId& marginalization_residual_id,
            BackendId& gnss_extrinsics_id,
            std::deque<GnssSolution>& gnss_solution_measurements,
            ImuMeasurements& imu_measurements)
{
  if (!finished_) return false;

  // check if we need apply marginalization
  if (states_.size() < window_length) {
    states = states_;
    marginalization_residual_id = marginalization_residual_id_;
    gnss_extrinsics_id = gnss_extrinsics_id_;
    gnss_solution_measurements = gnss_solution_measurements_;
    imu_measurements = imu_measurements_;
    return true;
  }

  // Check if marginalization input valid
  CHECK_NOTNULL(marginalization_error);
  CHECK(marginalization_residual_id == nullptr);
  marginalization_error_ = marginalization_error;

  // Fill marginalization terms
  states.clear();
  std::deque<State>::reverse_iterator it_state = states_.rbegin();
  for (int size = 0; it_state != states_.rend(); it_state++, size++)
  {
    State& state = *it_state;
    if (size < window_length - 1) {
      states.push_front(state);
      continue;
    }

    // pose and speed and bias parameter and corresponding residuals
    addImuStateMarginBlockWithResiduals(state);
  }

  // Apply marginalization and add the item into graph
  applyMarginalization();
  marginalization_residual_id = marginalization_residual_id_;
  gnss_extrinsics_id = gnss_extrinsics_id_;
  gnss_solution_measurements = gnss_solution_measurements_;
  imu_measurements = imu_measurements_;

  return true;
}

// Get initial pitch, roll, and anguler rate bias under slow motion
void GnssImuInitializer::slowMotionInitialization()
{
  if (!gravity_setted_) return;
  if (imu_measurements_.size() == 0) return;
  // if (zero_motion_finished_) return;
  const double timestamp_end = imu_measurements_.back().timestamp;
  const double timestamp_start = 
    timestamp_end - options_.time_window_length_slow_motion;
  if (timestamp_start >= imu_measurements_.front().timestamp) {
    imu_mutex_.lock();
    if (!zero_motion_finished_)
    while (timestamp_start > imu_measurements_.front().timestamp) {
      imu_measurements_.pop_front();
    }
    imu_mutex_.unlock();
    Transformation T_WS_store = T_WS_0_;
    SpeedAndBias speed_and_bias_store = speed_and_bias_0_;
    if (initPoseAndBiases(imu_measurements_, 
        imu_base_options_.imu_parameters.g, T_WS_0_, speed_and_bias_0_)) {
      zero_motion_finished_ = true;
    }
    else {
      // keep the nearest stady state
      T_WS_0_ = T_WS_store;
      speed_and_bias_0_ = speed_and_bias_store;
    }
  }
}

// Put state and measurements to graph with given initial values
void GnssImuInitializer::putMeasurementAndStateToGraph(
  const Eigen::Quaterniond& q_WS_0, const SpeedAndBias& speed_and_bias_0)
{
  // Clear old graph
  states_.clear();
  gnss_extrinsics_id_ = BackendId(0);
  Graph::ParameterBlockCollection parameters = graph_->parameters();
  for (auto& parameter : parameters) {
    graph_->removeParameterBlock(parameter.first);
  }

  // Add new
  Transformation T_WS = Transformation(Eigen::Vector3d::Zero(), q_WS_0);
  SpeedAndBias speed_and_bias = speed_and_bias_0;
  for (size_t i = 0; i < gnss_solution_measurements_.size(); i++)
  {
    states_.push_back(State());
    GnssSolution& gnss = gnss_solution_measurements_[i];
    double timestamp = gnss.timestamp;

    // pose prior
    Eigen::Vector3d cur_position = coordinate_->convert(
      gnss.position, GeoType::ECEF, GeoType::ENU);
    if (i == 0) {
      T_WS.getPosition() = cur_position;
    }
    else {
      double last_timestamp = gnss_solution_measurements_[i - 1].timestamp;
      imuIntegration(last_timestamp, timestamp, T_WS, speed_and_bias);
    }

    // Add parameter blocks
    // pose and speed and bias block
    const int32_t bundle_id = gnss.id;
    BackendId pose_id = createGnssPoseId(bundle_id);
     size_t index = insertImuState(timestamp, pose_id, T_WS, speed_and_bias, true);
    states_[index].status = gnss.status;
    // GNSS extrinsics
    if (!gnss_extrinsics_id_.valid()) {
      gnss_extrinsics_id_ = 
        addGnssExtrinsicsParameterBlock(gnss.id, options_.gnss_extrinsics);
    }
    
    // Add residual blocks
    // GNSS position
    if (gnss.status != GnssSolutionStatus::Fixed) {
      gnss.covariance.topLeftCorner(3, 3) = Eigen::Matrix3d::Identity() * 1.0e2;
    }
    addGnssPositionResidualBlock(gnss, states_[index]);
    // GNSS velocity
    addGnssVelocityResidualBlock(gnss, states_[index], 
      getImuMeasurementNear(timestamp).angular_velocity);
    // Initial errors
    if (isFirstEpoch()) {
      // GNSS extrinsics
      addGnssExtrinsicsResidualBlock(gnss_extrinsics_id_, 
        options_.gnss_extrinsics, options_.gnss_extrinsics_initial_std);
      // bias error
      double speed_std = speed_and_bias.head<3>().norm() * 2.0;
      if (speed_std < 1.0) speed_std = 1.0;
      addImuSpeedAndBiasResidualBlock(states_[index], speed_and_bias, 
        speed_std,  
        imu_base_options_.imu_parameters.sigma_bg,
        imu_base_options_.imu_parameters.sigma_ba);
      // yaw error
      if (imu_base_options_.car_motion) {
        double std_yaw = sqrt(square(imu_base_options_.body_to_imu_rotation_std * D2R) + 
          square(0.1 / speed_and_bias.head<2>().norm()));
        addPoseResidualBlock(states_[index], T_WS, 0.1, std_yaw);
      }
    }
    // Car motion
    if (imu_base_options_.car_motion) {
      if (!isFirstEpoch()) addHMCResidualBlock(states_[index]);
      addNHCResidualBlock(states_[index]);
    }
  }
}

}