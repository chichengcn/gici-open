/**
* @Function: Base class for IMU estimators
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
*
* Copyright (C) 2023 by Cheng Chi, All rights reserved.
**/
#include "gici/imu/imu_estimator_base.h"

#include "gici/estimate/speed_and_bias_parameter_block.h"
#include "gici/imu/imu_error.h"
#include "gici/imu/speed_and_bias_error.h"
#include "gici/estimate/pose_error.h"
#include "gici/imu/yaw_error.h"
#include "gici/imu/roll_and_pitch_error.h"
#include "gici/imu/hmc_error.h"
#include "gici/imu/nhc_error.h"
#include "gici/utility/transform.h"

namespace gici {

// The default constructor
ImuEstimatorBase::ImuEstimatorBase(
                    const ImuEstimatorBaseOptions& options,
                    const EstimatorBaseOptions& base_options) :
  imu_base_options_(options), EstimatorBase(base_options)
{
  Eigen::Quaterniond q_BI = 
    eulerAngleToQuaternion(imu_base_options_.body_to_imu_rotation * D2R);
  T_BI_ = Transformation(Eigen::Vector3d::Zero(), q_BI);
}

// The default destructor
ImuEstimatorBase::~ImuEstimatorBase()
{}

// Add IMU meausrement
void ImuEstimatorBase::addImuMeasurement(const ImuMeasurement& imu_measurement)
{
  if (imu_measurements_.size() != 0 && 
      imu_measurements_.back().timestamp > imu_measurement.timestamp) {
    LOG(WARNING) << "Received IMU with previous timestamp!";
  }
  else {
    imu_mutex_.lock();
    imu_measurements_.push_back(imu_measurement);
    imu_measurements_.back().angular_velocity = 
      rotateImuToBody(imu_measurements_.back().angular_velocity);
    imu_measurements_.back().linear_acceleration = 
      rotateImuToBody(imu_measurements_.back().linear_acceleration);
    imu_mutex_.unlock();
  }

  // delete used IMU measurement
  imu_mutex_.lock();
  if (states_.size() > 0 && !do_not_remove_imu_measurements_)
  while (imu_measurements_.front().timestamp < oldestState().timestamp - 1.0) {
    imu_measurements_.pop_front();
  }
  imu_mutex_.unlock();
}

// IMU integration
bool ImuEstimatorBase::imuIntegration(
  const double last_timestamp, const double timestamp, 
  Transformation& T_WS, SpeedAndBias& speed_and_bias)
{
  // Do not need to integrate
  if (checkEqual(timestamp, last_timestamp)) return true;

  // Integrate to desired timestamp
  imu_mutex_.lock();
  ImuError::propagation(
    imu_measurements_, imu_base_options_.imu_parameters, T_WS, speed_and_bias,
    last_timestamp, timestamp, nullptr, nullptr);
  imu_mutex_.unlock();
  T_WS.getRotation().normalize();

  return true;
}

// IMU integration with covariance
bool ImuEstimatorBase::imuIntegration(
  const double last_timestamp, const double timestamp, 
  Transformation& T_WS, SpeedAndBias& speed_and_bias, 
  Eigen::Matrix<double, 15, 15>& covariance)
{
  // Do not need to integrate
  if (checkEqual(timestamp, last_timestamp)) return true;

  // Integrate to desired timestamp
  Eigen::Matrix<double, 15, 15> delta_P, F;
  imu_mutex_.lock();
  ImuError::propagation(
    imu_measurements_, imu_base_options_.imu_parameters, T_WS, speed_and_bias,
    last_timestamp, timestamp, &delta_P, &F);
  imu_mutex_.unlock();
  covariance = F * covariance * F.transpose() + delta_P;
  T_WS.getRotation().normalize();

  return true;
}

// Get a pose and speed and bias from estimated state and IMU integration
bool ImuEstimatorBase::getMotionFromEstimateAndImuIntegration(
  const State& estimated_state, const double timestamp, 
  Transformation& T_WS, SpeedAndBias& speed_and_bias)
{
  // Get from estimate
  T_WS = getPoseEstimate(estimated_state);
  speed_and_bias = getSpeedAndBiasEstimate(estimated_state);

  // Do not need to integrate
  if (checkEqual(timestamp, estimated_state.timestamp)) return true;

  // Integrate to desired timestamp
  return imuIntegration(estimated_state.timestamp, timestamp, T_WS, speed_and_bias);
}

// Get a pose and speed and bias from estimated state and IMU integration with covariance
bool ImuEstimatorBase::getMotionFromEstimateAndImuIntegration(
  const State& estimated_state, const double timestamp, 
  Transformation& T_WS, SpeedAndBias& speed_and_bias,
  Eigen::Matrix<double, 15, 15>& covariance)
{
  // Get from estimate
  T_WS = getPoseEstimate(estimated_state);
  speed_and_bias = getSpeedAndBiasEstimate(estimated_state);
  covariance = getCovariance(estimated_state);

  // Do not need to integrate
  if (checkEqual(timestamp, estimated_state.timestamp)) return true;

  // Integrate to desired timestamp
  return imuIntegration(
    estimated_state.timestamp, timestamp, T_WS, speed_and_bias, covariance);
}

// Get pose estimate at given timestmap
bool ImuEstimatorBase::getPoseEstimateAt(
  const double timestamp, Transformation& T_WS)
{
  // Check if we have already applied integration
  if (checkEqual(timestamp, last_timestamp_)) {
    T_WS = last_T_WS_; return true;
  }

  imu_state_mutex_.lock();
  State base_state;
  int end_index = states_.back().valid() ? states_.size() - 1 : states_.size() - 2;
  for (int i = states_.size() - 1; i >= 0; i--) {
    State& state = states_[i];
    if (!state.valid()) continue;
    if (i == end_index && state.timestamp <= timestamp) {
      base_state = state; break;
    }
    else if (state.timestamp <= timestamp && states_[i + 1].timestamp > timestamp) {
      base_state = state; break;
    }
  }
  // no suitable state
  if (!base_state.valid()) { 
    imu_state_mutex_.unlock(); return false;
  }
  // not sufficiant IMU data
  if (imu_measurements_.back().timestamp < timestamp) { 
    imu_state_mutex_.unlock(); return false;
  }

  // check duration
  double dt = timestamp - base_state.timestamp;
  if (dt > 2.0) {
    LOG(WARNING) << "Large integration duration, " 
                 << dt << "s. The result maybe incorrect!";
  }

  // integrate to given timestamp
  bool ret;
  SpeedAndBias speed_and_bias;
  Eigen::Matrix<double, 15, 15> covariance; covariance.setZero();
  // start from base state
  if (!checkEqual(last_base_state_.timestamp, base_state.timestamp) || 
      !checkLessEqual(last_timestamp_, timestamp)) {
    if (need_covariance_) {
      ret = getMotionFromEstimateAndImuIntegration(
        base_state, timestamp, T_WS, speed_and_bias, covariance);
    }
    else {
      ret = getMotionFromEstimateAndImuIntegration(
        base_state, timestamp, T_WS, speed_and_bias);
    }
  }
  // start from last dataset
  else {
    T_WS = last_T_WS_;
    speed_and_bias = last_speed_and_bias_;
    covariance = last_covariance_;
    if (need_covariance_) {
      ret = imuIntegration(
        last_timestamp_, timestamp, T_WS, speed_and_bias, covariance);
    }
    else {
      ret = imuIntegration(
        last_timestamp_, timestamp, T_WS, speed_and_bias);
    }
  }

  imu_state_mutex_.unlock();

  // store for other function call or latter call
  last_timestamp_ = timestamp;
  last_base_state_ = base_state;
  last_T_WS_ = T_WS;
  last_speed_and_bias_ = speed_and_bias;
  if (need_covariance_) last_covariance_ = covariance;

  return ret;
}

// Get speed and bias estimate at given timestmap
bool ImuEstimatorBase::getSpeedAndBiasEstimateAt(
  const double timestamp, SpeedAndBias& speed_and_bias)
{
  // Check if we have already applied integration
  if (checkEqual(timestamp, last_timestamp_)) {
    speed_and_bias = last_speed_and_bias_; return true;
  }

  // Apply integration
  Transformation T_WS;
  if (!getPoseEstimateAt(timestamp, T_WS)) return false;
  speed_and_bias = last_speed_and_bias_;

  return true;
}

// Get covariance at given timestamp
bool ImuEstimatorBase::getCovarianceAt(
  const double timestamp, Eigen::Matrix<double, 15, 15>& covariance)
{
  // Check if we have already applied integration
  if (checkEqual(timestamp, last_timestamp_) && need_covariance_) {
    covariance = last_covariance_; return true;
  }

  // We have not setted the flag yet
  if (!need_covariance_) {
    need_covariance_ = true;
    // re-integrate
    last_timestamp_ = 0.0;
    last_base_state_ = State();
  }

  // Apply integration
  Transformation T_WS;
  if (!getPoseEstimateAt(timestamp, T_WS)) return false;
  covariance = last_covariance_;

  return true;
}

// Add IMU speed and bias block to graph
void ImuEstimatorBase::addImuSpeedAndBiasParameterBlock(
  const BackendId backend_id, 
  const SpeedAndBias& prior)
{
  BackendId speed_and_bias_id = changeIdType(backend_id, IdType::ImuStates);
  std::shared_ptr<SpeedAndBiasParameterBlock> speed_and_bias_parameter_block = 
    std::make_shared<SpeedAndBiasParameterBlock>(
    prior, speed_and_bias_id.asInteger());
  CHECK(graph_->addParameterBlock(speed_and_bias_parameter_block));
}

// Add pose block to graph
void ImuEstimatorBase::addPoseParameterBlock(
  const BackendId backend_id, 
  const Transformation& T_WS_prior)
{
  if (graph_->parameterBlockExists(backend_id.asInteger())) return;
  std::shared_ptr<PoseParameterBlock> pose_parameter_block = 
    std::make_shared<PoseParameterBlock>(T_WS_prior, backend_id.asInteger());
  CHECK(graph_->addParameterBlock(pose_parameter_block, Graph::Pose6d));
}

// Add IMU pre-integration block to graph
void ImuEstimatorBase::addImuResidualBlock(const State& last_state, State& cur_state)
{
  const double last_timestamp = last_state.timestamp;
  const double timestamp = cur_state.timestamp;
  BackendId last_pose_id = last_state.id_in_graph;
  BackendId cur_pose_id = cur_state.id_in_graph;
  BackendId last_speed_and_bias_id = changeIdType(last_pose_id, IdType::ImuStates);
  BackendId speed_and_bias_id = changeIdType(cur_pose_id, IdType::ImuStates);
  imu_mutex_.lock();
  std::shared_ptr<ImuError> imu_error =
    std::make_shared<ImuError>(imu_measurements_, imu_base_options_.imu_parameters,
                              last_timestamp, timestamp);
  imu_mutex_.unlock();
  cur_state.imu_residual_to_lhs = 
    graph_->addResidualBlock(imu_error, nullptr, 
      graph_->parameterBlockPtr(last_pose_id.asInteger()), 
      graph_->parameterBlockPtr(last_speed_and_bias_id.asInteger()), 
      graph_->parameterBlockPtr(cur_pose_id.asInteger()), 
      graph_->parameterBlockPtr(speed_and_bias_id.asInteger()));
    
  // overlapped state
  State::syncOverlap(cur_state, states_);
  State::syncOverlap(last_state, states_);
}

// Add IMU speed and bias residual block to graph
void ImuEstimatorBase::addImuSpeedAndBiasResidualBlock(
  const State& state, 
  const SpeedAndBias& speed_and_bias, 
  const double std_speed, const double std_bg, const double std_ba)
{
  BackendId speed_and_bias_id = changeIdType(state.id_in_graph, IdType::ImuStates);
  std::shared_ptr<SpeedAndBiasError> speed_and_bias_error = 
    std::make_shared<SpeedAndBiasError>(speed_and_bias, 
    square(std_speed), 
    square(std_bg), 
    square(std_ba));
  ceres::ResidualBlockId residual_id = 
    graph_->addResidualBlock(speed_and_bias_error, nullptr,
      graph_->parameterBlockPtr(speed_and_bias_id.asInteger()));
}

// Add pose residual block to graph
void ImuEstimatorBase::addPoseResidualBlock(
  const State& state, const Transformation& T_WS, 
  const double std_roll_pitch, const double std_yaw)
{
  BackendId pose_id = state.id_in_graph;
  Eigen::Matrix<double, 6, 6> information;
  information.setIdentity(); information *= 1.0e-6;
  information(3, 3) = 1.0 / square(std_roll_pitch);
  information(4, 4) = 1.0 / square(std_roll_pitch);
  information(5, 5) = 1.0 / square(std_yaw);
  std::shared_ptr<PoseError> pose_error = 
    std::make_shared<PoseError>(T_WS, information);
  ceres::ResidualBlockId residual_id = 
    graph_->addResidualBlock(pose_error, nullptr,
      graph_->parameterBlockPtr(pose_id.asInteger()));
}

// Add yaw residual block to graph
void ImuEstimatorBase::addYawResidualBlock(
  const State& state, const double yaw, const double std_yaw)
{
  BackendId pose_id = state.id_in_graph;
#if 0
  std::shared_ptr<YawError> yaw_error = 
    std::make_shared<YawError>(yaw, 1.0 / square(std_yaw));
#else
  Transformation T_WS = getPoseEstimate(state);
  Eigen::Vector3d rpy = quaternionToEulerAngle(T_WS.getEigenQuaternion());
  rpy.z() = yaw;
  T_WS = Transformation(T_WS.getPosition(), eulerAngleToQuaternion(rpy));
  Eigen::Matrix<double, 6, 6> information;
  information.setIdentity(); information *= 1.0e-6;
  information(5, 5) = 1.0 / square(std_yaw);
  std::shared_ptr<PoseError> yaw_error = 
    std::make_shared<PoseError>(T_WS, information);
#endif
  ceres::ResidualBlockId residual_id = 
    graph_->addResidualBlock(yaw_error, nullptr,
      graph_->parameterBlockPtr(pose_id.asInteger()));
}

// Add roll and pitch residual block to graph
void ImuEstimatorBase::addRollPitchResidualBlock(
  const State& state, const Eigen::Vector2d& pitch_and_roll, 
  const double std_pitch_and_roll)
{
  BackendId pose_id = state.id_in_graph;
#if 0
  Eigen::Matrix2d information = 
    Eigen::Matrix2d::Identity() / square(std_pitch_and_roll);
  std::shared_ptr<RollAndPitchError> roll_pitch_error = 
    std::make_shared<RollAndPitchError>(pitch_and_roll, information);
#else
  Transformation T_WS = getPoseEstimate(state);
  Eigen::Vector3d rpy = quaternionToEulerAngle(T_WS.getEigenQuaternion());
  rpy.head<2>() = pitch_and_roll;
  T_WS = Transformation(T_WS.getPosition(), eulerAngleToQuaternion(rpy));
  Eigen::Matrix<double, 6, 6> information;
  information.setIdentity(); information *= 1.0e-6;
  information(3, 3) = 1.0 / square(std_pitch_and_roll);
  information(4, 4) = 1.0 / square(std_pitch_and_roll);
  std::shared_ptr<PoseError> roll_pitch_error = 
    std::make_shared<PoseError>(T_WS, information);
#endif
  ceres::ResidualBlockId residual_id = 
    graph_->addResidualBlock(roll_pitch_error, nullptr,
      graph_->parameterBlockPtr(pose_id.asInteger()));
}

// Add heading measurement constraint residual block
void ImuEstimatorBase::addHMCResidualBlock(const State& state)
{
  BackendId pose_id = state.id_in_graph;
  BackendId speed_and_bias_id = changeIdType(pose_id, IdType::ImuStates);

  Eigen::Vector3d speed = getSpeedAndBiasEstimate(state).head<3>();
  if (speed.norm() < imu_base_options_.car_motion_min_velocity) return;
  double angular_velocity_norm = 
    getImuMeasurementNear(state.timestamp).angular_velocity.norm();
  if (angular_velocity_norm * R2D > 
    imu_base_options_.car_motion_max_anguler_velocity) return;

  double std = sqrt(square(imu_base_options_.body_to_imu_rotation_std * D2R) + 
    square(0.1 / speed.head<2>().norm()));
  std::shared_ptr<HMCError> hmc_error = std::make_shared<HMCError>(std);
  ceres::ResidualBlockId residual_id = 
    graph_->addResidualBlock(hmc_error, nullptr,
      graph_->parameterBlockPtr(pose_id.asInteger()),
      graph_->parameterBlockPtr(speed_and_bias_id.asInteger()));
}

// Add non-holonomic constraint error
void ImuEstimatorBase::addNHCResidualBlock(const State& state)
{
  BackendId pose_id = state.id_in_graph;
  BackendId speed_and_bias_id = changeIdType(pose_id, IdType::ImuStates);

  Eigen::Vector3d speed = getSpeedAndBiasEstimate(state).head<3>();
  if (speed.norm() < imu_base_options_.car_motion_min_velocity) return;
  double angular_velocity_norm = 
    getImuMeasurementNear(state.timestamp).angular_velocity.norm();
  if (angular_velocity_norm * R2D > 
    imu_base_options_.car_motion_max_anguler_velocity) return;

  double std = imu_base_options_.body_to_imu_rotation_std * D2R;
  std::shared_ptr<NHCError> nhc_error = std::make_shared<NHCError>(std);
  ceres::ResidualBlockId residual_id = 
    graph_->addResidualBlock(nhc_error, nullptr,
      graph_->parameterBlockPtr(pose_id.asInteger()),
      graph_->parameterBlockPtr(speed_and_bias_id.asInteger()));
}

// Add zero-motion update constraint error
void ImuEstimatorBase::addZUPTResidualBlock(const State& state)
{
  // Check zero motion
  imu_mutex_.lock();
  std::vector<double> acc[3], gyro[3];
  for (int i = imu_measurements_.size() - 1; i >= 0; i--) {
    ImuMeasurement& imu = imu_measurements_[i];
    if (imu.timestamp > state.timestamp) continue;
    if (imu.timestamp < state.timestamp - imu_base_options_.zupt_duration) break;
    if (i == 0 && imu.timestamp > 
        state.timestamp - imu_base_options_.zupt_duration) return;
    for (int j = 0; j < 3; j++) {
      acc[j].push_back(imu.linear_acceleration(j));
      gyro[j].push_back(imu.angular_velocity(j));
    }
  }
  imu_mutex_.unlock();
  double median_acc[3], median_gyro[3];
  double std_acc[3], std_gyro[3];
  for (int i = 0; i < 3; i++) {
    median_acc[i] = getMedian(acc[i]);
    median_gyro[i] = getMedian(gyro[i]);
    std_acc[i] = getStandardDeviation(acc[i], median_acc[i]);
    std_gyro[i] = getStandardDeviation(gyro[i], median_gyro[i]);
  }
  LOG(INFO) << "ZUPT: " << std::fixed << Eigen::Vector3d(std_acc).transpose() << " | " << Eigen::Vector3d(std_gyro).transpose() << " | " 
    << Eigen::Vector3d(median_gyro).transpose() << " | " << Eigen::Vector3d(median_acc).transpose() << " | bias = " << getSpeedAndBiasEstimate(state).tail<3>().transpose()
    << " | Velocity = " << getSpeedAndBiasEstimate(state).head<3>().transpose();
  for (int i = 0; i < 3; i++) {
    if (std_acc[i] > imu_base_options_.zupt_max_acc_std) return;
    if (std_gyro[i] > imu_base_options_.zupt_max_gyro_std) return;
    if (fabs(median_gyro[i]) > imu_base_options_.zupt_max_gyro_median) return;
  }

  // Add constraint
  SpeedAndBias speed_and_bias;
  speed_and_bias.head<3>() = Eigen::Vector3d::Zero();
  speed_and_bias.tail<6>().setZero();
  const double speed_information = 
    1.0 / square(imu_base_options_.zupt_sigma_zero_velocity);
  const double bias_information = 1.0e-6;
  Eigen::Matrix<double, 9, 9> information;
  information.setIdentity();
  information.topLeftCorner<3, 3>() *= speed_information;
  information.bottomRightCorner<6, 6>() *= bias_information;
  BackendId speed_and_bias_id = changeIdType(state.id_in_graph, IdType::ImuStates);
  std::shared_ptr<SpeedAndBiasError> error = 
    std::make_shared<SpeedAndBiasError>(speed_and_bias, information);
  graph_->addResidualBlock(error, 
    huber_loss_function_ ? huber_loss_function_.get() : nullptr,
    graph_->parameterBlockPtr(speed_and_bias_id.asInteger()));
  
  // set velocity parameter as zero
  auto speed_and_bias_parameter = 
    graph_->parameterBlockPtr(speed_and_bias_id.asInteger());
  Eigen::Map<Eigen::Vector3d>(speed_and_bias_parameter->parameters()).setZero();
  graph_->setParameterBlockVariable(speed_and_bias_parameter);

  LOG(INFO) << "Added ZUPT at " << std::fixed << state.timestamp;
}

// Inserts a state inside or at the ends of state window
size_t ImuEstimatorBase::insertImuState(
  const double timestamp, const BackendId& backend_id,
  const Transformation& T_WS_prior, 
  const SpeedAndBias& speed_and_bias_prior,
  const bool use_prior)
{
  imu_state_mutex_.lock();

  // Get the latest state
  bool has_invalid_state = false;
  double latest_timestamp = 0.0;
  int valid_state_index = states_.size();
  for (auto it = states_.rbegin(); it != states_.rend(); it++) {
    valid_state_index--;
    if (it->valid()) {
      latest_timestamp = it->timestamp; break;
    }
    else has_invalid_state = true;
  }

  // Free the pre-allocated memory
  if (has_invalid_state) states_.pop_back();

  // Check if it overlaps with existing state
  int overlap_index = -1;
  for (size_t i = 0; i < states_.size(); i++) {
    if (checkEqual(states_[i].timestamp, timestamp)) {
      overlap_index = i; break;
    }
  }

  // Overlap with existing state
  if (overlap_index >= 0) 
  {
    auto it_lhs = states_.begin(); 
    std::advance(it_lhs, overlap_index + 1);
    State state;
    state.timestamp = timestamp;
    state.id = backend_id;
    state.id_in_graph = states_[overlap_index].id_in_graph;
    state.imu_residual_to_lhs = states_[overlap_index].imu_residual_to_lhs;
    auto it_cur = states_.insert(it_lhs, state);
    if (State::overlaps.count(state.id_in_graph) == 0) {
      State::overlaps.insert(std::make_pair(state.id_in_graph, states_[overlap_index]));
    }
    State::overlaps.insert(std::make_pair(state.id_in_graph, (*it_cur)));

    imu_state_mutex_.unlock();
    return overlap_index + 1;
  }
  // At the front of the window
  else if (latest_timestamp == 0.0 || 
      checkLessEqual(timestamp, oldestState().timestamp))
  {
    CHECK(use_prior) << "Cannot add new IMU state at the front of the state "
      << "window without motion prior!";
    // add parameter block at front
    addPoseParameterBlock(backend_id, T_WS_prior);
    addImuSpeedAndBiasParameterBlock(backend_id, speed_and_bias_prior);
    State state;
    state.timestamp = timestamp;
    state.id = backend_id;
    state.id_in_graph = backend_id;
    states_.push_front(state);
    // connect current state to the next
    if (states_.size() > 1) {
      addImuResidualBlock(states_[0], states_[1]);
    }

    imu_state_mutex_.unlock();
    return 0;
  }
  // At the end of the window
  else if (checkLessEqual(latest_timestamp, timestamp)) 
  {
    // add parameter block at back
    const State& last_state = states_[valid_state_index];
    Transformation T_WS = T_WS_prior;
    SpeedAndBias speed_and_bias = speed_and_bias_prior;
    if (!use_prior) {
      getMotionFromEstimateAndImuIntegration(
        last_state, timestamp, T_WS, speed_and_bias);
    }
    addPoseParameterBlock(backend_id, T_WS);
    addImuSpeedAndBiasParameterBlock(backend_id, speed_and_bias);
    State state;
    state.timestamp = timestamp;
    state.id = backend_id;
    state.id_in_graph = backend_id;
    states_.push_back(state);
    // connect the last state to current
    addImuResidualBlock(lastState(), curState());
    
    imu_state_mutex_.unlock();
    return states_.size() - 1;
  }
  // Inside the window, we break the IMU connections and reform them
  else 
  {
    // find two states around the timestamp
    size_t index_lhs, index_rhs;
    for (size_t i = states_.size() - 1; i >= 0; i--) {
      State& state = states_[i];
      if (state.valid() && state.timestamp > timestamp) {
        index_rhs = i;
      }
      if (state.valid() && state.timestamp <= timestamp) {
        index_lhs = i;
        break;
      }
    }

    // add parameter blocks
    Transformation T_WS = T_WS_prior;
    SpeedAndBias speed_and_bias = speed_and_bias_prior;
    if (!use_prior) {
      getMotionFromEstimateAndImuIntegration(
        states_[index_lhs], timestamp, T_WS, speed_and_bias);
    }
    addPoseParameterBlock(backend_id, T_WS);
    addImuSpeedAndBiasParameterBlock(backend_id, speed_and_bias);
    auto it_lhs = states_.begin(); std::advance(it_lhs, index_lhs + 1);
    State state;
    state.timestamp = timestamp;
    state.id = backend_id;
    state.id_in_graph = backend_id;
    auto it_cur = states_.insert(it_lhs, state);
    State& cur_state = *it_cur;
    auto it_rhs_new = it_cur; it_rhs_new++;
    auto it_lhs_new = it_cur; it_lhs_new--;
    const State& state_lhs = *it_lhs_new;
    State& state_rhs = *it_rhs_new;
    // erase old IMU connection
    eraseImuResidualBlock(state_lhs, state_rhs);
    // add LHS IMU connection
    addImuResidualBlock(state_lhs, cur_state);
    // add RHS IMU connection
    addImuResidualBlock(cur_state, state_rhs);

    imu_state_mutex_.unlock();
    return index_lhs + 1;
  }

  imu_state_mutex_.unlock();
  return 0;
}

// Add IMU speed and bias block to marginalizer
void ImuEstimatorBase::addImuSpeedAndBiasParameterMarginBlock(const State& state, bool keep)
{
  BackendId speed_and_bias_id = changeIdType(state.id_in_graph, IdType::ImuStates);
  CHECK(graph_->parameterBlockExists(speed_and_bias_id.asInteger()));
  marginalization_parameter_ids_.push_back(speed_and_bias_id);
  marginalization_keep_parameter_blocks_.push_back(keep);
}

// Add IMU speed and bias block with its residuals to marginalizer
void ImuEstimatorBase::addImuSpeedAndBiasParameterMarginBlockWithResiduals(const State& state, bool keep)
{
  BackendId speed_and_bias_id = changeIdType(state.id_in_graph, IdType::ImuStates);
  CHECK(graph_->parameterBlockExists(speed_and_bias_id.asInteger()));
  Graph::ResidualBlockCollection residuals = 
    graph_->residuals(speed_and_bias_id.asInteger());
  for (size_t r = 0; r < residuals.size(); ++r) {
    marginalization_error_->addResidualBlock(
          residuals[r].residual_block_id);
  }
  marginalization_parameter_ids_.push_back(speed_and_bias_id);
  marginalization_keep_parameter_blocks_.push_back(keep);
}

// Add IMU pose block to marginalizer
void ImuEstimatorBase::addPoseParameterMarginBlock(const State& state, bool keep)
{
  BackendId pose_id = state.id_in_graph;
  CHECK(graph_->parameterBlockExists(pose_id.asInteger()));
  marginalization_parameter_ids_.push_back(pose_id);
  marginalization_keep_parameter_blocks_.push_back(keep);
}

// Add IMU pose block with its residuals to marginalizer
void ImuEstimatorBase::addPoseParameterMarginBlockWithResiduals(const State& state, bool keep)
{
  BackendId pose_id = state.id_in_graph;
  CHECK(graph_->parameterBlockExists(pose_id.asInteger()));
  Graph::ResidualBlockCollection residuals = 
    graph_->residuals(pose_id.asInteger());
  for (size_t r = 0; r < residuals.size(); ++r) {
    marginalization_error_->addResidualBlock(
          residuals[r].residual_block_id);
  }
  marginalization_parameter_ids_.push_back(pose_id);
  marginalization_keep_parameter_blocks_.push_back(keep);
}

// Add IMU state to marginalizer
void ImuEstimatorBase::addImuStateMarginBlock(const State& state, bool keep)
{
  // skip if it is a overlaped state
  if (State::overlaps.count(state.id_in_graph) > 1) {
    // erase in overlap state map
    State::eraseOverlap(state);
    return;
  }
  
  addPoseParameterMarginBlock(state, keep);
  addImuSpeedAndBiasParameterMarginBlock(state, keep);
}

// Add IMU state with its residuals to marginalizer
void ImuEstimatorBase::addImuStateMarginBlockWithResiduals(const State& state, bool keep)
{
  // skip if it is a overlaped state
  if (State::overlaps.count(state.id_in_graph) > 1) {
    // erase in overlap state map
    State::eraseOverlap(state);
    return;
  }

  addPoseParameterMarginBlockWithResiduals(state, keep);
  addImuSpeedAndBiasParameterMarginBlockWithResiduals(state, keep);
}

// Add IMU pre-integration block to marginalizer
void ImuEstimatorBase::addImuResidualMarginBlock(const State& state)
{
  CHECK(graph_->parameterBlockExists(state.id_in_graph.asInteger()));
  Graph::ResidualBlockCollection residuals = 
    graph_->residuals(state.id_in_graph.asInteger());
  for (size_t r = 0; r < residuals.size(); ++r) {
    if (residuals[r].error_interface_ptr->typeInfo() 
        != ErrorType::kIMUError) continue;
    marginalization_error_->addResidualBlock(
          residuals[r].residual_block_id);
  }
}

// Add IMU speed and bias residual block to marginalizer
void ImuEstimatorBase::addImuSpeedAndBiasResidualMarginBlock(const State& state)
{
  BackendId speed_and_bias_id = changeIdType(state.id_in_graph, IdType::ImuStates);
  CHECK(graph_->parameterBlockExists(speed_and_bias_id.asInteger()));
  Graph::ResidualBlockCollection residuals = 
    graph_->residuals(speed_and_bias_id.asInteger());
  for (size_t r = 0; r < residuals.size(); ++r) {
    if (residuals[r].error_interface_ptr->typeInfo() 
        != ErrorType::kSpeedAndBiasError) continue;
    marginalization_error_->addResidualBlock(
          residuals[r].residual_block_id);
  }
}

// Add pose residual block to marginalizer
void ImuEstimatorBase::addPoseResidualMarginBlock(const State& state)
{
  CHECK(graph_->parameterBlockExists(state.id_in_graph.asInteger()));
  Graph::ResidualBlockCollection residuals = 
    graph_->residuals(state.id_in_graph.asInteger());
  for (size_t r = 0; r < residuals.size(); ++r) {
    if (residuals[r].error_interface_ptr->typeInfo() 
        != ErrorType::kPoseError) continue;
    marginalization_error_->addResidualBlock(
          residuals[r].residual_block_id);
  }
}

// Add heading measurement constraint residual block to marginalizer
void ImuEstimatorBase::addHMCResidualMarginBlock(const State& state)
{
  CHECK(graph_->parameterBlockExists(state.id_in_graph.asInteger()));
  Graph::ResidualBlockCollection residuals = 
    graph_->residuals(state.id_in_graph.asInteger());
  for (size_t r = 0; r < residuals.size(); ++r) {
    if (residuals[r].error_interface_ptr->typeInfo() 
        != ErrorType::kHMCError) continue;
    marginalization_error_->addResidualBlock(
          residuals[r].residual_block_id);
  }
}

// Add non-holonomic constraint residual block to marginalizer
void ImuEstimatorBase::addNHCResidualMarginBlock(const State& state)
{
  CHECK(graph_->parameterBlockExists(state.id_in_graph.asInteger()));
  Graph::ResidualBlockCollection residuals = 
    graph_->residuals(state.id_in_graph.asInteger());
  for (size_t r = 0; r < residuals.size(); ++r) {
    if (residuals[r].error_interface_ptr->typeInfo() 
        != ErrorType::kNHCError) continue;
    marginalization_error_->addResidualBlock(
          residuals[r].residual_block_id);
  }
}

// Add all IMU residual blocks to marginalizer
void ImuEstimatorBase::addImuResidualMarginBlocks(const State& state)
{
  BackendId speed_and_bias_id = changeIdType(state.id_in_graph, IdType::ImuStates);
  CHECK(graph_->parameterBlockExists(speed_and_bias_id.asInteger()));
  Graph::ResidualBlockCollection residuals = 
    graph_->residuals(speed_and_bias_id.asInteger());
  for (size_t r = 0; r < residuals.size(); ++r) {
    if (residuals[r].error_interface_ptr->typeInfo() 
        != ErrorType::kSpeedAndBiasError && 
        residuals[r].error_interface_ptr->typeInfo() 
        != ErrorType::kIMUError && 
        residuals[r].error_interface_ptr->typeInfo() 
        != ErrorType::kPoseError && 
        residuals[r].error_interface_ptr->typeInfo() 
        != ErrorType::kHMCError && 
        residuals[r].error_interface_ptr->typeInfo() 
        != ErrorType::kNHCError) continue;
    marginalization_error_->addResidualBlock(
          residuals[r].residual_block_id);
  }
}

// Erase a IMU residual block
void ImuEstimatorBase::eraseImuResidualBlock(const State& last_state, State& cur_state)
{
  if (cur_state.imu_residual_to_lhs == nullptr) return;
  CHECK(graph_->removeResidualBlock(cur_state.imu_residual_to_lhs));
  cur_state.imu_residual_to_lhs = nullptr;
  // overlapped state
  State::syncOverlap(cur_state, states_);
}

// Erase a state inside or at the ends of windows
void ImuEstimatorBase::eraseImuState(const State& state)
{
  imu_state_mutex_.lock();

  // Overlaped state
  if (State::overlaps.count(state.id_in_graph) > 1) {
    // erase in overlap state map
    State::eraseOverlap(state);
    // erase in state que
    for (auto it = states_.begin(); it != states_.end(); it++) {
      if (it->id == state.id) { states_.erase(it); break; }
    }

    imu_state_mutex_.unlock();
    return;
  }

  // Erase state in graph
  const BackendId& parameter_id = state.id_in_graph;
  if (graph_->parameterBlockExists(parameter_id.asInteger())) {
    graph_->removeParameterBlock(parameter_id.asInteger());
  }
  const BackendId speed_and_bias_id = 
    changeIdType(parameter_id, IdType::ImuStates);
  if (graph_->parameterBlockExists(speed_and_bias_id.asInteger())) {
    graph_->removeParameterBlock(speed_and_bias_id.asInteger());
  }

  // Check if the state is inside the window
  size_t index;
  for (size_t i = 0; i < states_.size(); i++) {
    if (state.id == states_[i].id) {
      index = i; break;
    }
  }
  // not inside
  if (index == 0 || index == states_.size() - 1) {
    // erase state in window
    if (index == 0) states_.pop_front();
    else states_.pop_back();
  }
  // inside the window, we should reform IMU connection
  else {
    // connect lhs and rhs state
    const State& state_lhs = states_[index - 1];
    State& state_rhs = states_[index + 1];
    addImuResidualBlock(state_lhs, state_rhs);
    // erase state in window
    for (auto it = states_.begin(); it != states_.end(); it++) {
      if (it->id == state.id) {
        states_.erase(it); break;
      }
    }
  }

  imu_state_mutex_.unlock();
}

// Down-weight IMU residual block
void ImuEstimatorBase::downWeightImuResidualBlock(
  const ceres::ResidualBlockId residual_id, double factor)
{
  if (residual_id == nullptr) return;
  auto residual = graph_->residualBlockIdToResidualBlockSpecMap().at(residual_id);
  auto interface = residual.error_interface_ptr;
  std::shared_ptr<ImuError> imu_error = 
    std::dynamic_pointer_cast<ImuError>(interface);
  imu_error->downWeight(factor);
}

// Get a IMU measurement near given timestamp
ImuMeasurement ImuEstimatorBase::getImuMeasurementNear(const double timestamp) 
{
  imu_mutex_.lock();
  if (timestamp <= imu_measurements_.front().timestamp) {
    ImuMeasurement imu = imu_measurements_.front();
    imu_mutex_.unlock();
    return imu;
  }
  if (timestamp >= imu_measurements_.back().timestamp) {
    ImuMeasurement imu = imu_measurements_.back();
    imu_mutex_.unlock();
    return imu;
  }

  for (size_t i = 1; i < imu_measurements_.size(); i++) {
    ImuMeasurement last_imu = imu_measurements_[i - 1];
    ImuMeasurement cur_imu = imu_measurements_[i];
    double last_dt = last_imu.timestamp - timestamp;
    double cur_dt = cur_imu.timestamp - timestamp;
    if (cur_dt > 0.0 && last_dt <= 0.0) {
      imu_mutex_.unlock();
      return (fabs(cur_dt) > fabs(last_dt)) ? last_imu : cur_imu;
    }
  }

  ImuMeasurement imu = imu_measurements_.back();
  imu_mutex_.unlock();
  return imu;
}

}