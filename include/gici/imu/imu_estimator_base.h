/**
* @Function: Base class for IMU estimators
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
*
* Copyright (C) 2023 by Cheng Chi, All rights reserved.
**/
#pragma once

#include "gici/estimate/estimator_base.h"
#include "gici/imu/imu_common.h"
#include "gici/imu/imu_types.h"
#include "gici/utility/transform.h"

namespace gici {

// IMU estimator common options
struct ImuEstimatorBaseOptions {
  // IMU parameters
  ImuParameters imu_parameters;

  // IMU to car rotation (deg) (body frame: right, forward, up)
  // This is to avoid the y-axis rotating to 90 degree, which will cause gimbal lock.
  Eigen::Vector3d body_to_imu_rotation = Eigen::Vector3d::Zero();

  // STD of IMU to body rotation (deg)
  // This will be used in car motion mode to apply non-holonomic constraints.
  double body_to_imu_rotation_std = 3.0;

  // ZUPT window length, we think the vehicle is stable if it keeps stady within the window.
  double zupt_duration = 1.0;

  // Maximum IMU acceleration STD to consider as stady
  double zupt_max_acc_std = 0.5;

  // Maximum IMU angular velocity STD to consider as stady
  double zupt_max_gyro_std = 0.05;

  // Maximum IMU angular velocity median to consider as stady
  double zupt_max_gyro_median = 0.01;

  // Standard deviation for ZUPT zero velocity constriant
  double zupt_sigma_zero_velocity = 0.01;

  // If car motion
  bool car_motion = false;

  // Minimum velocity to apply car motion constraints (m/s)
  double car_motion_min_velocity = 3.0;

  // Maximum angular rate to apply car motion constraints (deg/s)
  double car_motion_max_anguler_velocity = 5.0;
};

// Estimator
class ImuEstimatorBase : public virtual EstimatorBase {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ImuEstimatorBase(const ImuEstimatorBaseOptions& options,
                    const EstimatorBaseOptions& base_options);
  ~ImuEstimatorBase();

  // Add IMU meausrement
  virtual void addImuMeasurement(const ImuMeasurement& imu_measurement);

  // Rotate translation variable from IMU frame to body frame
  inline Eigen::Vector3d rotateImuToBody(const Eigen::Vector3d& p_I) {
    return T_BI_ * p_I;
  }

  static inline Eigen::Vector3d rotateImuToBody(
    const Eigen::Vector3d& p_I, const ImuEstimatorBaseOptions& options) {
    Eigen::Quaterniond q_BI = 
      eulerAngleToQuaternion(options.body_to_imu_rotation * D2R);
    return q_BI * p_I;
  }

  static inline Eigen::Vector3d rotateStdImuToBody(
    const Eigen::Vector3d& std_p_I, const ImuEstimatorBaseOptions& options) {
    Eigen::Quaterniond q_BI = 
      eulerAngleToQuaternion(options.body_to_imu_rotation * D2R);
    Eigen::Matrix3d R_BI = q_BI.toRotationMatrix();
    Eigen::Matrix3d cov = cwiseSquare(std_p_I).asDiagonal();
    Eigen::Matrix3d cov_rot = R_BI * cov * R_BI.transpose();
    Eigen::Vector3d std_rot;
    std_rot << sqrt(cov_rot(0, 0)), sqrt(cov_rot(1, 1)), sqrt(cov_rot(2, 2));
    return std_rot;
  }

  // Rotate transformation variable from IMU frame to body frame
  inline Transformation rotateImuToBody(const Transformation& T_IX) {
    return T_BI_ * T_IX;
  }

  static inline Transformation rotateImuToBody(
    const Transformation& T_IX, const ImuEstimatorBaseOptions& options) {
    Eigen::Quaterniond q_BI = 
      eulerAngleToQuaternion(options.body_to_imu_rotation * D2R);
    Transformation T_BI = Transformation(Eigen::Vector3d::Zero(), q_BI);
    return T_BI * T_IX;
  }

  // Set gravity
  void setGravity(double gravity) { 
    imu_base_options_.imu_parameters.g = gravity;
    gravity_setted_ = true;
  }

  // IMU integration
  bool imuIntegration(const double last_timestamp, const double timestamp, 
    Transformation& T_WS, SpeedAndBias& speed_and_bias);

  // IMU integration with covariance
  bool imuIntegration(const double last_timestamp, const double timestamp, 
    Transformation& T_WS, SpeedAndBias& speed_and_bias, 
    Eigen::Matrix<double, 15, 15>& covariance);

  // Get a pose and speed and bias from estimated state and IMU integration
  bool getMotionFromEstimateAndImuIntegration(
    const State& estimated_state, const double timestamp, 
    Transformation& T_WS, SpeedAndBias& speed_and_bias);

  // Get a pose and speed and bias from estimated state and IMU integration with covariance
  bool getMotionFromEstimateAndImuIntegration(
    const State& estimated_state, const double timestamp, 
    Transformation& T_WS, SpeedAndBias& speed_and_bias,
    Eigen::Matrix<double, 15, 15>& covariance);

  // Get pose estimate at given timestmap
  bool getPoseEstimateAt(
    const double timestamp, Transformation& T_WS) override;

  // Get speed and bias estimate at given timestmap
  bool getSpeedAndBiasEstimateAt(
    const double timestamp, SpeedAndBias& speed_and_bias) override;

  // Get covariance at given timestamp
  bool getCovarianceAt(
    const double timestamp, Eigen::Matrix<double, 15, 15>& covariance) override;

  // Get lastest IMU measurement timestamp
  double latestImuMeasurementTimestamp() 
  { return imu_measurements_.size() == 0 ? 0.0 : imu_measurements_.back().timestamp; }

protected:
  // Add IMU speed and bias block to graph
  void addImuSpeedAndBiasParameterBlock(
    const BackendId backend_id, 
    const SpeedAndBias& prior);

  // Add pose block to graph
  void addPoseParameterBlock(
    const BackendId backend_id, 
    const Transformation& T_WS_prior);

  // Add IMU pre-integration block to graph
  void addImuResidualBlock(const State& last_state, State& cur_state);

  // Add IMU speed and bias residual block to graph
  void addImuSpeedAndBiasResidualBlock(
    const State& state, 
    const SpeedAndBias& speed_and_bias, 
    const double std_speed, const double std_bg, const double std_ba);

  // Add pose residual block to graph
  void addPoseResidualBlock(
    const State& state, const Transformation& T_WS, 
    const double std_roll_pitch, const double std_yaw);

  // Add yaw residual block to graph
  void addYawResidualBlock(
    const State& state, const double yaw, const double std_yaw);

  // Add roll and pitch residual block to graph
  void addRollPitchResidualBlock(
    const State& state, const Eigen::Vector2d& pitch_and_roll, 
    const double std_pitch_and_roll);

  // Add heading measurement constraint residual block
  void addHMCResidualBlock(const State& state);

  // Add non-holonomic constraint error
  void addNHCResidualBlock(const State& state);

  // Add zero-motion update constraint error
  void addZUPTResidualBlock(const State& state);

  // Inserts a state inside or at ends of state window
  // Retures the index of current added state in states_ buffer
  // The pose block and speed and bias block will be added here, note that we do not extrinsics
  // blocks here, it should be added by other sensor bases.
  // If you want add a state at the front, you must specify the prior values.
  size_t insertImuState(
    const double timestamp, const BackendId& backend_id,
    const Transformation& T_WS_prior = Transformation(), 
    const SpeedAndBias& speed_and_bias_prior = SpeedAndBias::Zero(),
    const bool use_prior = false);

  // Add IMU speed and bias block to marginalizer
  void addImuSpeedAndBiasParameterMarginBlock(const State& state, bool keep = false);

  // Add IMU speed and bias block with its residuals to marginalizer
  void addImuSpeedAndBiasParameterMarginBlockWithResiduals(const State& state, bool keep = false);

  // Add IMU pose block to marginalizer
  void addPoseParameterMarginBlock(const State& state, bool keep = false);

  // Add IMU pose block with its residuals to marginalizer
  void addPoseParameterMarginBlockWithResiduals(const State& state, bool keep = false);

  // Add IMU state to marginalizer with overlaping state check
  void addImuStateMarginBlock(const State& state, bool keep = false);

  // Add IMU state with its residuals to marginalizer with overlaping state check
  void addImuStateMarginBlockWithResiduals(const State& state, bool keep = false);

  // Add IMU pre-integration block to marginalizer
  void addImuResidualMarginBlock(const State& state);

  // Add IMU speed and bias residual block to marginalizer
  void addImuSpeedAndBiasResidualMarginBlock(const State& state);

  // Add IMU pose residual block to marginalizer
  void addPoseResidualMarginBlock(const State& state);

  // Add heading measurement constraint residual block to marginalizer
  void addHMCResidualMarginBlock(const State& state);

  // Add non-holonomic constraint residual block to marginalizer
  void addNHCResidualMarginBlock(const State& state);

  // Add all IMU residual blocks to marginalizer
  void addImuResidualMarginBlocks(const State& state);

  // Erase a IMU residual block
  void eraseImuResidualBlock(const State& last_state, State& cur_state);

  // Erase IMU speed and bias residual block
  void eraseImuSpeedAndBiasResidualBlock(const State& state);

  // Erase a state inside or at the ends of state window
  // The pose block and speed and bias block will be erased here, note that we do not extrinsics
  // blocks here, it should be erased by other sensor bases.
  void eraseImuState(const State& state);

  // Down-weight IMU residual block
  void downWeightImuResidualBlock(
    const ceres::ResidualBlockId residual_id, double factor);

  // Get a IMU measurement near given timestamp
  ImuMeasurement getImuMeasurementNear(const double timestamp);

protected:
  // Options
  ImuEstimatorBaseOptions imu_base_options_;

  // Measurements
  ImuMeasurements imu_measurements_;
  bool do_not_remove_imu_measurements_ = false;
  std::mutex imu_mutex_, imu_state_mutex_;

  // Body to IMU transformation to rotate variables
  Transformation T_BI_;

  // Flags
  bool gravity_setted_ = false;

  // For getXXXAt functions
  double last_timestamp_;
  State last_base_state_;
  Transformation last_T_WS_;
  SpeedAndBias last_speed_and_bias_;
  Eigen::Matrix<double, 15, 15> last_covariance_;
  bool need_covariance_ = false;
};

}