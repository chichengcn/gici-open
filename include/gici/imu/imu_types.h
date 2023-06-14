/**
* @Function: IMU types
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
*
* Copyright (C) 2023 by Cheng Chi, All rights reserved.
**/
#pragma once

#include <iostream>
#include <vector>
#include <deque>
#include <memory>
#include <Eigen/Core>

namespace gici {

// Role of formator
enum class ImuRole {
  None,
  Major, 
  Minor
};

// IMU measurement
struct ImuMeasurement
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  double timestamp; ///< In seconds.
  Eigen::Vector3d angular_velocity;
  Eigen::Vector3d linear_acceleration;
  ImuMeasurement() {}
  ImuMeasurement(
      const double timestamp,
      const Eigen::Vector3d& angular_velocity,
      const Eigen::Vector3d& linear_acceleration)
  : timestamp(timestamp)
  , angular_velocity(angular_velocity)
  , linear_acceleration(linear_acceleration)
  {}
};
typedef std::deque<ImuMeasurement,
Eigen::aligned_allocator<ImuMeasurement> > ImuMeasurements;

// A simple struct to specify properties of an IMU.
struct ImuParameters
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  double a_max = 150;  ///< Accelerometer saturation. [m/s^2]
  double g_max = 7.8;  ///< Gyroscope saturation. [rad/s]
  double sigma_g_c = 1.0e-4;  ///< Gyroscope noise density. [rad/s*1/sqrt(Hz)]
  double sigma_bg = 0.01; // Initial gyroscope bias uncertainty
  ///< Initial gyroscope bias uncertainty. [rad/s*1/sqrt(Hz)]
  double sigma_a_c = 2.0e-3;  ///< Accelerometer noise density. [m/s^2*1/sqrt(Hz)]
  double sigma_ba = 0.1; // Initial accelerometer bias uncertainty
  ///< Initial accelerometer bias uncertainty. [m/s^2*1/sqrt(Hz)]
  double sigma_gw_c = 2.1e-5; ///< Gyroscope drift noise density. [rad/s^2*1/sqrt(Hz)]
  double sigma_aw_c = 8.4e-4; ///< Accelerometer drift noise density. [m/s^3*1/sqrt(Hz)]
  double g = 9.8;  ///< Earth acceleration. [m/s^2]
  ///< Mean of the prior acceleration bias. [m/s^2*1/sqrt(Hz)]
  double rate = 400;  ///< IMU rate. [Hz].
  double delay_imu_cam = 0.0;
  ///< Camera-IMU delay: delay_imu_cam = cam_timestamp - imu_timestamp [s]
};

// [velocity, gyro biases, accel biases]
typedef Eigen::Matrix<double, 9, 1> SpeedAndBias;

}
