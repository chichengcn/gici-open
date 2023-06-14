/**
* @Function: IMU common functions
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
*
* Copyright (C) 2023 by Cheng Chi, All rights reserved.
**/
#include "gici/imu/imu_common.h"

namespace gici {

// Initialize IMU pose and biases with gravity vector
bool initPoseAndBiases(const ImuMeasurements& imu_measurements,
                       const double gravity,
                       Transformation& T_WS,
                       SpeedAndBias& speed_and_bias, 
                       const double timestamp_start, 
                       const double timestamp_end)
{
  // set to zero
  T_WS.setIdentity();
  speed_and_bias.setZero();

  if (imu_measurements.size() == 0) return false;

  // Check zero motion
#if 1
  int n_measurements = 0;
  int n_valid_measurements = 0;
  Eigen::Vector3d acc_B = Eigen::Vector3d::Zero();
  Eigen::Vector3d gyro_B = Eigen::Vector3d::Zero();
  for (size_t i = 0; i < imu_measurements.size(); ++i)
  {
    if (imu_measurements[i].timestamp < timestamp_start || 
        imu_measurements[i].timestamp > timestamp_end) continue;
    n_measurements++;

    // zero motion check
    // if (fabs(imu_measurements[i].linear_acceleration.norm() - gravity) > 0.5 || 
    //     fabs(imu_measurements[i].angular_velocity.norm()) > 0.05) continue;
    double acc_norm = fabs(imu_measurements[i].linear_acceleration.norm() - gravity);
    if (acc_norm > 0.5) continue;

    acc_B += imu_measurements[i].linear_acceleration;
    gyro_B += imu_measurements[i].angular_velocity;
    n_valid_measurements++;
  }
  if (n_measurements == 0) return false;
  else if (n_valid_measurements != n_measurements) return false;
  acc_B /= static_cast<double>(n_measurements);
  gyro_B /= static_cast<double>(n_measurements);
#else
  std::vector<double> acc[3], gyro[3];
  Eigen::Vector3d acc_B = Eigen::Vector3d::Zero();
  Eigen::Vector3d gyro_B = Eigen::Vector3d::Zero();
  for (auto imu : imu_measurements) {
    for (int i = 0; i < 3; i++) {
      acc[i].push_back(imu.linear_acceleration(i));
      gyro[i].push_back(imu.angular_velocity(i));
    }
    acc_B += imu.linear_acceleration;
    gyro_B += imu.angular_velocity;
  }
  double median_acc[3], median_gyro[3];
  double std_acc[3], std_gyro[3];
  for (int i = 0; i < 3; i++) {
    median_acc[i] = getMedian(acc[i]);
    median_gyro[i] = getMedian(gyro[i]);
    std_acc[i] = getStandardDeviation(acc[i], median_acc[i]);
    std_gyro[i] = getStandardDeviation(gyro[i], median_gyro[i]);
  }
  for (int i = 0; i < 3; i++) {
    if (std_acc[i] > 0.5) return false;
    if (std_gyro[i] > 0.02) return false;
    if (fabs(median_gyro[i]) > 0.008) return false;
  }
  acc_B /= static_cast<double>(imu_measurements.size());
  gyro_B /= static_cast<double>(imu_measurements.size());
#endif

  Eigen::Vector3d e_acc = acc_B.normalized();

  // align with ez_W:
  Eigen::Vector3d ez_W(0.0, 0.0, 1.0);
  Eigen::Matrix<double, 6, 1> pose_increment;
  pose_increment.head<3>() = Eigen::Vector3d::Zero();
  //! @todo this gives a bad result if ez_W.cross(e_acc) norm is
  //! close to zero, deal with it!
  pose_increment.tail<3>() = ez_W.cross(e_acc).normalized();
  double angle = std::acos(ez_W.transpose() * e_acc);
  pose_increment.tail<3>() *= angle;
  T_WS = Transformation::exp(-pose_increment) * T_WS;
  T_WS.getRotation().normalize();

  // biases
  // Eigen::Vector3d g(0.0, 0.0, gravity);
  // speed_and_bias.segment<3>(3) = gyro_B;
  // speed_and_bias.segment<3>(6) = 
  //   acc_B - T_WS.getRotationMatrix().transpose() * g;

  return true;
}

// Initialize IMU pose with body velocity in world frame
bool initYawFromVelocity(const Eigen::Vector3d& v_WS_W, double& yaw)
{
  yaw = atan2(v_WS_W(1), v_WS_W(0));

  return true;
}

}