/**
* @Function: IMU common functions
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
*
* Copyright (C) 2023 by Cheng Chi, All rights reserved.
**/
#pragma once

#include <iostream>
#include <memory>
#include <map>
#include <Eigen/Core>

#include "gici/imu/imu_types.h"
#include "gici/utility/svo.h"
#include "gici/utility/common.h"

namespace gici {

// Initialize IMU pose and biases with gravity vector
bool initPoseAndBiases(const ImuMeasurements& imu_measurements,
                       const double gravity,
                       Transformation& T_WS,
                       SpeedAndBias& speed_and_bias, 
                       const double timestamp_start = 0.0, 
                       const double timestamp_end = 1.0e12);

// Initialize IMU pose with body velocity in world frame
bool initYawFromVelocity(const Eigen::Vector3d& v_WS_W, double& yaw);

// Get earth gravity at a given position
// input should be in rad
inline double earthGravity(const Eigen::Vector3d& lla) {
  return 9.7803267715 * (1 + 0.0052790414 * square(sin(lla[0])) + 0.0000232718 * 
         pow(sin(lla[0]), 4)) + lla[2] * (0.0000000043977311 * square(sin(lla[0])) - 
         0.0000030876910891) + 0.0000000000007211 * square(lla[2]);
}

}