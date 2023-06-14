/*********************************************************************************
 *  OKVIS - Open Keyframe-based Visual-Inertial SLAM
 *  Copyright (c) 2015, Autonomous Systems Lab / ETH Zurich
 *  Copyright (c) 2016, ETH Zurich, Wyss Zurich, Zurich Eye
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *   * Neither the name of Autonomous Systems Lab / ETH Zurich nor the names of
 *     its contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  Created on: Sep 3, 2013
 *      Author: Stefan Leutenegger (s.leutenegger@imperial.ac.uk)
 *    Modified: Andreas Forster (an.forster@gmail.com)
 *    Modified: Zurich Eye
 *    Modified: Cheng Chi
 *********************************************************************************/

/**
 * @file ImuError.cpp
 * @brief Source file for the ImuError class.
 * @author Stefan Leutenegger
 * @author Andreas Forster
 */

#include "gici/imu/imu_error.h"

#include <math.h>

#include "gici/estimate/pose_local_parameterization.h"
#include "gici/utility/transform.h"

namespace gici {

// Construct with measurements and parameters.
ImuError::ImuError(const ImuMeasurements &imu_measurements,
                   const ImuParameters& imu_parameters,
                   const double& t_0, const double& t_1)
{
  std::lock_guard<std::mutex> lock(preintegration_mutex_);
  setT0(t_0 - imu_parameters.delay_imu_cam);
  setT1(t_1 - imu_parameters.delay_imu_cam);
  setImuParameters(imu_parameters);
  setImuMeasurements(imu_measurements);

  double dt0 = t0_ - imu_measurements_.front().timestamp;
  double dt1 = t1_ - imu_measurements_.back().timestamp;
  if (dt0 < 0.0) {
    LOG(ERROR) << "First IMU measurement included in ImuError is not old enough: "
      << std::fixed << t0_ << " vs " << imu_measurements_.front().timestamp;
    imu_measurements_.push_front(imu_measurements_.front());
    imu_measurements_.front().timestamp = t0_;
  }
  if (dt1 > 0.0) {
    LOG(ERROR) << "Last IMU measurement included in ImuError is not new enough: "
      << std::fixed << t1_ << " vs " << imu_measurements_.back().timestamp;
    imu_measurements_.push_back(imu_measurements_.back());
    imu_measurements_.back().timestamp = t1_;
  }
}

// Propagates pose, speeds and biases with given IMU measurements.
int ImuError::redoPreintegration(const Transformation& /*T_WS*/,
                                 const SpeedAndBias& speed_and_biases) const
{
  // ensure unique access
  std::lock_guard<std::mutex> lock(preintegration_mutex_);

  // now the propagation
  double time = t0_;

  // sanity check:
  CHECK_LE(imu_measurements_.front().timestamp, time);
  CHECK_GE(imu_measurements_.back().timestamp, t1_);

  // increments (initialise with identity)
  Delta_q_ = Eigen::Quaterniond(1, 0, 0, 0);
  C_integral_ = Eigen::Matrix3d::Zero();
  C_doubleintegral_ = Eigen::Matrix3d::Zero();
  acc_integral_ = Eigen::Vector3d::Zero();
  acc_doubleintegral_ = Eigen::Vector3d::Zero();

  // sub-Jacobians
  dalpha_db_g_ = Eigen::Matrix3d::Zero();
  dv_db_g_ = Eigen::Matrix3d::Zero();
  dp_db_g_ = Eigen::Matrix3d::Zero();

  // the Jacobian of the increment (w/o biases)
  P_delta_ = Eigen::Matrix<double, 15, 15>::Zero();

  //Eigen::Matrix<double, 15, 15> F_tot;
  //F_tot.setIdentity();

  double delta_t = 0;
  bool has_started = false;
  bool last_iteration = false;
  int n_integrated = 0;
  for (size_t i = 0; i < imu_measurements_.size() - 1; i++)
  {
    if (!has_started && 
        !(imu_measurements_[i].timestamp <= t0_ && 
        imu_measurements_[i + 1].timestamp >= t0_)) continue;

    Eigen::Vector3d omega_S_0 = imu_measurements_[i].angular_velocity;
    Eigen::Vector3d acc_S_0 = imu_measurements_[i].linear_acceleration;
    Eigen::Vector3d omega_S_1 = imu_measurements_[i + 1].angular_velocity;
    Eigen::Vector3d acc_S_1 = imu_measurements_[i + 1].linear_acceleration;
    double nexttime = imu_measurements_[i + 1].timestamp;

    // time delta
    double dt = nexttime - time;

    if (t1_ < nexttime)
    {
      double interval = nexttime - imu_measurements_[i].timestamp;
      nexttime = t1_;
      last_iteration = true;
      dt = nexttime - time;
      const double r = dt / interval;
      omega_S_1 = ((1.0 - r) * omega_S_0 + r * omega_S_1).eval();
      acc_S_1 = ((1.0 - r) * acc_S_0 + r * acc_S_1).eval();
    }

    if (dt <= 0.0)
    {
      continue;
    }
    delta_t += dt;

    if (!has_started)
    {
      has_started = true;
      const double r = dt / (nexttime - imu_measurements_[i].timestamp);
      omega_S_0 = (r * omega_S_0 + (1.0 - r) * omega_S_1).eval();
      acc_S_0 = (r * acc_S_0 + (1.0 - r) * acc_S_1).eval();
    }

    // ensure integrity
    double sigma_g_c = imu_parameters_.sigma_g_c;
    double sigma_a_c = imu_parameters_.sigma_a_c;

    if (std::abs(omega_S_0[0]) > imu_parameters_.g_max
        || std::abs(omega_S_0[1]) > imu_parameters_.g_max
        || std::abs(omega_S_0[2]) > imu_parameters_.g_max
        || std::abs(omega_S_1[0]) > imu_parameters_.g_max
        || std::abs(omega_S_1[1]) > imu_parameters_.g_max
        || std::abs(omega_S_1[2]) > imu_parameters_.g_max)
    {
      sigma_g_c *= 100;
      LOG(WARNING)<< "gyr saturation: " 
                  << omega_S_0.norm() << ", " << omega_S_1.norm();
    }

    if (std::abs(acc_S_0[0]) > imu_parameters_.a_max
        || std::abs(acc_S_0[1]) > imu_parameters_.a_max
        || std::abs(acc_S_0[2]) > imu_parameters_.a_max
        || std::abs(acc_S_1[0]) > imu_parameters_.a_max
        || std::abs(acc_S_1[1]) > imu_parameters_.a_max
        || std::abs(acc_S_1[2]) > imu_parameters_.a_max)
    {
      sigma_a_c *= 100;
      LOG(WARNING) << "acc saturation: " 
                   << acc_S_0.norm() << ", " << acc_S_1.norm();
    }

    // actual propagation
    // orientation:
    Eigen::Quaterniond dq;
    const Eigen::Vector3d omega_S_true =
        (0.5 * (omega_S_0 + omega_S_1) - speed_and_biases.segment<3>(3));
    const double theta_half = omega_S_true.norm() * 0.5 * dt;
    const double sinc_theta_half = sinc(theta_half);
    const double cos_theta_half = cos(theta_half);
    dq.vec() = sinc_theta_half * omega_S_true * 0.5 * dt;
    dq.w() = cos_theta_half;
    Eigen::Quaterniond Delta_q_1 = Delta_q_ * dq;
    // rotation matrix integral:
    const Eigen::Matrix3d C = Delta_q_.toRotationMatrix();
    const Eigen::Matrix3d C_1 = Delta_q_1.toRotationMatrix();
    const Eigen::Vector3d acc_S_true =
        (0.5 * (acc_S_0 + acc_S_1) - speed_and_biases.segment<3>(6));
    const Eigen::Matrix3d C_integral_1 = C_integral_ + 0.5 * (C + C_1) * dt;
    const Eigen::Vector3d acc_integral_1 =
        acc_integral_ + 0.5 * (C + C_1) * acc_S_true * dt;
    // rotation matrix double integral:
    C_doubleintegral_ += C_integral_ * dt + 0.25 * (C + C_1) * dt * dt;
    acc_doubleintegral_ +=
        acc_integral_ * dt + 0.25 * (C + C_1) * acc_S_true * dt * dt;

    // Jacobian parts
    Eigen::Matrix3d dalpha_db_g_0 = dalpha_db_g_;
    dalpha_db_g_ += -C_1.transpose() * expmapDerivativeSO3(omega_S_true) * dt;
    const Eigen::Matrix3d acc_S_x = skewSymmetric(acc_S_true);
    Eigen::Matrix3d dv_db_g_1 = dv_db_g_ - 
      0.5 * dt * (C * acc_S_x * dalpha_db_g_0 + C_1 * acc_S_x * dalpha_db_g_);
    dp_db_g_ +=
        dt * dv_db_g_
        - 0.25 * dt * dt * (C * acc_S_x * dalpha_db_g_0 + C_1 * acc_S_x * dalpha_db_g_);

    // covariance propagation
    Eigen::Matrix<double, 15, 15> F_delta =
        Eigen::Matrix<double, 15, 15>::Identity();
    // transform
    F_delta.block<3, 3>(0, 6) = Eigen::Matrix3d::Identity() * dt;
    F_delta.block<3, 3>(3, 3) = -skewSymmetric(omega_S_true * dt);
    F_delta.block<3, 3>(3, 9) = -C * dt;
    F_delta.block<3, 3>(6, 3) =
        -skewSymmetric(0.5 * (C + C_1) * acc_S_true * dt);
    F_delta.block<3, 3>(6, 12) = -C * dt;
    P_delta_ = F_delta * P_delta_ * F_delta.transpose();
    // add noise. Note that transformations with rotation matrices can be
    // ignored, since the noise is isotropic.
    //F_tot = F_delta*F_tot;
    const double sigma2_dalpha = dt * sigma_g_c * sigma_g_c;
    P_delta_(3, 3) += sigma2_dalpha;
    P_delta_(4, 4) += sigma2_dalpha;
    P_delta_(5, 5) += sigma2_dalpha;
    const double sigma2_v = dt * sigma_a_c * sigma_a_c;
    P_delta_(6, 6) += sigma2_v;
    P_delta_(7, 7) += sigma2_v;
    P_delta_(8, 8) += sigma2_v;
    const double sigma2_p = 0.5 * dt * dt * sigma2_v;
    P_delta_(0, 0) += sigma2_p;
    P_delta_(1, 1) += sigma2_p;
    P_delta_(2, 2) += sigma2_p;
    const double sigma2_b_g =
        dt * imu_parameters_.sigma_gw_c * imu_parameters_.sigma_gw_c;
    P_delta_(9, 9) += sigma2_b_g;
    P_delta_(10, 10) += sigma2_b_g;
    P_delta_(11, 11) += sigma2_b_g;
    const double sigma2_b_a =
        dt * imu_parameters_.sigma_aw_c* imu_parameters_.sigma_aw_c;
    P_delta_(12, 12) += sigma2_b_a;
    P_delta_(13, 13) += sigma2_b_a;
    P_delta_(14, 14) += sigma2_b_a;

    // memory shift
    Delta_q_ = Delta_q_1;
    C_integral_ = C_integral_1;
    acc_integral_ = acc_integral_1;
    dv_db_g_ = dv_db_g_1;
    time = nexttime;

    ++n_integrated;

    if (last_iteration)
      break;

  }

  // store the reference (linearisation) point
  speed_and_biases_ref_ = speed_and_biases;

  // get the weighting:
  // enforce symmetric
  P_delta_ = 0.5 * P_delta_ + 0.5 * P_delta_.transpose().eval();

  // calculate inverse
  information_ = P_delta_.inverse();
  information_ = 0.5 * information_ + 0.5 * information_.transpose().eval();

  // square root
  Eigen::LLT<information_t> lltOfInformation(information_);
  square_root_information_ = lltOfInformation.matrixL().transpose();

  //std::cout << F_tot;

  return n_integrated;
}

// Propagates pose, speeds and biases with given IMU measurements.
int ImuError::propagation(const ImuMeasurements& imu_measurements,
                          const ImuParameters& imu_params,
                          Transformation& T_WS,
                          SpeedAndBias & speed_and_biases,
                          const double& t_start,
                          const double& t_end,
                          covariance_t* covariance,
                          jacobian_t* jacobian)
{
  const double t_start_adjusted = t_start - imu_params.delay_imu_cam;
  const double t_end_adjusted = t_end - imu_params.delay_imu_cam;
  // sanity check:
  bool modify_front = false, modify_back = false;;
  ImuMeasurements imu_measurements_modified;
  double dt0 = t_start_adjusted - imu_measurements.front().timestamp;
  double dt1 = t_end_adjusted - imu_measurements.back().timestamp;
  if (dt0 < 0.0) {
    LOG(ERROR) << "First IMU measurement included in imu_measurements is not old enough: "
      << std::fixed << t_start_adjusted << " vs " << imu_measurements.front().timestamp;
    modify_front = true;
  }
  if (dt1 > 0.0) {
    LOG(ERROR) << "Last IMU measurement included in imu_measurements is not new enough: "
      << std::fixed << t_end_adjusted << " vs " << imu_measurements.back().timestamp;
    modify_back = true;
  }
  if (modify_front || modify_back) imu_measurements_modified = imu_measurements;
  if (modify_front) {
    imu_measurements_modified.push_front(imu_measurements.front());
    imu_measurements_modified.front().timestamp = t_start_adjusted;
  }
  if (modify_back) {
    imu_measurements_modified.push_back(imu_measurements.back());
    imu_measurements_modified.back().timestamp = t_end_adjusted;
  }
  const ImuMeasurements *imu_measurements_ptr = 
    (modify_front || modify_back) ? &imu_measurements_modified : &imu_measurements;
  CHECK_LE(imu_measurements_ptr->front().timestamp, t_start_adjusted);
  CHECK_GE(imu_measurements_ptr->back().timestamp, t_end_adjusted);

  // initial condition
  Eigen::Vector3d r_0 = T_WS.getPosition();
  Eigen::Quaterniond q_WS_0 = T_WS.getEigenQuaternion();
  Eigen::Matrix3d C_WS_0 = T_WS.getRotationMatrix();

  // increments (initialise with identity)
  Eigen::Quaterniond Delta_q(1,0,0,0);
  Eigen::Matrix3d C_integral = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d C_doubleintegral = Eigen::Matrix3d::Zero();
  Eigen::Vector3d acc_integral = Eigen::Vector3d::Zero();
  Eigen::Vector3d acc_doubleintegral = Eigen::Vector3d::Zero();

  // jacobian
  Eigen::Matrix<double, 15, 15> F = Eigen::Matrix<double, 15, 15>::Identity();

  // the Jacobian of the increment (w/o biases)
  Eigen::Matrix<double,15,15> P_delta = Eigen::Matrix<double,15,15>::Zero();

  double Delta_t = 0;
  bool has_started = false;
  int num_propagated = 0;

  double time = t_start_adjusted;
  const ImuMeasurements& imu = *imu_measurements_ptr;
  for (size_t i = 0; i < imu.size() - 1; i++)
  {
    if (!has_started && 
        !(imu[i].timestamp <= t_start_adjusted && 
        imu[i + 1].timestamp >= t_start_adjusted)) continue;

    Eigen::Vector3d omega_S_0 = imu[i].angular_velocity;
    Eigen::Vector3d acc_S_0 = imu[i].linear_acceleration;
    Eigen::Vector3d omega_S_1 = imu[i + 1].angular_velocity;
    Eigen::Vector3d acc_S_1 = imu[i + 1].linear_acceleration;
    double nexttime = imu[i + 1].timestamp;

    // time delta
    double dt = nexttime - time;

    // interpolate to end time point
    if (t_end_adjusted < nexttime)
    {
      double interval = nexttime - imu[i].timestamp;
      nexttime = t_end_adjusted;
      dt = nexttime - time;
      const double r = dt / interval;
      omega_S_1 = ((1.0 - r) * omega_S_0 + r * omega_S_1).eval();
      acc_S_1 = ((1.0 - r) * acc_S_0 + r * acc_S_1).eval();
    }

    if (dt <= 0.0)
    {
      continue;
    }
    Delta_t += dt;

    // interpolate to first time point
    if (!has_started)
    {
      has_started = true;
      const double r = dt / (nexttime - imu[i].timestamp);
      omega_S_0 = (r * omega_S_0 + (1.0 - r) * omega_S_1).eval();
      acc_S_0 = (r * acc_S_0 + (1.0 - r) * acc_S_1).eval();
    }

    // ensure integrity
    double sigma_g_c = imu_params.sigma_g_c;
    double sigma_a_c = imu_params.sigma_a_c;

    if (std::abs(omega_S_0[0]) > imu_params.g_max
        || std::abs(omega_S_0[1]) > imu_params.g_max
        || std::abs(omega_S_0[2]) > imu_params.g_max
        || std::abs(omega_S_1[0]) > imu_params.g_max
        || std::abs(omega_S_1[1]) > imu_params.g_max
        || std::abs(omega_S_1[2]) > imu_params.g_max)
    {
      sigma_g_c *= 100;
      LOG(WARNING)<< "gyr saturation: " 
                  << omega_S_0.norm() << ", " << omega_S_1.norm();
    }

    if (std::abs(acc_S_0[0]) > imu_params.a_max
        || std::abs(acc_S_0[1]) > imu_params.a_max
        || std::abs(acc_S_0[2]) > imu_params.a_max
        || std::abs(acc_S_1[0]) > imu_params.a_max
        || std::abs(acc_S_1[1]) > imu_params.a_max
        || std::abs(acc_S_1[2]) > imu_params.a_max)
    {
      sigma_a_c *= 100;
      LOG(WARNING) << "acc saturation: " 
                   << acc_S_0.norm() << ", " << acc_S_1.norm();
    }

    // actual propagation
    // orientation:
    Eigen::Quaterniond dq;
    const Eigen::Vector3d omega_S_true =
        (0.5 *(omega_S_0+omega_S_1) - speed_and_biases.segment<3>(3));
    const double theta_half = omega_S_true.norm() * 0.5 * dt;
    const double sinc_theta_half = sinc(theta_half);
    const double cos_theta_half = cos(theta_half);
    dq.vec() = sinc_theta_half * omega_S_true * 0.5 * dt;
    dq.w() = cos_theta_half;
    Eigen::Quaterniond Delta_q_1 = Delta_q * dq;
    // rotation matrix integral:
    const Eigen::Matrix3d C = Delta_q.toRotationMatrix();
    const Eigen::Matrix3d C_1 = Delta_q_1.toRotationMatrix();
    const Eigen::Vector3d acc_S_true =
        (0.5 * (acc_S_0 + acc_S_1) - speed_and_biases.segment<3>(6));
    const Eigen::Matrix3d C_integral_1 = C_integral + 0.5 *(C + C_1) * dt;
    const Eigen::Vector3d acc_integral_1 =
        acc_integral + 0.5 * (C + C_1) * acc_S_true * dt;
    // rotation matrix double integral:
    C_doubleintegral += C_integral * dt + 0.25 * (C + C_1) * dt * dt;
    acc_doubleintegral +=
        acc_integral * dt + 0.25 * (C + C_1) * acc_S_true * dt * dt;

    // Jacobian 
    Eigen::Matrix<double,15,15> F_delta =
        Eigen::Matrix<double,15,15>::Identity();
    if (jacobian || covariance)
    {
      F_delta.block<3, 3>(0, 6) = Eigen::Matrix3d::Identity() * dt;
      F_delta.block<3, 3>(3, 3) = -skewSymmetric(omega_S_true * dt);
      F_delta.block<3, 3>(3, 9) = -C * dt;
      F_delta.block<3, 3>(6, 3) =
          -skewSymmetric(0.5 * (C + C_1) * acc_S_true * dt);
      F_delta.block<3, 3>(6, 12) = -C * dt;
    }
    if (jacobian)
    {
      F = F_delta * F;
    }

    // covariance propagation
    if (covariance)
    {
      // transform
      P_delta = F_delta * P_delta * F_delta.transpose();
      // add noise. Note that transformations with rotation matrices can be
      // ignored, since the noise is isotropic.
      //F_tot = F_delta*F_tot;
      const double sigma2_dalpha = dt * sigma_g_c * sigma_g_c;
      P_delta(3,3) += sigma2_dalpha;
      P_delta(4,4) += sigma2_dalpha;
      P_delta(5,5) += sigma2_dalpha;
      const double sigma2_v = dt * sigma_a_c * imu_params.sigma_a_c;
      P_delta(6,6) += sigma2_v;
      P_delta(7,7) += sigma2_v;
      P_delta(8,8) += sigma2_v;
      const double sigma2_p = 0.5 * dt * dt *sigma2_v;
      P_delta(0,0) += sigma2_p;
      P_delta(1,1) += sigma2_p;
      P_delta(2,2) += sigma2_p;
      const double sigma2_b_g =
          dt * imu_params.sigma_gw_c * imu_params.sigma_gw_c;
      P_delta(9,9)   += sigma2_b_g;
      P_delta(10,10) += sigma2_b_g;
      P_delta(11,11) += sigma2_b_g;
      const double sigma2_b_a =
          dt * imu_params.sigma_aw_c * imu_params.sigma_aw_c;
      P_delta(12,12) += sigma2_b_a;
      P_delta(13,13) += sigma2_b_a;
      P_delta(14,14) += sigma2_b_a;
    }

    // memory shift
    Delta_q = Delta_q_1;
    C_integral = C_integral_1;
    acc_integral = acc_integral_1;
    time = nexttime;

    ++num_propagated;

    if (nexttime == t_end_adjusted)
      break;

  }

  // actual propagation output:
  const Eigen::Vector3d g_W = imu_params.g * Eigen::Vector3d(0, 0, 1.0);
  T_WS =
      Transformation(
        r_0 + speed_and_biases.head<3>() * Delta_t
        + C_WS_0 * (acc_doubleintegral/*-C_doubleintegral*speedAndBiases.segment<3>(6)*/)
        - 0.5 * g_W * Delta_t * Delta_t,
        q_WS_0 * Delta_q);
  speed_and_biases.head<3>() +=
      C_WS_0 * (acc_integral/*-C_integral*speedAndBiases.segment<3>(6)*/)
      - g_W * Delta_t;

  // assign Jacobian, if requested
  if (jacobian)
  {
    *jacobian = F;
  }

  // overall covariance, if requested
  if (covariance)
  {
    Eigen::Matrix<double,15,15>& P = *covariance;
    // transform from local increments to actual states
    Eigen::Matrix<double,15,15> T = Eigen::Matrix<double,15,15>::Identity();
    T.topLeftCorner<3,3>() = C_WS_0;
    T.block<3,3>(3,3) = C_WS_0;
    T.block<3,3>(6,6) = C_WS_0;
    P = T * P_delta * T.transpose();
  }
  return num_propagated;
}

// This evaluates the error term and additionally computes the Jacobians.
bool ImuError::Evaluate(double const* const * parameters, double* residuals,
                        double** jacobians) const
{
  return EvaluateWithMinimalJacobians(parameters, residuals, jacobians, nullptr);
}

// This evaluates the error term and additionally computes
// the Jacobians in the minimal internal representation.
bool ImuError::EvaluateWithMinimalJacobians(double const* const * parameters,
                                            double* residuals,
                                            double** jacobians,
                                            double** jacobians_minimal) const
{
  // get poses
  const Transformation T_WS_0(
        Eigen::Vector3d(parameters[0][0], parameters[0][1], parameters[0][2]),
      Eigen::Quaterniond(parameters[0][6], parameters[0][3],
      parameters[0][4], parameters[0][5]));

  const Transformation T_WS_1(
        Eigen::Vector3d(parameters[2][0], parameters[2][1], parameters[2][2]),
      Eigen::Quaterniond(parameters[2][6], parameters[2][3],
      parameters[2][4], parameters[2][5]));

  // get speed and bias
  SpeedAndBias speed_and_biases_0;
  SpeedAndBias speed_and_biases_1;
  for (size_t i = 0; i < 9; ++i)
  {
    speed_and_biases_0[i] = parameters[1][i];
    speed_and_biases_1[i] = parameters[3][i];
  }

  // this will NOT be changed:
  const Eigen::Matrix3d C_WS_0 = T_WS_0.getRotationMatrix();
  const Eigen::Matrix3d C_S0_W = C_WS_0.transpose();

  // call the propagation
  const double delta_t = t1_ - t0_;
  Eigen::Matrix<double, 6, 1> Delta_b;
  // ensure unique access
  {
    std::lock_guard<std::mutex> lock(preintegration_mutex_);
    Delta_b = speed_and_biases_0.tail<6>()
        - speed_and_biases_ref_.tail<6>();
  }
  redo_ = redo_ || (Delta_b.head<3>().norm() * delta_t > 0.0001);
  if (redo_)
  {
    redoPreintegration(T_WS_0, speed_and_biases_0);
    redoCounter_++;
    Delta_b.setZero();
    redo_ = false;
    /*if (redoCounter_ > 1) {
      std::cout << "pre-integration no. " << redoCounter_ << std::endl;
    }*/
  }

  // actual propagation output:
  {
    std::lock_guard<std::mutex> lock(preintegration_mutex_);
    // this is a bit stupid, but shared read-locks only come in C++14
    const Eigen::Vector3d g_W = Eigen::Vector3d(0, 0, imu_parameters_.g);

    // assign Jacobian w.r.t. x0
    Eigen::Matrix<double,15,15> F0 =
        Eigen::Matrix<double,15,15>::Identity(); // holds for d/db_g, d/db_a
    const Eigen::Vector3d delta_p_est_W =
        T_WS_0.getPosition() - T_WS_1.getPosition()
        + speed_and_biases_0.head<3>()* delta_t - 0.5 * g_W* delta_t * delta_t;
    const Eigen::Vector3d delta_v_est_W = speed_and_biases_0.head<3>()
        - speed_and_biases_1.head<3>() - g_W * delta_t;
    const Eigen::Quaterniond Dq =
        deltaQ(dalpha_db_g_*Delta_b.head<3>())*Delta_q_;
    F0.block<3,3>(0,0) = C_S0_W;
    F0.block<3,3>(0,3) = C_S0_W * skewSymmetric(delta_p_est_W);
    F0.block<3,3>(0,6) = C_S0_W * Eigen::Matrix3d::Identity()* delta_t;
    F0.block<3,3>(0,9) = dp_db_g_;
    F0.block<3,3>(0,12) = -C_doubleintegral_;
    F0.block<3,3>(3,3) =
        (quaternionPlusMatrix(Dq * T_WS_1.getEigenQuaternion().inverse()) *
         quaternionOplusMatrix(T_WS_0.getEigenQuaternion())).topLeftCorner<3,3>();
    F0.block<3,3>(3,9) =
        (quaternionOplusMatrix(T_WS_1.getEigenQuaternion().inverse() *
                               T_WS_0.getEigenQuaternion()) *
         quaternionOplusMatrix(Dq)).topLeftCorner<3,3>() * (dalpha_db_g_);
    F0.block<3,3>(6,3) = C_S0_W * skewSymmetric(delta_v_est_W);
    F0.block<3,3>(6,6) = C_S0_W;
    F0.block<3,3>(6,9) = dv_db_g_;
    F0.block<3,3>(6,12) = -C_integral_;

    // assign Jacobian w.r.t. x1
    Eigen::Matrix<double,15,15> F1 =
        -Eigen::Matrix<double,15,15>::Identity(); // holds for the biases
    F1.block<3,3>(0,0) = -C_S0_W;
    F1.block<3,3>(3,3) =
        -(quaternionPlusMatrix(Dq) *
          quaternionOplusMatrix(T_WS_0.getEigenQuaternion()) *
          quaternionPlusMatrix(T_WS_1.getEigenQuaternion().inverse()))
        .topLeftCorner<3,3>();
    F1.block<3,3>(6,6) = -C_S0_W;

    // the overall error vector
    Eigen::Matrix<double, 15, 1> error;
    error.segment<3>(0) =
        C_S0_W * delta_p_est_W + acc_doubleintegral_ + F0.block<3,6>(0,9)*Delta_b;
    error.segment<3>(3) =
        2.0 * (Dq * (T_WS_1.getEigenQuaternion().inverse() *
                     T_WS_0.getEigenQuaternion())).vec();
    //2*T_WS_0.q()*Dq*T_WS_1.q().inverse();//
    error.segment<3>(6) =
        C_S0_W * delta_v_est_W + acc_integral_ + F0.block<3,6>(6,9)*Delta_b;
    error.tail<6>() = speed_and_biases_0.tail<6>() - speed_and_biases_1.tail<6>();

    // error weighting
    Eigen::Map<Eigen::Matrix<double, 15, 1> > weighted_error(residuals);
    weighted_error = (1.0 / down_weight_factor_) * square_root_information_ * error;

    // get the Jacobians
    if (jacobians != nullptr)
    {
      if (jacobians[0] != nullptr)
      {
        // Jacobian w.r.t. minimal perturbance
        Eigen::Matrix<double, 15, 6> J0_minimal =
            (1.0 / down_weight_factor_) * square_root_information_ * F0.block<15, 6>(0, 0);

        // pseudo inverse of the local parametrization Jacobian:
        Eigen::Matrix<double, 6, 7, Eigen::RowMajor> J_lift;
        PoseLocalParameterization::liftJacobian(parameters[0], J_lift.data());

        // hallucinate Jacobian w.r.t. state
        Eigen::Map<Eigen::Matrix<double, 15, 7, Eigen::RowMajor> > J0(
              jacobians[0]);
        J0 = J0_minimal * J_lift;

        // if requested, provide minimal Jacobians
        if (jacobians_minimal != nullptr)
        {
          if (jacobians_minimal[0] != nullptr)
          {
            Eigen::Map<Eigen::Matrix<double, 15, 6, Eigen::RowMajor> >
                J0_minimal_mapped(jacobians_minimal[0]);
            J0_minimal_mapped = J0_minimal;
          }
        }
      }
      if (jacobians[1] != nullptr)
      {
        Eigen::Map<Eigen::Matrix<double, 15, 9, Eigen::RowMajor> >
            J1(jacobians[1]);
        J1 = (1.0 / down_weight_factor_) * square_root_information_ * F0.block<15, 9>(0, 6);

        // if requested, provide minimal Jacobians
        if (jacobians_minimal != nullptr)
        {
          if (jacobians_minimal[1] != nullptr)
          {
            Eigen::Map<Eigen::Matrix<double, 15, 9, Eigen::RowMajor> >
                J1_minimal_mapped(jacobians_minimal[1]);
            J1_minimal_mapped = J1;
          }
        }
      }
      if (jacobians[2] != nullptr)
      {
        // Jacobian w.r.t. minimal perturbance
        Eigen::Matrix<double, 15, 6> J2_minimal = 
            (1.0 / down_weight_factor_) * square_root_information_
            * F1.block<15, 6>(0, 0);

        // pseudo inverse of the local parametrization Jacobian:
        Eigen::Matrix<double, 6, 7, Eigen::RowMajor> J_lift;
        PoseLocalParameterization::liftJacobian(parameters[2], J_lift.data());

        // hallucinate Jacobian w.r.t. state
        Eigen::Map<Eigen::Matrix<double, 15, 7, Eigen::RowMajor> > J2(
              jacobians[2]);
        J2 = J2_minimal * J_lift;

        // if requested, provide minimal Jacobians
        if (jacobians_minimal != nullptr)
        {
          if (jacobians_minimal[2] != nullptr)
          {
            Eigen::Map<Eigen::Matrix<double, 15, 6, Eigen::RowMajor> >
                J2_minimal_mapped(jacobians_minimal[2]);
            J2_minimal_mapped = J2_minimal;
          }
        }
      }
      if (jacobians[3] != nullptr)
      {
        Eigen::Map<Eigen::Matrix<double, 15, 9, Eigen::RowMajor> >
            J3(jacobians[3]);
        J3 = (1.0 / down_weight_factor_) * square_root_information_ * F1.block<15, 9>(0, 6);

        // if requested, provide minimal Jacobians
        if (jacobians_minimal != nullptr)
        {
          if (jacobians_minimal[3] != nullptr)
          {
            Eigen::Map<Eigen::Matrix<double, 15, 9, Eigen::RowMajor> >
                J3_minimal_mapped(jacobians_minimal[3]);
            J3_minimal_mapped = J3;
          }
        }
      }
    }
  }
  return true;
}

}  // namespace gici
