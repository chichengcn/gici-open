/**
* @Function: Non-holonomic constraint error
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
*
* Copyright (C) 2023 by Cheng Chi, All rights reserved.
**/
#include "gici/imu/nhc_error.h"
#include "gici/estimate/estimator_types.h"
#include "gici/estimate/pose_local_parameterization.h"
#include "gici/utility/transform.h"
#include "gici/utility/common.h"

namespace gici {

// Construct with STD.
NHCError::NHCError(const double std)
{
  attitude_std_ = std;
}

// This evaluates the error term and additionally computes the Jacobians.
bool NHCError::Evaluate(double const* const * parameters, double* residuals,
                         double** jacobians) const
{
  return EvaluateWithMinimalJacobians(parameters, residuals, jacobians, nullptr);
}

// This evaluates the error term and additionally computes
// the Jacobians in the minimal internal representation.
bool NHCError::EvaluateWithMinimalJacobians(double const* const * parameters,
                                             double* residuals,
                                             double** jacobians,
                                             double** jacobians_minimal) const
{
  // compute error
  Transformation T_WS(
      Eigen::Vector3d(parameters[0][0], parameters[0][1], parameters[0][2]),
      Eigen::Quaterniond(parameters[0][6], parameters[0][3], parameters[0][4],
                         parameters[0][5]));
  SpeedAndBias speed_and_biases;
  for (size_t i = 0; i < 9; ++i) speed_and_biases[i] = parameters[1][i];
  Eigen::Vector3d v_WS_W = speed_and_biases.head<3>();
  Eigen::Vector3d v_WS_S = T_WS.getRotationMatrix().transpose() * v_WS_W;
  Eigen::Matrix<double, 2, 3> J_lower; 
  J_lower.setZero();
  J_lower(0, 0) = 1.0; J_lower(1, 2) = 1.0;
  Eigen::Vector2d error = J_lower * v_WS_S;

  // weigh it
  Eigen::Matrix3d J_q_full = 
    T_WS.getRotationMatrix().transpose() * skewSymmetric(v_WS_W);
  Eigen::Matrix<double, 2, 3> J_q = J_lower * J_q_full;
  Eigen::Matrix3d covariance_q = 
    Eigen::Matrix3d::Identity() * square(attitude_std_);
  covariance_ = J_q * covariance_q * J_q.transpose();
  information_ = covariance_.inverse();
  Eigen::LLT<information_t> lltOfInformation(information_);
  square_root_information_ = lltOfInformation.matrixL().transpose();
  square_root_information_inverse_ = square_root_information_.inverse();
  Eigen::Map<Eigen::Matrix<double, 2, 1> > weighted_error(residuals);
  weighted_error = square_root_information_ * error;

  // compute Jacobian...
  if (jacobians != nullptr)
  {
    if (jacobians[0] != nullptr)
    {
      Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor> >
          J0(jacobians[0]);
      Eigen::Matrix<double, 2, 6, Eigen::RowMajor> J0_minimal;
      Eigen::Matrix<double, 3, 6> J0_full;
      J0_full.setZero();
      J0_full.bottomRightCorner(3, 3) = 
        T_WS.getRotationMatrix().transpose() * skewSymmetric(v_WS_W);

      J0_minimal = J_lower * J0_full;
      J0_minimal = (square_root_information_ * J0_minimal).eval();

      // pseudo inverse of the local parametrization Jacobian:
      Eigen::Matrix<double, 6, 7, Eigen::RowMajor> J_lift;
      PoseLocalParameterization::liftJacobian(parameters[0], J_lift.data());

      // hallucinate Jacobian w.r.t. state
      J0 = J0_minimal * J_lift;

      if (jacobians_minimal != nullptr)
      {
        if (jacobians_minimal[0] != nullptr)
        {
          Eigen::Map<Eigen::Matrix<double, 2, 6, Eigen::RowMajor> >
              J0_minimal_mapped(jacobians_minimal[0]);
          J0_minimal_mapped = J0_minimal;
        }
      }
    }
    if (jacobians[1] != nullptr)
    {
      Eigen::Map<Eigen::Matrix<double, 2, 9, Eigen::RowMajor> >
          J1(jacobians[1]);
      Eigen::Matrix<double, 3, 9> J1_full;
      J1_full.setZero();
      J1_full.topLeftCorner(3, 3) = T_WS.getRotationMatrix().transpose();
      J1 = J_lower * J1_full;

      if (jacobians_minimal != nullptr)
      {
        if (jacobians_minimal[1] != nullptr)
        {
          Eigen::Map<Eigen::Matrix<double, 2, 9, Eigen::RowMajor> >
              J1_minimal_mapped(jacobians_minimal[1]);
          J1_minimal_mapped = J1;
        }
      }
    }
  }

  return true;
}

}  // namespace gici
