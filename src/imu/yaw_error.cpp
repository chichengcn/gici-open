/**
* @Function: Yaw error
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
*
* Copyright (C) 2023 by Cheng Chi, All rights reserved.
**/
#include "gici/imu/yaw_error.h"
#include "gici/estimate/estimator_types.h"
#include "gici/estimate/pose_local_parameterization.h"
#include "gici/utility/transform.h"
#include "gici/utility/svo.h"

namespace gici {

// Construct with measurement and information matrix.
YawError::YawError(const double measurement,
                   const double information)
{
  setMeasurement(measurement);
  information_t information_mat(information);
  setInformation(information_mat);
}

// Set the information.
void YawError::setInformation(const information_t& information)
{
  information_ = information;
  covariance_ = information.inverse();
  // perform the Cholesky decomposition on order to obtain the correct error weighting
  Eigen::LLT<information_t> lltOfInformation(information_);
  square_root_information_ = lltOfInformation.matrixL().transpose();
  square_root_information_inverse_ = square_root_information_.inverse();
}

// This evaluates the error term and additionally computes the Jacobians.
bool YawError::Evaluate(double const* const * parameters, double* residuals,
                         double** jacobians) const
{
  return EvaluateWithMinimalJacobians(parameters, residuals, jacobians, nullptr);
}

// This evaluates the error term and additionally computes
// the Jacobians in the minimal internal representation.
bool YawError::EvaluateWithMinimalJacobians(double const* const * parameters,
                                             double* residuals,
                                             double** jacobians,
                                             double** jacobians_minimal) const
{
  // compute error
  Transformation T_WS(
      Eigen::Vector3d(parameters[0][0], parameters[0][1], parameters[0][2]),
      Eigen::Quaterniond(parameters[0][6], parameters[0][3], parameters[0][4],
                         parameters[0][5]));
  // delta pose
  Eigen::Vector3d rpy = quaternionToEulerAngle(T_WS.getEigenQuaternion());
  rpy.z() = measurement_;
  Eigen::Quaterniond q_WS_meas = eulerAngleToQuaternion(rpy);
  Transformation T_WS_meas = Transformation(T_WS.getPosition(), q_WS_meas);
  Transformation dp = T_WS_meas * T_WS.inverse();
  // get the error
  Eigen::Matrix<double, 1, 1> error;
  const Eigen::Vector3d dtheta = 2 * dp.getRotation().imaginary();
  error = Eigen::Vector3d::UnitZ().transpose() * dtheta;

  // weigh it
  Eigen::Map<Eigen::Matrix<double, 1, 1> > weighted_error(residuals);
  weighted_error = square_root_information_ * error;

  // compute Jacobian...
  if (jacobians != nullptr)
  {
    if (jacobians[0] != nullptr)
    {
      Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor> >
          J0(jacobians[0]);
      Eigen::Matrix<double, 1, 6, Eigen::RowMajor> J0_minimal;
      Eigen::Matrix<double, 6, 6> J0_p2p;
      J0_p2p.setIdentity();
      J0_p2p *= -1.0;
      J0_p2p.block<3, 3>(3, 3) =
          -quaternionPlusMatrix(dp.getEigenQuaternion()).topLeftCorner<3, 3>();
      Eigen::Matrix<double, 1, 6> J_p2p_lift;
      J_p2p_lift.setZero(); J_p2p_lift(0, 5) = 1.0;
      J0_minimal = J_p2p_lift * J0_p2p;
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
          Eigen::Map<Eigen::Matrix<double, 1, 6, Eigen::RowMajor> >
              J0_minimal_mapped(jacobians_minimal[0]);
          J0_minimal_mapped = J0_minimal;
        }
      }
    }
  }

  return true;
}

}  // namespace gici
