/**
* @Function: GNSS Position error used for loosely integration with other sensors
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
*
* Copyright (C) 2023 by Cheng Chi, All rights reserved.
**/
#include "gici/gnss/position_error.h"

#include "gici/gnss/gnss_common.h"
#include "gici/utility/transform.h"
#include "gici/estimate/pose_local_parameterization.h"

namespace gici {

// Construct with measurement and information matrix
template<int... Ns>
PositionError<Ns ...>::PositionError(
                    const Eigen::Vector3d& position_measurement,
                    const Eigen::Matrix3d& information)
{
  setMeasurement(position_measurement);
  setInformation(information);

  // Check parameter block types
  // Group 1
  if (dims_.kNumParameterBlocks == 1 && 
      dims_.GetDim(0) == 3) {
    is_estimate_body_ = false;
    parameter_block_group_ = 1;
  }
  // Group 2
  else if (dims_.kNumParameterBlocks == 2 &&
      dims_.GetDim(0) == 7 && dims_.GetDim(1) == 3) {
    is_estimate_body_ = true;
    parameter_block_group_ = 2;
  }
  else {
    LOG(FATAL) << "PositionError parameter blocks setup invalid!";
  }
}

// This evaluates the error term and additionally computes the Jacobians.
template<int... Ns>
bool PositionError<Ns ...>::Evaluate(double const* const * parameters,
                                 double* residuals, double** jacobians) const
{
  return EvaluateWithMinimalJacobians(parameters, residuals, jacobians, nullptr);
}

// This evaluates the error term and additionally computes
// the Jacobians in the minimal internal representation.
template<int... Ns>
bool PositionError<Ns ...>::EvaluateWithMinimalJacobians(
    double const* const * parameters, double* residuals, double** jacobians,
    double** jacobians_minimal) const
{
  Eigen::Vector3d t_WR_ECEF, t_WS_W, t_SR_S;
  Eigen::Quaterniond q_WS;
  
  // Position and clock
  if (!is_estimate_body_) 
  {
    t_WR_ECEF = Eigen::Map<const Eigen::Vector3d>(parameters[0]);
  }
  else 
  {
    // pose in ENU frame
    t_WS_W = Eigen::Map<const Eigen::Vector3d>(&parameters[0][0]);
    q_WS = Eigen::Map<const Eigen::Quaterniond>(&parameters[0][3]);

    // relative position
    t_SR_S = Eigen::Map<const Eigen::Vector3d>(parameters[1]);

    // receiver position
    Eigen::Vector3d t_WR_W = t_WS_W + q_WS * t_SR_S;

    if (!coordinate_) {
      LOG(FATAL) << "Coordinate not set!";
    }
    if (!coordinate_->isZeroSetted()) {
      LOG(FATAL) << "Coordinate zero not set!";
    }
    t_WR_ECEF = coordinate_->convert(t_WR_W, GeoType::ENU, GeoType::ECEF);
  }

  // Compute error
  Eigen::Vector3d error = measurement_ - t_WR_ECEF;

  // weigh it
  Eigen::Map<Eigen::Vector3d> weighted_error(residuals);
  weighted_error = square_root_information_ * error;

  // compute Jacobian
  if (jacobians != nullptr)
  {
    // Receiver position in ECEF
    Eigen::Matrix<double, 3, 3> J_t_ECEF = -Eigen::Matrix3d::Identity();
    
    // Poses
    Eigen::Matrix<double, 3, 6> J_T_WS;
    Eigen::Matrix<double, 3, 3> J_t_SR_S;
    if (is_estimate_body_) {
      // Body position in ENU
      Eigen::Matrix<double, 3, 3> J_t_W = J_t_ECEF * 
        coordinate_->rotationMatrix(GeoType::ENU, GeoType::ECEF);

      // Body rotation in ENU
      Eigen::Matrix<double, 3, 3> J_q_WS = J_t_W * 
        -skewSymmetric(q_WS.toRotationMatrix() * t_SR_S);

      // Body pose in ENU
      J_T_WS.setZero();
      J_T_WS.topLeftCorner(3, 3) = J_t_W;
      J_T_WS.topRightCorner(3, 3) = J_q_WS;

      // Relative position 
      J_t_SR_S = J_t_W * q_WS.toRotationMatrix();
    }

    // Group 1
    if (parameter_block_group_ == 1) 
    {
      // Position
      if (jacobians[0] != nullptr) {
        Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> J0(jacobians[0]);
        J0 = square_root_information_ * J_t_ECEF;
        
        if (jacobians_minimal != nullptr && jacobians_minimal[0] != nullptr) {
          Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor> >
              J0_minimal_mapped(jacobians_minimal[0]);
          J0_minimal_mapped = J0;
        }
      }
    }
    // Group 2
    if (parameter_block_group_ == 2)
    {
      // Pose
      if (jacobians[0] != nullptr) {
        Eigen::Map<Eigen::Matrix<double, 3, 7, Eigen::RowMajor>> J0(jacobians[0]);
        Eigen::Matrix<double, 3, 6, Eigen::RowMajor> J0_minimal;
        J0_minimal = square_root_information_ * J_T_WS;

        // pseudo inverse of the local parametrization Jacobian:
        Eigen::Matrix<double, 6, 7, Eigen::RowMajor> J_lift;
        PoseLocalParameterization::liftJacobian(parameters[0], J_lift.data());

        J0 = J0_minimal * J_lift;

        if (jacobians_minimal != nullptr && jacobians_minimal[0] != nullptr) {
          Eigen::Map<Eigen::Matrix<double, 3, 6, Eigen::RowMajor> >
              J0_minimal_mapped(jacobians_minimal[0]);
          J0_minimal_mapped = J0_minimal;
        }
      }
      // Relative position
      if (jacobians[1] != nullptr) {
        Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> J1(jacobians[1]);
        J1 = square_root_information_ * J_t_SR_S;

        if (jacobians_minimal != nullptr && jacobians_minimal[1] != nullptr) {
          Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor> >
              J1_minimal_mapped(jacobians_minimal[1]);
          J1_minimal_mapped = J1;
        }
      }
    }
  }

  return true;
}

}  // namespace gici
