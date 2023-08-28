/**
* @Function: Relative Inter-System Bias (ISB) error
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
*
* Copyright (C) 2023 by Cheng Chi, All rights reserved.
**/
#include "gici/gnss/relative_isb_error.h"

#include "gici/gnss/gnss_common.h"
#include "gici/utility/transform.h"
#include "gici/estimate/pose_local_parameterization.h"

namespace gici {

// Construct with measurement and information matrix
RelativeIsbError::RelativeIsbError(const double information)
{
  setInformation(Eigen::Matrix<double, 1, 1>(information));
}

// This evaluates the error term and additionally computes the Jacobians.
bool RelativeIsbError::Evaluate(double const* const * parameters,
                                 double* residuals, double** jacobians) const
{
  return EvaluateWithMinimalJacobians(parameters, residuals, jacobians, nullptr);
}

// This evaluates the error term and additionally computes
// the Jacobians in the minimal internal representation.
bool RelativeIsbError::EvaluateWithMinimalJacobians(
    double const* const * parameters, double* residuals, double** jacobians,
    double** jacobians_minimal) const
{
  // Get parameters
  double clock_s0_t0 = parameters[0][0];
  double clock_s1_t0 = parameters[1][0];
  double clock_s0_t1 = parameters[2][0];
  double clock_s1_t1 = parameters[3][0];

  // Get estimate derivated measurement
  double isb_estimate = clock_s1_t1 - clock_s0_t1 - (clock_s1_t0 - clock_s0_t0);

  // Compute error
  Eigen::Matrix<double, 1, 1> error = 
    Eigen::Matrix<double, 1, 1>(0 - isb_estimate);

  // weigh it
  Eigen::Map<Eigen::Matrix<double, 1, 1> > weighted_error(residuals);
  weighted_error = square_root_information_ * error;

  // compute Jacobian
  if (jacobians != nullptr)
  {
    for (size_t i = 0; i < 4; i++) {
      double coefficient = 1.0;
      if (i == 1 || i == 2) coefficient = -1.0;
      if (jacobians[i] != nullptr) {
        Eigen::Map<Eigen::Matrix<double, 1, 1, Eigen::RowMajor>> Ji(jacobians[i]);
        Ji = -square_root_information_ * 
              Eigen::MatrixXd::Identity(1, 1) * coefficient;

        if (jacobians_minimal != nullptr && jacobians_minimal[i] != nullptr) {
          Eigen::Map<Eigen::Matrix<double, 1, 1, Eigen::RowMajor> >
              Ji_minimal_mapped(jacobians_minimal[i]);
          Ji_minimal_mapped = Ji;
        }
      }
    }
  }

  return true;
}

}  // namespace gici
