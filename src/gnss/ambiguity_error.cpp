/**
* @Function: Ambiguity measurement from ambiguity resolution
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
*
* Copyright (C) 2023 by Cheng Chi, All rights reserved.
**/
#include "gici/gnss/ambiguity_error.h"

#include "gici/gnss/gnss_common.h"
#include "gici/utility/transform.h"
#include "gici/estimate/pose_local_parameterization.h"

namespace gici {

// Construct with measurement and information matrix
template<int... Ns>
AmbiguityError<Ns ...>::AmbiguityError(
                    const double measurement, 
                    const double information,
                    const std::vector<double>& coefficients)
{
  CHECK(coefficients.size() == parameterBlocks());

  setMeasurement(measurement);
  setInformation(Eigen::Matrix<double, 1, 1>(information));
  coefficients_ = coefficients;
}

// This evaluates the error term and additionally computes the Jacobians.
template<int... Ns>
bool AmbiguityError<Ns ...>::Evaluate(double const* const * parameters,
                                 double* residuals, double** jacobians) const
{
  return EvaluateWithMinimalJacobians(parameters, residuals, jacobians, nullptr);
}

// This evaluates the error term and additionally computes
// the Jacobians in the minimal internal representation.
template<int... Ns>
bool AmbiguityError<Ns ...>::EvaluateWithMinimalJacobians(
    double const* const * parameters, double* residuals, double** jacobians,
    double** jacobians_minimal) const
{
  // Get parameters
  std::vector<double> ambiguities;
  for (size_t i = 0; i < parameterBlocks(); i++) {
    ambiguities.push_back(parameters[i][0]);
  }

  // Get estimate derivated measurement
  double ambiguity_estimate = 0.0;
  for (size_t i = 0; i < parameterBlocks(); i++) {
    ambiguity_estimate += ambiguities[i] * coefficients_[i];
  }

  // Compute error
  Eigen::Matrix<double, 1, 1> error = 
    Eigen::Matrix<double, 1, 1>(ambiguity_ - ambiguity_estimate);

  // weigh it
  Eigen::Map<Eigen::Matrix<double, 1, 1> > weighted_error(residuals);
  weighted_error = square_root_information_ * error;

  // compute Jacobian
  if (jacobians != nullptr)
  {
    for (size_t i = 0; i < parameterBlocks(); i++) {
      if (jacobians[i] != nullptr) {
        Eigen::Map<Eigen::Matrix<double, 1, 1, Eigen::RowMajor>> Ji(jacobians[i]);
        Ji = -square_root_information_ * 
              Eigen::MatrixXd::Identity(1, 1) * coefficients_[i];

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
