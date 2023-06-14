/**
* @Function: Ambiguity measurement from ambiguity resolution
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
*
* Copyright (C) 2023 by Cheng Chi, All rights reserved.
**/
#pragma once

#pragma diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
// Eigen 3.2.7 uses std::binder1st and std::binder2nd which are deprecated since c++11
// Fix is in 3.3 devel (http://eigen.tuxfamily.org/bz/show_bug.cgi?id=872).
#include <ceres/ceres.h>
#include <Eigen/Core>
#pragma diagnostic pop

#include "gici/estimate/error_interface.h"
#include "gici/gnss/geodetic_coordinate.h"
#include "gici/gnss/gnss_types.h"

namespace gici {

// Ambiguity measurement from any combination
template<int... Ns>
class AmbiguityError :
    public ceres::SizedCostFunction<
    1 /* number of residuals */,
    Ns ... /* parameter blocks */>,
    public ErrorInterface
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// \brief The base class type.
  typedef ceres::SizedCostFunction<1, Ns ...> base_t;

  /// \brief Number of residuals (1).
  static const int kNumResiduals = 1;

  /// \brief The information matrix type (1x1).
  typedef Eigen::Matrix<double, 1, 1> information_t;

  /// \brief The covariance matrix type (same as information).
  typedef Eigen::Matrix<double, 1, 1> covariance_t;

  /// \brief Default constructor.
  AmbiguityError();

  /// \brief Construct with measurement and information matrix
  /// The sequency should be consistant with template parameter blocks
  AmbiguityError(const double measurement, 
                 const double information,
                 const std::vector<double>& coefficients);

  /// \brief Trivial destructor.
  virtual ~AmbiguityError() {}

  // setters
  /// \brief Set the measurement.
  /// @param[in] measurement The measurement.
  void setMeasurement(const double measurement)
  {
    ambiguity_ = measurement;
  }

  // Set information
  void setInformation(const information_t information)
  {
    information_ = information;  
    Eigen::LLT<information_t> lltOfInformation(information_); 
    square_root_information_ = lltOfInformation.matrixL().transpose();  
    square_root_information_inverse_ = square_root_information_.inverse(); 
  }

  // error term and Jacobian implementation
  /**
    * @brief This evaluates the error term and additionally computes the Jacobians.
    * @param parameters Pointer to the parameters (see ceres)
    * @param residuals Pointer to the residual vector (see ceres)
    * @param jacobians Pointer to the Jacobians (see ceres)
    * @return success of th evaluation.
    */
  virtual bool Evaluate(double const* const * parameters, double* residuals,
                        double** jacobians) const;

  /**
   * @brief This evaluates the error term and additionally computes
   *        the Jacobians in the minimal internal representation.
   * @param parameters Pointer to the parameters (see ceres)
   * @param residuals Pointer to the residual vector (see ceres)
   * @param jacobians Pointer to the Jacobians (see ceres)
   * @param jacobians_minimal Pointer to the minimal Jacobians (equivalent to jacobians).
   * @return Success of the evaluation.
   */
  bool EvaluateWithMinimalJacobians(double const* const * parameters,
                                    double* residuals, double** jacobians,
                                    double** jacobians_minimal) const;

  // sizes
  /// \brief Residual dimension.
  size_t residualDim() const { return kNumResiduals; }

  /// \brief Number of parameter blocks.
  size_t parameterBlocks() const { return dims_.kNumParameterBlocks; }

  /// \brief Dimension of an individual parameter block.
  size_t parameterBlockDim(size_t parameter_block_idx) const
  {
    return dims_.GetDim(parameter_block_idx);
  }

  /// @brief Residual block type as string
  virtual ErrorType typeInfo() const
  {
    return ErrorType::kAmbiguityError;
  }

  // Convert normalized residual to raw residual
  virtual void deNormalizeResidual(double *residuals) const
  {
    Eigen::Map<Eigen::Matrix<double, 1, 1>> Residual(residuals);
    Residual = square_root_information_inverse_ * Residual;
  }

protected:
  double ambiguity_;
  information_t information_;
  information_t square_root_information_; ///< The DimxDim square root information matrix.
  information_t square_root_information_inverse_;
  std::vector<double> coefficients_;

  // Parameter dimensions
  ceres::internal::StaticParameterDims<Ns...> dims_;
};

// Explicitly instantiate template classes
template class AmbiguityError<1>; 
template class AmbiguityError<1, 1>;  
template class AmbiguityError<1, 1, 1>;  
template class AmbiguityError<1, 1, 1, 1>;
template class AmbiguityError<1, 1, 1, 1, 1>;
template class AmbiguityError<1, 1, 1, 1, 1, 1>;
template class AmbiguityError<1, 1, 1, 1, 1, 1, 1>;
template class AmbiguityError<1, 1, 1, 1, 1, 1, 1, 1>;
using AmbiguityError1Coef = AmbiguityError<1>;
using AmbiguityError2Coef = AmbiguityError<1, 1>;  
using AmbiguityError3Coef = AmbiguityError<1, 1, 1>;  
using AmbiguityError4Coef = AmbiguityError<1, 1, 1, 1>;
using AmbiguityError5Coef = AmbiguityError<1, 1, 1, 1, 1>;
using AmbiguityError6Coef = AmbiguityError<1, 1, 1, 1, 1, 1>;
using AmbiguityError7Coef = AmbiguityError<1, 1, 1, 1, 1, 1, 1>;
using AmbiguityError8Coef = AmbiguityError<1, 1, 1, 1, 1, 1, 1, 1>;

}  

