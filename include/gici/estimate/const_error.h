/**
* @Function: Const state error for prior
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
#pragma diagnostic pop

#include "gici/estimate/error_interface.h"

namespace gici {

/// \brief Constant relative error between two epochs.
template<int Dim, ErrorType Type>
class ConstError :
    public ceres::SizedCostFunction<
    Dim /* number of residuals */,
    Dim /* size of first parameter */>,
    public ErrorInterface
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// \brief The base class type.
  typedef ceres::SizedCostFunction<Dim, Dim> base_t;

  /// \brief Number of residuals (Dim).
  static const int kNumResiduals = Dim;

  /// \brief The information matrix type (DimxDim).
  typedef Eigen::Matrix<double, Dim, Dim> information_t;

  /// \brief The covariance matrix type (same as information).
  typedef Eigen::Matrix<double, Dim, Dim> covariance_t;

  /// \brief Default constructor.
  ConstError();

  /// \brief Construct with measurement and information matrix
  /// @param[in] information The information (weight) matrix.
  ConstError(const Eigen::VectorXd& value,
             const Eigen::Matrix<double, Dim, Dim>& information) {
    CHECK(value.size() == Dim);
    value_ = value;
    setInformation(information);
  }

  /// \brief Trivial destructor.
  virtual ~ConstError() {}

  // setters
  /// \brief Set the information.
  /// @param[in] information The information (weight) matrix.
  void setInformation(const information_t& information) {
    information_ = information;
    covariance_ = information.inverse();
    // perform the Cholesky decomposition on order to obtain the correct error weighting
    Eigen::LLT<information_t> lltOfInformation(information_);
    square_root_information_ = lltOfInformation.matrixL().transpose();
    square_root_information_inverse_ = square_root_information_.inverse();
  }

  // getters
  /// \brief Get the information matrix.
  /// \return The information (weight) matrix.
  const information_t& information() const { return information_; }

  /// \brief Get the covariance matrix.
  /// \return The inverse information (covariance) matrix.
  const information_t& covariance() const { return covariance_; }

  // error term and Jacobian implementation
  /**
    * @brief This evaluates the error term and additionally computes the Jacobians.
    * @param parameters Pointer to the parameters (see ceres)
    * @param residuals Pointer to the residual vector (see ceres)
    * @param jacobians Pointer to the Jacobians (see ceres)
    * @return success of th evaluation.
    */
  virtual bool Evaluate(double const* const * parameters, double* residuals,
                        double** jacobians) const {
    return EvaluateWithMinimalJacobians(parameters, residuals, jacobians, nullptr);
  }

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
                                    double** jacobians_minimal) const {
    // Residual
    Eigen::Map<const Eigen::VectorXd> para(parameters[0], Dim);
    Eigen::Map<Eigen::VectorXd> Res(residuals, Dim);
    Res = square_root_information_ * (value_ - para);

    // Jacobian
    if (jacobians != nullptr)
    {
      if (jacobians[0] != nullptr)
      {
        Eigen::Map<Eigen::Matrix<double, Dim, Dim, Eigen::RowMajor>> J0(jacobians[0]);
        J0.setIdentity();
        J0 = -square_root_information_ * J0;
        if (jacobians_minimal != nullptr && jacobians_minimal[0] != nullptr)
        {
          Eigen::Map<Eigen::Matrix<double, Dim, Dim, Eigen::RowMajor> >
              J0_minimal_mapped(jacobians_minimal[0]);
          J0_minimal_mapped = J0;
        }
      }
    }
    return true;
  }

  // sizes
  /// \brief Residual dimension.
  size_t residualDim() const { return kNumResiduals; }

  /// \brief Number of parameter blocks.
  size_t parameterBlocks() const { return 2; }

  /// \brief Dimension of an individual parameter block.
  size_t parameterBlockDim(size_t parameter_block_idx) const
  {
    return Dim;
  }

  /// @brief Residual block type as string
  virtual ErrorType typeInfo() const
  {
    return Type;
  }

  // Convert normalized residual to raw residual
  virtual void deNormalizeResidual(double *residuals) const
  {
    Eigen::Map<Eigen::Matrix<double, Dim, 1>> Residual(residuals);
    Residual = square_root_information_inverse_ * Residual;
  }

protected:
  // measurement
  Eigen::VectorXd value_;

  // weighting related
  information_t information_; ///< The DimxDim information matrix.
  information_t square_root_information_; ///< The DimxDim square root information matrix.
  information_t square_root_information_inverse_;
  covariance_t covariance_; ///< The DimxDim covariance matrix.

};

}  // namespace gici

