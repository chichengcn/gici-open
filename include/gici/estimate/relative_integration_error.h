/**
* @Function: Relative first-order integration error
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

/// \brief Relative first-order integration error
// e.g. p_cur = p_pre + v_pre * dt, 
//      v_cur = v_pre
template<int Dim, ErrorType Type>
class RelativeIntegrationError :
    public ceres::SizedCostFunction<
    2*Dim /* number of residuals */,
    Dim, /* size of first parameter: previous */
    Dim, /* size of second parameter: current */
    Dim, /* size of second parameter: previous differential term */
    Dim /* size of second parameter: current differential term */>,
    public ErrorInterface
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// \brief The base class type.
  typedef ceres::SizedCostFunction<Dim, Dim, Dim, Dim> base_t;

  /// \brief Number of residuals (Dim).
  static const int kNumResiduals = 2*Dim;

  /// \brief The information matrix type (DimxDim).
  typedef Eigen::Matrix<double, 2*Dim, 2*Dim> information_t;

  /// \brief The covariance matrix type (same as information).
  typedef Eigen::Matrix<double, 2*Dim, 2*Dim> covariance_t;

  /// \brief Default constructor.
  RelativeIntegrationError();

  /// \brief Construct with measurement and information matrix
  /// @param[in] PSD The Power Spectral Density
  RelativeIntegrationError(const Eigen::Matrix<double, Dim, Dim>& psd, double dt) {
    Eigen::MatrixXd covariance = Eigen::MatrixXd::Zero(2*Dim, 2*Dim);
    covariance.topLeftCorner(Dim, Dim) = psd * pow(dt, 3) / 3.0;
    covariance.topRightCorner(Dim, Dim) = psd * pow(dt, 2) / 2.0;
    covariance.bottomLeftCorner(Dim, Dim) = psd * pow(dt, 2) / 2.0;
    covariance.bottomRightCorner(Dim, Dim) = psd * dt;
    setCovariance(covariance);
    dt_ = dt;
  }

  /// \brief Trivial destructor.
  virtual ~RelativeIntegrationError() {}

  // setters
  /// \brief Set the covariance.
  /// @param[in] covariance The covariance (weight) matrix.
  void setCovariance(const covariance_t& covariance) {
    covariance_ = covariance;
    information_ = covariance.inverse();
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
    Eigen::Map<const Eigen::VectorXd> para_pre(parameters[0], Dim);
    Eigen::Map<const Eigen::VectorXd> para_cur(parameters[1], Dim);
    Eigen::Map<const Eigen::VectorXd> para_inte_pre(parameters[2], Dim);
    Eigen::Map<const Eigen::VectorXd> para_inte_cur(parameters[3], Dim);
    Eigen::Map<Eigen::VectorXd> Res(residuals, 2*Dim);
    Eigen::VectorXd error = Eigen::VectorXd::Zero(2*Dim);
    error.topRows(Dim) = para_cur - (para_pre + para_inte_pre * dt_);
    error.bottomRows(Dim) = para_inte_cur - para_inte_pre;
    Res = square_root_information_ * error;

    // Jacobian
    if (jacobians != nullptr)
    {
      if (jacobians[0] != nullptr)
      {
        Eigen::Map<Eigen::Matrix<double, 2*Dim, Dim, Eigen::RowMajor>> J0(jacobians[0]);
        J0.setZero();
        J0.topLeftCorner(Dim, Dim).setIdentity();
        J0 = -square_root_information_ * J0;
        if (jacobians_minimal != nullptr && jacobians_minimal[0] != nullptr)
        {
          Eigen::Map<Eigen::Matrix<double, 2*Dim, Dim, Eigen::RowMajor> >
              J0_minimal_mapped(jacobians_minimal[0]);
          J0_minimal_mapped = J0;
        }
      }
      if (jacobians[1] != nullptr)
      {
        Eigen::Map<Eigen::Matrix<double, 2*Dim, Dim, Eigen::RowMajor>> J1(jacobians[1]);
        J1.setZero();
        J1.topLeftCorner(Dim, Dim).setIdentity();
        J1 = square_root_information_ * J1;
        if (jacobians_minimal != nullptr && jacobians_minimal[1] != nullptr)
        {
          Eigen::Map<Eigen::Matrix<double, 2*Dim, Dim, Eigen::RowMajor> >
              J1_minimal_mapped(jacobians_minimal[1]);
          J1_minimal_mapped = J1;
        }
      }
      if (jacobians[2] != nullptr)
      {
        Eigen::Map<Eigen::Matrix<double, 2*Dim, Dim, Eigen::RowMajor>> J2(jacobians[2]);
        J2.setZero();
        J2.topLeftCorner(Dim, Dim) = -dt_ * Eigen::MatrixXd::Identity(Dim, Dim);
        J2.bottomLeftCorner(Dim, Dim) = -Eigen::MatrixXd::Identity(Dim, Dim);
        J2 = square_root_information_ * J2;
        if (jacobians_minimal != nullptr && jacobians_minimal[2] != nullptr)
        {
          Eigen::Map<Eigen::Matrix<double, 2*Dim, Dim, Eigen::RowMajor> >
              J2_minimal_mapped(jacobians_minimal[2]);
          J2_minimal_mapped = J2;
        }
      }
      if (jacobians[3] != nullptr)
      {
        Eigen::Map<Eigen::Matrix<double, 2*Dim, Dim, Eigen::RowMajor>> J3(jacobians[3]);
        J3.setZero();
        J3.bottomLeftCorner(Dim, Dim).setIdentity();
        J3 = square_root_information_ * J3;
        if (jacobians_minimal != nullptr && jacobians_minimal[3] != nullptr)
        {
          Eigen::Map<Eigen::Matrix<double, 2*Dim, Dim, Eigen::RowMajor> >
              J3_minimal_mapped(jacobians_minimal[3]);
          J3_minimal_mapped = J3;
        }
      }
    }
    return true;
  }

  // sizes
  /// \brief Residual dimension.
  size_t residualDim() const { return kNumResiduals; }

  /// \brief Number of parameter blocks.
  size_t parameterBlocks() const { return 4; }

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
    Eigen::Map<Eigen::Matrix<double, 2*Dim, 1>> Residual(residuals);
    Residual = square_root_information_inverse_ * Residual;
  }

protected:

  // weighting related
  information_t information_; ///< The DimxDim information matrix.
  information_t square_root_information_; ///< The DimxDim square root information matrix.
  information_t square_root_information_inverse_;
  covariance_t covariance_; ///< The DimxDim covariance matrix.

  // step period
  double dt_;

};

}  // namespace gici

