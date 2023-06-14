/**
* @Function: Yaw error
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

/// \brief Absolute error of yaw angle.
class YawError :
    public ceres::SizedCostFunction<1 /* number of residuals */,
                                    7 /* size of first parameter */>,
    public ErrorInterface
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// \brief The base class type.
  typedef ceres::SizedCostFunction<1, 7> base_t;

  /// \brief The number of residuals (1).
  static const int kNumResiduals = 1;

  /// \brief The information matrix type (6x6).
  typedef Eigen::Matrix<double, 1, 1> information_t;

  /// \brief The covariance matrix type (same as information).
  typedef Eigen::Matrix<double, 1, 1> covariance_t;

  /// \brief Default constructor.
  YawError() = default;

  /// \brief Construct with measurement and information matrix.
  /// @param[in] measurement The yaw angle.
  /// @param[in] information The information (weight) matrix.
  YawError(const double measurement,
           const double information);

  /// \brief Trivial destructor.
  virtual ~YawError() = default;

  /// \name Setters
  /// \{

  /// \brief Set the measurement.
  /// @param[in] measurement The measurement.
  void setMeasurement(const double& measurement)
  {
    measurement_ = measurement;
  }

  /// \brief Set the information.
  /// @param[in] information The information (weight) matrix.
  void setInformation(const information_t& information);

  /// \}
  /// \name Getters
  /// \{

  /// \brief Get the measurement.
  /// \return The measurement vector.
  const double& measurement() const { return measurement_; }

  /// \brief Get the information matrix.
  /// \return The information (weight) matrix.
  const information_t& information() const { return information_; }

  /// \brief Get the covariance matrix.
  /// \return The inverse information (covariance) matrix.
  const information_t& covariance() const { return covariance_; }

  /// \}

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
  virtual bool EvaluateWithMinimalJacobians(double const* const * parameters,
                                            double* residuals,
                                            double** jacobians,
                                            double** jacobians_minimal) const;

  // sizes
  /// \brief Residual dimension.
  size_t residualDim() const { return kNumResiduals; }

  /// \brief Number of parameter blocks.
  size_t parameterBlocks() const
  {
    return base_t::parameter_block_sizes().size();
  }

  /// \brief Dimension of an individual parameter block.
  size_t parameterBlockDim(size_t parameter_block_idx) const
  {
    return base_t::parameter_block_sizes().at(parameter_block_idx);
  }

  /// @brief Return parameter block type as string
  virtual ErrorType typeInfo() const
  {
    return ErrorType::kPoseError;
  }

  // Convert normalized residual to raw residual
  virtual void deNormalizeResidual(double *residuals) const
  {
    Eigen::Map<Eigen::Matrix<double, 1, 1>> Residual(residuals);
    Residual = square_root_information_inverse_ * Residual;
  }

protected:
  // the measurement
  double measurement_; ///< The yaw measurement.

  // weighting related
  information_t information_; ///< The 1x1 information matrix.
  information_t square_root_information_; ///< The 1x1 square root information matrix.
  information_t square_root_information_inverse_;
  covariance_t covariance_; ///< The 1x1 covariance matrix.

};

}  // namespace gici
