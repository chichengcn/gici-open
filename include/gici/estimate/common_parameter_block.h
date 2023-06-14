/**
* @Function: Common multiple dimension parameter block for ceres backend
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
*
* Copyright (C) 2023 by Cheng Chi, All rights reserved.
**/
#pragma once

#include "gici/estimate/parameter_block.h"

namespace gici {

enum class CommonParameterBlockType {
  Position,
  Velocity,
  Clock,
  Ifb,
  Frequency,
  Ambiguity,
  Ionosphere,
  Troposphere
};

/// \brief Wraps the parameter block for a common n dimension estimate
template<int Dim, CommonParameterBlockType Type>
class CommonParameterBlock : public ParameterBlock
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef Eigen::VectorXd estimate_t;

  static constexpr size_t c_dimension = Dim;
  static constexpr size_t c_minimal_dimension = Dim;

  /// \brief Default constructor (assumes not fixed).
  CommonParameterBlock(): ParameterBlock::ParameterBlock() {
    setFixed(false);
  }

  /// \brief Constructor with estimate and id.
  /// @param[in] parameter The parameter estimate.
  /// @param[in] id The (unique) ID of this block.
  CommonParameterBlock(const Eigen::VectorXd& parameter, uint64_t id) {
    setEstimate(parameter);
    setId(id);
    setFixed(false);
  }

  virtual ~CommonParameterBlock() {}

  // ---------------------------------------------------------------------------
  // Setters

  virtual void setEstimate(const Eigen::VectorXd& parameter)
  {
    estimate_ = parameter;
  }

  // ---------------------------------------------------------------------------
  // Getters

  virtual const Eigen::VectorXd& estimate() const { return estimate_; }

  virtual double* parameters() { return estimate_.data(); }

  virtual const double* parameters() const { return estimate_.data(); }

  virtual size_t dimension() const { return c_dimension; }

  virtual size_t minimalDimension() const { return c_minimal_dimension; }

  // minimal internal parameterization
  // x0_plus_Delta=Delta_Chi[+]x0
  /// \brief Generalization of the addition operation,
  ///        x_plus_delta = Plus(x, delta)
  ///        with the condition that Plus(x, 0) = x.
  /// @param[in] x0 Variable.
  /// @param[in] Delta_Chi Perturbation.
  /// @param[out] x0_plus_Delta Perturbed x.
  virtual void plus(const double* x0, const double* Delta_Chi,
                    double* x0_plus_Delta) const
  {
    Eigen::Map<const Eigen::VectorXd> x0_(x0, Dim);
    Eigen::Map<const Eigen::VectorXd> Delta_Chi_(Delta_Chi, Dim);
    Eigen::Map<Eigen::VectorXd> x0_plus_Delta_(x0_plus_Delta, Dim);
    x0_plus_Delta_ = x0_ + Delta_Chi_;
  }

  /// \brief The jacobian of Plus(x, delta) w.r.t delta at delta = 0.
  /// @param[in] x0 Variable.
  /// @param[out] jacobian The Jacobian.
  virtual void plusJacobian(const double* /*unused: x*/,
                            double* jacobian) const
  {
    Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, 
      Eigen::Dynamic, Eigen::RowMajor>> identity(jacobian, Dim, Dim);
    identity.setIdentity();
  }

  // Delta_Chi=x0_plus_Delta[-]x0
  /// \brief Computes the minimal difference between a variable x and a
  ///        perturbed variable x_plus_delta
  /// @param[in] x0 Variable.
  /// @param[in] x0_plus_Delta Perturbed variable.
  /// @param[out] Delta_Chi Minimal difference.
  /// \return True on success.
  virtual void minus(const double* x0, const double* x0_plus_Delta,
                     double* Delta_Chi) const
  {
    Eigen::Map<const Eigen::VectorXd> x0_(x0, Dim);
    Eigen::Map<Eigen::VectorXd> Delta_Chi_(Delta_Chi, Dim);
    Eigen::Map<const Eigen::VectorXd> x0_plus_Delta_(x0_plus_Delta, Dim);
    Delta_Chi_ = x0_plus_Delta_ - x0_;
  }

  /// \brief Computes the Jacobian from minimal space to naively
  ///        overparameterised space as used by ceres.
  /// @param[out] jacobian the Jacobian (dimension minDim x dim).
  /// \return True on success.
  virtual void liftJacobian(const double* /*unused: x*/,
                            double* jacobian) const
  {
    Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, 
      Eigen::Dynamic, Eigen::RowMajor>> identity(jacobian, Dim, Dim);
    identity.setIdentity();
  }

  /// @brief Return parameter block type as string
  virtual std::string typeInfo() const
  {
#define PRINT_MAP(x) \
  if (Type == CommonParameterBlockType::x) return std::string(#x) + "ParameterBlock";
    PRINT_MAP(Position);
    PRINT_MAP(Velocity);
    PRINT_MAP(Clock);
    PRINT_MAP(Ifb);
    PRINT_MAP(Frequency);
    PRINT_MAP(Ambiguity);
    PRINT_MAP(Ionosphere);
    PRINT_MAP(Troposphere);
    return "CommonParameterBlock";
  }

private:
  Eigen::VectorXd estimate_;
};

} 
