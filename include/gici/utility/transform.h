/**
* @Function: Coordinate transform functions
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
*
* Copyright (C) 2023 by Cheng Chi, All rights reserved.
**/
#pragma once

#include <iostream>
#include <Eigen/Core>

#include <svo/vio_common/matrix.hpp>
#include <svo/vio_common/matrix_operations.hpp>

namespace gici {

//! Skew symmetric matrix.
inline Eigen::Matrix<double,3,3> skewSymmetric(const double w1,
                             const double w2,
                             const double w3)
{ return svo::skewSymmetric(w1, w2, w3); }

inline Eigen::Matrix<double,3,3> skewSymmetric(
    const Eigen::Ref<const Eigen::Matrix<double,3,1> >& w)
{ return svo::skewSymmetric(w(0), w(1), w(2)); }

// Right Jacobian for Exponential map in SO(3)
inline Eigen::Matrix<double,3,3> expmapDerivativeSO3(
    const Eigen::Matrix<double,3,1>& omega)
{ return svo::expmapDerivativeSO3(omega); }

// -----------------------------------------------------------------------------
// Quaternion utils

//! Plus matrix for a quaternion. q_AB x q_BC = plus(q_AB) * q_BC.coeffs().
inline Eigen::Matrix<double,4,4> quaternionPlusMatrix(
    const Eigen::Quaternion<double>& q_AB)
{ return svo::quaternionPlusMatrix(q_AB); }

//! Opposite-Plus matrix for a quaternion q_AB x q_BC = oplus(q_BC) * q_AB.coeffs().
inline Eigen::Matrix<double,4,4> quaternionOplusMatrix(
    const Eigen::Quaternion<double>& q_BC)
{ return svo::quaternionOplusMatrix(q_BC); }

// Convert quarternion to euler angle
Eigen::Vector3d quaternionToEulerAngle(const Eigen::Quaternion<double>& q);

// Convert euler angle to quarternion
Eigen::Quaternion<double> eulerAngleToQuaternion(const Eigen::Vector3d rpy);

}

