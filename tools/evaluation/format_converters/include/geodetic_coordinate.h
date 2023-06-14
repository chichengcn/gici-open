/**
* @Function: Geodetic coordinate for GNSS processing
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
*
* Copyright (C) 2023 by Cheng Chi, All rights reserved.
**/
#pragma once

#include <Eigen/Core>
#include <memory>
#include <rtklib.h>

namespace gici {

// Geodetic coordinate types
enum class GeoType {
  ECEF,
  LLA,
  ENU,
  NED
};

class GeoCoordinate {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  GeoCoordinate(const Eigen::Vector3d& position_zero, 
                const GeoType type);
  GeoCoordinate() : 
    lla_zero_(Eigen::Vector3d::Zero()), lla_zero_setted_(false) { }
  ~GeoCoordinate() { }

  // Set zero position for ENU convertion
  void setZero(const Eigen::Vector3d& position, 
               const GeoType type);

  // Check if zero position was setted
  bool isZeroSetted() { return lla_zero_setted_; }

  // Convert coordinate
  Eigen::Vector3d convert(const Eigen::Vector3d& position,
              const GeoType in_type, const GeoType out_type);

  // Rotate coordinate
  Eigen::Vector3d rotate(const Eigen::Vector3d& position,
              const GeoType in_type, const GeoType out_type);

  // Convert covariance
  Eigen::Matrix3d convertCovariance(const Eigen::Matrix3d& cov,
              const GeoType in_type, const GeoType out_type);

  // Rotation matrices
  Eigen::Matrix3d rotationMatrix(GeoType from, GeoType to);

  // Get coordinate zero
  Eigen::Vector3d getZero(const GeoType type);

  // Convert LLA in degree to LLA in rad
  inline static Eigen::Vector3d degToRad(Eigen::Vector3d& deg) {
    Eigen::Vector3d rad = deg;
    rad(0) *= D2R; rad(1) *= D2R;
    return rad;
  }

  // Convert LLA in rad to LLA in degree
  inline static Eigen::Vector3d radToDeg(Eigen::Vector3d& rad) {
    Eigen::Vector3d deg = rad;
    deg(0) *= R2D; deg(1) *= R2D;
    return deg;
  }

private:
  // Initial Latitude, Longitude and Altitude stored in rad
  Eigen::Vector3d lla_zero_;
  bool lla_zero_setted_;
};

using GeoCoordinatePtr = std::shared_ptr<GeoCoordinate>;

} // namespace gici