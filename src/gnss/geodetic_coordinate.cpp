/**
* @Function: Geodetic coordinate for GNSS processing
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
*
* Copyright (C) 2023 by Cheng Chi, All rights reserved.
**/
#include "gici/gnss/geodetic_coordinate.h"

#include <glog/logging.h>
#include <Eigen/Geometry>

namespace gici {

GeoCoordinate::GeoCoordinate(const Eigen::Vector3d& position_zero, 
                             const GeoType type) : GeoCoordinate()
{
  setZero(position_zero, type);
}

// Set zero position for ENU convertion
void GeoCoordinate::setZero(const Eigen::Vector3d& position, 
              const GeoType type)
{
  if (type == GeoType::ECEF) {
    double pos[3];
    ecef2pos(position.data(), pos);
    lla_zero_ = Eigen::Vector3d(pos);
  }
  else if (type == GeoType::LLA) {
    lla_zero_ = position;
  }
  else {
    LOG(ERROR) << "Input type not supported!";
  }
  lla_zero_setted_ = true;
}

// Convert coordinate
Eigen::Vector3d GeoCoordinate::convert(const Eigen::Vector3d& position,
            const GeoType in_type, const GeoType out_type)
{
  // Check if do not need convertion
  if (in_type == out_type) return position;

  // Convert input coordinate to LLA
  Eigen::Vector3d lla;
  if (in_type == GeoType::ECEF) {
    double pos[3];
    ecef2pos(position.data(), pos);
    lla = Eigen::Vector3d(pos);
  }
  else if (in_type == GeoType::LLA) {
    lla = position;
  }
  else if (in_type == GeoType::ENU && lla_zero_setted_) {
    Eigen::Vector3d dxyz, xyz0;
    enu2ecef(lla_zero_.data(), position.data(), dxyz.data());
    pos2ecef(lla_zero_.data(), xyz0.data());
    Eigen::Vector3d xyz = xyz0 + dxyz;
    ecef2pos(xyz.data(), lla.data());
  }
  else if (in_type == GeoType::NED && lla_zero_setted_) {
    const Eigen::Vector3d& ned = position;
    Eigen::Vector3d enu(ned(1), ned(0), -ned(2));
    Eigen::Vector3d dxyz, xyz0;
    enu2ecef(lla_zero_.data(), enu.data(), dxyz.data());
    pos2ecef(lla_zero_.data(), xyz0.data());
    Eigen::Vector3d xyz = xyz0 + dxyz;
    ecef2pos(xyz.data(), lla.data());
  }
  else {
    LOG(ERROR) << "Input type not supported!";
  }

  // Convert LLA to output type
  if (out_type == GeoType::LLA) {
    return lla;
  }
  if (out_type == GeoType::ECEF) {
    double r[3];
    pos2ecef(lla.data(), r);
    return Eigen::Vector3d(r);
  }
  if (out_type == GeoType::ENU || out_type == GeoType::NED) {
    if (!lla_zero_setted_) {
      LOG(ERROR) << "Inital position not setted!";
      return Eigen::Vector3d::Zero();
    }

    double r[3], r0[3], dr[3], enu[3];
    pos2ecef(lla.data(), r);
    pos2ecef(lla_zero_.data(), r0);
    for (int i = 0; i < 3; i++) dr[i] = r[i] - r0[i];
    ecef2enu(lla_zero_.data(), dr, enu);
    if (out_type == GeoType::ENU) {
      return Eigen::Vector3d(enu);
    }
    if (out_type == GeoType::NED) {
      return Eigen::Vector3d(enu[1], enu[0], -enu[2]);
    }
  }

  return Eigen::Vector3d::Zero();
}

// Rotate coordinate
Eigen::Vector3d GeoCoordinate::rotate(const Eigen::Vector3d& position,
            const GeoType in_type, const GeoType out_type)
{
  return (rotationMatrix(in_type, out_type) * position);
}

// Convert covariance
Eigen::Matrix3d GeoCoordinate::convertCovariance(const Eigen::Matrix3d& cov,
            const GeoType in_type, const GeoType out_type)
{
  Eigen::Matrix3d R = rotationMatrix(in_type, out_type);
  Eigen::Matrix3d rot_cov = R * cov * R.transpose();
  return ((rot_cov + rot_cov.transpose()) / 2.0);
}

// Rotation matrices
Eigen::Matrix3d GeoCoordinate::rotationMatrix(GeoType from, GeoType to)
{
  if (!lla_zero_setted_) {
    LOG(ERROR) << "Inital position not setted!";
    return Eigen::Matrix3d::Identity();
  }

  if (from == GeoType::ECEF) {
    if (to == GeoType::ENU) {
      double E[9];
      xyz2enu(lla_zero_.data(), E);
      return Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::ColMajor>>(E);
    }
    if (to == GeoType::NED) {
      Eigen::Matrix3d R_ECEF_ENU = rotationMatrix(GeoType::ECEF, GeoType::ENU);
      Eigen::Matrix3d R_ENU_NED = rotationMatrix(GeoType::ENU, GeoType::NED);
      return R_ECEF_ENU * R_ENU_NED;
    }
  }

  if (from == GeoType::ENU) {
    if (to == GeoType::ECEF) {
      return rotationMatrix(GeoType::ECEF, GeoType::ENU).transpose();
    }
    if (to == GeoType::NED) {
      Eigen::AngleAxisd x(PI, Eigen::Vector3d::UnitX());
      Eigen::AngleAxisd y(0.0, Eigen::Vector3d::UnitY());
      Eigen::AngleAxisd z(PI / 2.0, Eigen::Vector3d::UnitZ());
      Eigen::Quaterniond q_ENU_NED = z * y * x;
      return q_ENU_NED.toRotationMatrix();
    }
  }

  if (from == GeoType::NED) {
    if (to == GeoType::ECEF) {
      Eigen::Matrix3d R_NED_ENU = rotationMatrix(GeoType::NED, GeoType::ENU);
      Eigen::Matrix3d R_ENU_ECEF = rotationMatrix(GeoType::ENU, GeoType::ECEF);
      return R_NED_ENU * R_ENU_ECEF;
    }
    if (to == GeoType::ENU) {
      return rotationMatrix(GeoType::ENU, GeoType::NED).transpose();
    }
  }

  LOG(ERROR) << "Invalid rotation matrix type!";
  return Eigen::Matrix3d::Identity();
}

// Get coordinate zero
Eigen::Vector3d GeoCoordinate::getZero(const GeoType type)
{
  return convert(Eigen::Vector3d::Zero(), GeoType::ENU, type);
}

}
