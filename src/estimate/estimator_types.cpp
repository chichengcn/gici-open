/**
* @Function: Estimator types
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
*
* Copyright (C) 2023 by Cheng Chi, All rights reserved.
**/
#include "gici/estimate/estimator_types.h"

namespace gici {

// Static variables
std::multimap<BackendId, State> State::overlaps;

// Convert from estimator type to string
std::string estimatorTypeToString(const EstimatorType& type)
{
  if (type == EstimatorType::None) return "None";
  if (type == EstimatorType::Spp) return "SPP";
  if (type == EstimatorType::Sdgnss) return "SDGNSS";
  if (type == EstimatorType::Dgnss) return "DGNSS";
  if (type == EstimatorType::Rtk) return "RTK";
  if (type == EstimatorType::Ppp) return "PPP";
  if (type == EstimatorType::GnssImuLc) return "GNSS/IMU LC";
  if (type == EstimatorType::SppImuTc) return "SPP/IMU TC";
  if (type == EstimatorType::DgnssImuTc) return "DGNSS/IMU TC";
  if (type == EstimatorType::RtkImuTc) return "RTK/IMU TC";
  if (type == EstimatorType::PppImuTc) return "PPP/IMU TC";
  if (type == EstimatorType::GnssImuCameraSrr) return "GNSS/IMU/Camera SRR";
  if (type == EstimatorType::SppImuCameraRrr) return "SPP/IMU/Camera RRR";
  if (type == EstimatorType::DgnssImuCameraRrr) return "DGNSS/IMU/Camera RRR";
  if (type == EstimatorType::RtkImuCameraRrr) return "RTK/IMU/Camera RRR";
  if (type == EstimatorType::PppImuCameraRrr) return "PPP/IMU/Camera RRR";
  return "";
}

} // namespace gici
