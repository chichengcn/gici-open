/**
* @Function: Convert pose from IMU coordinate to GNSS antenna coordinate
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
*
* Copyright (C) 2023 by Cheng Chi, All rights reserved.
**/
#include "rtklib.h"
#include "nmea_formator.h"

#include <Eigen/Dense>
#include <iostream>
#include <string>
#include <vector>

const Eigen::Vector3d t_SR_S(0.354, -0.042, -0.029);

// Convert euler angle to quarternion
Eigen::Quaternion<double> eulerAngleToQuaternion(const Eigen::Vector3d rpy)
{
  // Abbreviations for the various angular functions
  double roll = rpy(0);
  double pitch = rpy(1);
  double yaw = rpy(2);
  double cy = cos(yaw * 0.5);
  double sy = sin(yaw * 0.5);
  double cp = cos(pitch * 0.5);
  double sp = sin(pitch * 0.5);
  double cr = cos(roll * 0.5);
  double sr = sin(roll * 0.5);

  Eigen::Quaternion<double> q;
  q.w() = cr * cp * cy + sr * sp * sy;
  q.x() = sr * cp * cy - cr * sp * sy;
  q.y() = cr * sp * cy + sr * cp * sy;
  q.z() = cr * cp * sy - sr * sp * cy;

  return q;
}

int main(int argc, char ** argv)
{
  char nmea_buf[1024];
	if (argc < 2) {
		return -1;
	} else if (argc == 2) {
		strcpy(nmea_buf, argv[1]);
	}

  std::vector<NmeaEpoch> epochs;
  loadNmeaFile(nmea_buf, epochs);

  std::vector<NmeaEpoch> to_gnss_epochs;
  Eigen::Vector3d ref_ecef = Eigen::Vector3d::Zero();
  for (int i = 0; i < epochs.size(); i++)
  {
    NmeaEpoch& epoch = epochs[i];
    esa_t& esa = epoch.esa;
    sol_t& sol = epoch.sol;

    Eigen::Quaterniond q = eulerAngleToQuaternion(Eigen::Map<Eigen::Vector3d>(esa.att));
    Eigen::Vector3d p_enu = Eigen::Vector3d::Zero();
    Eigen::Vector3d p_ecef(sol.rr);
    if (ref_ecef == Eigen::Vector3d::Zero()) {
      ref_ecef = Eigen::Map<Eigen::Vector3d>(sol.rr);
    }
    double ref_position[3];
    ecef2pos(ref_ecef.data(), ref_position);
    Eigen::Vector3d dp = p_ecef - ref_ecef;
    ecef2enu(ref_position, dp.data(), p_enu.data());

    p_enu = p_enu.eval() + q * t_SR_S;

    enu2ecef(ref_position, p_enu.data(), dp.data());
    p_ecef = ref_ecef + dp;
    for (int i = 0; i < 3; i++) sol.rr[i] = p_ecef(i);

    NmeaEpoch to_gnss_epoch;
    to_gnss_epoch.sol = sol;
    to_gnss_epoch.esa.time = sol.time;
    for (int j = 0; j < 3; j++) {
      to_gnss_epoch.esa.vel[j] = 0.0;
      to_gnss_epoch.esa.att[j] = 0.0;
    }
    to_gnss_epochs.push_back(to_gnss_epoch);
  }

  char to_gnss_nmea_path[1040];
  sprintf(to_gnss_nmea_path, "%s.translated", nmea_buf);
  writeNmeaFile(to_gnss_epochs, to_gnss_nmea_path);

  return 0;
}