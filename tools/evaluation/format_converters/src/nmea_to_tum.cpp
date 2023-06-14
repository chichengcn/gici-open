/**
* @Function: Convert file from IE output format to TUM pose format
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

const double ref_position[] = {31.02605369 * D2R, 121.44236563 * D2R, 14.5};

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

  char buf[1034];
  sprintf(buf, "%s.tum", nmea_buf);
  FILE *fp_tum = fopen(buf, "w");
  
  fprintf(fp_tum, "# timestamp tx ty tz qx qy qz qw\r\n");

  for (int i = 0; i < epochs.size(); i++)
  {
    NmeaEpoch& epoch = epochs[i];

    Eigen::Quaterniond q; q.setIdentity();
    Eigen::Vector3d p_enu = Eigen::Vector3d::Zero();
    Eigen::Vector3d p_ecef = Eigen::Vector3d::Zero();
    Eigen::Vector3d p_ref_ecef = Eigen::Vector3d::Zero();
    pos2ecef(ref_position, p_ref_ecef.data());
    Eigen::Vector3d dp = Eigen::Map<Eigen::Vector3d>(epoch.sol.rr) - p_ref_ecef;
    ecef2enu(ref_position, dp.data(), p_enu.data());
    q = eulerAngleToQuaternion(Eigen::Map<Eigen::Vector3d>(epoch.esa.att));
    double time = (double)epoch.sol.time.time + epoch.sol.time.sec;

    fprintf(fp_tum, "%.4lf %.4lf %.4lf %.4lf %.4lf %.4lf %.4lf %.4lf\r\n", 
      time, p_enu(0), p_enu(1), p_enu(2), q.x(), q.y(), q.z(), q.w());
  }

  fclose(fp_tum);

  return 0;
}