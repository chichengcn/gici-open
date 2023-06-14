/**
* @Function: Convert file from GICI NMEA to IE format
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
  sprintf(buf, "%s.ie", nmea_buf);
  FILE *fp_ie = fopen(buf, "w");

  fprintf(fp_ie, "Week GPSTime UTCTime Longitude Latitude  H-Ell Heading Pitch Roll\r\n");
  for (int i = 0; i < epochs.size(); i++)
  {
    NmeaEpoch& epoch = epochs[i];

    gtime_t utc = gpst2utc(epoch.sol.time);
    int gpsweek;
    double gpstow = time2gpst(epoch.sol.time, &gpsweek);
    double utctow = time2gpst(utc, NULL);
    double lla[3];
    ecef2pos(epoch.sol.rr, lla);

    fprintf(fp_ie, "%4d %10.3lf %10.3lf %15.10lf %14.10lf %10.3lf %15.10lf %15.10lf %15.10lf\r\n", 
      gpsweek, gpstow, utctow, lla[1]*R2D, lla[0]*R2D, lla[2], 
      -epoch.esa.att[2]*R2D, epoch.esa.att[0]*R2D, epoch.esa.att[1]*R2D);
  }
      
  fclose(fp_ie);

  return 0;
}