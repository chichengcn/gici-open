/**
* @Function: Convert file from GICI NMEA to rosbag with topic type NavSatFix
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
*
* Copyright (C) 2023 by Cheng Chi, All rights reserved.
**/
#include <rosbag/bag.h>
#include <sensor_msgs/NavSatFix.h>
#include "gici/ros_utility/nmea_formator.h"
#include "gici/gnss/gnss_common.h"

using namespace gici;

const std::string topic_name = "/gici/gnss_solution";

int main(int argc, char** argv)
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
  sprintf(buf, "%s.bag", nmea_buf);
  rosbag::Bag bag;
  bag.open(buf, rosbag::bagmode::Write);

  // Write rosbag
  for (int i = 0; i < epochs.size(); i++)
  {
    NmeaEpoch& epoch = epochs[i];

    if (epoch.sol.stat == SOLQ_NONE) continue;

    sensor_msgs::NavSatFix msg;
    msg.header.seq = i + 1;
    msg.header.stamp = ros::Time(gnss_common::gtimeToDouble(gpst2utc(epoch.sol.time)));
    double lla[3];
    ecef2pos(epoch.sol.rr, lla);
    msg.latitude = lla[0] * R2D;
    msg.longitude = lla[1] * R2D;
    msg.altitude = lla[2];
    msg.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_KNOWN;
    for (size_t j = 0; j < 9; j++) {
      msg.position_covariance[j] = 0.0;
    }
    // ENU to NED
    msg.position_covariance[0 + 0 * 3] = square(epoch.esd.std_pos[1]);
    msg.position_covariance[1 + 1 * 3] = square(epoch.esd.std_pos[0]);
    msg.position_covariance[2 + 2 * 3] = square(epoch.esd.std_pos[2]);
    msg.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS | 
                         sensor_msgs::NavSatStatus::SERVICE_GLONASS | 
                         sensor_msgs::NavSatStatus::SERVICE_GALILEO |
                         sensor_msgs::NavSatStatus::SERVICE_COMPASS;
    if (epoch.sol.stat == SOLQ_FIX) {
      msg.status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
    }
    else {
      msg.status.status = sensor_msgs::NavSatStatus::STATUS_NO_FIX;
    }
    bag.write(topic_name, msg.header.stamp, msg);
  }
  
  bag.close();

  return 0;
}