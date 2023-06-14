/**
* @Function: Convert file from GICI IMU pack to rosbag
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
*
* Copyright (C) 2023 by Cheng Chi, All rights reserved.
**/
#include <rosbag/bag.h>
#include <sensor_msgs/Imu.h>
#include "gici/stream/format_imu.h"
#include "gici/gnss/gnss_common.h"

using namespace gici;

const std::string topic_name = "/gici/imu_raw";

int main(int argc, char** argv)
{
  char imu_pack_path[1024];
	if (argc < 2) {
		return -1;
	} else if (argc == 2) {
		strcpy(imu_pack_path, argv[1]);
	}

  FILE *fp_imu_pack = fopen(imu_pack_path, "r");
  char buf[1034];
  sprintf(buf, "%s.bag", imu_pack_path);
  rosbag::Bag bag;
  bag.open(buf, rosbag::bagmode::Write);

  // Write rosbag
  imu_t imu;
  sensor_msgs::Imu msg;
  int cnt = 0;
  init_imu(&imu);
  while (size_t n = fread(buf, 1, 1034, fp_imu_pack))
  {
    for (size_t i = 0; i < n; i++) {
      if (!(input_imu(&imu, buf[i]) == 1)) continue;
      msg.header.seq = ++cnt;
      msg.header.stamp = ros::Time(gnss_common::gtimeToDouble(imu.time));
      msg.angular_velocity.x = imu.gyro[0];
      msg.angular_velocity.y = imu.gyro[1];
      msg.angular_velocity.z = imu.gyro[2];
      msg.linear_acceleration.x = imu.acc[0];
      msg.linear_acceleration.y = imu.acc[1];
      msg.linear_acceleration.z = imu.acc[2];
      bag.write(topic_name, msg.header.stamp, msg);
    }
  }
  
  free_imu(&imu);
  fclose(fp_imu_pack);
  bag.close();

  return 0;
}