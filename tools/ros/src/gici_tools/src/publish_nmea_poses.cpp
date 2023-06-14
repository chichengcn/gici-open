/**
* @Function: Publish poses
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
*
* Copyright (C) 2023 by Cheng Chi, All rights reserved.
**/
#include "gici/ros_utility/ros_publisher.h"
#include "gici/ros_utility/nmea_formator.h"

using namespace gici;

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

int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "gici");
  ros::NodeHandle nh("~");

  // Get file
  if (argc != 4) {
    std::cerr << "Invalid input variables! Supported variables are: "
              << "<path-to-executable> <path-to-file> <topic-name> <time-duration>" << std::endl;
    return -1;
  }
  std::string file_path = argv[1];
  std::string topic_name = argv[2];
  double duration = atof(argv[3]);
  
  ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("/" + topic_name + "/path", 10);
  ros::Publisher pose_pub = nh.advertise<nav_msgs::Odometry>("/" + topic_name + "/pose", 3);

  std::unique_ptr<tf::TransformBroadcaster> tranform_broadcaster_ = std::make_unique<tf::TransformBroadcaster>();
  std::unique_ptr<PathPublisher> path_publisher_ = std::make_unique<PathPublisher>();

  char buf[1024];
  FILE *fp_nmea = fopen(file_path.data(), "r");

  sol_t sol = {0};
  esa_t esa = {0};
  int num_gga = 0;
  bool has_esa = false;
  Eigen::Vector3d ref_ecef = Eigen::Vector3d::Zero();
  ros::Rate r(1.0 / duration);
  while (ros::ok() && !(fgets(buf, 1024 * sizeof(char), fp_nmea) == NULL))
  {
    std::vector<std::string> strs;
    strs.push_back("");
    for (int i = 0; i < strlen(buf); i++) {
      if (buf[i] == ',') {
        if (strs.back().size() > 0) strs.push_back("");
        continue;
      }
      if (buf[i] == '\r' || buf[i] == '\n') break;
      strs.back().push_back(buf[i]);
    }
    
    if (strs[0].substr(3, 3) == "GGA") {
      decodeGGA(buf, &sol);
      num_gga++;
    }
    else if (strs[0].substr(3, 3) == "RMC") {
      decodeRMC(buf, &sol);
    }
    else if (strs[0].substr(3, 3) == "ESA") {
      decodeESA(buf, &sol, &esa);
      has_esa = true;
    }

    if (num_gga > 1)
    {
      double time = (double)sol.time.time + sol.time.sec;
      if (has_esa) {
        if (timediff(esa.time, sol.time) == 0.0) {
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

          Transformation T_WS(p_enu, q.normalized());

          path_publisher_->addPoseAndPublish(path_pub, T_WS, ros::Time(time), "World");
          publishOdometry(pose_pub, *tranform_broadcaster_, T_WS, 
            Eigen::Vector3d::Zero(), Eigen::Matrix<double, 9, 9>::Zero(), 
            ros::Time(time), "World", "Body");

          r.sleep();
        }
      }
      else {
        std::cerr << "Error: Cannot find GNESA message!" << std::endl;
        return -1;
      }
    }
  }

  fclose(fp_nmea);

  return 0;
}