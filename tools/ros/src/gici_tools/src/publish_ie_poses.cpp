/**
* @Function: Publish poses
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
*
* Copyright (C) 2023 by Cheng Chi, All rights reserved.
**/
#include "gici/ros_utility/ros_publisher.h"

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
  FILE *fp_ie = fopen(file_path.data(), "r");

  // Eigen::Vector3d acc(0.0, 0.0, 1.0);
  // Eigen::Quaterniond rot = eulerAngleToQuaternion(Eigen::Vector3d(-60.0, 0.0, 180.0) * D2R);
  // acc = rot.inverse() * acc;
  // std::cout << std::fixed << acc.transpose() << std::endl;
  // return -1;

  ros::Rate r(1.0 / duration);
  Eigen::Vector3d ref_ecef = Eigen::Vector3d::Zero();
  while (ros::ok() && !(fgets(buf, 1024 * sizeof(char), fp_ie) == NULL))
  {
    if (buf[0] == 'W') continue;
    std::vector<std::string> strs;
    strs.push_back("");
    for (int i = 0; i < strlen(buf); i++) {
      if (buf[i] == ' ' || buf[i] == '\t') {
        if (strs.back().size() > 0) strs.push_back("");
        continue;
      }
      if (buf[i] == '\r' || buf[i] == '\n') break;
      strs.back().push_back(buf[i]);
    }
    int gpsweek, status;
    double gpstow, lla[3], vel[3], att[3], pos[3];
    gpsweek = atoi(strs[0].data());
    gpstow = atof(strs[1].data());
    lla[1] = atof(strs[3].data()) * D2R;
    lla[0] = atof(strs[4].data()) * D2R;
    lla[2] = atof(strs[5].data());
    att[2] = atof(strs[6].data()) * D2R;
    att[0] = atof(strs[7].data()) * D2R;
    att[1] = atof(strs[8].data()) * D2R;

    pos2ecef(lla, pos);
    att[2] = -att[2];

    gtime_t gtime = gpst2time(gpsweek, gpstow);
    double time = (double)gtime.time + gtime.sec;

    Eigen::Quaterniond q = eulerAngleToQuaternion(Eigen::Map<Eigen::Vector3d>(att));

    Eigen::Vector3d p_enu = Eigen::Vector3d::Zero();
    Eigen::Vector3d p_ecef = Eigen::Vector3d::Zero();
    pos2ecef(lla, p_ecef.data());
    if (ref_ecef == Eigen::Vector3d::Zero()) {
      ref_ecef = p_ecef;
    }
    double ref_position[3];
    ecef2pos(ref_ecef.data(), ref_position);
    Eigen::Vector3d dp = p_ecef - ref_ecef;
    ecef2enu(ref_position, dp.data(), p_enu.data());

    /// !!!!!!
    // Eigen::Quaterniond rot = eulerAngleToQuaternion(Eigen::Vector3d(60.0, 0.0, 180.0) * D2R);
    // q = q * rot.inverse();

    Transformation T_WS(p_enu, q.normalized());

    path_publisher_->addPoseAndPublish(path_pub, T_WS, ros::Time(time), "World");
    publishOdometry(pose_pub, *tranform_broadcaster_, T_WS, 
      Eigen::Vector3d::Zero(), Eigen::Matrix<double, 9, 9>::Zero(), 
      ros::Time(time), "World", "Body");

    r.sleep();
  }

  fclose(fp_ie);

  return 0;
}