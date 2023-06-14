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
  FILE *fp_tum = fopen(file_path.data(), "r");

  ros::Rate r(1.0 / duration);
  while (ros::ok() && !(fgets(buf, 1024 * sizeof(char), fp_tum) == NULL))
  {
    if (buf[0] == '#') continue;
    double time, position[3], quaternion[4];
    sscanf(buf, "%lf %lf %lf %lf %lf %lf %lf %lf", 
      &time, &position[0], &position[1], &position[2], 
      &quaternion[0], &quaternion[1], &quaternion[2], &quaternion[3]);
    Eigen::Quaterniond q(quaternion[3], quaternion[0], quaternion[1], quaternion[2]);
    Transformation T_WS(Eigen::Vector3d(position), q.normalized());

    path_publisher_->addPoseAndPublish(path_pub, T_WS, ros::Time(time), "World");
    publishOdometry(pose_pub, *tranform_broadcaster_, T_WS, 
      Eigen::Vector3d::Zero(), Eigen::Matrix<double, 9, 9>::Zero(), 
      ros::Time(time), "World", "Body");

    r.sleep();
  }

  fclose(fp_tum);

  return 0;
}