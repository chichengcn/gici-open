/**
* @Function: ROS publishers
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
*
* Copyright (C) 2023 by Cheng Chi, All rights reserved.
**/
#pragma once

#include <iostream>
#include <boost/shared_ptr.hpp>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/ColorRGBA.h>
#include <tf/transform_broadcaster.h>
#include <image_transport/image_transport.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include "gici/utility/svo.h"
#include "gici/stream/formator.h"

namespace gici {

// Publish raw image
void publishImage(ros::Publisher& pub, 
  const cv::Mat& image, const ros::Time time);
void publishImage(ros::Publisher& pub, 
  const FramePtr& frame, const ros::Time time, const std::string& encoding);

// Publish image with features
void publishFeaturedImage(ros::Publisher& pub, 
  const FramePtr& frame, const ros::Time time);

// Publish landmarks
void publishLandmarks(ros::Publisher& pub, 
  const MapPtr& map, const ros::Time time, 
  std::string frame_id, double marker_scale = 0.1);

// Publish pose
void publishPoseStamped(ros::Publisher& pub, 
  const Transformation& pose, const ros::Time time, 
  std::string frame_id);

// Publish pose with covariance
void publishPoseWithCovarianceStamped(ros::Publisher& pub, 
  const Transformation& pose, const Eigen::Matrix<double, 6, 6>& covariance,
  const ros::Time time, std::string frame_id);

// Publish pose with transform
void publishPoseWithTransform(ros::Publisher& pub, 
  tf::TransformBroadcaster& broadcaster, 
  const Transformation& pose, const ros::Time time, 
  std::string frame_id, std::string child_frame_id);

// Publish pose with covariance and transform
void publishPoseWithCovarianceAndTransform(ros::Publisher& pub, 
  tf::TransformBroadcaster& broadcaster, 
  const Transformation& pose, const Eigen::Matrix<double, 6, 6>& covariance,
  const ros::Time time, std::string frame_id, std::string child_frame_id);

// Publish odometry
void publishOdometry(ros::Publisher& pub, tf::TransformBroadcaster& broadcaster,
  const Transformation& pose, const Eigen::Vector3d& velocity, 
  const Eigen::Matrix<double, 9, 9>& covariance, const ros::Time time, 
  std::string frame_id, std::string child_frame_id);

// Path publisher
class PathPublisher {
public:
  // Add a pose and publish all previous poses
  void addPoseAndPublish(ros::Publisher& pub, 
    const Transformation& pose, const ros::Time time, 
    std::string frame_id);

  // Clear previous poses
  inline void clear() { 
    path_.poses.clear();
    is_initialized_ = false;
  }

protected:
  bool is_initialized_ = false;
  nav_msgs::Path path_;
};

// Publish 3D error
void publishError3d(ros::Publisher& pub, 
  const Eigen::Vector3d& error, const ros::Time time, 
  std::string frame_id);

// Publish IMU message
void publishImu(ros::Publisher& pub, const DataCluster::IMU& imu);
void publishImu(ros::Publisher& pub, const ImuMeasurement& imu);

}