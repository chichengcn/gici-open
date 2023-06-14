/**
* @Function: ROS publishers
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
*
* Copyright (C) 2023 by Cheng Chi, All rights reserved.
**/
#include "gici/ros_interface/ros_publisher.h"

#include <opencv2/opencv.hpp>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/image_encodings.h>
#include <tf/tf.h>
#include <ros/package.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <gici_ros/GlonassEphemeris.h>
#include <gici_ros/GnssAntennaPosition.h>
#include <gici_ros/GnssEphemerides.h>
#include <gici_ros/GnssIonosphereParameter.h>
#include <gici_ros/GnssObservations.h>
#include <gici_ros/GnssSsrCodeBiases.h>
#include <gici_ros/GnssSsrPhaseBiases.h>
#include <gici_ros/GnssSsrEphemerides.h>

#include "gici/gnss/gnss_common.h"

namespace gici {

// Configures
const double position_scale = 1.0;

// Draw features on image
static void drawFeatures(
    const Frame& frame,
    const bool only_matched_features,
    cv::Mat* img_rgb)
{
  CHECK_NOTNULL(img_rgb);

  *img_rgb = cv::Mat(frame.img_pyr_[0].size(), CV_8UC3);
  cv::cvtColor(frame.img_pyr_[0], *img_rgb, cv::COLOR_GRAY2RGB);
  for(size_t i = 0; i < frame.num_features_; ++i)
  {
    const auto& px = frame.px_vec_.col(i);
    if(frame.landmark_vec_[i] == nullptr
        && frame.seed_ref_vec_[i].keyframe == nullptr
        && only_matched_features)
      continue;

    const auto& g = frame.grad_vec_.col(i);
    switch (frame.type_vec_[i])
    {
      case FeatureType::kEdgelet:
      case FeatureType::kEdgeletSeed:
      case FeatureType::kEdgeletSeedConverged:
        cv::line(*img_rgb, cv::Point2f(px(0) + 3 * g(1), px(1) - 3 * g(0)),
                  cv::Point2f(px(0) - 3 * g(1), px(1) + 3 * g(0)),
                  cv::Scalar(0, 0, 255), 2);
      break;
      case FeatureType::kCorner:
      case FeatureType::kMapPoint:
      case FeatureType::kFixedLandmark:
      case FeatureType::kCornerSeed:
      case FeatureType::kCornerSeedConverged:
      case FeatureType::kMapPointSeed:
      case FeatureType::kMapPointSeedConverged: {
        size_t obs_size = frame.landmark_vec_[i]->obs_.size();
        cv::Scalar bgr = cv::Scalar(0, 0, 0);
        const int thickness = 3;
        const int max_size = 10;
        const int min_size = 5;
        if (obs_size < min_size) obs_size = 0;
        else obs_size -= min_size;
        bgr(2) = 150 + obs_size * 105 / (max_size - min_size);
        bgr(1) = 150 - obs_size * 150 / (max_size - min_size);
        bgr(0) = 150 - obs_size * 150 / (max_size - min_size);
        if (frame.landmark_vec_[i]->obs_.size() < min_size) {
          cv::circle(*img_rgb, cv::Point2f(px(0), px(1)), thickness, bgr, -1);
        }
        else  {
          cv::circle(*img_rgb, cv::Point2f(px(0), px(1)), thickness, bgr, -1);
          auto& obs = frame.landmark_vec_[i]->obs_[frame.landmark_vec_[i]->obs_.size() - 2];
          if (obs_size >= max_size)
          if (auto last_frame = obs.frame.lock()) {
            const auto& px_last = last_frame->px_vec_.col(obs.keypoint_index_);
            cv::Point2f begin = cv::Point2f(px_last(0), px_last(1));
            cv::Point2f end = cv::Point2f(px(0), px(1));
            cv::Point2f begin_extend = end - (begin - end) * 2.0;
            cv::line(*img_rgb, begin_extend, end, bgr, 1);
          }
        }
        break;
      }
      default:
        cv::circle(*img_rgb, cv::Point2f(px(0), px(1)),
                    5, cv::Scalar(0, 0, 255), -1);
        break;
    }
  }
}

// Publish image
void publishImage(ros::Publisher& pub, 
  const cv::Mat& image, const ros::Time time)
{
  sensor_msgs::ImagePtr img_msg = 
    cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::
    image_encodings::MONO8, image).toImageMsg();
  img_msg->header.stamp = time;
  pub.publish(img_msg);
}

// Publish raw image
void publishImage(ros::Publisher& pub, 
  const FramePtr& frame, const ros::Time time, const std::string& encoding)
{
  sensor_msgs::ImagePtr img_msg = 
    cv_bridge::CvImage(std_msgs::Header(), 
    encoding, frame->img_pyr_[0]).toImageMsg();
  img_msg->header.stamp = time;
  pub.publish(img_msg);
}

// Publish image with features
void publishFeaturedImage(ros::Publisher& pub, 
  const FramePtr& frame, const ros::Time time)
{
  cv::Mat img_rgb;
  drawFeatures(*frame, false, &img_rgb);
  sensor_msgs::ImagePtr img_msg = 
    cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::
    image_encodings::BGR8, img_rgb).toImageMsg();
  img_msg->header.stamp = time;
  pub.publish(img_msg);
}

// Publish landmarks
void publishLandmarks(ros::Publisher& pub, 
  const MapPtr& map, const ros::Time time, 
  std::string frame_id, double marker_scale)
{
  marker_scale *= position_scale;
  visualization_msgs::Marker m;
  m.header.frame_id = frame_id;
  m.header.stamp = time;
  m.ns = "landmarks";
  m.id = 0;
  m.type = visualization_msgs::Marker::SPHERE_LIST;
  m.action = 0;  // add/modify
  m.scale.x = marker_scale;
  m.scale.y = marker_scale;
  m.scale.z = marker_scale;
  m.color.a = 1.0;
  m.color.r = 0.0;
  m.color.g = 0.0;
  m.color.b = 0.0;
  m.pose.orientation.x = 0.0;
  m.pose.orientation.y = 0.0;
  m.pose.orientation.z = 0.0;
  m.pose.orientation.w = 1.0;
  for (auto kf : map->keyframes_)
  {
    const FramePtr& frame = kf.second;
    const Transformation T_w_f = frame->T_world_cam();
    for (size_t i = 0; i < frame->num_features_; ++i)
    {
      if (!isSeed(frame->type_vec_[i])) continue;
      Eigen::Vector3d xyz = frame->landmark_vec_[i]->pos();
      geometry_msgs::Point p;
      p.x = xyz.x() * position_scale;
      p.y = xyz.y() * position_scale;
      p.z = xyz.z() * position_scale;
      m.points.push_back(p);
    }
  }
  pub.publish(m);
}

// Publish pose
void publishPoseStamped(ros::Publisher& pub, 
  const Transformation& pose, const ros::Time time, 
  std::string frame_id)
{
  geometry_msgs::PoseStamped pose_msg;
  pose_msg.header.frame_id = frame_id;
  pose_msg.header.stamp = time;
  pose_msg.pose.position.x = pose.getPosition()[0] * position_scale;
  pose_msg.pose.position.y = pose.getPosition()[1] * position_scale;
  pose_msg.pose.position.z = pose.getPosition()[2] * position_scale;
  pose_msg.pose.orientation.w = pose.getRotation().w();
  pose_msg.pose.orientation.x = pose.getRotation().x();
  pose_msg.pose.orientation.y = pose.getRotation().y();
  pose_msg.pose.orientation.z = pose.getRotation().z();
  pub.publish(pose_msg);
}

// Publish pose with covariance
void publishPoseWithCovarianceStamped(ros::Publisher& pub, 
  const Transformation& pose, const Eigen::Matrix<double, 6, 6>& covariance,
  const ros::Time time, std::string frame_id)
{
  geometry_msgs::PoseWithCovarianceStamped pose_msg;
  pose_msg.header.frame_id = frame_id;
  pose_msg.header.stamp = time;
  pose_msg.pose.pose.position.x = pose.getPosition()[0] * position_scale;
  pose_msg.pose.pose.position.y = pose.getPosition()[1] * position_scale;
  pose_msg.pose.pose.position.z = pose.getPosition()[2] * position_scale;
  pose_msg.pose.pose.orientation.w = pose.getRotation().w();
  pose_msg.pose.pose.orientation.x = pose.getRotation().x();
  pose_msg.pose.pose.orientation.y = pose.getRotation().y();
  pose_msg.pose.pose.orientation.z = pose.getRotation().z();
  for (size_t i = 0; i < 6; i++) {
    for (size_t j = 0; j < 6; j++) {
      pose_msg.pose.covariance[i * 6 + j] = covariance(i, j);
    }
  }
  pub.publish(pose_msg);
}

// Publish pose with transform
void publishPoseWithTransform(ros::Publisher& pub, 
  tf::TransformBroadcaster& broadcaster, 
  const Transformation& pose, const ros::Time time, 
  std::string frame_id, std::string child_frame_id)
{
  // Publish transform
  tf::Transform transform_msg;
  auto& T = pose;
  const Eigen::Quaterniond& q = T.getRotation().toImplementation();
  transform_msg.setOrigin(
    tf::Vector3(T.getPosition().x() * position_scale, 
                T.getPosition().y() * position_scale, 
                T.getPosition().z() * position_scale));
  tf::Quaternion tf_q; 
  tf_q.setX(q.x()); tf_q.setY(q.y()); tf_q.setZ(q.z()); tf_q.setW(q.w());
  transform_msg.setRotation(tf_q);
  broadcaster.sendTransform(tf::StampedTransform(transform_msg, time, frame_id, child_frame_id));

  // Publish pose
  publishPoseStamped(pub, pose, time, frame_id);
}

// Publish pose with covariance and transform
void publishPoseWithCovarianceAndTransform(ros::Publisher& pub, 
  tf::TransformBroadcaster& broadcaster, 
  const Transformation& pose, const Eigen::Matrix<double, 6, 6>& covariance,
  const ros::Time time, std::string frame_id, std::string child_frame_id)
{
  // Publish transform
  tf::Transform transform_msg;
  auto& T = pose;
  const Eigen::Quaterniond& q = T.getRotation().toImplementation();
  transform_msg.setOrigin(
    tf::Vector3(T.getPosition().x() * position_scale, 
                T.getPosition().y() * position_scale, 
                T.getPosition().z() * position_scale));
  tf::Quaternion tf_q; 
  tf_q.setX(q.x()); tf_q.setY(q.y()); tf_q.setZ(q.z()); tf_q.setW(q.w());
  transform_msg.setRotation(tf_q);
  broadcaster.sendTransform(tf::StampedTransform(transform_msg, time, frame_id, child_frame_id));

  // Publish pose
  publishPoseWithCovarianceStamped(pub, pose, covariance, time, frame_id);
}

// Publish odometry
void publishOdometry(ros::Publisher& pub, tf::TransformBroadcaster& broadcaster,
  const Transformation& pose, const Eigen::Vector3d& velocity, 
  const Eigen::Matrix<double, 9, 9>& covariance, const ros::Time time, 
  std::string frame_id, std::string child_frame_id)
{
  // Publish transform
  tf::Transform transform_msg;
  auto& T = pose;
  const Eigen::Quaterniond& q = T.getRotation().toImplementation();
  transform_msg.setOrigin(
    tf::Vector3(T.getPosition().x() * position_scale, 
                T.getPosition().y() * position_scale, 
                T.getPosition().z() * position_scale));
  tf::Quaternion tf_q; 
  tf_q.setX(q.x()); tf_q.setY(q.y()); tf_q.setZ(q.z()); tf_q.setW(q.w());
  transform_msg.setRotation(tf_q);
  broadcaster.sendTransform(tf::StampedTransform(transform_msg, time, frame_id, child_frame_id));

  // Publish odometry
  nav_msgs::Odometry odometry_msg;
  odometry_msg.child_frame_id = child_frame_id;
  odometry_msg.header.frame_id = frame_id;
  odometry_msg.header.stamp = time;
  odometry_msg.pose.pose.position.x = pose.getPosition()[0] * position_scale;
  odometry_msg.pose.pose.position.y = pose.getPosition()[1] * position_scale;
  odometry_msg.pose.pose.position.z = pose.getPosition()[2] * position_scale;
  odometry_msg.pose.pose.orientation.w = pose.getRotation().w();
  odometry_msg.pose.pose.orientation.x = pose.getRotation().x();
  odometry_msg.pose.pose.orientation.y = pose.getRotation().y();
  odometry_msg.pose.pose.orientation.z = pose.getRotation().z();
  odometry_msg.twist.twist.linear.x = velocity.x();
  odometry_msg.twist.twist.linear.y = velocity.y();
  odometry_msg.twist.twist.linear.z = velocity.z();
  for (size_t i = 0; i < 6; i++) {
    for (size_t j = 0; j < 6; j++) {
      odometry_msg.pose.covariance[i * 6 + j] = covariance(i, j);
    }
  }
  for (size_t i = 0; i < 3; i++) {
    for (size_t j = 0; j < 3; j++) {
      odometry_msg.twist.covariance[i * 6 + j] = covariance(i + 6, j + 6);
    }
  }
  pub.publish(odometry_msg);
}

// Publish NavSatFix
void publishNavSatFix(ros::Publisher& pub, const Eigen::Vector3d& lla, 
  Eigen::Matrix3d& covariance, const ros::Time time, GnssSolutionStatus status)
{
  sensor_msgs::NavSatFix sat_msg;
  sat_msg.header.stamp = time;
  sat_msg.latitude = lla(0) * R2D;
  sat_msg.longitude = lla(1) * R2D;
  sat_msg.altitude = lla(2);
  if (covariance == Eigen::Matrix3d::Zero()) {
    sat_msg.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
  }
  else {
    sat_msg.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_KNOWN;
    for (size_t i = 0; i < 3; i++) {
      for (size_t j = 0; j < 3; j++) {
        sat_msg.position_covariance[j + i * 3] = covariance(i, j);
      }
    }
  }
  sat_msg.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS | 
                           sensor_msgs::NavSatStatus::SERVICE_GLONASS | 
                           sensor_msgs::NavSatStatus::SERVICE_GALILEO |
                           sensor_msgs::NavSatStatus::SERVICE_COMPASS;
  if (status == GnssSolutionStatus::Fixed) {
    sat_msg.status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
  }
  else {
    sat_msg.status.status = sensor_msgs::NavSatStatus::STATUS_NO_FIX;
  }
  pub.publish(sat_msg);
}

// Path publisher
void PathPublisher::addPoseAndPublish(ros::Publisher& pub, 
  const Transformation& pose, const ros::Time time, 
  std::string frame_id)
{
  if (!is_initialized_) {
    path_.header.stamp = time;
    path_.header.frame_id = frame_id;
    is_initialized_ = true;
  }

  // check timestamp, we force the frequency less than 10 Hz
  if (path_.poses.size() > 0 && 
      !((time - path_.poses.back().header.stamp).toSec() >= 0.1 - 1e-4)) return;

  geometry_msgs::PoseStamped pose_msg;
  pose_msg.header.frame_id = path_.header.frame_id;
  pose_msg.header.stamp = time;
  pose_msg.pose.position.x = pose.getPosition()[0] * position_scale;
  pose_msg.pose.position.y = pose.getPosition()[1] * position_scale;
  pose_msg.pose.position.z = pose.getPosition()[2] * position_scale;
  pose_msg.pose.orientation.w = pose.getRotation().w();
  pose_msg.pose.orientation.x = pose.getRotation().x();
  pose_msg.pose.orientation.y = pose.getRotation().y();
  pose_msg.pose.orientation.z = pose.getRotation().z();
  path_.poses.push_back(pose_msg);

  pub.publish(path_);
}

// Publish 3D error
void publishError3d(ros::Publisher& pub, 
  const Eigen::Vector3d& error, const ros::Time time, 
  std::string frame_id)
{
  geometry_msgs::Vector3Stamped error_msg;
  error_msg.header.frame_id = frame_id;
  error_msg.header.stamp = time;
  error_msg.vector.x = error(0);
  error_msg.vector.y = error(1);
  error_msg.vector.z = error(2);

  pub.publish(error_msg);
}

// Publish IMU message
void publishImu(ros::Publisher& pub, const DataCluster::IMU& imu)
{
  sensor_msgs::Imu imu_msg;
  imu_msg.header.stamp = ros::Time(imu.time);
  imu_msg.linear_acceleration.x = imu.acceleration[0];
  imu_msg.linear_acceleration.y = imu.acceleration[1];
  imu_msg.linear_acceleration.z = imu.acceleration[2];
  imu_msg.angular_velocity.x = imu.angular_velocity[0];
  imu_msg.angular_velocity.y = imu.angular_velocity[1];
  imu_msg.angular_velocity.z = imu.angular_velocity[2];

  pub.publish(imu_msg);
}

// Publish IMU message
void publishImu(ros::Publisher& pub, const ImuMeasurement& imu)
{
  sensor_msgs::Imu imu_msg;
  imu_msg.header.stamp = ros::Time(imu.timestamp);
  imu_msg.linear_acceleration.x = imu.linear_acceleration[0];
  imu_msg.linear_acceleration.y = imu.linear_acceleration[1];
  imu_msg.linear_acceleration.z = imu.linear_acceleration[2];
  imu_msg.angular_velocity.x = imu.angular_velocity[0];
  imu_msg.angular_velocity.y = imu.angular_velocity[1];
  imu_msg.angular_velocity.z = imu.angular_velocity[2];
  
  pub.publish(imu_msg);
}

// Publish GNSS message
void publishGnssObservations(
  ros::Publisher& pub, const DataCluster::GNSS& gnss)
{
  gici_ros::GnssObservations msg;
  for (int i = 0; i < gnss.observation->n; i++) {
    gici_ros::GnssObservation o;
    obsd_t *obs = gnss.observation->data + i;
    int week = 0;
    o.tow = time2gpst(obs->time, &week);
    o.week = week;
    char prn_buf[5];
    satno2id(obs->sat, prn_buf);
    o.prn = prn_buf;
    const char system = o.prn[0];
    if (system != 'G' && system != 'R' && system != 'E' && system != 'C') continue;
    for (int j = 0; j < NFREQ+NEXOBS; j++) {
      if (obs->code[j] == CODE_NONE) continue;
      o.SNR.push_back(obs->SNR[j]);
      o.LLI.push_back(obs->LLI[j]);
      o.code.push_back(gnss_common::codeTypeToRinexType(o.prn[0], obs->code[j]));
      o.L.push_back(obs->L[j]);
      o.P.push_back(obs->P[j]);
      o.D.push_back(obs->D[j]);
    }
    msg.observations.push_back(o);
  }
  msg.header.stamp = ros::Time::now();
  
  pub.publish(msg);
}

void publishGnssEphemerides(
  ros::Publisher& pub, const DataCluster::GNSS& gnss)
{
  gici_ros::GnssEphemerides msg;
  nav_t *nav = gnss.ephemeris;
  for (int i = 0; i < MAXSAT; i++) {
    eph_t *eph = nav->eph + i;
    if (eph->sat == 0) continue;
    gici_ros::GnssEphemeris e;
    char prn_buf[5];
    satno2id(eph->sat, prn_buf);
    e.prn = prn_buf;
    e.week = eph->week;
    if (e.prn[0] == 'C') {
      e.toes = time2bdt(gpst2bdt(eph->toe), NULL);
      e.toc = time2bdt(gpst2bdt(eph->toc), NULL);
    }
    else {
      e.toes = time2gpst(eph->toe, NULL);
      e.toc = time2gpst(eph->toc, NULL);
    }
    e.A = eph->A;
    e.sva = eph->sva;
    e.code = eph->code;
    e.idot = eph->idot;
    e.iode = eph->iode;
    e.f2 = eph->f2;
    e.f1 = eph->f1;
    e.f0 = eph->f0;
    e.iodc = eph->iodc;
    e.crs = eph->crs;
    e.deln = eph->deln;
    e.M0 = eph->M0;
    e.cuc = eph->cuc;
    e.e = eph->e;
    e.cus = eph->cus;
    e.toes = eph->toes;
    e.cic = eph->cic;
    e.OMG0 = eph->OMG0;
    e.cis = eph->cis;
    e.i0 = eph->i0;
    e.crc = eph->crc;
    e.omg = eph->omg;
    e.OMGd = eph->OMGd;
    for (int j = 0; j < 6; j++) {
      if (eph->tgd[j] != 0.0) {
        e.tgd.push_back(eph->tgd[j]);
      }
    }
    e.svh = eph->svh;
    msg.ephemerides.push_back(e);
  }
  for (int i = 0; i < MAXPRNGLO; i++) {
    geph_t *geph = nav->geph + i;
    if (geph->sat == 0) continue;
    gici_ros::GlonassEphemeris e;
    char prn_buf[5];
    satno2id(geph->sat, prn_buf);
    e.prn = prn_buf;
    e.svh = geph->svh;
    e.iode = geph->iode;
    int week = 0;
    e.tof = time2gpst(geph->tof, &week);
    e.toe = time2gpst(geph->toe, &week);
    e.week = week;
    e.frq = geph->frq;
    for (int j = 0; j < 3; j++) {
      e.vel.push_back(geph->vel[j]);
      e.pos.push_back(geph->pos[j]);
      e.acc.push_back(geph->acc[j]);
    }
    e.gamn = geph->gamn;
    e.taun = geph->taun;
    e.dtaun = geph->dtaun;
    e.age = geph->age;
    msg.glonass_ephemerides.push_back(e);
  }
  msg.header.stamp = ros::Time::now();

  pub.publish(msg);
}

void publishGnssAntennaPosition(
  ros::Publisher& pub, const DataCluster::GNSS& gnss)
{
  gici_ros::GnssAntennaPosition msg;
  for (size_t i = 0; i < 3; i++) {
    msg.pos.push_back(gnss.antenna->pos[i]);
  }
  msg.header.stamp = ros::Time::now();

  pub.publish(msg);
}

void publishGnssIonosphereParameter(
  ros::Publisher& pub, const DataCluster::GNSS& gnss)
{
  gici_ros::GnssIonosphereParameter msg;
  // use GPS parameters
  msg.type = 0;
  for (int i = 0; i < 8; i++) {
    msg.parameters.push_back(gnss.ephemeris->ion_gps[i]);
  }
  msg.header.stamp = ros::Time::now();

  pub.publish(msg);
}

void publishGnssSsrCodeBiases(
  ros::Publisher& pub, const DataCluster::GNSS& gnss)
{
  gici_ros::GnssSsrCodeBiases msg;
  for (int i = 0; i < MAXSAT; i++) {
    gici_ros::GnssSsrCodeBias b;
    ssr_t *ssr = gnss.ephemeris->ssr + i;
    char prn_buf[5];
    satno2id(i + 1, prn_buf);
    b.prn = prn_buf;
    int week;
    b.tow = time2gpst(ssr->t0[4], &week);
    b.week = week;
    b.udi = ssr->udi[4];
    b.isdcb = ssr->isdcb;
    const char system = b.prn[0];
    if (system != 'G' && system != 'R' && system != 'E' && system != 'C') continue;
    for (int j = 0; j < MAXCODE; j++) {
      if (ssr->cbias[j] == 0.0) continue;
      b.code.push_back(gnss_common::codeTypeToRinexType(b.prn[0], j + 1));
      b.bias.push_back(ssr->cbias[j]);
    }
    if (b.code.size() == 0) continue;
    msg.biases.push_back(b);
  }
  if (msg.biases.size() == 0) return;
  msg.header.stamp = ros::Time::now();

  pub.publish(msg);
}

void publishGnssSsrPhaseBiases(
  ros::Publisher& pub, const DataCluster::GNSS& gnss)
{
  gici_ros::GnssSsrPhaseBiases msg;
  for (int i = 0; i < MAXSAT; i++) {
    gici_ros::GnssSsrPhaseBias b;
    ssr_t *ssr = gnss.ephemeris->ssr + i;
    char prn_buf[5];
    satno2id(i + 1, prn_buf);
    b.prn = prn_buf;
    int week;
    b.tow = time2gpst(ssr->t0[4], &week);
    b.week = week;
    b.udi = ssr->udi[4];
    b.isdpb = ssr->isdpb;
    const char system = b.prn[0];
    if (system != 'G' && system != 'R' && system != 'E' && system != 'C') continue;
    for (int j = 0; j < MAXCODE; j++) {
      if (ssr->pbias[j] == 0.0) continue;
      int phase = gnss_common::getPhaseID(b.prn[0], j + 1);
      b.phase.push_back(gnss_common::phaseTypeToPhaseString(b.prn[0], phase));
      b.bias.push_back(ssr->pbias[j]);
    }
    if (b.phase.size() == 0) continue;
    msg.biases.push_back(b);
  }
  if (msg.biases.size() == 0) return;
  msg.header.stamp = ros::Time::now();

  pub.publish(msg);
}

void publishGnssSsrEphemerides(
  ros::Publisher& pub, const DataCluster::GNSS& gnss)
{
  gici_ros::GnssSsrEphemerides msg;
  for (int i = 0; i < MAXSAT; i++) {
    gici_ros::GnssSsrEphemeris c;
    ssr_t *ssr = gnss.ephemeris->ssr + i;
    if (ssr->deph[0] == 0.0 || ssr->dclk[0] == 0.0) continue;
    char prn_buf[5];
    satno2id(i + 1, prn_buf);
    c.prn = prn_buf;
    int week;
    c.tow = time2gpst(ssr->t0[0], &week);
    c.week = week;
    c.udi = ssr->udi[0];
    c.iod = ssr->iod[0];
    c.iode = ssr->iode;
    c.iodcrc = ssr->iodcrc;
    c.refd = ssr->refd;
    for (int j = 0; j < 3; j++) {
      c.deph.push_back(ssr->deph[j]);
      c.ddeph.push_back(ssr->ddeph[j]);
      c.dclk.push_back(ssr->dclk[j]);
    }
    msg.corrections.push_back(c);
  }
  if (msg.corrections.size() == 0) return;
  msg.header.stamp = ros::Time::now();

  pub.publish(msg);
}

}