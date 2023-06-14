/*
 * output_helper.h
 *
 *  Created on: Jan 20, 2013
 *      Author: cforster
 */

#ifndef VIKIT_OUTPUT_HELPER_H_
#define VIKIT_OUTPUT_HELPER_H_

#include <string>
#include <ros/ros.h>
#include <Eigen/Core>
#include <sophus/se3.hpp>
#include <tf/transform_broadcaster.h>

namespace vk {
namespace output_helper {

using namespace std;
using namespace Eigen;

void
publishTfTransform      (const Sophus::SE3& T, const ros::Time& stamp,
                         const string& frame_id, const string& child_frame_id,
                         tf::TransformBroadcaster& br);

void
publishPointMarker      (ros::Publisher pub,
                         const Vector3d& pos,
                         const string& ns,
                         const ros::Time& timestamp,
                         int id,
                         int action,
                         double marker_scale,
                         const Vector3d& color,
                         ros::Duration lifetime = ros::Duration(0.0));

void
publishLineMarker       (ros::Publisher pub,
                         const Vector3d& start,
                         const Vector3d& end,
                         const string& ns,
                         const ros::Time& timestamp,
                         int id,
                         int action,
                         double marker_scale,
                         const Vector3d& color,
                         ros::Duration lifetime = ros::Duration(0.0));

void
publishArrowMarker      (ros::Publisher pub,
                         const Vector3d& pos,
                         const Vector3d& dir,
                         double scale,
                         const string& ns,
                         const ros::Time& timestamp,
                         int id,
                         int action,
                         double marker_scale,
                         const Vector3d& color);

void
publishHexacopterMarker (ros::Publisher pub,
                         const string& frame_id,
                         const string& ns,
                         const ros::Time& timestamp,
                         int id,
                         int action,
                         double marker_scale,
                         const Vector3d& color);

void
publishCameraMarker(ros::Publisher pub,
                    const string& frame_id,
                    const string& ns,
                    const ros::Time& timestamp,
                    int id,
                    double marker_scale,
                    const Vector3d& color);
void
publishFrameMarker     (ros::Publisher pub,
                        const Matrix3d& rot,
                        const Vector3d& pos,
                        const string& ns,
                        const ros::Time& timestamp,
                        int id,
                        int action,
                        double marker_scale,
                        ros::Duration lifetime = ros::Duration(0.0));


} // namespace output_helper
} // namespace vk


#endif /* VIKIT_OUTPUT_HELPER_H_ */
