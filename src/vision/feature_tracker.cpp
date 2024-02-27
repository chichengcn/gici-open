/**
* @Function: Feature tracking using LK optical flow
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
*
* Copyright (C) 2023 by Cheng Chi, All rights reserved.
**/
#include "gici/vision/feature_tracker.h"

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>

#include "gici/utility/common.h"

namespace gici {

FeatureTracker::FeatureTracker(FeatureTrackerOptions options)
{
  CHECK(options.window_size.size() == 2);
  options_ = options;
} 

// Track features by LK optical flow
void FeatureTracker::track(const FramePtr& ref_frame,
    const FramePtr& cur_frame, OccupandyGrid2D& grid,
    const Eigen::Quaterniond& q_cur_ref)
{
  // Apply LK optical flow
  std::vector<cv::Point2f> ref_points;
  std::vector<cv::Point2f> cur_points;
  for (size_t i = 0; i < ref_frame->num_features_; i++) {
    ref_points.push_back(cv::Point2f(
      ref_frame->px_vec_.col(i)[0], ref_frame->px_vec_.col(i)[1]));
  }
  
  std::vector<unsigned char> status;
  if (options_.use_relative_rotation && 
      !checkEqual(q_cur_ref.coeffs(), Eigen::Quaterniond::Identity().coeffs())) {
    // Predict feature pixel using relative orientation
    predictFeatureTracking(ref_frame, cur_frame, q_cur_ref, ref_points, cur_points);

    cv::calcOpticalFlowPyrLK(ref_frame->img_pyr_[0], cur_frame->img_pyr_[0],
        ref_points, cur_points, status, cv::noArray(), 
        cv::Size(options_.window_size[0], options_.window_size[1]), 
        options_.max_level, 
        cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 
        options_.max_count, options_.epsilon),
        cv::OPTFLOW_USE_INITIAL_FLOW);
  }
  else {
    cv::calcOpticalFlowPyrLK(ref_frame->img_pyr_[0], cur_frame->img_pyr_[0],
        ref_points, cur_points, status, cv::noArray(), 
        cv::Size(options_.window_size[0], options_.window_size[1]), 
        options_.max_level, 
        cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 
        options_.max_count, options_.epsilon));
  }

  // Apply RANSAC
  std::vector<cv::Point2f> ref_bearings;
  std::vector<cv::Point2f> cur_bearings;
  std::unordered_map<int, int> index_map;
  int num_tracked = 0;
  for (size_t i = 0; i < ref_frame->num_features_; i++) {
    if (!status[i]) continue;

    // remove out bound features
    if ((cur_points[i].x < 0 || cur_points[i].x > ref_frame->cam()->imageWidth()) ||
        (cur_points[i].y < 0 || cur_points[i].y > ref_frame->cam()->imageHeight())) {
      status[i] = 0; continue;
    }

    BearingVector bearing;
    cv::Point2f point;
    Eigen::Vector2d px = Eigen::Vector2d(ref_points[i].x, ref_points[i].y);
    ref_frame->cam()->backProject3(px, &bearing);
    point.x = bearing.x() / bearing.z();
    point.y = bearing.y() / bearing.z();
    ref_bearings.push_back(point);

    px = Eigen::Vector2d(cur_points[i].x, cur_points[i].y);
    cur_frame->cam()->backProject3(px, &bearing);
    point.x = bearing.x() / bearing.z();
    point.y = bearing.y() / bearing.z();
    cur_bearings.push_back(point);

    index_map.insert(std::make_pair(num_tracked, i));
    num_tracked++;
  }
  status.clear();

  // Apply ransac to reject error matches
  cv::findFundamentalMat(ref_bearings, cur_bearings, 
    cv::FM_RANSAC, options_.ransac_threshold / cur_frame->cam()->errorMultiplier(), 
    options_.ransac_confidence, status);

  // Put valid features in current frame
  const cv::Mat& mask = cur_frame->cam()->getMask();
  for (int i = 0; i < num_tracked; i++) {
    if (!status[i]) continue;

    int j = index_map.at(i);

    if (ref_frame->type_vec_[j] == FeatureType::kOutlier) continue;

    if(!mask.empty() && mask.at<uint8_t>(
      static_cast<int>(cur_points[j].y), static_cast<int>(cur_points[j].x)) == 0) {
      continue;
    }

    size_t grid_index = grid.getCellIndex(cur_points[j].x,cur_points[j].y, 1);
    if (!grid.isOccupied(grid_index)) {
      cur_frame->resizeFeatureStorage(cur_frame->num_features_ + 1);
      size_t& index = cur_frame->num_features_;
      cur_frame->px_vec_.col(index) = 
        Eigen::Vector2d(cur_points[j].x, cur_points[j].y);
      cur_frame->track_id_vec_[index] = ref_frame->track_id_vec_[j];
      cur_frame->grad_vec_.col(index) = ref_frame->grad_vec_.col(j);
      cur_frame->score_vec_[index] = ref_frame->score_vec_[j];
      cur_frame->level_vec_[index] = ref_frame->level_vec_[j];
      cur_frame->num_features_++;
      grid.setOccupied(grid_index);
    }
  }
  frame_utils::computeBearingVectors(
    cur_frame->px_vec_, *cur_frame->cam(), &cur_frame->f_vec_);
}

// Predict initial point for optical flow tracking
void FeatureTracker::predictFeatureTracking(const FramePtr& ref_frame,
    const FramePtr& cur_frame,
    const Eigen::Quaterniond& q_cur_ref, 
    const std::vector<cv::Point2f>& ref_points,
    std::vector<cv::Point2f>& pre_points)
{
  Eigen::Matrix3d R_cur_ref = q_cur_ref.toRotationMatrix();
  
  if (pre_points.size() != ref_points.size()) pre_points.resize(ref_points.size());
  for (size_t i = 0; i < ref_points.size(); i++) {
    Eigen::Vector2d px(ref_points[i].x, ref_points[i].y);
    BearingVector ref_bearing;
    ref_frame->cam()->backProject3(px, &ref_bearing);
    BearingVector pre_bearing = R_cur_ref * ref_bearing;
    Eigen::Vector2d px_pre;
    cur_frame->cam()->project3(pre_bearing, &px_pre);
    pre_points[i] = cv::Point2f(px_pre(0), px_pre(1));
  }
}

}