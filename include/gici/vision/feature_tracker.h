/**
* @Function: Feature tracking using LK optical flow
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
*
* Copyright (C) 2023 by Cheng Chi, All rights reserved.
**/
#pragma once

#include "gici/utility/svo.h"

namespace gici {

// LK optical flow tracking options
struct FeatureTrackerOptions {
  // Size of the search window at each pyramid level
  std::vector<int> window_size = {21, 21};

  // 0-based maximal pyramid level number; if set to 0, pyramids are not used (single
  // level), if set to 1, two levels are used, and so on
  int max_level = 3;

  // The maximum number of iterations or elements to compute
  int max_count = 30;

  // The desired accuracy or change in parameters at which the iterative algorithm stops
  double epsilon = 0.01;

  // Use relative rotation to set initial pixel of LK optical flow iteration
  // when it is avaible
  bool use_relative_rotation = true;

  // Threshold of ransac to reject tracking outliers (pixel)
  double ransac_threshold = 1.0;

  // specifies a desirable level of confidence (probability) that the estimated 
  // matrix is correct
  double ransac_confidence = 0.99;
};

class FeatureTracker {
public:
  FeatureTracker(FeatureTrackerOptions options);
  ~FeatureTracker() { }

  // Track features by LK optical flow
  void track(const FramePtr& ref_frame,
      const FramePtr& cur_frame, OccupandyGrid2D& grid,
      const Eigen::Quaterniond& q_cur_ref = Eigen::Quaterniond::Identity());

private:
  // Predict initial point for optical flow tracking
  void predictFeatureTracking(const FramePtr& ref_frame,
      const FramePtr& cur_frame,
      const Eigen::Quaterniond& q_cur_ref, 
      const std::vector<cv::Point2f>& ref_points,
      std::vector<cv::Point2f>& pre_points);

protected: 
  FeatureTrackerOptions options_;
};

using FeatureTrackerPtr = std::shared_ptr<FeatureTracker>;

}