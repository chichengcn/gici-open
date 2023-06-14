/**
* @Function: Initialize relative pose, landmarks, and scale
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
*
* Copyright (C) 2023 by Cheng Chi, All rights reserved.
**/
#pragma once

#include "gici/utility/svo.h"
#include "gici/vision/feature_tracker.h"

namespace gici {

enum class VisualInitializerType {
  kHomography,       // Estimates a plane from the first two views
  kFundamental,      // Estimates fundamental matrix from the first two views
};

// Common options for all initializers
struct VisualInitializationOptions
{

  // Initializer method. See options above.
  VisualInitializerType init_type = VisualInitializerType::kFundamental;

  // Minimum disparity (length of feature tracks) required to select the
  // second frame. After that we triangulate the first pointcloud. For easy
  // VisualInitialization you want to make this small (minimum 20px) but actually
  // it is much better to have more disparity to ensure the initial pointcloud
  // is good.
  double init_min_disparity = 50.0;

  // When checking whether the disparity of the tracked features are large enough,
  // we check that a certain percentage of tracked features have disparities large
  // than init_min_disparity. The default percentage is 0.5, which means the median
  // is checked. For example, if this parameter is set to 0.25, it means we go for
  // realtive pose estimation only when at least 25% of the tracked features have
  //  disparities large than init_min_disparity
  double init_disparity_pivot_ratio = 0.5;

  // If less features than init_min_features can be extracted at the
  // first frame, the first frame is not accepted and we check the next frame.
  size_t init_min_features = 30;

  // At the end of VisualInitialization, we triangulate the first pointcloud
  // and check the quality of the triangulation by evaluating the reprojection
  // errors. All points that have more reprojection error than reproj_error_thresh
  // are considered outliers. Only return SUCCESS if we have more inliers
  // than  init_min_inliers.
  size_t init_min_inliers = 20;

  // Initial map average depth
  // Only used for homography init
  double init_map_scale = 100.0;

  // Reprojection threshold in pixels
  double reproj_error_thresh = 2.0;
};

enum class VisualInitResult
{
  kFailure,
  kNoKeyframe,
  kTracking,
  kSuccess
};

// Bootstrapping the map from the first two views.
class AbstractVisualInitialization
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<AbstractVisualInitialization>;
  using UniquePtr = std::unique_ptr<AbstractVisualInitialization>;

  VisualInitializationOptions options_;       // Initializer options
  DetectorPtr detector_;                // Feature detector
  FeatureTrackerPtr tracker_;           // Feature tracker
  FrameBundlePtr ref_frames_;           // reference frames
  Eigen::Quaterniond rotation_prior_; 
  bool has_rotation_prior_ = false;

  AbstractVisualInitialization(
      const VisualInitializationOptions& init_options,
      const DetectorPtr& detector,
      const FeatureTrackerPtr& tracker);

  virtual ~AbstractVisualInitialization();

  bool trackFeaturesAndCheckDisparity(const FrameBundlePtr& frames);

  // Set the rotation of reference frame. For multi-sensor fusion, you can just set 
  // a coarse value, the entile poses will be adjusted after this visual init step.
  inline void setRotationPrior(const Eigen::Quaterniond& R_WS) {
    rotation_prior_ = R_WS;
    if (ref_frames_) {
      ref_frames_->set_T_W_B(Transformation(Eigen::Vector3d::Zero(), rotation_prior_));
    }
    has_rotation_prior_ = true;
  }

  // Name a reference frame and detect features
  void newRefFrames(const FrameBundlePtr& frames);

  // Clear frames to make sure it can be freed properly
  void clearFrames() { ref_frames_ = nullptr; }

  virtual VisualInitResult addFrameBundle(const FrameBundlePtr& frames_cur) = 0;

  virtual void reset();

public:
  static bool triangulateAndInitializePoints(
      const FramePtr& frame_cur,
      const FramePtr& frame_ref,
      const Transformation& T_cur_ref,
      const double reprojection_threshold,
      const size_t min_inliers_threshold,
      std::vector<std::pair<size_t, size_t>>& matches_cur_ref, 
      const double depth_at_current_frame = 0.0);

  static void triangulatePoints(
      const Frame& frame_cur,
      const Frame& frame_ref,
      const Transformation& T_cur_ref,
      const double reprojection_threshold,
      std::vector<std::pair<size_t, size_t>>& matches_cur_ref,
      Positions& points_in_cur);

  static void rescaleAndInitializePoints(
      const FramePtr& frame_cur,
      const FramePtr& frame_ref,
      const std::vector<std::pair<size_t, size_t>>& matches_cur_ref,
      const Positions& points_in_cur,
      const Transformation& T_cur_ref,
      const double depth_at_current_frame = 0.0);

  static int estimateFundamental(
      const Bearings& f_cur,
      const Bearings& f_ref,
      const double focal_length,
      const double reproj_error_thresh,
      Transformation& T_cur_ref);

  static inline double angleError(
    const Eigen::Vector3d& f1, const Eigen::Vector3d& f2)
  {
    return std::acos(f1.dot(f2) / (f1.norm()*f2.norm()));
  }
};

// Tracks features using Lucas-Kanade tracker and then estimates a homography.
class HomographyVisualInit : public AbstractVisualInitialization
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using AbstractVisualInitialization::AbstractVisualInitialization;
  virtual ~HomographyVisualInit() = default;

  virtual VisualInitResult addFrameBundle(
      const FrameBundlePtr& frames_cur) override;
};

// Tracks features using Lucas-Kanade tracker and then estimates a fundamental.
class FundamentalVisualInit : public AbstractVisualInitialization
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using AbstractVisualInitialization::AbstractVisualInitialization;
  virtual ~FundamentalVisualInit() = default;

  virtual VisualInitResult addFrameBundle(
      const FrameBundlePtr& frames_cur) override;
};

AbstractVisualInitialization::UniquePtr makeVisualInitializer(
    const VisualInitializationOptions& init_options,
    const DetectorPtr& detector,
    const FeatureTrackerPtr& tracker);

} 
