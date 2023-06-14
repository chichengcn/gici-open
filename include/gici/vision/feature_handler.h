/**
* @Function: Feature detecting and tracking
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
*
* Copyright (C) 2023 by Cheng Chi, All rights reserved.
**/
#pragma once

#include <memory>

#include "gici/utility/svo.h"
#include "gici/imu/imu_types.h"
#include "gici/vision/feature_matcher.h"
#include "gici/vision/feature_tracker.h"
#include "gici/vision/visual_initialization.h"
#include "gici/utility/common.h"
#include "gici/imu/imu_estimator_base.h"

namespace gici {

// Feature handler options
struct FeatureHandlerOptions {
  // Max number of keyframes to keep
  size_t max_n_kfs = 10;

  // Max features per frame
  size_t max_features_per_frame = 120;

  // If we have less than this amount of features we always select a new keyframe.
  size_t kfselect_min_numkfs = 60;

  // Minimum disparity to select a new keyframe
  double kfselect_min_disparity = 10.0;

  // Minimum distance in meters before a new keyframe is selected.
  double kfselect_min_dist_metric = 1.5;

  // Minimum angle in degrees to closest KF
  double kfselect_min_angle = 10.0;

  // Minimum time duration in seconds to forcely select a new keyframe
  // This is used to control the long duration IMU drift under a slow or static motion.
  double kfselect_min_dt = 2.0;

  // Image max pyramid level
  int max_pyramid_level = 3;

  // Minimum disparity to triangulate a landmark
  double min_disparity_init_landmark = 5.0;

  // Feature detector options
  DetectorOptions detector;

  // Feature tracker options
  FeatureTrackerOptions tracker;

  // Initialization options
  VisualInitializationOptions initialization;

  // Camera model, can be ATAN, Pinhole or Ocam (see vikit)
  CameraBundlePtr cameras;
};

// Estimator
class FeatureHandler {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using PoseRequest = std::function<bool(const double, Transformation&)>;

  FeatureHandler(const FeatureHandlerOptions& options, 
                 const ImuEstimatorBaseOptions& imu_options);
  ~FeatureHandler();

  // Add images
  bool addImageBundle(const std::vector<std::shared_ptr<cv::Mat>>& imgs, 
                      const double timestamp);

  // Add images with poses
  bool addImageBundle(const std::vector<std::shared_ptr<cv::Mat>>& imgs, 
                      const double timestamp, const std::vector<Transformation>& T_WSs);

  // Process current added image bundle. Should be called after addImageBundle.
  bool processImageBundle();

  // Initialize landmarks 
  void initializeLandmarks(const FramePtr& keyframe);

  // Set pose for a frame and adjust the frames behind
  void setPoseAndAdjust(const double timestamp, const Transformation& T_WS);

  // Set a rotation prior for initialization
  inline void setRotationPrior(const Eigen::Quaterniond& R_WS) {
    initializer_->setRotationPrior(R_WS);
  }

  // Set a function to be called for getting a pose at given timestamp
  void setPoseRequestSource(PoseRequest& f) { pose_request_ = f; }

  // Get current map
  inline const MapPtr& getMap() const { return map_; }

  // Get camera bundle parameters
  inline const CameraBundle::Ptr& getNCamera() const { return cams_; }

  // Get camera parameters
  inline CameraPtr getCamera() const { return cams_->getCameraShared(0); }

  // Get feature detector
  inline const DetectorPtr& getDetector() const { return detector_; }

  // Get current processed frame bundle
  FrameBundlePtr getFrameBundle() { return lastFrames(); }

  // Get all frame bundles
  std::deque<FrameBundlePtr>& getFrameBundles() { return frame_bundles_; }

  // Check if it is the first frame
  bool isFirstFrame() { return frame_bundles_.size() < 2; }

  // Get current processed frame
  FramePtr getFrame() { return lastFrame(); }

  // Set global initialization flag
  void setGlobalScaleInitialized() { global_scale_initialized_ = true; }

private:
  // Get last frame
  const FramePtr& lastFrame() {
    return getLast(frame_bundles_)->at(0);
  }

  // Get current frame
  const FramePtr& curFrame() {
    return getCurrent(frame_bundles_)->at(0);
  }

  // Get last frame bundle
  FrameBundlePtr& lastFrames() {
    return getLast(frame_bundles_);
  }

  // Get current frame bundle
  FrameBundlePtr& curFrames() {
    return getCurrent(frame_bundles_);
  }

  // Keyframe selection criterion.
  bool needKeyFrame(const FramePtr& last_keyframe, const FramePtr& frame);

  // Detect features
  void detectFeatures(const FramePtr& frame);

  // Track featuers using LK optical flow
  void trackFeatures();

  // Optimize pose of new frame using tracked (and initialized) landmarks
  void optimizePose();

  // Check disparity between two frames
  double getDisparity(const FramePtr& ref_frame, 
                      const FramePtr& cur_frame);

  // Add observation to landmarks
  void addObservation(const FramePtr& frame);

  // Set the new frame as keyframe
  void setKeyFrame(const FrameBundlePtr& frame_bundle);

  // Initialize scale and landmarks at first
  bool initializeScale();

  // Processes frame bundle
  bool processFrameBundle();

  // Processes frames
  bool processFrame();

protected:
  // Options
  FeatureHandlerOptions options_;

  // Camera model
  CameraBundlePtr cams_;

  // Frames
  std::deque<FrameBundlePtr> frame_bundles_;
  size_t frame_counter_ = 0;
  bool local_scale_initialized_;
  bool global_scale_initialized_;

  // IMU
  double speed_and_bias_timestamp_;
  SpeedAndBias speed_and_bias_;

  // Modules
  DetectorPtr detector_;
  FeatureTrackerPtr tracker_;
  AbstractVisualInitialization::UniquePtr initializer_;

  // Map that handles keyframes and keypoints
  MapPtr map_;

  // Mutex lock to protect landmarks
  std::timed_mutex mutex_;

  // Call other functions to get pose at given timestamp
  PoseRequest pose_request_;
};

}