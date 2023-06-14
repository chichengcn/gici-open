/**
* @Function: Base class for visual estimator
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
*
* Copyright (C) 2023 by Cheng Chi, All rights reserved.
**/
#pragma once

#include "gici/estimate/estimator_base.h"
#include "gici/utility/common.h"
#include "gici/vision/feature_handler.h"

namespace gici {

// Visual estimator common options
struct VisualEstimatorBaseOptions {
  // Feature error STD for normal features (pixel)
  double feature_error_std = 10.0;

  // Feature error STD for stable features (pixel)
  double stable_feature_error_std = 1.0;

  // Minimum number of tracked frames for a landmark after triangulation to judge as a
  // stable landmark
  int min_observation_stable = 20;

  // Landmark outliter rejection threshold (pixel)
  double landmark_outlier_rejection_threshold = 2.0;

  // Maximum frequency of visual backend processing (Hz)
  double max_frequency = 10.0;

  // Maximum outlier rejection ratio to be considered as diverging
  double diverge_max_reject_ratio = 0.5;

  // Minimum number of continuous large amount rejection to be considered as divergence
  size_t diverge_min_num_continuous_reject = 10;
};

// Estimator
class VisualEstimatorBase : public virtual EstimatorBase {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  VisualEstimatorBase(const VisualEstimatorBaseOptions& options,
                    const EstimatorBaseOptions& base_options);
  ~VisualEstimatorBase();

  // Set visual front-end handler. 
  inline void setFeatureHandler(
    const std::shared_ptr<FeatureHandler>& feature_handler) {
    feature_handler_ = feature_handler;
  }

  // Add Image solution measurement and state
  virtual bool addImageMeasurementAndState(const FrameBundlePtr& frame_bundle)
  { return false; }

protected:
  // Add camera extrinsics block to graph
  BackendId addCameraExtrinsicsParameterBlock(
    const int32_t id, 
    const Transformation& T_SC_S_prior);

  // Add landmark blocks to graph
  void addLandmarkParameterBlocksWithResiduals(const FramePtr& frame);

  // Add landmark reprojection error block
  ceres::ResidualBlockId addReprojectionErrorResidualBlock(
    const State& state, const FramePtr& frame, const size_t keypoint_idx);

  // Add landmark reprojection error blocks
  void addReprojectionErrorResidualBlocks(
    const State& state, const FramePtr& frame);

  // Add initial camera extrinsics error
  void addCameraExtrinsicsResidualBlock(
    const BackendId& camera_extrinsics_id, 
    const Transformation& T_SC_S_prior, 
    const double std_translation, const double std_orientation);

  // Number of reprojection error
  size_t numReprojectionError(const FramePtr& frame);

  // Reject landmark reprojection error outlier
  // feature_handler_ should be locked when calling this function.
  bool rejectReprojectionErrorOutlier(const FramePtr& frame);

  // Add landmark blocks in corresponding with given camera state to marginalizer
  void addLandmarkParameterMarginBlocksWithResiduals(const State& state, bool keep = false);

  // Erase landmark observation
  void eraseReprojectionErrorResidualBlock(
    ceres::ResidualBlockId residual_block_id);

  // Erase landmark observations at a given state
  void eraseReprojectionErrorResidualBlocks(const State& state);

  // Erase landmarks which have no observations
  void eraseEmptyLandmarks();

  // Set all features as outliers to make them untracked
  void untrackAllLandmarks();

  // Update landmarks from estimator and set to frontend
  // feature_handler_ should be locked when calling this function.
  void updateLandmarks();

  // Update frame pose and speed and bias to frontend
  // feature_handler_ should be locked when calling this function.
  void updateFrameStateToFrontend(const State& state, const FramePtr& frame);

  // Get current frame bundle
  inline FrameBundlePtr& curFrameBundle() { 
    return getCurrent(frame_bundles_); 
  }

  // Get last frame bundle
  inline FrameBundlePtr& lastFrameBundle() { 
    return getLast(frame_bundles_); 
  }

  // Get oldest frame bundle
  inline FrameBundlePtr& oldestFrameBundle() { 
    return getOldest(frame_bundles_); 
  }

  // Get current frame
  inline FramePtr& curFrame() { 
    return getCurrent(frame_bundles_)->frames_.at(0); 
  }

  // Get last frame
  inline FramePtr& lastFrame() { 
    return getLast(frame_bundles_)->frames_.at(0); 
  }

  // Get oldest frame
  inline FramePtr& oldestFrame() { 
    return getOldest(frame_bundles_)->frames_.at(0); 
  }

  // Get latest camera state
  inline State& latestCameraState() {
    for (auto it = states_.rbegin(); it != states_.rend(); it++) {
      State& state = *it;
      if (!state.valid()) continue;
      if (state.id.type() == IdType::cPose) return state;
    }
    return null_state_;
  }

  // Get last camera state 
  inline State& lastCameraState() {
    int pass_cnt = 0;
    for (auto it = states_.rbegin(); it != states_.rend(); it++) {
      State& state = *it;
      if (!state.valid()) continue;
      if (state.id.type() == IdType::cPose) {
        pass_cnt++;
        if (pass_cnt == 2) return state;
      }
    }
    return null_state_;
  }

  // Count size of the keyframe states
  inline int sizeOfKeyframeStates() {
    return std::count_if(states_.begin(), states_.end(), 
      [](State& state) { return state.valid() && state.is_keyframe; });
  }

  // Check if a landmark is in estimator
  inline bool isLandmarkInEstimator(BackendId landmark_id) const {
    return landmarks_map_.find(landmark_id) != landmarks_map_.end();
  }

protected:
  // Options
  VisualEstimatorBaseOptions visual_base_options_;

  // Measurements
  std::deque<FrameBundlePtr> frame_bundles_;

  // States
  BackendId camera_extrinsics_id_;
  PointMap landmarks_map_;
  std::deque<FramePtr> active_keyframes_;

  // Feature handler
  std::shared_ptr<FeatureHandler> feature_handler_;
};

}