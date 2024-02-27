/**
* @Function: Initialize relative pose, landmarks, and scale
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
*
* Copyright (C) 2023 by Cheng Chi, All rights reserved.
**/
#include "gici/vision/visual_initialization.h"

#include <vikit/homography.h>
#include <svo/common/container_helpers.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>

#include "gici/utility/svo.h"
#include "gici/vision/feature_tracker.h"
#include "gici/vision/feature_matcher.h"
#include "gici/utility/transform.h"

namespace gici {

AbstractVisualInitialization::AbstractVisualInitialization(
      const VisualInitializationOptions& init_options,
      const DetectorPtr& detector,
      const FeatureTrackerPtr& tracker)
  : options_(init_options), tracker_(tracker), detector_(detector)
{}

AbstractVisualInitialization::~AbstractVisualInitialization()
{}

void AbstractVisualInitialization::reset()
{
  ref_frames_.reset();
}

bool AbstractVisualInitialization::trackFeaturesAndCheckDisparity(const FrameBundlePtr& frames)
{
  // Set reference frame
  if (ref_frames_ == nullptr) {
    newRefFrames(frames);
    return false;
  }

  // Track features
  for (size_t i = 0; i < ref_frames_->frames_.size(); i++) {
    detector_->grid_.reset();
    const FramePtr& ref_frame = ref_frames_->frames_.at(i);
    const FramePtr& cur_frame = frames->frames_.at(i);
    tracker_->track(ref_frame, cur_frame, detector_->grid_);
    // erase observations of landmarks, we do not need the un-initiliazed observations
    for (size_t i = 0; i < cur_frame->numFeatures(); i++) {
      cur_frame->landmark_vec_[i]->obs_.clear();
    }
  }

  // Get number of tracked features and 
  std::vector<size_t> num_tracked;
  std::vector<double> disparity;
  for (size_t i = 0; i < ref_frames_->frames_.size(); i++) {
    num_tracked.push_back(frames->frames_.at(i)->numFeatures());
    disparity.push_back(getDisparityPercentile(
      ref_frames_->frames_.at(i), frames->frames_.at(i), 
      options_.init_disparity_pivot_ratio));
  }

  const size_t num_tracked_tot =
      std::accumulate(num_tracked.begin(), num_tracked.end(), 0u);
  const double avg_disparity =
      std::accumulate(disparity.begin(), disparity.end(), 0.0) / disparity.size();
  if(num_tracked_tot < options_.init_min_features)
  {
    LOG(INFO) << "Reseting reference frame because there are too few tracked features!";
    // set current frame as reference
    newRefFrames(frames);
    return false;
  }
  if(avg_disparity < options_.init_min_disparity)
    return false;
  return true;
}

// Name a reference frame and detect features
void AbstractVisualInitialization::newRefFrames(const FrameBundlePtr& frames)
{
  ref_frames_ = frames;
  // we set the pose of referece frame as identity or user input, one can adjust the entire poses
  // and landmarks after initialization outside this class.
  if (has_rotation_prior_) {
    ref_frames_->set_T_W_B(Transformation(Eigen::Vector3d::Zero(), rotation_prior_));
  }
  else {
    ref_frames_->set_T_W_B(Transformation());
  }

  for (auto& frame : ref_frames_->frames_) 
  {
    // Clear features
    frame->clearFeatureStorage();

    // Detect new features.
    Keypoints new_px;
    Scores new_scores;
    Levels new_levels;
    Gradients new_grads;
    FeatureTypes new_types;
    Bearings new_f;

    const int max_n_features = options_.init_min_features * 5;
    detector_->detect(
          frame->img_pyr_, frame->getMask(), max_n_features, new_px, new_scores,
          new_levels, new_grads, new_types);
    frame_utils::computeBearingVectors(new_px, *frame->cam(), &new_f);

    // Add features to frame.
    const size_t n_old = frame->num_features_;
    const size_t n_new = new_px.cols();
    frame->resizeFeatureStorage(n_old + n_new);
    for(size_t i = 0, j = n_old; i < n_new; ++i, ++j) {
      frame->px_vec_.col(j) = new_px.col(i);
      frame->f_vec_.col(j) = new_f.col(i);
      frame->grad_vec_.col(j) = new_grads.col(i);
      frame->score_vec_(j) = new_scores(i);
      frame->level_vec_(j) = new_levels(i);

      frame->landmark_vec_[j] = std::make_shared<Point>(Eigen::Vector3d::Zero());
      frame->track_id_vec_(j) = frame->landmark_vec_[j]->id();

      frame->num_features_++;
    }
  }
}

bool AbstractVisualInitialization::triangulateAndInitializePoints(
    const FramePtr& frame_cur,
    const FramePtr& frame_ref,
    const Transformation& T_cur_ref,
    const double reprojection_threshold,
    const size_t min_inliers_threshold,
    std::vector<std::pair<size_t, size_t>>& matches_cur_ref, 
    const double depth_at_current_frame)
{
  Positions points_in_cur;
  triangulatePoints(
        *frame_cur, *frame_ref, T_cur_ref, reprojection_threshold, matches_cur_ref, points_in_cur);

  if(matches_cur_ref.size() < min_inliers_threshold)
  {
    LOG(WARNING) << "Init WARNING: " <<min_inliers_threshold <<" inliers minimum required. "
                 << "Have only " << matches_cur_ref.size();
    return false;
  }

  // Scale 3D points to given scene depth and initialize Points
  rescaleAndInitializePoints(
        frame_cur, frame_ref, matches_cur_ref, points_in_cur, T_cur_ref, depth_at_current_frame);

  return true;
}

void AbstractVisualInitialization::triangulatePoints(
    const Frame& frame_cur,
    const Frame& frame_ref,
    const Transformation& T_cur_ref,
    const double reprojection_threshold,
    std::vector<std::pair<size_t, size_t>>& matches_cur_ref,
    Positions& points_in_cur)
{
  points_in_cur.resize(Eigen::NoChange, frame_cur.num_features_);
  const Transformation T_ref_cur = T_cur_ref.inverse();
  std::vector<size_t> outliers;
  size_t num_inliers = 0;
  const double angle_threshold =
      frame_cur.getAngleError(reprojection_threshold);
  for(size_t i = 0; i < matches_cur_ref.size(); ++i)
  {
    BearingVector f_cur = frame_cur.f_vec_.col(matches_cur_ref[i].first);
    BearingVector f_ref = frame_ref.f_vec_.col(matches_cur_ref[i].second);
    // TODO(cfo): should take reference to eigen
    const Position xyz_in_cur =
        vk::triangulateFeatureNonLin(
          T_cur_ref.getRotationMatrix(), T_cur_ref.getPosition(), f_cur, f_ref);

    const double e1 = angleError(f_cur, xyz_in_cur);
    const double e2 = angleError(f_ref, T_ref_cur * xyz_in_cur);
    if(e1 > angle_threshold || e2 > angle_threshold ||
       (frame_cur.cam()->getType() == Camera::Type::kPinhole &&
           xyz_in_cur.z() < 0.0))
    {
      outliers.push_back(i);
    }
    else
    {
      points_in_cur.col(num_inliers) = xyz_in_cur;
      ++num_inliers;
    }
  }
  if(!outliers.empty())
  {
    svo::common::container_helpers::eraseIndicesFromVector(
        outliers, &matches_cur_ref);
  }
}

void AbstractVisualInitialization::rescaleAndInitializePoints(
    const FramePtr& frame_cur,
    const FramePtr& frame_ref,
    const std::vector<std::pair<size_t, size_t>>& matches_cur_ref,
    const Positions& points_in_cur,
    const Transformation& T_cur_ref,
    const double depth_at_current_frame)
{
  // compute scale factor
  std::vector<double> depth_vec;
  for(size_t i = 0; i < matches_cur_ref.size(); ++i)
  {
    depth_vec.push_back(points_in_cur.col(i).norm());
  }
  CHECK_GT(depth_vec.size(), 1u);
  double scale = 1.0;
  if (depth_at_current_frame != 0.0) {
    const double scene_depth_median = vk::getMedian(depth_vec);
    depth_at_current_frame / scene_depth_median;
  }

  // reset pose of current frame to have right scale
  frame_cur->T_f_w_ = T_cur_ref * frame_ref->T_f_w_;
  frame_cur->T_f_w_.getPosition() =
      - frame_cur->T_f_w_.getRotation().rotate(
        frame_ref->pos() + scale * (frame_cur->pos() - frame_ref->pos()));

  // Rescale 3D points and add to features
  Transformation T_world_cur = frame_cur->T_f_w_.inverse();
  for(size_t i = 0; i < matches_cur_ref.size(); ++i)
  {
    const Eigen::Vector3d xyz_in_world = T_world_cur * (points_in_cur.col(i) * scale);
    const int point_id_cur = frame_cur->track_id_vec_(matches_cur_ref[i].first);
    const int point_id_ref = frame_ref->track_id_vec_(matches_cur_ref[i].second);
    CHECK_EQ(point_id_cur, point_id_ref);
    changeFeatureTypeToSeed(frame_cur->type_vec_[matches_cur_ref[i].first]);
    changeFeatureTypeToSeed(frame_ref->type_vec_[matches_cur_ref[i].second]);
    PointPtr& new_point = frame_cur->landmark_vec_.at(matches_cur_ref[i].first);
    new_point->id_ = point_id_cur;
    new_point->pos_ = xyz_in_world;
    new_point->addObservation(frame_ref, matches_cur_ref[i].second);
    new_point->addObservation(frame_cur, matches_cur_ref[i].first);
  }
}

int AbstractVisualInitialization::estimateFundamental(
    const Bearings& f_cur,
    const Bearings& f_ref,
    const double focal_length,
    const double reproj_error_thresh,
    Transformation& T_cur_ref)
{
  CHECK_EQ(f_cur.cols(), f_ref.cols());
  const size_t N = f_cur.cols();
  const double thresh = reproj_error_thresh/focal_length;

  // select valid matchs
  std::vector<cv::Point2f> ref_pts(N), cur_pts(N);
  for (size_t i = 0; i < N; i++) {
    const Eigen::Vector2d& uv_ref(vk::project2(f_ref.col(i)));
    const Eigen::Vector2d& uv_cur(vk::project2(f_cur.col(i)));
    ref_pts[i] = cv::Point2f(uv_ref[0], uv_ref[1]);
    cur_pts[i] = cv::Point2f(uv_cur[0], uv_cur[1]);
  }

  // compute essential matrix
  cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
  cv::Mat essential = cv::findEssentialMat(
    ref_pts, cur_pts, camera_matrix, cv::FM_RANSAC, 0.999, thresh);

  // recover pose
  cv::Mat R_cv, t_cv;
  int inlier_cnt = cv::recoverPose(
    essential, ref_pts, cur_pts, camera_matrix, R_cv, t_cv);
  Eigen::Vector3d t; 
  Eigen::Matrix3d R;
  for (size_t i = 0; i < 3; i++) {   
    t(i) = t_cv.at<double>(i, 0);
    for (size_t j = 0; j < 3; j++) R(i, j) = R_cv.at<double>(i, j);
  }
  T_cur_ref = Transformation(t, Eigen::Quaterniond(R));

  return inlier_cnt;
}

VisualInitResult HomographyVisualInit::addFrameBundle(
    const FrameBundlePtr& frames_cur)
{
  // Track and detect features.
  if(!trackFeaturesAndCheckDisparity(frames_cur))
    return VisualInitResult::kTracking;

  // Create vector of bearing vectors
  const FrameBundlePtr ref_frames = ref_frames_;
  const Frame& frame_ref = *ref_frames->at(0);
  const Frame& frame_cur = *frames_cur->at(0);
  std::vector<std::pair<size_t, size_t>> matches_cur_ref;
  getFeatureMatches(frame_cur, frame_ref, &matches_cur_ref);
  const size_t n = matches_cur_ref.size();
  Bearings f_cur(3, n);
  Bearings f_ref(3, n);
  for(size_t i = 0; i < n; ++i)
  {
    f_cur.col(i) = frame_cur.f_vec_.col(matches_cur_ref[i].first);
    f_ref.col(i) = frame_ref.f_vec_.col(matches_cur_ref[i].second);
  }

  // Compute model
  const vk::Homography H_cur_ref = vk::estimateHomography(
        f_cur, f_ref, ref_frames_->at(0)->getErrorMultiplier(),
        options_.reproj_error_thresh, options_.init_min_inliers);
  if(H_cur_ref.score < options_.init_min_inliers)
  {
    LOG(WARNING) << "Init Homography: Have " << H_cur_ref.score << "inliers. "
                    << options_.init_min_inliers << " inliers minimum required.";
    return VisualInitResult::kFailure;
  }
  Transformation T_cur_ref = 
    Transformation(Quaternion(H_cur_ref.R_cur_ref), H_cur_ref.t_cur_ref);

  // Triangulate
  if(triangulateAndInitializePoints(
        frames_cur->at(0), ref_frames_->at(0), T_cur_ref, options_.reproj_error_thresh,
        options_.init_min_inliers, matches_cur_ref, options_.init_map_scale))
  {
    return VisualInitResult::kSuccess;
  }

  // Restart
  frames_cur->at(0)->clearFeatureStorage();
  return VisualInitResult::kTracking;
}

VisualInitResult FundamentalVisualInit::addFrameBundle(
    const FrameBundlePtr& frames_cur)
{
  // Track and detect features.
  if(!trackFeaturesAndCheckDisparity(frames_cur))
    return VisualInitResult::kTracking;

  // Create vector of bearing vectors
  const FrameBundlePtr ref_frames = ref_frames_;
  const Frame& frame_ref = *ref_frames->at(0);
  const Frame& frame_cur = *frames_cur->at(0);
  std::vector<std::pair<size_t, size_t>> matches_cur_ref;
  getFeatureMatches(frame_cur, frame_ref, &matches_cur_ref);
  const size_t n = matches_cur_ref.size();
  Bearings f_cur(3, n);
  Bearings f_ref(3, n);
  for(size_t i = 0; i < n; ++i)
  {
    f_cur.col(i) = frame_cur.f_vec_.col(matches_cur_ref[i].first);
    f_ref.col(i) = frame_ref.f_vec_.col(matches_cur_ref[i].second);
  }

  // Compute model
  Transformation T_cur_ref;
  int num_valid = estimateFundamental(
    f_cur, f_ref, ref_frames_->at(0)->getErrorMultiplier(), 
    options_.reproj_error_thresh, T_cur_ref);
  if(num_valid < options_.init_min_inliers)
  {
    LOG(WARNING) << "Init Fundamental: Have " << num_valid << " inliers. "
                    << options_.init_min_inliers << " inliers minimum required.";
    return VisualInitResult::kFailure;
  }

  // Triangulate
  if(triangulateAndInitializePoints(
        frames_cur->at(0), ref_frames_->at(0), T_cur_ref, options_.reproj_error_thresh,
        options_.init_min_inliers, matches_cur_ref))
  {
    return VisualInitResult::kSuccess;
  }

  // Restart
  frames_cur->at(0)->clearFeatureStorage();
  return VisualInitResult::kTracking;
}

AbstractVisualInitialization::UniquePtr makeVisualInitializer(
    const VisualInitializationOptions& init_options,
    const DetectorPtr& detector,
    const FeatureTrackerPtr& tracker)
{
  AbstractVisualInitialization::UniquePtr initializer;
  switch(init_options.init_type)
  {
    case VisualInitializerType::kHomography:
      initializer.reset(new HomographyVisualInit(init_options, detector, tracker));
      break;
    case VisualInitializerType::kFundamental:
      initializer.reset(new FundamentalVisualInit(init_options, detector, tracker));
      break;
    default:
      LOG(FATAL) << "Initializer type not known.";
  }
  return initializer;
}

} 
