/**
* @Function: Feature detecting and tracking
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
*
* Copyright (C) 2023 by Cheng Chi, All rights reserved.
**/
#include "gici/vision/feature_handler.h"

#include "gici/utility/common.h"
#include "gici/estimate/graph.h"
#include "gici/estimate/estimator_types.h"
#include "gici/estimate/pose_parameter_block.h"
#include "gici/estimate/homogeneous_point_parameter_block.h"
#include "gici/estimate/pose_error.h"
#include "gici/vision/reprojection_error.h"
#include "gici/imu/imu_error.h"

namespace gici {

// The default constructor
FeatureHandler::FeatureHandler(const FeatureHandlerOptions& options,
                               const ImuEstimatorBaseOptions& imu_options) :
  options_(options), cams_(options.cameras), map_(new Map()),
  local_scale_initialized_(false), speed_and_bias_timestamp_(0.0), 
  speed_and_bias_(SpeedAndBias::Zero()), global_scale_initialized_(false)
{
  // initialize modules
  detector_ = feature_detection_utils::makeDetector(
    options.detector, cams_->getCameraShared(0));
  tracker_ = std::make_shared<FeatureTracker>(options.tracker);
  initializer_ = makeVisualInitializer(options.initialization, detector_, tracker_);

  frame_bundles_.push_back(std::make_shared<FrameBundle>(std::vector<FramePtr>()));
}

// The default destructor
FeatureHandler::~FeatureHandler()
{}

// Add images
bool FeatureHandler::addImageBundle(
  const std::vector<std::shared_ptr<cv::Mat>>& imgs, const double timestamp)
{
  if (!isFirstFrame())
  {
    // check if the timestamp is valid
    if (lastFrames()->getMinTimestampSeconds() >= timestamp) {
      LOG(WARNING) << "Dropping frame: timestamp older than last frame of id " 
      << lastFrames()->getBundleId();
      return false;
    }
  }

  CHECK_EQ(imgs.size(), cams_->getNumCameras());
  std::vector<FramePtr> frames;
  for (size_t i = 0; i < imgs.size(); ++i)
  {
    frames.push_back(std::make_shared<Frame>(
      cams_->getCameraShared(i), imgs[i]->clone(), 
      static_cast<int64_t>(timestamp * 1.0e9), options_.max_pyramid_level + 1));
    frames.back()->set_T_cam_imu(cams_->get_T_C_B(i));
    frames.back()->setNFrameIndex(i);
  }
  FrameBundlePtr frame_bundle(new FrameBundle(frames));

  // Add to pipeline.
  mutex_.lock();
  curFrames() = frame_bundle;
  mutex_.unlock();
  ++frame_counter_;

  return true;
}

// Add images with poses
bool FeatureHandler::addImageBundle(const std::vector<std::shared_ptr<cv::Mat>>& imgs, 
                    const double timestamp, const std::vector<Transformation>& T_WSs)
{
  if (!addImageBundle(imgs, timestamp)) return false;

  CHECK(T_WSs.size() == curFrames()->size());
  for (size_t i = 0; i < curFrames()->size(); i++) {
    curFrames()->frames_[i]->set_T_w_imu(T_WSs[i]);
  }

  return true;
} 

// Process current added image bundle. Should be called after addImageBundle.
bool FeatureHandler::processImageBundle()
{
  // Perform detecting and tracking
  bool ret = processFrameBundle();

  // Shift memory
  frame_bundles_.push_back(std::make_shared<FrameBundle>(std::vector<FramePtr>()));
  if (map_->size() == 0) {
    while (frame_bundles_.size() > 3) {
      frame_bundles_.pop_front();
    }
  }
  else {
    while (frame_bundles_.front()->at(0)->id() < 
          map_->getOldsestKeyframe()->id()) {
      frame_bundles_.pop_front();
    }
  }

  return true;
}

// Set pose for a frame and adjust the frames behind
void FeatureHandler::setPoseAndAdjust(const double timestamp, const Transformation& T_WS)
{
  mutex_.lock();

  // Get the start iterator
  std::deque<FrameBundlePtr>::iterator it_start = frame_bundles_.end();
  for (auto it = frame_bundles_.begin(); it != frame_bundles_.end(); it++) {
    if (checkEqual((*it)->getMinTimestampSeconds(), timestamp)) {
      it_start = it; break;
    }
  }
  if (it_start == frame_bundles_.end()) {
    LOG(WARNING) << "Cannot find a frame bundle at timestamp " << std::fixed << timestamp;
  }

  // Adjust poses
  std::vector<Transformation> old_poses;
  Transformation last_pose;
  for (auto it = it_start; it != frame_bundles_.end(); it++) {
    if ((*it)->size() == 0) continue;
    const FramePtr& frame = (*it)->at(0);
    old_poses.push_back(frame->T_world_imu());
    if (it == it_start) {
      frame->set_T_w_imu(T_WS);
    }
    else {
      const Transformation& last_old_pose = old_poses[old_poses.size() - 2];
      const Transformation& cur_old_pose = frame->T_world_imu();
      const Transformation T_last_cur = last_old_pose.inverse() * cur_old_pose;
      Transformation T_WS_cur = last_pose * T_last_cur;
      T_WS_cur.getRotation().normalize();
      frame->set_T_w_imu(T_WS_cur);
    }
    last_pose = frame->T_world_imu();
  }

  // Adjust landmarks
  size_t index = 0;
  std::unordered_set<int> adjusted_ids;
  for (auto it = it_start; it != frame_bundles_.end(); it++, index++) {
    if ((*it)->size() == 0) continue;
    const FramePtr& frame = (*it)->at(0);
    for (size_t i = 0; i < frame->landmark_vec_.size(); i++) {
      const auto& landmark = frame->landmark_vec_[i];
      // just not-in-graph landmarks 
      if (landmark->in_ba_graph_) continue;
      // just initialized landmarks
      if (!isSeed(frame->type_vec_[i])) continue;
      // adjusted
      if (adjusted_ids.find(landmark->id()) != adjusted_ids.end()) continue;
      
      // adjust landmark position
      Eigen::Vector3d t_C = (frame->T_cam_imu() * 
        old_poses[index].inverse()) * landmark->pos();
      landmark->pos_ = frame->T_world_cam() * t_C;
      adjusted_ids.insert(landmark->id());
    }
  }

  mutex_.unlock();
}

// Keyframe selection criterion.
bool FeatureHandler::needKeyFrame(
  const FramePtr& last_keyframe, const FramePtr& frame)
{
  // Get last keyframe
  const FramePtr keyframe = last_keyframe;

  // If first frame
  if (keyframe == nullptr) return true;

  // Number of tracked features are too little
  std::vector<std::pair<size_t, size_t>> matches_ref_cur;
  getFeatureMatches(*keyframe, *frame, &matches_ref_cur);
  size_t n_tracked_fts = matches_ref_cur.size();
  if (n_tracked_fts < options_.kfselect_min_numkfs) {
    LOG(INFO) << "Select new keyframe by number: " 
              << n_tracked_fts << " vs " << options_.kfselect_min_numkfs;
    return true;
  }

  // If global scale initialized, we use distance and angle matric.
  if (global_scale_initialized_)
  {
    const double a =
        Quaternion::log(frame->T_f_w_.getRotation() *
                        keyframe->T_f_w_.getRotation().inverse()).norm()
            * 180/M_PI;
    const double d = (frame->pos() - keyframe->pos()).norm();
    if (!(a < options_.kfselect_min_angle
        && d < options_.kfselect_min_dist_metric))
    {
      LOG(INFO) << "Select new keyframe by motivation: angle = " 
                << a << ", distance = " << d;
      return true;
    }
  }
  // Select keyframe with disparity
  else {
    double disparity = getDisparity(keyframe, frame);
    if (disparity > options_.kfselect_min_disparity) {
      LOG(INFO) << "Select new keyframe by disparity: " 
                << disparity << " vs " << options_.kfselect_min_disparity;
      return true;
    }
  }

  // Check time duration  
  const double time_duration = 
    getCurrent(frame_bundles_)->at(0)->getTimestampSec() - keyframe->getTimestampSec();
  if (time_duration > options_.kfselect_min_dt) {
    LOG(INFO) << "Select new keyframe by time duration: " 
              << time_duration << " vs " << options_.kfselect_min_dt;
    return true;
  }

  // Not a new keyframe
  return false;
}

// Detect features
void FeatureHandler::detectFeatures(const FramePtr& frame)
{
  // Detect new features.
  Keypoints new_px;
  Scores new_scores;
  Levels new_levels;
  Gradients new_grads;
  FeatureTypes new_types;
  Bearings new_f;

  const int max_n_features = options_.max_features_per_frame - frame->numFeatures();
  if(max_n_features <= 0) return;

  detector_->detect(
        frame->img_pyr_, frame->getMask(), max_n_features, new_px, new_scores,
        new_levels, new_grads, new_types);

  frame_utils::computeBearingVectors(new_px, *frame->cam(), &new_f);

  // Add features to frame.
  mutex_.lock();
  const size_t n_old = frame->num_features_;
  const size_t n_new = new_px.cols();
  frame->resizeFeatureStorage(n_old + n_new);
  for(size_t i = 0, j = n_old; i < n_new; ++i, ++j) {
    frame->px_vec_.col(j) = new_px.col(i);
    frame->f_vec_.col(j) = new_f.col(i);
    frame->grad_vec_.col(j) = new_grads.col(i);
    frame->score_vec_(j) = new_scores(i);
    frame->level_vec_(j) = new_levels(i);
    frame->type_vec_[j] = FeatureType::kCorner;

    frame->landmark_vec_[j] = std::make_shared<Point>(Eigen::Vector3d::Zero());
    frame->track_id_vec_(j) = frame->landmark_vec_[j]->id();
    frame->landmark_vec_[j]->addObservation(frame, j);

    frame->num_features_++;
  }
  mutex_.unlock();
}

// Track featuers using LK optical flow
void FeatureHandler::trackFeatures()
{
  // Integrate attitude
  bool has_rotation_prior = false;
  Eigen::Quaterniond q_cur_ref;
  if (lastFrame()->has_transform_ && curFrame()->has_transform_) {
    Transformation T_WC_ref = lastFrame()->T_world_cam();
    Transformation T_WC_cur = curFrame()->T_world_cam();
    q_cur_ref = (T_WC_cur.inverse() * T_WC_ref).getEigenQuaternion();
    has_rotation_prior = true;
  }

  if (has_rotation_prior) {
    tracker_->track(getLast(frame_bundles_)->at(0), 
      getCurrent(frame_bundles_)->at(0), detector_->grid_, q_cur_ref);
  }
  else {
    tracker_->track(getLast(frame_bundles_)->at(0), 
      getCurrent(frame_bundles_)->at(0), detector_->grid_);
  }

  // Add landmark observations
  addObservation(getCurrent(frame_bundles_)->at(0));
}

// Optimize pose of new frame using tracked (and initialized) landmarks
void FeatureHandler::optimizePose()
{
  mutex_.lock();

  FrameBundlePtr frame_bundle = getCurrent(frame_bundles_);
  FramePtr frame = getCurrent(frame_bundles_)->at(0);
  Transformation T_WS;
  
  // Could get from outside
  if (pose_request_ && pose_request_(frame->getTimestampSec(), T_WS)) {
    frame->set_T_w_imu(T_WS);
    mutex_.unlock();
    return;
  }

  // Could not get from outside, we optimize by reprojection error
  std::unique_ptr<Graph> graph = std::make_unique<Graph>();
  std::shared_ptr<ceres::LossFunction> loss_function = 
    std::make_shared<ceres::CauchyLoss>(1);

  // Approximate pose
  FramePtr last_frame = getLast(frame_bundles_)->at(0);
  Transformation T_WS_aprox = last_frame->T_world_imu();
  if (frame_bundles_.size() > 2) {
    FramePtr last_last_frame = frame_bundles_.at(frame_bundles_.size() - 3)->at(0);
    Eigen::Vector3d t_last_cur = last_frame->T_imu_world().getRotation().rotate(
        last_frame->imuPos() - last_last_frame->imuPos());
    double dt1 = last_frame->getTimestampSec() - last_last_frame->getTimestampSec();
    double dt2 = frame->getTimestampSec() - last_frame->getTimestampSec();
    Transformation T_last_cur = Transformation(Quaternion(), t_last_cur * (dt2 / dt1));
    T_WS_aprox = last_frame->T_world_imu() * T_last_cur;
  }

  // Pose of current frame
  BackendId pose_id = createNFrameId(frame->bundleId());
  std::shared_ptr<PoseParameterBlock> pose_parameter_block = 
    std::make_shared<PoseParameterBlock>(T_WS_aprox, pose_id.asInteger());
  CHECK(graph->addParameterBlock(pose_parameter_block, Graph::Pose6d));

  // Extrinsics parameter (constant)
  const Transformation T_SC = frame->T_imu_cam();
  BackendId extrinsics_id = changeIdType(pose_id, IdType::cExtrinsics, size_t(0));
  std::shared_ptr<PoseParameterBlock> extrinsics_parameter_block = 
    std::make_shared<PoseParameterBlock>(T_SC, extrinsics_id.asInteger());
  CHECK(graph->addParameterBlock(extrinsics_parameter_block, Graph::Pose6d));
  graph->setParameterBlockConstant(extrinsics_id.asInteger());

  // Landmarks
  int num_valid_features = 0;
  for (size_t kp_idx = 0; kp_idx < frame->numFeatures(); ++kp_idx)
  {
    const auto& landmark = frame->landmark_vec_[kp_idx];

    // check if feature is associated to landmark
    if (landmark == nullptr || !isSeed(frame->type_vec_[kp_idx])) {
      continue;
    }

    // Add landmark parameter block and set as constant. We do not optimize
    // landmark here to save time consumption. It will be optimized in the 
    // backend graph.
    BackendId landmark_id = createLandmarkId(landmark->id());
    std::shared_ptr<HomogeneousPointParameterBlock> landmark_parameter_block =
      std::make_shared<HomogeneousPointParameterBlock>(
        landmark->pos(), landmark_id.asInteger());
    CHECK(graph->addParameterBlock(
      landmark_parameter_block, Graph::HomogeneousPoint));
    graph->setParameterBlockConstant(landmark_id.asInteger());

    // Add reprojection errors
    Eigen::Matrix2d information = Eigen::Matrix2d::Identity();
    information *= 1.0 / 
      static_cast<double>(1 << frame->level_vec_(kp_idx));
    // we do not believe not-in-graph landmarks, relatively.
    if (global_scale_initialized_ && !landmark->in_ba_graph_) information /= 1.0e4;

    std::shared_ptr<ReprojectionError> reprojection_error =
        std::make_shared<ReprojectionError>(
          frame->cam(),
          frame->px_vec_.col(kp_idx), information);
    graph->addResidualBlock(
          reprojection_error,
          loss_function.get(),
          graph->parameterBlockPtr(pose_id.asInteger()),
          graph->parameterBlockPtr(landmark_id.asInteger()),
          graph->parameterBlockPtr(extrinsics_id.asInteger()));
    
    num_valid_features++;
  }

  // If insufficient features are tracked, we set pose as the approximate one.
  if (num_valid_features < 10) {
    LOG(WARNING) << "Insufficient tracked features to estimate "
      << "current pose! num_valid_features = " << num_valid_features 
      << " " << frame->numFeatures();
    frame->set_T_w_imu(T_WS_aprox);
    mutex_.unlock();
    return;
  }

  // Optimize
  graph->options.linear_solver_type = ceres::DENSE_SCHUR;
  graph->options.trust_region_strategy_type = ceres::DOGLEG;
  graph->options.max_num_iterations = 5;
  graph->options.max_solver_time_in_seconds = 0.01;
  graph->solve();

  // Get solution
  std::shared_ptr<PoseParameterBlock> block_ptr =
      std::static_pointer_cast<PoseParameterBlock>(
        graph->parameterBlockPtr(pose_id.asInteger()));
  T_WS = block_ptr->estimate();
  frame->set_T_w_imu(T_WS);

  mutex_.unlock();
}

// Check disparity between two frames
double FeatureHandler::getDisparity(
                    const FramePtr& ref_frame, 
                    const FramePtr& cur_frame)
{
  std::vector<std::pair<size_t, size_t>> matches_ref_cur;
  getFeatureMatches(*ref_frame, *cur_frame, &matches_ref_cur);

  std::vector<double> disparities;
  for (size_t i = 0; i < matches_ref_cur.size(); i++) {
    double disparity = (ref_frame->px_vec_.col(matches_ref_cur[i].first) - 
      cur_frame->px_vec_.col(matches_ref_cur[i].second)).norm();
    disparities.push_back(disparity);
  }

  if (!disparities.empty()) return vk::getMedian(disparities);
  else return 0.0;
}

// Add observation to landmarks
void FeatureHandler::addObservation(const FramePtr& frame)
{
  mutex_.lock();

  FramePtr ref_frame = getLast(frame_bundles_)->at(0);
  std::vector<std::pair<size_t, size_t>> matches;
  getFeatureMatches(*ref_frame, *frame, &matches);
  for (size_t i = 0; i < matches.size(); i++) {
    PointPtr& landmark = ref_frame->landmark_vec_[matches[i].first];
    CHECK(landmark != nullptr);

    frame->type_vec_[matches[i].second] = ref_frame->type_vec_[matches[i].first];
    frame->landmark_vec_[matches[i].second] = landmark;
    landmark->addObservation(frame, matches[i].second);
  }

  mutex_.unlock();
}

// Set the new frame as keyframe
void FeatureHandler::setKeyFrame(const FrameBundlePtr& frame_bundle)
{
  // check if this frame tracked enough landmarks
  const FramePtr& frame = frame_bundle->at(0);
  int num_landmarks = 0;
  for (const auto& type : frame->type_vec_) {
    if (isSeed(type)) num_landmarks++;
  }
  if (num_landmarks < options_.kfselect_min_numkfs) {
    // LOG(INFO) << "Too few landmarks tracked: " << num_landmarks
    //           << ". Cannot set this frame as keyframe.";
    // return;
  }

  // set as keyframe
  frame_bundle->at(0)->setKeyframe();
  frame_bundle->setKeyframe();

  // add keyframe to map
  map_->addKeyframe(frame_bundle->at(0), true);

  // if limited number of keyframes, remove the one furthest apart
  if(options_.max_n_kfs > 2)
  {
    while(map_->size() > options_.max_n_kfs)
    {
      map_->removeOldestKeyframe();
    }
  }
}

// Initialize landmarks
void FeatureHandler::initializeLandmarks(const FramePtr& keyframe)
{
  const FramePtr& frame = keyframe;
  if (!frame->isKeyframe()) return;

  mutex_.lock();

  // Initialize landmarks
  for (size_t i = 0; i < frame->numFeatures(); i++) {
    auto& landmark = frame->landmark_vec_[i];
    CHECK(landmark != nullptr);

    // already initialized
    if (isSeed(frame->type_vec_[i])) continue;

    // rejected by bundle adjuster
    if (frame->type_vec_[i] == FeatureType::kOutlier) continue;

    // get the last keyframe
    FramePtr ref_frame = nullptr;
    size_t index_ref = 0;
    for (auto obs = landmark->obs_.rbegin(); obs != landmark->obs_.rend(); obs++) {
      if (FramePtr f = obs->frame.lock()) {
        if (f->isKeyframe() && f->id() < frame->id() && f->has_transform_) {
          ref_frame = f; 
          index_ref = obs->keypoint_index_;
        }
      }
    }
    if (ref_frame == nullptr) continue;
    if (ref_frame->id() == frame->id()) continue;

    // check disparity
    double disparity = (ref_frame->px_vec_.col(index_ref) - 
      frame->px_vec_.col(i)).norm();
    if (disparity < options_.min_disparity_init_landmark) continue;

    Transformation T_cur_ref = frame->T_f_w_ * ref_frame->T_f_w_.inverse();

    // check translation
    if (T_cur_ref.getPosition().norm() < options_.min_translation_init_landmark) continue;

    // check parallax angle
    BearingVector f_ref = ref_frame->f_vec_.col(index_ref);
    BearingVector f_cur = frame->f_vec_.col(i);
    BearingVector f_ref_rot = T_cur_ref.getRotation().rotate(f_ref);
    double angle = acos(f_ref_rot.dot(f_cur) / (f_ref_rot.norm() * f_cur.norm())) * R2D;
    if (angle < options_.min_parallax_angle_init_landmark) continue;
    
    // apply traingulation
    double depth;
    if (!(depthFromTriangulation(T_cur_ref, f_ref, f_cur, &depth) 
        == FeatureMatcher::MatchResult::kSuccess)) continue;
    if (depth < 0.1) continue;
    // Note that the follow equation should not be T * f_ref * depth.
    // Because the operater "*" is applied in homogeneous coordinate
    landmark->pos_ = ref_frame->T_world_cam() * (f_ref * depth);

    // Change feature type
    for (auto obs : landmark->obs_) {
      if (FramePtr f = obs.frame.lock()) {
        changeFeatureTypeToSeed(f->type_vec_[obs.keypoint_index_]);
      }
    }
  } 

  mutex_.unlock();
}

// Initialize scale and landmarks at first
bool FeatureHandler::initializeScale()
{
  mutex_.lock();

  VisualInitResult ret = initializer_->addFrameBundle(curFrames());
  if (ret == VisualInitResult::kTracking) {
    mutex_.unlock();
    return false;
  }
  if (ret == VisualInitResult::kFailure) {
    initializer_->reset();
    mutex_.unlock();
    return false;
  }

  // Set the first and the latest frames as keyframes
  initializer_->ref_frames_->setKeyframe();
  initializer_->ref_frames_->at(0)->setKeyframe();
  map_->addKeyframe(initializer_->ref_frames_->at(0), true);
  curFrames()->setKeyframe();
  curFrames()->at(0)->setKeyframe();
  map_->addKeyframe(curFrames()->at(0), true);

  // Clear frames to make sure it can be freed properly
  initializer_->clearFrames();

  // Set flag
  local_scale_initialized_ = true;

  mutex_.unlock();

  return true;
}

// Processes frame bundle
bool FeatureHandler::processFrameBundle()
{
  // currently we only support one camera
  return processFrame();
}

// Processes frames
bool FeatureHandler::processFrame()
{
  // Reset grid
  detector_->grid_.reset();

  // Track features
  if (frame_bundles_.size() > 1) trackFeatures();

  // Detect features in new frame
  detectFeatures(getCurrent(frame_bundles_)->at(0));

  // Select keyframe
  if(!needKeyFrame(map_->getLastKeyframe(), curFrame())) return true;

  // set as keyframe
  setKeyFrame(curFrames());

  return true;
}

}