/**
* @Function: Base class for visual estimator
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
*
* Copyright (C) 2023 by Cheng Chi, All rights reserved.
**/
#include "gici/vision/visual_estimator_base.h"

#include "gici/estimate/pose_parameter_block.h"
#include "gici/vision/reprojection_error.h"
#include "gici/estimate/pose_error.h"

namespace gici {

// The default constructor
VisualEstimatorBase::VisualEstimatorBase(
                    const VisualEstimatorBaseOptions& options,
                    const EstimatorBaseOptions& base_options) :
  visual_base_options_(options), EstimatorBase(base_options)
{}

// The default destructor
VisualEstimatorBase::~VisualEstimatorBase()
{}

// Add camera extrinsics block to graph
BackendId VisualEstimatorBase::addCameraExtrinsicsParameterBlock(
  const int32_t id, 
  const Transformation& T_SC_prior)
{
  BackendId pose_id = createNFrameId(id);
  BackendId camera_extrinsics_id = changeIdType(pose_id, IdType::cExtrinsics);
  std::shared_ptr<PoseParameterBlock> camera_extrinsic_parameter_block = 
    std::make_shared<PoseParameterBlock>(
      T_SC_prior, camera_extrinsics_id.asInteger());
  CHECK(graph_->addParameterBlock(camera_extrinsic_parameter_block, Graph::Pose6d));

  return camera_extrinsics_id;
}

// Add landmark blocks to graph
void VisualEstimatorBase::addLandmarkParameterBlocksWithResiduals(const FramePtr& frame)
{
  // only add new landmarks at keyframe
  if (!frame->isKeyframe()) return;

  double timestamp = frame->getTimestampSec();
  for (size_t kp_idx = 0; kp_idx < frame->numFeatures(); ++kp_idx)
  {
    const PointPtr& point = frame->landmark_vec_[kp_idx];
    const FeatureType& type = frame->type_vec_[kp_idx];

    // check if feature is associated to landmark
    if (point == nullptr || !isSeed(frame->type_vec_[kp_idx])) {
      continue;
    }

    // add new landmarks and corresponding observations at keyframe
    if (!isLandmarkInEstimator(createLandmarkId(point->id()))) {
      // check if the first frame is current keyframe
      if (point->obs_.front().frame_id == frame->id()) {
        continue;
      }

      CHECK(!std::isnan(point->pos_[0])) << "Point is nan!";

      // add the landmark
      BackendId landmark_backend_id = createLandmarkId(point->id());
      std::shared_ptr<HomogeneousPointParameterBlock>
          point_parameter_block =
          std::make_shared<HomogeneousPointParameterBlock>(
            point->pos(), landmark_backend_id.asInteger());
      CHECK(graph_->addParameterBlock(
        point_parameter_block, Graph::HomogeneousPoint));

      // add landmark to map
      auto it_landmark = landmarks_map_.emplace_hint(
            landmarks_map_.end(),
            landmark_backend_id, MapPoint(point));
      point->in_ba_graph_ = true;

      // add two observations (current frame and the last keyframe)
      size_t added_cnt = 0;
      for (auto obs = point->obs_.rbegin(); obs != point->obs_.rend(); obs++) {
        // check if frames ahead
        if (obs->frame_id > frame->id()) continue;
        if (FramePtr f = obs->frame.lock()) {
          if (f->isKeyframe()) {
            State state;
            for (auto s = states_.rbegin(); s != states_.rend(); s++) {
              if (!s->valid()) continue;
              if (s->id != createNFrameId(f->bundleId())) continue;
              state = *s; break;
            }
            if (!state.valid()) continue;
            addReprojectionErrorResidualBlock(state, f, obs->keypoint_index_);
            added_cnt++;
          }
        }
        if (added_cnt >= 2) break;
      }
      // invalid landmark
      if (added_cnt < 2) {
        graph_->removeParameterBlock(landmark_backend_id.asInteger());
        point->in_ba_graph_ = false;
        landmarks_map_.erase(it_landmark);
      }
    }
  }

  // Store keyframes
  active_keyframes_.push_back(frame);
}

// Add landmark reprojection error block
ceres::ResidualBlockId VisualEstimatorBase::addReprojectionErrorResidualBlock(
  const State& state, const FramePtr& frame, const size_t keypoint_idx)
{
  const BackendId pose_id = createNFrameId(frame->bundleId());
  CHECK(state.id == pose_id);
  CHECK_GE(frame->level_vec_(keypoint_idx), 0);
  const int cam_idx = frame->getNFrameIndex();
  // get Landmark ID.
  const BackendId landmark_backend_id = createLandmarkId(
        frame->track_id_vec_[keypoint_idx]);
  if (!isLandmarkInEstimator(landmark_backend_id)) {
    LOG(WARNING) << "Landmark " << landmark_backend_id << " not added!";
    return nullptr;
  }

  KeypointIdentifier kid(frame, keypoint_idx);
  // check for double observations
  const auto& obs = landmarks_map_.at(landmark_backend_id).observations.find(kid);
  if (obs != landmarks_map_.at(landmark_backend_id).observations.end()) {
    return ceres::ResidualBlockId(obs->second);
  }

  Eigen::Matrix2d information = Eigen::Matrix2d::Identity();
  if (landmarks_map_.at(landmark_backend_id).num_observations_historical > 
      visual_base_options_.min_observation_stable) {
    information /= square(visual_base_options_.stable_feature_error_std * 
      static_cast<double>(1 << frame->level_vec_(keypoint_idx)));
  }
  else {
    information /= square(visual_base_options_.feature_error_std * 
      static_cast<double>(1 << frame->level_vec_(keypoint_idx)));
  }

  // create error term
  std::shared_ptr<ReprojectionError> reprojection_error =
      std::make_shared<ReprojectionError>(
        frame->cam(),
        frame->px_vec_.col(keypoint_idx), information);
  ceres::ResidualBlockId ret_val = graph_->addResidualBlock(
        reprojection_error,
        cauchy_loss_function_ ? cauchy_loss_function_.get() : nullptr,
        graph_->parameterBlockPtr(state.id_in_graph.asInteger()),
        graph_->parameterBlockPtr(landmark_backend_id.asInteger()),
        graph_->parameterBlockPtr(camera_extrinsics_id_.asInteger()));

  // remember
  landmarks_map_.at(landmark_backend_id).observations.insert(
        std::pair<KeypointIdentifier, uint64_t>(
          kid, reinterpret_cast<uint64_t>(ret_val)));
  landmarks_map_.at(landmark_backend_id).num_observations_historical++;

  return ret_val;
}

// Add landmark reprojection error blocks
void VisualEstimatorBase::addReprojectionErrorResidualBlocks(
  const State& state, const FramePtr& frame)
{
  for (size_t kp_idx = 0; kp_idx < frame->numFeatures(); ++kp_idx)
  {
    const PointPtr& point = frame->landmark_vec_[kp_idx];
    const FeatureType& type = frame->type_vec_[kp_idx];

    // check if feature is associated to landmark
    if (point == nullptr || !isSeed(frame->type_vec_[kp_idx])) {
      continue;
    }
    // check if landmark is in backend
    if (!isLandmarkInEstimator(createLandmarkId(point->id()))) continue;

    if (!addReprojectionErrorResidualBlock(state, frame, kp_idx)) {
      LOG(WARNING) << "Failed to add an observation!";
      continue;
    }
  }
}

// Add initial camera extrinsics error
void VisualEstimatorBase::addCameraExtrinsicsResidualBlock(
  const BackendId& camera_extrinsics_id, 
  const Transformation& T_SC_prior, 
  const Eigen::Vector3d& std_translation, const Eigen::Vector3d& std_orientation)
{
  // set the parameter as constant
  if ((std_translation[0] * std_translation[1] * std_translation[2] == 0.0) && 
      (std_orientation[0] * std_orientation[1] * std_orientation[2] == 0.0)) {
    graph_->setParameterBlockConstant(camera_extrinsics_id.asInteger());
    return;
  }

  Eigen::Matrix<double, 6, 6> information;
  information.setIdentity(); 
  for (size_t i = 0; i < 3; i++) {
    double std_tr = std_translation[i] == 0.0 ? 1.0e-4 : std_translation[i];
    double std_or = std_orientation[i] == 0.0 ? 1.0e-4 : std_orientation[i];
    information(i, i) = 1.0 / square(std_tr);
    information(i + 3, i + 3) = 1.0 / square(std_or);
  }
  std::shared_ptr<PoseError> pose_error = 
    std::make_shared<PoseError>(T_SC_prior, information);
  ceres::ResidualBlockId residual_id = 
    graph_->addResidualBlock(pose_error, nullptr,
      graph_->parameterBlockPtr(camera_extrinsics_id.asInteger()));
}

// Number of reprojection error
size_t VisualEstimatorBase::numReprojectionError(const FramePtr& frame)
{
  size_t num = 0;
  for(auto it = landmarks_map_.begin(); it != landmarks_map_.end(); it++)
  {
    const BackendId& landmark_id = it->first;
    MapPoint& map_point = it->second;

    for (auto obs : map_point.observations) {
      if (obs.first.frame_id != frame->id()) continue;
      num++;
    }
  }
  return num;
}

// Reject landmark reprojection error outlier
bool VisualEstimatorBase::rejectReprojectionErrorOutlier(const FramePtr& frame)
{
  const double outlier_threshold = 
    visual_base_options_.landmark_outlier_rejection_threshold;
  std::vector<PointPtr> rejected_landmarks;
  std::vector<std::pair<int, double>> residual_vec;
  for(auto it = landmarks_map_.begin(); it != landmarks_map_.end(); )
  {
    const BackendId& landmark_id = it->first;
    MapPoint& map_point = it->second;

    // calculate residual
    bool need_add_it = true;
    for (auto obs : map_point.observations) {
      if (obs.first.frame_id != frame->id()) continue;

      ceres::ResidualBlockId residual_block_id = ceres::ResidualBlockId(obs.second);
      std::shared_ptr<ErrorInterface> interface = 
        graph_->residualBlockIdToResidualBlockSpecMap().at(residual_block_id).error_interface_ptr;
      Eigen::VectorXd Residuals = Eigen::VectorXd::Zero(2);
      graph_->problem()->EvaluateResidualBlock(
          residual_block_id, false, nullptr, Residuals.data(), nullptr);
      interface->deNormalizeResidual(Residuals.data());
      // outlier detected
      if (Residuals.norm() > outlier_threshold) {
        rejected_landmarks.push_back(it->second.point);
#if 0  // only reject current observation
        // erase in backend
        eraseReprojectionErrorResidualBlock(residual_block_id);
        residual_vec.push_back(std::make_pair((int)landmark_id.trackId(), Residuals.norm()));

        // notify to frontend
        frame->type_vec_[obs.first.keypoint_index_] = FeatureType::kOutlier;

        // check if no sufficient observation left
        if (map_point.observations.size() <= 1) {
          graph_->removeParameterBlock(landmark_id.asInteger());
          map_point.point->in_ba_graph_ = false;
          it = landmarks_map_.erase(it);
          need_add_it = false;
        }
#else  // reject landmark parameter
        residual_vec.push_back(std::make_pair((int)landmark_id.trackId(), Residuals.norm()));

        // erase landmark
        bool has_margin_residual = false;
        auto residuals = graph_->residuals(landmark_id.asInteger());
        for (auto r : residuals) {
          if (r.error_interface_ptr->typeInfo() == ErrorType::kMarginalizationError) {
            has_margin_residual = true;
            break;
          }
        }
        // not connected to margin error, directly erase
        if (!has_margin_residual) {
          graph_->removeParameterBlock(landmark_id.asInteger());
          map_point.point->in_ba_graph_ = false;
          it = landmarks_map_.erase(it);
          need_add_it = false;
        }
        // connected to margin error, erase current reprojection error
        else {
          eraseReprojectionErrorResidualBlock(residual_block_id);
        }

        // notify to frontend
        for (auto o : rejected_landmarks.back()->obs_) {
          if (FramePtr f = o.frame.lock()) {
            f->type_vec_[o.keypoint_index_] = FeatureType::kOutlier;
          }
        }
#endif
        break;
      }
    }

    if (need_add_it) it++;
  }

  if (base_options_.verbose_output && rejected_landmarks.size() > 0) {
    LOG(INFO) << "Rejected " << rejected_landmarks.size() << " landmark outliers. Remaining " 
              << landmarks_map_.size() << " landmarks.";
    for (size_t i = 0; i < residual_vec.size(); i++) {
      LOG(INFO) << "Rejected landmark outlier " 
        << std::fixed << residual_vec[i].first 
        << ": residual = " << residual_vec[i].second;
    }
  }

  return rejected_landmarks.size() > 0 ? true : false;
}

// Add landmark blocks in corresponding with given camera state to marginalizer
void VisualEstimatorBase::addLandmarkParameterMarginBlocksWithResiduals(
  const State& state, bool keep)
{
  // only apply marginalization for keyframe
  if (!state.is_keyframe) return;

  const BackendId& margin_keyframe_id = state.id_in_graph;
  for(PointMap::iterator pit = landmarks_map_.begin();
      pit != landmarks_map_.end();)
  {
    Graph::ResidualBlockCollection residuals =
        graph_->residuals(pit->first.asInteger());
    CHECK(residuals.size() != 0) << ": " << pit->second.observations.size();

    // First loop: check if we can skip
    bool at_pose_to_margin = false;
    size_t num_at_other_poses = 0;
    for (size_t r = 0; r < residuals.size(); ++r)
    {
      if (residuals[r].error_interface_ptr->typeInfo() 
          != ErrorType::kReprojectionError) continue;

      BackendId pose_id(graph_->parameters(
          residuals[r].residual_block_id).at(0).first);
      bool is_keyframe = false;
      for (auto s : states_) {
        if (s.is_keyframe && (s.id == pose_id)) {
          is_keyframe = true; break;
        }
      }
      // in case that we have already erased this state
      if (pose_id == margin_keyframe_id) is_keyframe = true;

      // if the landmark is visible in the frames to marginalize
      if(pose_id == margin_keyframe_id) at_pose_to_margin = true;

      // the landmark is still visible in keyframes of the sliding window
      if(is_keyframe && (pose_id != margin_keyframe_id)) {
        num_at_other_poses++;
      }
    }

    // the landmark is not affected by the marginalization
    if(!at_pose_to_margin) {
      pit++;
      continue;
    }

    // Second loop: actually collect residuals to marginalize
    bool margin_parameter = false;
    for (size_t r = 0; r < residuals.size(); ++r)
    {
      if (residuals[r].error_interface_ptr->typeInfo() 
          != ErrorType::kReprojectionError) continue;
      
      BackendId pose_id(graph_->parameters(
        residuals[r].residual_block_id).at(0).first);
      
      // can be observed by at least two keyframes, margin the observation 
      // at current keyframe
      if (at_pose_to_margin && num_at_other_poses >= 2)
      {
        if (pose_id == margin_keyframe_id) {
          marginalization_error_->addResidualBlock(
                residuals[r].residual_block_id);
        }
      }
      // cannot be observed by at least two other keyframes, marginalize the landmark 
      // and its keyframe observations, and erase non-keyframe observations.
      else if(at_pose_to_margin && num_at_other_poses < 2)
      {
        // add information to be considered in marginalization later.
        margin_parameter = true;
        // check if keyframe
        bool is_keyframe = false;
        for (auto s : states_) {
          if (s.is_keyframe && (s.id == pose_id)) {
            is_keyframe = true; break;
          }
        }
        // in case that we have already erased this state
        if (pose_id == margin_keyframe_id) is_keyframe = true;
        // marginalize keyframe observation
        if (is_keyframe) {
          marginalization_error_->addResidualBlock(
                residuals[r].residual_block_id);
        }
        // erase non-keyframe observation
        else {
          eraseReprojectionErrorResidualBlock(residuals[r].residual_block_id);
        }
      }
    }

    // deal with parameter blocks
    if (margin_parameter) {
      marginalization_parameter_ids_.push_back(pit->first);
      marginalization_keep_parameter_blocks_.push_back(keep);
      pit->second.point->in_ba_graph_ = false;
      pit = landmarks_map_.erase(pit);
      continue;
    }

    pit++;
  } 

  // Erase active keyframe
  BackendId pose_id = state.id;
  for (auto frame = active_keyframes_.begin(); frame != active_keyframes_.end(); frame++) {
    if ((*frame)->bundleId() != pose_id.bundleId()) continue;
    active_keyframes_.erase(frame); break;
  }
}

// Erase landmark observation
void VisualEstimatorBase::eraseReprojectionErrorResidualBlock(
  ceres::ResidualBlockId residual_block_id)
{
  const Graph::ParameterBlockCollection parameters =
      graph_->parameters(residual_block_id);
  const BackendId landmark_id(parameters.at(1).first);
  CHECK(landmark_id.type() == IdType::cLandmark);
  // remove in landmarksMap
  MapPoint& map_point = landmarks_map_.at(landmark_id);
  for(auto it = map_point.observations.begin();
      it!= map_point.observations.end();)
  {
    if(it->second == uint64_t(residual_block_id))
    {
      it = map_point.observations.erase(it);
    }
    else
    {
      ++it;
    }
  }
  // remove residual block
  graph_->removeResidualBlock(residual_block_id);
}

// Erase landmark observations at a given state
void VisualEstimatorBase::eraseReprojectionErrorResidualBlocks(const State& state)
{
  Graph::ResidualBlockCollection residuals = 
    graph_->residuals(state.id_in_graph.asInteger());
  for (size_t r = 0; r < residuals.size(); ++r) {
    if (residuals[r].error_interface_ptr->typeInfo() 
        != ErrorType::kReprojectionError) continue;
        
    const Graph::ParameterBlockCollection parameters =
        graph_->parameters(residuals[r].residual_block_id);
    const BackendId landmark_id(parameters.at(1).first);
    MapPoint& map_point = landmarks_map_.at(landmark_id);

    eraseReprojectionErrorResidualBlock(residuals[r].residual_block_id);
  }
}

// Erase landmarks which have no observations
void VisualEstimatorBase::eraseEmptyLandmarks()
{
  for(PointMap::iterator pit = landmarks_map_.begin();
      pit != landmarks_map_.end();) {
    if (pit->second.observations.size() == 0) {
      graph_->removeParameterBlock(pit->first.asInteger());
      pit->second.point->in_ba_graph_ = false;
      pit = landmarks_map_.erase(pit);
    }
    else pit++;
  }
}

// Set all features as outliers to make them untracked
void VisualEstimatorBase::untrackAllLandmarks()
{
  for(auto it = landmarks_map_.begin(); it != landmarks_map_.end(); it++) {
    for (auto o : it->second.point->obs_) {
      if (FramePtr f = o.frame.lock()) {
        f->type_vec_[o.keypoint_index_] = FeatureType::kOutlier;
      }
    }
  }
}

// Update landmarks by estimator
void VisualEstimatorBase::updateLandmarks()
{
  for(auto &id_and_map_point : landmarks_map_) {
    // update coordinates
    id_and_map_point.second.hom_coordinates =
        std::static_pointer_cast<HomogeneousPointParameterBlock>(
          graph_->parameterBlockPtr(
            id_and_map_point.first.asInteger()))->estimate();
    CHECK(id_and_map_point.second.hom_coordinates(3) == 1.0);
    // update map points to frontend
    id_and_map_point.second.point->pos_=
        id_and_map_point.second.hom_coordinates.head<3>();
  }
}

// Update frame pose and speed and bias to frontend
void VisualEstimatorBase::updateFrameStateToFrontend(
  const State& state, const FramePtr& frame)
{
  frame->set_T_w_imu(getPoseEstimate(state));
}

// Get extrinsics estimate
Transformation VisualEstimatorBase::getCameraExtrinsicsEstimate()
{
  std::shared_ptr<PoseParameterBlock> block_ptr =
      std::static_pointer_cast<PoseParameterBlock>(
        graph_->parameterBlockPtr(camera_extrinsics_id_.asInteger()));
  CHECK(block_ptr != nullptr);
  return block_ptr->estimate();
}

}