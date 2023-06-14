/**
* @Function: GNSS/IMU/Camera semi-tightly couple estimator (GNSS solution + IMU raw + camera raw)
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
*
* Copyright (C) 2023 by Cheng Chi, All rights reserved.
**/
#include "gici/fusion/gnss_imu_camera_srr_estimator.h"

namespace gici {

// The default constructor
GnssImuCameraSrrEstimator::GnssImuCameraSrrEstimator(
               const GnssImuCameraSrrEstimatorOptions& options, 
               const GnssImuInitializerOptions& init_options, 
               const GnssLooseEstimatorBaseOptions& gnss_loose_base_options, 
               const VisualEstimatorBaseOptions& visual_base_options,
               const ImuEstimatorBaseOptions& imu_base_options,
               const EstimatorBaseOptions& base_options) :
  srr_options_(options), 
  GnssLooseEstimatorBase(gnss_loose_base_options, base_options),
  VisualEstimatorBase(visual_base_options, base_options),
  ImuEstimatorBase(imu_base_options, base_options),
  EstimatorBase(base_options)
{
  type_ = EstimatorType::GnssImuCameraSrr;
  states_.push_back(State());
  gnss_solution_measurements_.push_back(GnssSolution());
  frame_bundles_.push_back(nullptr);

  // Initialization control
  gnss_imu_initializer_.reset(new GnssImuInitializer(
    init_options, gnss_loose_base_options, imu_base_options, 
    base_options, graph_));
}

// The default destructor
GnssImuCameraSrrEstimator::~GnssImuCameraSrrEstimator()
{}

// Add measurement
bool GnssImuCameraSrrEstimator::addMeasurement(const EstimatorDataCluster& measurement)
{
  // GNSS/IMU initialization
  if (coordinate_ == nullptr || !gravity_setted_) return false;
  if (!gnss_imu_initializer_->finished()) {
    if (gnss_imu_initializer_->getCoordinate() == nullptr) {
      gnss_imu_initializer_->setCoordinate(coordinate_);
      gnss_imu_initializer_->setGravity(imu_base_options_.imu_parameters.g);
    }
    if (gnss_imu_initializer_->addMeasurement(measurement)) {
      gnss_imu_initializer_->estimate();
      // set result to estimator
      setInitializationResult(gnss_imu_initializer_);
    }
    return false;
  }

  // Add IMU
  if (measurement.imu && measurement.imu_role == ImuRole::Major) {
    addImuMeasurement(*measurement.imu);
  }

  // Add GNSS by solution measurement
  if (measurement.solution && 
      (measurement.solution_role == SolutionRole::Position || 
       measurement.solution_role == SolutionRole::Velocity ||
       measurement.solution_role == SolutionRole::PositionAndVelocity)) {
    GnssSolution gnss_solution = convertSolutionToGnssSolution(
      *measurement.solution, measurement.solution_role);
    return addGnssSolutionMeasurementAndState(gnss_solution);
  }

  // Add images
  if (measurement.frame_bundle) {
    if (!visual_initialized_) return visualInitialization(measurement.frame_bundle);
    return addImageMeasurementAndState(measurement.frame_bundle);
  }

  return false;
}

// Add GNSS measurements and state
bool GnssImuCameraSrrEstimator::addGnssSolutionMeasurementAndState(
  const GnssSolution& measurement)
{
  // Set to local measurement handle
  curGnssSolution() = measurement;

  // Add parameter blocks
  double timestamp = curGnssSolution().timestamp;
  // pose and speed and bias block
  const int32_t bundle_id = curGnssSolution().id;
  BackendId pose_id = createGnssPoseId(bundle_id);
  size_t index = insertImuState(timestamp, pose_id);
  states_[index].status = curGnssSolution().status;
  latest_state_index_ = index;
  // GNSS extrinsics, it should be added at initialization step
  CHECK(gnss_extrinsics_id_.valid());

  // Add residual blocks
  // GNSS position
  addGnssPositionResidualBlock(curGnssSolution(), states_[index]);
  // GNSS velocity
  addGnssVelocityResidualBlock(curGnssSolution(), states_[index], 
    getImuMeasurementNear(timestamp).angular_velocity);
  // Car motion
  if (imu_base_options_.car_motion) {
    // heading measurement constraint
    addHMCResidualBlock(states_[index]);
    // non-holonomic constraint
    addNHCResidualBlock(states_[index]);
  }

  return true;
}

// Add image measurements and state
bool GnssImuCameraSrrEstimator::addImageMeasurementAndState(
  const FrameBundlePtr& frame_bundle, const SpeedAndBias& speed_and_bias)
{
  // If initialized, we are supposed not at the first epoch
  CHECK(!isFirstEpoch());

  // Check frequency
  if (!frame_bundle->isKeyframe() && frame_bundles_.size() > 1) {
    const double t_last = lastFrameBundle()->getMinTimestampSeconds();
    const double t_cur = frame_bundle->getMinTimestampSeconds();
    if (t_cur - t_last < 1.0 / visual_base_options_.max_frequency) return false;
  }

  // Set to local measurement handle
  curFrameBundle() = frame_bundle;

  // Add parameter blocks
  double timestamp = curFrame()->getTimestampSec();
  // pose and speed and bias block
  const int32_t bundle_id = curFrame()->bundleId();
  BackendId pose_id = createNFrameId(bundle_id);
  size_t index;
  if (speed_and_bias != SpeedAndBias::Zero()) {
    index = insertImuState(timestamp, pose_id, 
      curFrame()->T_world_imu(), speed_and_bias, true);
  }
  else {
    index = insertImuState(timestamp, pose_id);
  }
  states_[index].status = latestGnssState().status;
  states_[index].is_keyframe = curFrame()->isKeyframe();
  latest_state_index_ = index;
  // camera extrinsics
  if (!camera_extrinsics_id_.valid()) {
    camera_extrinsics_id_ = 
      addCameraExtrinsicsParameterBlock(bundle_id, curFrame()->T_imu_cam());
  }

  // Initialize landmarks
  if (visual_initialized_ && curFrame()->isKeyframe()) {
    curFrame()->set_T_w_imu(getPoseEstimate(states_[index]));
    feature_handler_->initializeLandmarks(curFrame());
  }

  // Add Landmark parameters and minimal resiudals at keyframe
  if (curFrame()->isKeyframe()) {
    addLandmarkParameterBlocksWithResiduals(curFrame());
  }

  // Add landmark observations
  addReprojectionErrorResidualBlocks(states_[index], curFrame());

  return true;
}

// Visual initialization
bool GnssImuCameraSrrEstimator::visualInitialization(const FrameBundlePtr& frame_bundle)
{
  // store poses
  do_not_remove_imu_measurements_ = true;
  Solution solution;
  solution.timestamp = getTimestamp();
  solution.pose = getPoseEstimate();
  solution.speed_and_bias = getSpeedAndBiasEstimate();
  init_solution_store_.push_back(solution);
  // store keyframes
  if (frame_bundle->isKeyframe()) {
    init_keyframes_.push_back(frame_bundle);
    while (init_keyframes_.size() > 2) init_keyframes_.pop_front();
  } 
  if (init_keyframes_.size() < 2) return false;
  // check if GNSS/IMU estimator fully converged
  double std_yaw = sqrt(computeAndGetCovariance(lastState())(5, 5));
  if (std_yaw > srr_options_.min_yaw_std_init_visual * D2R) return false;
  // set poses
  std::vector<SpeedAndBias> speed_and_biases;
  for (auto& frame_bundle : init_keyframes_) {
    bool found = false;
    double timestamp = frame_bundle->getMinTimestampSeconds();
    for (size_t i = 1; i < init_solution_store_.size(); i++) {
      double dt1 = init_solution_store_[i].timestamp - timestamp;
      double dt2 = init_solution_store_[i - 1].timestamp - timestamp;
      if ((dt1 >= 0 && dt2 <= 0) || 
          (i == init_solution_store_.size() - 1) && dt1 < 0 && fabs(dt1) < 2.0) {
        size_t idx = (dt1 >= 0 && dt2 <= 0) ? (i - 1) : i;
        Transformation T_WS = init_solution_store_[idx].pose;
        SpeedAndBias speed_and_bias = init_solution_store_[idx].speed_and_bias;
        imuIntegration(init_solution_store_[idx].timestamp, 
          timestamp, T_WS, speed_and_bias);
        speed_and_biases.push_back(speed_and_bias);
        frame_bundle->set_T_W_B(T_WS);
        found = true;
        break;
      }
    }
    CHECK(found);
  }
  // initialize landmarks
  feature_handler_->initializeLandmarks(init_keyframes_.back()->at(0));
  feature_handler_->setGlobalScaleInitialized();
  // add two keyframes
  CHECK(addImageMeasurementAndState(init_keyframes_.front(), speed_and_biases.front()));
  CHECK(addImageMeasurementAndState(init_keyframes_.back(), speed_and_biases.back()));

  // set flag
  visual_initialized_ = true;
  do_not_remove_imu_measurements_ = false;
  can_compute_covariance_ = true;
  return true;
}

// Solve current graph
bool GnssImuCameraSrrEstimator::estimate()
{
  // Optimize
  optimize();

  // Log information
  State& new_state = states_[latest_state_index_];
  if (base_options_.verbose_output) {
    LOG(INFO) << estimatorTypeToString(type_) << ": " 
      << "Iterations: " << graph_->summary.iterations.size() << ", "
      << std::scientific << std::setprecision(3) 
      << "Initial cost: " << graph_->summary.initial_cost << ", "
      << "Final cost: " << graph_->summary.final_cost
      << ", Sensor type: " << std::setw(1) << 
        static_cast<int>(BackendId::sensorType(new_state.id.type()))
      << ", Fix status: " << std::setw(1) << static_cast<int>(new_state.status);
  }

  // Update parameters to frontend
  IdType new_state_type = states_[latest_state_index_].id.type();
  if (new_state_type == IdType::cPose) {
    // update landmarks to frontend
    updateLandmarks();
    // update states to frontend
    updateFrameStateToFrontend(states_[latest_state_index_], curFrame());
    // reject landmark outliers
    size_t n_reprojection = numReprojectionError(curFrame());
    rejectReprojectionErrorOutlier(curFrame());
    // check if we rejected too many reprojection errors
    double ratio_reprojection = n_reprojection == 0.0 ? 0.0 : 1.0 - 
      getDivide(numReprojectionError(curFrame()), n_reprojection);
    if (ratio_reprojection > visual_base_options_.diverge_max_reject_ratio) {
      num_cotinuous_reject_visual_++;
    }
    else num_cotinuous_reject_visual_ = 0;
    if (num_cotinuous_reject_visual_ > 
        visual_base_options_.diverge_min_num_continuous_reject) {
      LOG(WARNING) << "Estimator diverge: Too many visual outliers rejected!";
      status_ = EstimatorStatus::Diverged;
      num_cotinuous_reject_visual_ = 0;
    }
  }

  // Apply marginalization
  marginalization(new_state_type);

  // Shift memory for states and measurements
  if (new_state_type == IdType::gPose) 
    gnss_solution_measurements_.push_back(GnssSolution());
  if (new_state_type == IdType::cPose) frame_bundles_.push_back(nullptr);
  states_.push_back(State());
  while (!visual_initialized_ && 
         states_.size() > srr_options_.max_gnss_window_length_minor) {
    states_.pop_front();
  }
  // only keep measurement data for two epochs
  while (gnss_solution_measurements_.size() > 2) 
    gnss_solution_measurements_.pop_front();
  while (frame_bundles_.size() > 2) frame_bundles_.pop_front();

  return true;
}

// Set initializatin result
void GnssImuCameraSrrEstimator::setInitializationResult(
  const std::shared_ptr<MultisensorInitializerBase>& initializer)
{
  CHECK(initializer->finished());

  // Cast to desired initializer
  std::shared_ptr<GnssImuInitializer> gnss_imu_initializer = 
    std::static_pointer_cast<GnssImuInitializer>(initializer);
  CHECK_NOTNULL(gnss_imu_initializer);
  
  // Arrange to window length
  ImuMeasurements imu_measurements;
  gnss_imu_initializer->arrangeToEstimator(
    srr_options_.max_gnss_window_length_minor, marginalization_error_, states_, 
    marginalization_residual_id_, gnss_extrinsics_id_, 
    gnss_solution_measurements_, imu_measurements);
  for (auto it = imu_measurements.rbegin(); it != imu_measurements.rend(); it++) {
    imu_mutex_.lock();
    imu_measurements_.push_front(*it);
    imu_mutex_.unlock();
  }

  // Shift memory for states and measurements
  states_.push_back(State());
  gnss_solution_measurements_.push_back(GnssSolution());
}

// Marginalization
bool GnssImuCameraSrrEstimator::marginalization(const IdType& type)
{
  if (type == IdType::cPose) return frameMarginalization();
  else if (type == IdType::gPose) return gnssMarginalization();
  else return false;
}

// Marginalization when the new state is a frame state
bool GnssImuCameraSrrEstimator::frameMarginalization()
{
  // Check if we need marginalization
  if (isFirstEpoch()) return true;

  // Make sure that only current state can be non-keyframe
  for (int i = 0; i < states_.size() - 1; i++) {
    if (states_[i].id.type() != IdType::cPose) continue;
    if (!states_[i].is_keyframe) {
      eraseReprojectionErrorResidualBlocks(states_[i]);
      eraseImuState(states_[i]);
      i--;
    }
  }

  // If current frame is a keyframe. Marginalize the oldest keyframe and corresponding 
  // IMU and GNSS states out. And sparsify GNSS states.
  if (states_[latest_state_index_].is_keyframe &&
      sizeOfKeyframeStates() > srr_options_.max_keyframes) {
    // Erase old marginalization item
    if (!eraseOldMarginalization()) return false;

    // Add marginalization items
    // marginalize oldest keyframe state, which actually, all the states before the 
    // second keyframe state
    bool passed_first_keyframe = false;
    State margin_keyframe_state;
    for (auto it = states_.begin(); it != states_.end();) {
      State& state = *it;
      // reached the second keyframe
      if (passed_first_keyframe && state.is_keyframe) break;
      // passed the first keyframe
      if (state.is_keyframe) passed_first_keyframe = true;
      
      // mark marginalized keyframe state
      if (state.is_keyframe) margin_keyframe_state = state;

      // add margin blocks
      // IMU parameters
      // Keyframe, we add the reprojection errors latter.
      if (state.is_keyframe) {
        addImuStateMarginBlock(state);
        addImuResidualMarginBlocks(state);
      }
      // GNSS states
      else {
        CHECK(state.id.type() == IdType::gPose) << (int)state.id.asInteger();
        addImuStateMarginBlock(state);
        addImuResidualMarginBlocks(state);
        addGnssLooseResidualMarginBlocks(state);
      }

      // Erase state
      it = states_.erase(it);
    }

    // landmarks
    eraseEmptyLandmarks();
    addLandmarkParameterMarginBlocksWithResiduals(margin_keyframe_state);

    // Apply marginalization and add the item into graph
    bool ret = applyMarginalization();

    return ret;
  }

  return true;
}

// Marginalization when the new state is a GNSS state
bool GnssImuCameraSrrEstimator::gnssMarginalization()
{
  // Check if we need marginalization
  if (isFirstEpoch()) return true;

  // Visual not initialzed, only handle GNSS and INS
  if (!visual_initialized_) {
    // check if we need marginalization
    if (states_.size() < srr_options_.max_gnss_window_length_minor) {
      return true;
    }

    // Erase old marginalization item
    if (!eraseOldMarginalization()) return false;

    // Add marginalization items
    // IMU states and residuals
    addImuStateMarginBlockWithResiduals(oldestState());

    // Apply marginalization and add the item into graph
    bool ret = applyMarginalization();

    return ret;
  }
  // Visual initialized, handle all the sensors
  else {
    sparsifyGnssStates();
  }

  return true;
}

// Sparsify GNSS states to bound computational load
void GnssImuCameraSrrEstimator::sparsifyGnssStates()
{
  // Check if we need to sparsify
  std::vector<BackendId> gnss_ids;
  std::vector<int> num_neighbors;
  int max_num_neighbors = 0;
  for (int i = 0; i < states_.size(); i++) {
    if (states_[i].id.type() != IdType::gPose) continue;
    gnss_ids.push_back(states_[i].id);
    // find neighbors
    num_neighbors.push_back(0);
    int idx = num_neighbors.size() - 1;
    if (i - 1 >= 0) for (int j = i - 1; j >= 0; j--) {
      if (states_[j].id.type() != IdType::gPose) break;
      num_neighbors[idx]++;
    } 
    if (i + 1 < states_.size()) for (int j = i; j < states_.size(); j++) {
      if (states_[j].id.type() != IdType::gPose) break;
      num_neighbors[idx]++;
    } 
    if (max_num_neighbors < num_neighbors[idx]) {
      max_num_neighbors = num_neighbors[idx];
    }
  }
  if (gnss_ids.size() <= srr_options_.max_keyframes) return;

  // Erase some GNSS states
  // The states with most neighbors will be erased first
  int num_to_erase = gnss_ids.size() - srr_options_.max_keyframes;
  std::vector<BackendId> ids_to_erase;
  for (int m = max_num_neighbors; m >= 0; m--) {
    for (size_t i = 0; i < num_neighbors.size(); i++) {
      if (num_neighbors[i] != m) continue;
      if (i == 0) continue;  // in case the first one connects to margin block
      ids_to_erase.push_back(gnss_ids[i]);
      if (ids_to_erase.size() >= num_to_erase) break;
    }
    if (ids_to_erase.size() >= num_to_erase) break;
  }
  CHECK(ids_to_erase.size() >= num_to_erase);
  for (int i = 0; i < states_.size(); i++) {
    if (std::find(ids_to_erase.begin(), ids_to_erase.end(), states_[i].id) 
        == ids_to_erase.end()) {
      continue;
    }
    eraseGnssLooseResidualBlocks(states_[i]);
    eraseImuState(states_[i]);
    i--;
  }
}

};