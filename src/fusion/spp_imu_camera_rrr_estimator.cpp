/**
* @Function: SPP/IMU/Camera tightly couple estimator (GNSS raw (SPP formula) + IMU raw + camera raw)
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
*
* Copyright (C) 2023 by Cheng Chi, All rights reserved.
**/
#include "gici/fusion/spp_imu_camera_rrr_estimator.h"

#include "gici/gnss/position_error.h"

namespace gici {

// The default constructor
SppImuCameraRrrEstimator::SppImuCameraRrrEstimator(
               const SppImuCameraRrrEstimatorOptions& options, 
               const GnssImuInitializerOptions& init_options, 
               const SppEstimatorOptions spp_options,
               const GnssEstimatorBaseOptions& gnss_base_options, 
               const GnssLooseEstimatorBaseOptions& gnss_loose_base_options, 
               const VisualEstimatorBaseOptions& visual_base_options,
               const ImuEstimatorBaseOptions& imu_base_options,
               const EstimatorBaseOptions& base_options) :
  rrr_options_(options), spp_options_(spp_options),
  GnssEstimatorBase(gnss_base_options, base_options),
  VisualEstimatorBase(visual_base_options, base_options),
  ImuEstimatorBase(imu_base_options, base_options),
  EstimatorBase(base_options)
{
  type_ = EstimatorType::SppImuCameraRrr;
  states_.push_back(State());
  gnss_measurements_.push_back(GnssMeasurement());
  frame_bundles_.push_back(nullptr);
  num_satellites_ = 0;

  // Initialization control
  initializer_sub_estimator_.reset(new SppEstimator(
    spp_options, gnss_base_options, base_options));
  gnss_imu_initializer_.reset(new GnssImuInitializer(
    init_options, gnss_loose_base_options, imu_base_options, 
    base_options, graph_, initializer_sub_estimator_));
}

// The default destructor
SppImuCameraRrrEstimator::~SppImuCameraRrrEstimator()
{}

// Add measurement
bool SppImuCameraRrrEstimator::addMeasurement(const EstimatorDataCluster& measurement)
{
  // GNSS/IMU initialization
  if (coordinate_ == nullptr || !gravity_setted_) return false;
  if (!gnss_imu_initializer_->finished()) {
    if (gnss_imu_initializer_->getCoordinate() == nullptr) {
      gnss_imu_initializer_->setCoordinate(coordinate_);
      initializer_sub_estimator_->setCoordinate(coordinate_);
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

  // Add GNSS
  if (measurement.gnss && measurement.gnss_role == GnssRole::Rover) {
    return addGnssMeasurementAndState(*measurement.gnss);
  }

  // Add images
  if (measurement.frame_bundle) {
    if (!visual_initialized_) return visualInitialization(measurement.frame_bundle);
    return addImageMeasurementAndState(measurement.frame_bundle);
  }

  return false;
}

// Add GNSS measurements and state
bool SppImuCameraRrrEstimator::addGnssMeasurementAndState(
    const GnssMeasurement& measurement)
{
  // Get prior states
  Eigen::Vector3d position_prior = coordinate_->convert(
    getPoseEstimate().getPosition(), GeoType::ENU, GeoType::ECEF);

  // Set to local measurement handle
  curGnss() = measurement;
  curGnss().position = position_prior;

  // Correct code bias
  correctCodeBias(curGnss());

  // Compute ionosphere delays
  computeIonosphereDelay(curGnss(), true);

  // Add parameter blocks
  double timestamp = curGnss().timestamp;
  // pose and speed and bias block
  const int32_t bundle_id = curGnss().id;
  BackendId pose_id = createGnssPoseId(bundle_id);
  size_t index = insertImuState(timestamp, pose_id);
  states_[index].status = GnssSolutionStatus::Single;
  latest_state_index_ = index;
  // GNSS extrinsics, it should be added at initialization step
  CHECK(gnss_extrinsics_id_.valid());
  // clock block
  int num_valid_system = 0;
  addClockParameterBlocks(curGnss(), curGnss().id, num_valid_system, 
    std::map<char, double>(), true);
  // frequency block
  int num_valid_doppler_system = 0;
  addFrequencyParameterBlocks(curGnss(), curGnss().id, num_valid_doppler_system, 
    std::map<char, double>(), true);

  // Add pseudorange residual blocks
  int num_valid_satellite = 0;
  addPseudorangeResidualBlocks(curGnss(), curState(), num_valid_satellite, true);
  
  // We do not need to check if the number of satellites is sufficient in tightly fusion.
  if (!checkSufficientSatellite(num_valid_satellite, num_valid_system)) {
    // do nothing
  }
  num_satellites_ = num_valid_satellite;

  // No satellite
  if (num_satellites_ == 0) {
    // erase parameters in current state
    eraseFrequencyParameterBlocks(states_[index]);
    eraseClockParameterBlocks(states_[index]);
    eraseImuState(states_[index]);
    return false;
  }

  // Add doppler residual blocks
  addDopplerResidualBlocks(curGnss(), curState(), num_valid_satellite, 
    true, getImuMeasurementNear(timestamp).angular_velocity);

  // Add relative errors
  if (lastGnssState().valid()) {  // maybe invalid here because of long term GNSS absent
    // frequency
    addRelativeFrequencyResidualBlock(lastGnssState(), states_[index]);
  }

  // ZUPT
  addZUPTResidualBlock(curState());

  // Car motion
  if (imu_base_options_.car_motion) {
    // heading measurement constraint
    addHMCResidualBlock(states_[index]);
    // non-holonomic constraint
    addNHCResidualBlock(states_[index]);
  }

  // Compute DOP
  updateGdop(curGnss());

  return true;
}

// Add image measurements and state
bool SppImuCameraRrrEstimator::addImageMeasurementAndState(
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
    addCameraExtrinsicsResidualBlock(camera_extrinsics_id_, curFrame()->T_imu_cam(), 
      visual_base_options_.camera_extrinsics_initial_std.head<3>(), 
      visual_base_options_.camera_extrinsics_initial_std.tail<3>() * D2R);
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
bool SppImuCameraRrrEstimator::visualInitialization(const FrameBundlePtr& frame_bundle)
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
  if (std_yaw > rrr_options_.min_yaw_std_init_visual * D2R) return false;
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
bool SppImuCameraRrrEstimator::estimate()
{
  status_ = EstimatorStatus::Converged;

  // Optimize
  optimize();

  // Get current sensor type
  IdType new_state_type = states_[latest_state_index_].id.type();

  // GNSS processes
  double gdop = 0.0;
  if (new_state_type == IdType::gPose)
  {
    // Reject GNSS outliers
    size_t n_pseudorange = numPseudorangeError(states_[latest_state_index_]);
    size_t n_doppler = numDopplerError(states_[latest_state_index_]);
    if (gnss_base_options_.use_outlier_rejection) {
      rejectPseudorangeOutlier(states_[latest_state_index_]);
      rejectDopplerOutlier(states_[latest_state_index_]);
    }

    // Check if we rejected too many GNSS residuals
    double ratio_pseudorange = n_pseudorange == 0.0 ? 0.0 : 1.0 - 
      getDivide(numPseudorangeError(states_[latest_state_index_]), n_pseudorange);
    double ratio_doppler = n_doppler == 0.0 ? 0.0 : 1.0 - 
      getDivide(numDopplerError(states_[latest_state_index_]), n_doppler);
    const double thr = gnss_base_options_.diverge_max_reject_ratio;
    if (isGnssGoodObservation() && 
        (ratio_pseudorange > thr || ratio_doppler > thr)) {
      num_cotinuous_reject_gnss_++;
    }
    else num_cotinuous_reject_gnss_ = 0;
    if (num_cotinuous_reject_gnss_ > 
        gnss_base_options_.diverge_min_num_continuous_reject) {
      LOG(WARNING) << "Estimator diverge: Too many GNSS outliers rejected!";
      status_ = EstimatorStatus::Diverged;
      num_cotinuous_reject_gnss_ = 0;
    }
  }

  // Image processes
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
      << ", Sat number: " << std::setw(2) << num_satellites_
      << ", GDOP: " << std::setprecision(1) << std::fixed << gdop_;
  }

  // Apply marginalization
  marginalization(new_state_type);

  // Shift memory for states and measurements
  if (new_state_type == IdType::gPose) {
    gnss_measurements_.push_back(GnssMeasurement());
  }
  if (new_state_type == IdType::cPose) frame_bundles_.push_back(nullptr);
  states_.push_back(State());
  while (!visual_initialized_ && 
         states_.size() > rrr_options_.max_gnss_window_length_minor) {
    states_.pop_front();
    gnss_measurements_.pop_front();
  }
  // only keep frame measurement data for two epochs
  while (frame_bundles_.size() > 2) frame_bundles_.pop_front();
  
  return true;
}

// Set initializatin result
void SppImuCameraRrrEstimator::setInitializationResult(
  const std::shared_ptr<MultisensorInitializerBase>& initializer)
{
  CHECK(initializer->finished());

  // Cast to desired initializer
  std::shared_ptr<GnssImuInitializer> gnss_imu_initializer = 
    std::static_pointer_cast<GnssImuInitializer>(initializer);
  CHECK_NOTNULL(gnss_imu_initializer);
  
  // Arrange to window length
  ImuMeasurements imu_measurements;
  std::deque<GnssSolution> gnss_measurement_temp;  // we do not use this
  gnss_imu_initializer->arrangeToEstimator(
    rrr_options_.max_gnss_window_length_minor, marginalization_error_, states_, 
    marginalization_residual_id_, gnss_extrinsics_id_, 
    gnss_measurement_temp, imu_measurements);
  for (auto it = imu_measurements.rbegin(); it != imu_measurements.rend(); it++) {
    imu_mutex_.lock();
    imu_measurements_.push_front(*it);
    imu_mutex_.unlock();
  }

  // Shift memory for states and measurements
  states_.push_back(State());
  gnss_measurements_.resize(states_.size());
  frame_bundles_.push_back(nullptr);
}

// Marginalization
bool SppImuCameraRrrEstimator::marginalization(const IdType& type)
{
  if (type == IdType::cPose) return frameMarginalization();
  else if (type == IdType::gPose) return gnssMarginalization();
  else return false;
}

// Marginalization when the new state is a frame state
bool SppImuCameraRrrEstimator::frameMarginalization()
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
      sizeOfKeyframeStates() > rrr_options_.max_keyframes) {
    // Erase old marginalization item
    if (!eraseOldMarginalization()) return false;

    // Add marginalization items
    // marginalize oldest keyframe state
    bool reached_first_keyframe = false;
    State margin_keyframe_state;
    int n_state = 0, n_frame = 0, n_gnss = 0;
    for (auto it = states_.begin(); it != states_.end();) {
      State& state = *it;
      // reached the first keyframe
      if (state.is_keyframe) reached_first_keyframe = true;
      
      // mark marginalized keyframe state
      if (state.is_keyframe) margin_keyframe_state = state;

      // add margin blocks
      // Keyframe, we add the reprojection errors latter.
      if (state.is_keyframe) {
        addImuStateMarginBlock(state);
        addImuResidualMarginBlocks(state);
      }
      // GNSS state that not yet been marginalized by GNSS steps.
      else {
        addGnssLooseResidualMarginBlocks(state);
        addImuStateMarginBlock(state);
        addImuResidualMarginBlocks(state);
        addClockMarginBlocksWithResiduals(state);
        addFrequencyMarginBlocksWithResiduals(state);
        addGnssResidualMarginBlocks(state);

        // erase GNSS measurements
        if (gnssMeasurementAt(state.timestamp) != gnss_measurements_.end()) {
          gnss_measurements_.erase(gnssMeasurementAt(state.timestamp));
        }
      }

      // Erase state
      it = states_.erase(it);

      n_state++;

      if (reached_first_keyframe) break;
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
bool SppImuCameraRrrEstimator::gnssMarginalization()
{
  // Check if we need marginalization
  if (isFirstEpoch()) return true;

  // Visual not initialzed, only handle GNSS and INS
  if (!visual_initialized_) {
    // Check if we need marginalization
    if (states_.size() < rrr_options_.max_gnss_window_length_minor) {
      return true;
    }

    // Erase old marginalization item
    if (!eraseOldMarginalization()) return false;

    // Add marginalization items
    // IMU states and residuals
    addImuStateMarginBlockWithResiduals(oldestState());
    // clock
    addClockMarginBlocksWithResiduals(oldestState());
    // frequency
    addFrequencyMarginBlocksWithResiduals(oldestState());

    // Apply marginalization and add the item into graph
    bool ret = applyMarginalization();

    return ret;
  }
  // Visual initialized, handle all the sensors
  else {
    // Sparsify GNSS states
    sparsifyGnssStates();

    // Marginalize the GNSS states that in front of the oldest keyframe
    // We do this at both here and the visual margin step to smooth computational load
    for (auto it = states_.begin(); it != states_.end();) {
      State& state = *it;
      // reached the oldest keyframe
      if (state.is_keyframe) break;
      // check if GNSS state
      if (state.id.type() != IdType::gPose) continue;

      // Erase old marginalization item
      if (!eraseOldMarginalization()) return false;

      // margin
      addGnssLooseResidualMarginBlocks(state);
      addImuStateMarginBlock(state);
      addImuResidualMarginBlocks(state);
      addClockMarginBlocksWithResiduals(state);
      addFrequencyMarginBlocksWithResiduals(state);
      addGnssResidualMarginBlocks(state);

      // erase GNSS measurements
      if (gnssMeasurementAt(state.timestamp) != gnss_measurements_.end()) {
        gnss_measurements_.erase(gnssMeasurementAt(state.timestamp));
      }

      // Erase state
      it = states_.erase(it);

      // just handle one in one time
      bool ret = applyMarginalization();

      return ret;
    }
  }

  return true;
}

// Sparsify GNSS states to bound computational load
void SppImuCameraRrrEstimator::sparsifyGnssStates()
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
  if (gnss_ids.size() <= rrr_options_.max_keyframes) return;

  // Erase some GNSS states
  // The states with most neighbors will be erased first
  int num_to_erase = gnss_ids.size() - rrr_options_.max_keyframes;
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
    State& state = states_[i];

    eraseGnssMeasurementResidualBlocks(state);
    eraseClockParameterBlocks(state);
    eraseFrequencyParameterBlocks(state);
    eraseGnssLooseResidualBlocks(state);
    eraseImuState(state);

    if (gnssMeasurementAt(state.timestamp) != gnss_measurements_.end()) {
      gnss_measurements_.erase(gnssMeasurementAt(state.timestamp));
    }

    i--;
  }
}

};